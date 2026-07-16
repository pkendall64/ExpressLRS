#!/usr/bin/python

from enum import Enum
import shutil
import os

from elrs_helpers import ElrsUploadResult
import BFinitPassthrough
import ETXinitPassthrough
import serials_find
import upload_via_esp8266_backpack
from firmware import DeviceType, FirmwareOptions, MCUType

import sys
from os.path import dirname
sys.path.insert(0, dirname(__file__) + '/external/esptool')

from external.esptool import esptool
sys.path.append(dirname(__file__) + "/external")

class UploadMethod(Enum):
    wifi = 'wifi'
    uart = 'uart'
    betaflight = 'bf'
    edgetx = 'etx'
    stlink = 'stlink'
    stock = 'stock'
    dir = 'dir'

    def __str__(self):
        return self.value

def upload_wifi(args, options, upload_addr):
    wifi_mode = 'upload'
    if args.force == True:
        wifi_mode = 'uploadforce'
    elif args.confirm == True:
        wifi_mode = 'uploadconfirm'
    if args.port:
        upload_addr = [args.port]
    if options.mcuType == MCUType.ESP8266:
        return upload_via_esp8266_backpack.do_upload('firmware.bin.gz', wifi_mode, upload_addr, {})
    else:
        return upload_via_esp8266_backpack.do_upload(args.file.name, wifi_mode, upload_addr, {})

def _uart_upload_mode(args):
    return 'uploadforce' if args.force == True else 'upload'

def _reset_receiver_for_uart(args, options):
    if options.deviceType != DeviceType.RX:
        return ElrsUploadResult.Success
    return BFinitPassthrough.reset_to_bootloader(
        args.port,
        args.baud,
        options.firmware,
        getattr(args, 'target_path', None),
        _uart_upload_mode(args),
        getattr(args, 'accept', None),
        getattr(args, 'platform', 'ESP82'),
    )

def _detect_esp32_uart_mode(args):
    try:
        esp = esptool.cmds.detect_chip(args.port, args.baud, "no_reset")
        return 'stub' if esp.IS_STUB else 'rom'
    except Exception:
        return None

def _esp32_full_flash_cmd(args, before):
    dir = os.path.dirname(args.file.name)
    cmd = ['--chip', args.platform.replace('-', ''), '--port', args.port, '--baud', str(args.baud), '--before', before, '--after', 'hard_reset', 'write_flash']
    if args.erase:
        cmd.append('--erase-all')
    start_addr = '0x0000' if args.platform.startswith('esp32-') else '0x1000'
    cmd.extend(['-z', '--flash_mode', 'dio', '--flash_freq', '40m', '--flash_size', 'detect', start_addr, os.path.join(dir, 'bootloader.bin'), '0x8000', os.path.join(dir, 'partitions.bin'), '0xe000', os.path.join(dir, 'boot_app0.bin'), '0x10000', args.file.name])
    return cmd

def _esp32_app_flash_cmd(args):
    cmd = ['--chip', args.platform.replace('-', ''), '--port', args.port, '--baud', str(args.baud), '--before', 'no_reset', '--after', 'hard_reset', 'write_flash']
    cmd.extend(['-z', '--flash_mode', 'dio', '--flash_freq', '40m', '--flash_size', 'detect', '0x10000', args.file.name])
    return cmd

def upload_esp8266_uart(args, options):
    if args.port == None:
        args.port = serials_find.get_serial_port()
    retval = _reset_receiver_for_uart(args, options)
    if retval != ElrsUploadResult.Success:
        return retval
    before = 'no_reset' if options.deviceType == DeviceType.RX else 'default_reset'
    try:
        cmd = ['--chip', 'esp8266', '--port', args.port, '--baud', str(args.baud), '--before', before, '--after', 'soft_reset', 'write_flash']
        if args.erase:
            cmd.append('--erase-all')
        cmd.extend(['0x0000', args.file.name])
        esptool.main(cmd)
    except Exception:
        if before == 'default_reset':
            return ElrsUploadResult.ErrorGeneral
        try:
            print("UART reset command did not enter bootloader, falling back to default reset.")
            cmd = ['--chip', 'esp8266', '--port', args.port, '--baud', str(args.baud), '--before', 'default_reset', '--after', 'soft_reset', 'write_flash']
            if args.erase:
                cmd.append('--erase-all')
            cmd.extend(['0x0000', args.file.name])
            esptool.main(cmd)
        except Exception:
            return ElrsUploadResult.ErrorGeneral
    return ElrsUploadResult.Success

def upload_esp8266_bf(args, options):
    if args.port == None:
        args.port = serials_find.get_serial_port()
    mode = 'upload'
    if args.force == True:
        mode = 'uploadforce'
    passthrough_args = ['-p', args.port, '-b', str(args.baud), '-r', options.firmware, '-a', mode]
    if getattr(args, 'accept', None):
        passthrough_args.extend(['--accept', args.accept])
    if getattr(args, 'target_path', None):
        passthrough_args.extend(['--target-path', args.target_path])
    retval = BFinitPassthrough.main(passthrough_args)
    if retval != ElrsUploadResult.Success:
        return retval
    try:
        cmd = ['--passthrough', '--chip', 'esp8266', '--port', args.port, '--baud', str(args.baud), '--before', 'no_reset', '--after', 'soft_reset', 'write_flash']
        if args.erase: cmd.append('--erase-all')
        cmd.extend(['0x0000', args.file.name])
        esptool.main(cmd)
    except:
        return ElrsUploadResult.ErrorGeneral
    return ElrsUploadResult.Success

def upload_esp32_uart(args, options):
    if args.port == None:
        args.port = serials_find.get_serial_port()
    retval = _reset_receiver_for_uart(args, options)
    if retval != ElrsUploadResult.Success:
        return retval
    try:
        if options.deviceType != DeviceType.RX:
            esptool.main(_esp32_full_flash_cmd(args, 'default_reset'))
            return ElrsUploadResult.Success

        mode = _detect_esp32_uart_mode(args)
        if mode == 'stub':
            if args.erase:
                print("ESP32 serial update mode only supports flashing the main firmware image. Use ROM bootloader mode for erase-all.")
                return ElrsUploadResult.ErrorGeneral
            esptool.main(_esp32_app_flash_cmd(args))
        elif mode == 'rom':
            esptool.main(_esp32_full_flash_cmd(args, 'no_reset'))
        else:
            print("Could not detect ESP32 serial-update mode, falling back to ROM bootloader reset.")
            esptool.main(_esp32_full_flash_cmd(args, 'default_reset'))
    except Exception:
        return ElrsUploadResult.ErrorGeneral
    return ElrsUploadResult.Success

def upload_esp32_etx(args):
    if args.port == None:
        args.port = serials_find.get_serial_port()
    ETXinitPassthrough.etx_passthrough_init(args.port, args.baud)
    try:
        dir = os.path.dirname(args.file.name)
        cmd = ['--chip', args.platform.replace('-', ''), '--port', args.port, '--baud', str(args.baud), '--before', 'no_reset', '--after', 'hard_reset', 'write_flash']
        if args.erase: cmd.append('--erase-all')
        start_addr = '0x0000' if args.platform.startswith('esp32-') else '0x1000'
        cmd.extend(['-z', '--flash_mode', 'dio', '--flash_freq', '40m', '--flash_size', 'detect', start_addr, os.path.join(dir, 'bootloader.bin'), '0x8000', os.path.join(dir, 'partitions.bin'), '0xe000', os.path.join(dir, 'boot_app0.bin'), '0x10000', args.file.name])
        esptool.main(cmd)
    except:
        return ElrsUploadResult.ErrorGeneral
    return ElrsUploadResult.Success

def upload_esp32_bf(args, options):
    if args.port == None:
        args.port = serials_find.get_serial_port()
    mode = 'upload'
    if args.force == True:
        mode = 'uploadforce'
    passthrough_args = ['-p', args.port, '-b', str(args.baud), '-r', options.firmware, '-a', mode]
    if getattr(args, 'accept', None):
        passthrough_args.extend(['--accept', args.accept])
    if getattr(args, 'target_path', None):
        passthrough_args.extend(['--target-path', args.target_path])
    retval = BFinitPassthrough.main(passthrough_args)
    if retval != ElrsUploadResult.Success:
        return retval
    try:
        esptool.main(['--passthrough', '--chip', args.platform.replace('-', ''), '--port', args.port, '--baud', str(args.baud), '--before', 'no_reset', '--after', 'hard_reset', 'write_flash', '-z', '--flash_mode', 'dio', '--flash_freq', '40m', '--flash_size', 'detect', '0x10000', args.file.name])
    except:
        return ElrsUploadResult.ErrorGeneral
    return ElrsUploadResult.Success

def upload_dir(mcuType, args):
    if mcuType == MCUType.ESP8266:
        shutil.copy2('firmware.bin.gz', os.path.join(args.out, 'firmware.bin.gz'))
    elif mcuType == MCUType.ESP32:
        shutil.copy2(args.file.name, args.out)

def upload(options: FirmwareOptions, args):
    if args.baud == 0:
        args.baud = 460800
        if args.flash == UploadMethod.betaflight:
            args.baud = 420000

    if args.flash == UploadMethod.dir or args.flash == UploadMethod.stock:
        return upload_dir(options.mcuType, args)
    elif options.deviceType == DeviceType.RX:
        if options.mcuType == MCUType.ESP8266:
            if args.flash == UploadMethod.betaflight:
                return upload_esp8266_bf(args, options)
            elif args.flash == UploadMethod.uart:
                return upload_esp8266_uart(args, options)
            elif args.flash == UploadMethod.wifi:
                return upload_wifi(args, options, ['elrs_rx', 'elrs_rx.local'])
        elif options.mcuType == MCUType.ESP32:
            if args.flash == UploadMethod.betaflight:
                return upload_esp32_bf(args, options)
            elif args.flash == UploadMethod.uart:
                return upload_esp32_uart(args, options)
            elif args.flash == UploadMethod.wifi:
                return upload_wifi(args, options, ['elrs_rx', 'elrs_rx.local'])
    else:
        if options.mcuType == MCUType.ESP8266:
            if args.flash == UploadMethod.uart:
                return upload_esp8266_uart(args, options)
            elif args.flash == UploadMethod.wifi:
                return upload_wifi(args, options, ['elrs_tx', 'elrs_tx.local'])
        elif options.mcuType == MCUType.ESP32:
            if args.flash == UploadMethod.edgetx:
                return upload_esp32_etx(args)
            elif args.flash == UploadMethod.uart:
                return upload_esp32_uart(args, options)
            elif args.flash == UploadMethod.wifi:
                return upload_wifi(args, options, ['elrs_tx', 'elrs_tx.local'])
    print("Invalid upload method for firmware")
    return ElrsUploadResult.ErrorGeneral
