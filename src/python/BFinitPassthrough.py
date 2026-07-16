import serial, time, sys, re
import argparse
import serials_find
import SerialHelper
import bootloader
from query_yes_no import query_yes_no
from elrs_helpers import ElrsUploadResult


SCRIPT_DEBUG = False

TARGET_INFO_PREFIX = "ELRS_TARGET_INFO:"


class PassthroughEnabled(Exception):
    pass

class PassthroughFailed(Exception):
    pass

def dbg_print(line=''):
    sys.stdout.write(line + '\n')
    sys.stdout.flush()


def _validate_serialrx(rl, config, expected):
    found = False
    if type(expected) == str:
        expected = [expected]
    rl.set_delimiters(["# "])
    rl.clear()
    rl.write_str("get %s" % config)
    line = rl.read_line(1.).strip()
    for key in expected:
        key = " = %s" % key
        if key in line:
            found = True
            break
    return found


def bf_passthrough_init(port, requestedBaudrate):
    sys.stdout.flush()
    dbg_print("======== PASSTHROUGH INIT ========")
    dbg_print("  Trying to initialize %s @ %s" % (port, requestedBaudrate))

    s = serial.Serial(port=port, baudrate=115200,
        bytesize=8, parity='N', stopbits=1,
        timeout=1, xonxoff=0, rtscts=0)

    rl = SerialHelper.SerialHelper(s, 3., ['CCC', "# "])
    rl.clear()
    # Send start command '#'
    rl.write_str("#", False)
    start = rl.read_line(2.).strip()
    #dbg_print("BF INIT: '%s'" % start.replace("\r", ""))
    if "CCC" in start:
        raise PassthroughEnabled("Passthrough already enabled and bootloader active")
    elif not start or not start.endswith("#"):
        raise PassthroughEnabled("No CLI available. Already in passthrough mode?, If this fails reboot FC and try again!")

    serial_check = []
    if not _validate_serialrx(rl, "serialrx_provider", ["CRSF", "ELRS"]):
        serial_check.append("Serial Receiver Protocol is not set to CRSF! Hint: set serialrx_provider = CRSF")
    if not _validate_serialrx(rl, "serialrx_inverted", "OFF"):
        serial_check.append("Serial Receiver UART is inverted! Hint: set serialrx_inverted = OFF")
    if not _validate_serialrx(rl, "serialrx_halfduplex", ["OFF", "AUTO"]):
        serial_check.append("Serial Receiver UART is not in full duplex! Hint: set serialrx_halfduplex = OFF")
    if _validate_serialrx(rl, "rx_spi_protocol", "EXPRESSLRS" ) and serial_check:
        serial_check = [ "ExpressLRS SPI RX detected\n\nUpdate via betaflight to flash your RX\nhttps://www.expresslrs.org/2.0/hardware/spi-receivers/" ]

    if serial_check:
        error = "\n\n [ERROR] Invalid serial RX configuration detected:\n"
        for err in serial_check:
            error += "    !!! %s !!!\n" % err
        error += "\n    Please change the configuration and try again!\n"
        raise PassthroughFailed(error)

    SerialRXindex = ""

    dbg_print("\nAttempting to detect FC UART configuration...")

    rl.set_delimiters(["\n"])
    rl.clear()
    rl.write_str("serial")

    while True:
        line = rl.read_line().strip()
        #print("FC: '%s'" % line)
        if not line or "#" in line:
            break

        if line.startswith("serial"):
            if SCRIPT_DEBUG:
                dbg_print("  '%s'" % line)
            config = re.search('serial ((?:UART)?[0-9]+) ([0-9]+) ', line)
            if config and (int(config.group(2)) & 64 == 64):
                dbg_print("    ** Serial RX config detected: '%s'" % line)
                SerialRXindex = config.group(1)
                if not SCRIPT_DEBUG:
                    break

    if not SerialRXindex:
        raise PassthroughFailed("!!! RX Serial not found !!!!\n  Check configuration and try again...")

    cmd = "serialpassthrough %s %s" % (SerialRXindex, requestedBaudrate, )

    dbg_print("Enabling serial passthrough...")
    dbg_print("  CMD: '%s'" % cmd)
    rl.write_str(cmd)
    time.sleep(.2)
    s.close()
    dbg_print("======== PASSTHROUGH DONE ========")


def _parse_detected_target(line):
    payload = line
    if TARGET_INFO_PREFIX in payload:
        detected_target_path = payload.split(TARGET_INFO_PREFIX, 1)[1]
        return "", detected_target_path
    return payload, ""


def _format_detected_target(detected_target, detected_target_path):
    if detected_target_path:
        return detected_target_path
    return detected_target


def _extract_detected_target_from_bytes(data):
    marker = TARGET_INFO_PREFIX.encode()
    marker_offset = data.find(marker)
    if marker_offset >= 0:
        end = data.find(b'\n', marker_offset)
        if end < 0:
            end = len(data)
        line = data[marker_offset:end].decode('ascii', errors='ignore').strip()
        return _parse_detected_target(line)

    fallback_target = ("", "")
    for raw_line in data.splitlines():
        line = raw_line.decode('ascii', errors='ignore').strip()
        if line == "":
            continue
        detected_target = _parse_detected_target(line)
        if fallback_target == ("", ""):
            fallback_target = detected_target
        if TARGET_INFO_PREFIX in line:
            return detected_target
    return fallback_target

def _collect_reset_response(rl, timeout=1.5):
    buf = bytearray(rl.buf)
    rl.buf = bytearray()
    start = time.time()
    while (time.time() - start) < timeout:
        waiting = min(2048, rl.serial.in_waiting)
        data = rl.serial.read(waiting)
        if data:
            buf.extend(data)
            start = time.time()
        else:
            time.sleep(0.01)
    return bytes(buf)


def reset_to_bootloader(port, baud, target, target_path, action, accept=None, chip_type='ESP82') -> int:
    dbg_print("======== RESET TO BOOTLOADER ========")
    s = serial.Serial(port=port, baudrate=baud,
        bytesize=8, parity='N', stopbits=1,
        timeout=1, xonxoff=0, rtscts=0)
    rl = SerialHelper.SerialHelper(s, 3.)
    rl.clear()
    BootloaderInitSeq = bootloader.get_init_seq(chip_type)
    dbg_print("  * Using full duplex (CRSF)")
    rl.write(BootloaderInitSeq)
    time.sleep(0.3)
    detected_target, detected_target_path = _extract_detected_target_from_bytes(_collect_reset_response(rl))
    detected = _format_detected_target(detected_target, detected_target_path)
    if target is not None:
        flash_target = re.sub("_VIA_.*", "", target.upper())
        flash_target_path = target_path if target_path else None
        accept_target = accept.upper() if accept else None
        ignore_incorrect_target = action == "uploadforce"
        detected_target_cmp = detected_target.upper()
        detected_target_path_cmp = detected_target_path.casefold()
        flash_target_path_cmp = flash_target_path.casefold() if flash_target_path else None
        if flash_target_path is not None:
            target_matches = detected_target_path_cmp == flash_target_path_cmp
            if not target_matches and detected_target_path == "" and accept_target is not None:
                target_matches = detected_target_cmp == accept_target
        else:
            target_matches = detected_target_cmp == flash_target or detected_target_cmp == accept_target
        expected = flash_target_path if flash_target_path is not None else flash_target
        if detected_target == "":
            dbg_print("Cannot detect RX target, blindly flashing!")
        elif ignore_incorrect_target:
            dbg_print(f"Force flashing {expected}, detected {detected}")
        elif not target_matches:
            if query_yes_no("\n\n\nWrong target selected! your RX is '%s', trying to flash '%s', continue? Y/N\n" % (detected, expected)):
                dbg_print("Ok, flashing anyway!")
            else:
                dbg_print("Wrong target selected your RX is '%s', trying to flash '%s'" % (detected, expected))
                return ElrsUploadResult.ErrorMismatch
        elif flash_target != "":
            dbg_print("Verified RX target '%s'" % (expected))
    time.sleep(.5)
    s.close()

    return ElrsUploadResult.Success

def init_passthrough(source, target, env) -> int:
    env.AutodetectUploadPort([env])
    try:
        bf_passthrough_init(env['UPLOAD_PORT'], env['UPLOAD_SPEED'])
    except PassthroughEnabled as err:
        dbg_print(str(err))
    return reset_to_bootloader(env['UPLOAD_PORT'], env['UPLOAD_SPEED'], env['PIOENV'], env.GetProjectOption('board_config', None), source[0])

def main(custom_args = None):
    parser = argparse.ArgumentParser(
        description="Initialize BetaFlight passthrough and optionally send a reboot comamnd sequence")
    parser.add_argument("-b", "--baud", type=int, default=420000,
        help="Baud rate for passthrough communication")
    parser.add_argument("-p", "--port", type=str,
        help="Override serial port autodetection and use PORT")
    parser.add_argument("-r", "--target", type=str,
        help="The target firmware that is going to be uploaded")
    parser.add_argument("-nr", "--no-reset", action="store_false",
        dest="reset_to_bl", help="Do not send reset_to_bootloader command sequence")
    parser.add_argument("-t", "--type", type=str, default="ESP82",
        help="Defines flash target type which is sent to target in reboot command")
    parser.add_argument("-a", "--action", type=str, default="upload",
        help="Upload action: upload (default), or uploadforce to flash even on target mismatch")
    parser.add_argument("--accept", type=str, default=None,
        help="Acceptable target to auto-overwrite")
    parser.add_argument("--target-path", type=str, default=None,
        help="The selected targets.json path for the firmware being uploaded")

    args = parser.parse_args(custom_args)

    if (args.port == None):
        args.port = serials_find.get_serial_port()

    returncode = ElrsUploadResult.Success
    try:
        bf_passthrough_init(args.port, args.baud)
    except PassthroughEnabled as err:
        dbg_print(str(err))

    if args.reset_to_bl:
        returncode = reset_to_bootloader(args.port, args.baud, args.target, args.target_path, args.action, args.accept, args.type)

    return returncode

if __name__ == '__main__':
    returncode = main()
    exit(returncode)