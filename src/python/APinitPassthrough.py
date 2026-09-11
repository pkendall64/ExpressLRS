import argparse
import re
import struct
import sys
import time
from typing import Dict, Optional, Set, Tuple

try:
    import serial
except ImportError:
    serial = None

try:
    import bootloader
except ImportError:
    from . import bootloader
try:
    import serials_find
except ImportError:
    try:
        from . import serials_find
    except ImportError:
        serials_find = None
try:
    from elrs_helpers import ElrsUploadResult
except ImportError:
    from .elrs_helpers import ElrsUploadResult


MAVLINK1_STX = 0xFE
MAVLINK2_STX = 0xFD
MAV_COMP_ID = 190
MAV_SYS_ID = 255
MAV_PARAM_TYPE_INT8 = 1

MSG_HEARTBEAT = 0
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_VALUE = 22
MSG_PARAM_SET = 23
CRC_EXTRA = {
    MSG_HEARTBEAT: 50,
    MSG_PARAM_REQUEST_READ: 214,
    MSG_PARAM_VALUE: 220,
    MSG_PARAM_SET: 168,
}

SERIAL_PARAM_RE = re.compile(r'^SERIAL([1-9][0-9]*)_(PROTOCOL|BAUD)$')
CRSF_PROTOCOLS = {23}
MAVLINK_PROTOCOLS = {1, 2}
CRSF_BAUD = 420000
MAVLINK_BAUD = 460800

PASSTHROUGH_TIMEOUT_SECONDS = 120
PASSTHROUGH_SETTLE_SECONDS = 2.0
MAX_TRIGGER_ATTEMPTS = 10
TRIGGER_BURST_COUNT = 5
TRIGGER_BURST_DELAY_SECONDS = 0.01
BANNER_READ_WINDOW_SECONDS = 1.0
BANNER_POLL_SECONDS = 0.02
ESP_SYNC_WINDOW_SECONDS = 3.0
ESP_SYNC_INTERVAL_SECONDS = 0.01
ESP_SYNC_SLIP_FRAME = b"\xc0\x00\x08\x24\x00\x00\x00\x00\x00\x07\x07\x12\x20" + 32 * b"\x55" + b"\xc0"
ESP_SYNC_COMMAND = 0x08
BOOTLOADER_BANNER_REGEX = re.compile(br"UNIFIED_ESP.*?\r\n", re.IGNORECASE | re.DOTALL)
PARAM_READ_TIMEOUT_SECONDS = 0.5
RECOVERY_UNSUPPORTED_MESSAGE = (
    'Receiver is already in ESP bootloader; recovery flashing via ArduPilot passthrough is unreliable. '
    'Use direct UART or reboot the receiver into firmware and retry normal passthrough.'
)
STALE_DRAIN_QUIET_SECONDS = 0.1
STALE_DRAIN_MAX_SECONDS = 1.0
MAX_SERIAL_PROBE_PORTS = 32


class PassthroughFailed(Exception):
    pass


class SignedMavlink2Unsupported(PassthroughFailed):
    pass


def dbg_print(line=''):
    sys.stdout.write(line + '\n')
    sys.stdout.flush()


def _param_name(name):
    return name.encode('ascii')[:16].ljust(16, b'\x00')


def _clean_param_name(raw):
    return raw.rstrip(b'\x00').decode('ascii', 'ignore')


def _baud_value(value):
    baud = int(round(float(value)))
    if baud == 420:
        return CRSF_BAUD
    if baud == 460:
        return MAVLINK_BAUD
    return baud * 1000 if baud and baud < 10000 else baud


def x25_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def pack_mavlink1(msgid: int, payload: bytes, seq: int, sysid: int, compid: int, crc_extra: int) -> bytes:
    header = bytes([len(payload), seq & 0xFF, sysid & 0xFF, compid & 0xFF, msgid & 0xFF])
    crc = x25_crc(header + payload + bytes([crc_extra]))
    return bytes([MAVLINK1_STX]) + header + payload + struct.pack('<H', crc)


def collect_serial_params(params: Dict[str, float]) -> Tuple[Dict[int, Tuple[int, Optional[int]]], Dict[int, Set[str]]]:
    found = {}
    for name, value in params.items():
        match = SERIAL_PARAM_RE.match(name)
        if not match:
            continue
        port = int(match.group(1))
        suffix = match.group(2)
        found.setdefault(port, {})[suffix] = value
    complete = {}
    incomplete = {}

    for port, values in found.items():
        if 'PROTOCOL' not in values:
            incomplete[port] = {'PROTOCOL'} | ({'BAUD'} if 'BAUD' not in values else set())
            continue
        protocol = int(round(float(values['PROTOCOL'])))
        baud = _baud_value(values['BAUD']) if 'BAUD' in values else None
        if baud is None:
            incomplete[port] = {'BAUD'}
        complete[port] = (protocol, baud)
    return complete, incomplete

def _format_complete(complete):
    return ', '.join(
        f'SERIAL{port} protocol {protocol}' + (f' baud {baud}' if baud is not None else ' baud <missing>')
        for port, (protocol, baud) in sorted(complete.items()))


def _format_incomplete(incomplete):
    return ', '.join(
        f'SERIAL{port} missing {"/".join(sorted(missing))}'
        for port, missing in sorted(incomplete.items()))


def choose_receiver_port(params: Dict[str, float]) -> Tuple[int, int, str]:
    complete, incomplete = collect_serial_params(params)
    candidates = []
    unsupported = []
    for port, (protocol, baud) in complete.items():
        if protocol in CRSF_PROTOCOLS:
            candidates.append((port, CRSF_BAUD, 'CRSF'))
        elif protocol in MAVLINK_PROTOCOLS:
            if baud == MAVLINK_BAUD:
                candidates.append((port, baud, 'MAVLink'))
            else:
                unsupported.append((port, protocol, baud, MAVLINK_BAUD))

    if len(candidates) == 1:
        return candidates[0]
    if len(candidates) > 1:
        raise PassthroughFailed('Ambiguous receiver serial ports: ' + ', '.join(
            f'SERIAL{port} {mode} {baud}' for port, baud, mode in candidates))
    if unsupported:
        raise PassthroughFailed('Unsupported receiver serial baud: ' + ', '.join(
            f'SERIAL{port} protocol {protocol} baud {baud}, expected {expected}'
            for port, protocol, baud, expected in unsupported))

    details = _format_complete(complete) or 'none'
    if incomplete:
        details += '; incomplete: ' + _format_incomplete(incomplete)
    raise PassthroughFailed('No receiver serial port matched CRSF protocol 23 or MAVLink 460800; available: ' + details)


class MavlinkClient:
    def __init__(self, handle):
        self.handle = handle
        self.seq = 0
        self.target_system = None
        self.target_component = None
        self.saw_signed_v2 = False

    def _read_exact(self, count, deadline):
        data = bytearray()
        while len(data) < count and time.monotonic() < deadline:
            chunk = self.handle.read(count - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data) if len(data) == count else None

    def _read_message(self, deadline):
        while time.monotonic() < deadline:
            stx = self.handle.read(1)
            if not stx:
                continue
            stx = stx[0]
            if stx == MAVLINK1_STX:
                header = self._read_exact(5, deadline)
                if not header:
                    return None
                payload = self._read_exact(header[0], deadline)
                checksum = self._read_exact(2, deadline)
                if payload is None or checksum is None:
                    return None
                msgid = header[4]
                if msgid in CRC_EXTRA and struct.unpack('<H', checksum)[0] == x25_crc(header + payload + bytes([CRC_EXTRA[msgid]])):
                    return msgid, payload, header[2], header[3]
            elif stx == MAVLINK2_STX:
                header = self._read_exact(9, deadline)
                if not header:
                    return None
                signed = bool(header[1] & 0x01)
                payload = self._read_exact(header[0], deadline)
                checksum = self._read_exact(2, deadline)
                signature = self._read_exact(13, deadline) if signed else b''
                if payload is None or checksum is None or signature is None:
                    return None
                if signed:
                    self.saw_signed_v2 = True
                    continue
                msgid = header[6] | (header[7] << 8) | (header[8] << 16)
                if msgid in CRC_EXTRA and struct.unpack('<H', checksum)[0] == x25_crc(header + payload + bytes([CRC_EXTRA[msgid]])):
                    return msgid, payload, header[4], header[5]
        return None

    def _send(self, msgid, payload):
        self.handle.write(pack_mavlink1(msgid, payload, self.seq, MAV_SYS_ID, MAV_COMP_ID, CRC_EXTRA[msgid]))
        self.handle.flush()
        self.seq = (self.seq + 1) & 0xFF

    def wait_heartbeat(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self._read_message(deadline)
            if message and message[0] == MSG_HEARTBEAT:
                self.target_system = message[2]
                self.target_component = message[3]
                return
        if self.saw_signed_v2:
            raise SignedMavlink2Unsupported('Signed MAVLink2 unsupported by raw helper; configure unsigned MAVLink or use another upload path')
        raise PassthroughFailed('Timed out waiting for ArduPilot HEARTBEAT')

    def read_param(self, name, timeout=PARAM_READ_TIMEOUT_SECONDS):
        deadline = time.monotonic() + timeout
        self._send(MSG_PARAM_REQUEST_READ, struct.pack('<hBB16s', -1, self.target_system, self.target_component, _param_name(name)))
        while time.monotonic() < deadline:
            message = self._read_message(deadline)
            if message and message[0] == MSG_PARAM_VALUE and len(message[1]) >= 25:
                value, _count, _index, param_id, _param_type = struct.unpack('<fHH16sB', message[1][:25])
                if _clean_param_name(param_id) == name:
                    return value
        if self.saw_signed_v2:
            raise SignedMavlink2Unsupported('Signed MAVLink2 unsupported by raw helper; configure unsigned MAVLink or use another upload path')
        return None

    def set_param_int8(self, name, value, timeout=5.0):
        deadline = time.monotonic() + timeout
        next_write = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_write:
                self._send(MSG_PARAM_SET, struct.pack('<fBB16sB', float(value), self.target_system, self.target_component, _param_name(name), MAV_PARAM_TYPE_INT8))
                next_write = now + 1.0
            message = self._read_message(deadline)
            if message and message[0] == MSG_PARAM_VALUE and len(message[1]) >= 25:
                echoed, _count, _index, param_id, _param_type = struct.unpack('<fHH16sB', message[1][:25])
                if _clean_param_name(param_id) == name and int(round(echoed)) == value:
                    return
        if self.saw_signed_v2:
            raise SignedMavlink2Unsupported('Signed MAVLink2 unsupported by raw helper; configure unsigned MAVLink or use another upload path')
        raise PassthroughFailed(f'Timed out writing ArduPilot param {name}={value}')


def _read_serial_params(client):
    params = {}
    dbg_print(f'Probing SERIAL params with {PARAM_READ_TIMEOUT_SECONDS:.1f}s timeout...')
    for port in range(1, MAX_SERIAL_PROBE_PORTS + 1):
        protocol_name = f'SERIAL{port}_PROTOCOL'
        protocol = client.read_param(protocol_name)
        if protocol is None:
            dbg_print(f'  {protocol_name} not present; stopping SERIAL probe')
            break
        params[protocol_name] = protocol
        dbg_print(f'  {protocol_name} = {int(round(float(protocol)))}')

        baud_name = f'SERIAL{port}_BAUD'
        baud = client.read_param(baud_name)
        if baud is None:
            dbg_print(f'  {baud_name} not present; continuing SERIAL probe')
            continue
        params[baud_name] = baud
        dbg_print(f'  {baud_name} = {_baud_value(baud)}')
    return params


def _open_serial(**kwargs):
    if serial is None:
        raise PassthroughFailed("pyserial is required for ArduPilot passthrough")
    return serial.Serial(**kwargs)


def ap_passthrough_init(port, selected_baud):
    dbg_print('======== ARDUPILOT PASSTHROUGH INIT ========')
    dbg_print('  Trying to initialize %s @ %s' % (port, selected_baud))
    with _open_serial(port=port, baudrate=selected_baud, timeout=1, bytesize=8, parity='N', stopbits=1, xonxoff=0, rtscts=0) as handle:
        client = MavlinkClient(handle)
        dbg_print('Waiting for ArduPilot heartbeat...')
        client.wait_heartbeat()
        dbg_print(f'Connected to ArduPilot (system {client.target_system}, component {client.target_component})')
        params = _read_serial_params(client)
        complete, incomplete = collect_serial_params(params)
        dbg_print(f'Found {len(complete)} complete SERIAL ports' + (f', {len(incomplete)} incomplete' if incomplete else ''))
        if complete:
            dbg_print('  Available serial ports: ' + _format_complete(complete))
        if incomplete:
            dbg_print('  Incomplete serial ports: ' + _format_incomplete(incomplete))
        receiver_port, receiver_baud, mode = choose_receiver_port(params)
        dbg_print(f'  Receiver serial: SERIAL{receiver_port} {mode} @ {receiver_baud}; upload link will reopen @ {receiver_baud}')
        dbg_print('Engaging ArduPilot serial passthrough...')
        dbg_print(f'  Setting SERIAL_PASSTIMO = {PASSTHROUGH_TIMEOUT_SECONDS}')
        client.set_param_int8('SERIAL_PASSTIMO', PASSTHROUGH_TIMEOUT_SECONDS)
        dbg_print('  Setting SERIAL_PASS1 = 0')
        client.set_param_int8('SERIAL_PASS1', 0)
        dbg_print(f'  Setting SERIAL_PASS2 = {receiver_port}')
        client.set_param_int8('SERIAL_PASS2', receiver_port)
        dbg_print(f'Waiting {PASSTHROUGH_SETTLE_SECONDS:.1f}s for ArduPilot UART re-initialization...')
        time.sleep(PASSTHROUGH_SETTLE_SECONDS)
        _drain_serial(handle)
    dbg_print('======== ARDUPILOT PASSTHROUGH DONE ========')
    return receiver_baud


def _clear_serial(handle):
    try:
        handle.reset_input_buffer()
        handle.reset_output_buffer()
    except AttributeError:
        pass

def _drain_serial(handle, quiet_seconds=STALE_DRAIN_QUIET_SECONDS, max_seconds=STALE_DRAIN_MAX_SECONDS):
    deadline = time.monotonic() + max_seconds
    quiet_deadline = time.monotonic() + quiet_seconds
    drained = 0
    while time.monotonic() < deadline and time.monotonic() < quiet_deadline:
        waiting = getattr(handle, 'in_waiting', 0)
        if waiting:
            chunk = handle.read(waiting)
            drained += len(chunk)
            quiet_deadline = time.monotonic() + quiet_seconds
        else:
            time.sleep(min(0.01, quiet_seconds))
    if drained:
        dbg_print(f'Drained {drained} stale passthrough bytes')
    return drained


def _verify_target(rx_target, target, action, accept):
    if target is None:
        return ElrsUploadResult.Success
    flash_target = re.sub('_VIA_.*', '', str(target).upper())
    accept = accept.upper() if isinstance(accept, str) else accept
    ignore_incorrect_target = action == 'uploadforce'
    if rx_target == '':
        dbg_print('Cannot detect RX target, blindly flashing!')
    elif ignore_incorrect_target:
        dbg_print(f'Force flashing {flash_target}, detected {rx_target}')
    elif rx_target != flash_target and rx_target != accept:
        try:
            from query_yes_no import query_yes_no
        except ImportError:
            from .query_yes_no import query_yes_no
        if query_yes_no("\n\n\nWrong target selected! your RX is '%s', trying to flash '%s', continue? Y/N\n" % (rx_target, flash_target)):
            dbg_print('Ok, flashing anyway!')
        else:
            dbg_print("Wrong target selected your RX is '%s', trying to flash '%s'" % (rx_target, flash_target))
            return ElrsUploadResult.ErrorMismatch
    elif flash_target != '':
        dbg_print("Verified RX target '%s'" % flash_target)
    return ElrsUploadResult.Success


def _slip_packets(buffer):
    packets = []
    packet = None
    escaped = False
    for byte in buffer:
        if packet is None:
            if byte == 0xC0:
                packet = bytearray()
            continue
        if escaped:
            escaped = False
            if byte == 0xDC:
                packet.append(0xC0)
            elif byte == 0xDD:
                packet.append(0xDB)
            else:
                packet = None
        elif byte == 0xDB:
            escaped = True
        elif byte == 0xC0:
            packets.append(bytes(packet))
            packet = bytearray()
        else:
            packet.append(byte)
    remainder = b'' if packet is None else bytes([0xC0]) + bytes(packet)
    if escaped:
        remainder += b'\xDB'
    return packets, remainder


def _is_esp_response(packet):
    if len(packet) < 8:
        return False
    response, _op, length, _value = struct.unpack('<BBHI', packet[:8])
    return response == 1 and len(packet[8:]) >= length


def _sync_esp_rom(handle):
    dbg_print("\n--- Phase 3: Synchronizing with ESP Bootloader (SLIP Loop) ---")
    _clear_serial(handle)
    _drain_serial(handle)
    deadline = time.monotonic() + ESP_SYNC_WINDOW_SECONDS
    attempts = 0
    buffer = b''
    while time.monotonic() < deadline:
        attempts += 1
        handle.write(ESP_SYNC_SLIP_FRAME)
        handle.flush()
        time.sleep(ESP_SYNC_INTERVAL_SECONDS)
        waiting = getattr(handle, 'in_waiting', 0)
        if not waiting:
            continue
        data = handle.read(waiting)
        if data:
            dbg_print(f"Sync #{attempts} | RX <- {data.hex(' ')}")
            buffer += data
            packets, buffer = _slip_packets(buffer)
            for packet in packets:
                if _is_esp_response(packet):
                    dbg_print(f'ESP bootloader response received on sync #{attempts}: {packet.hex(" ")}')
                    return True
                elif packet:
                    dbg_print(f'Ignoring non-ESP SLIP packet: {packet.hex(" ")}')
    return False


def reset_to_bootloader_ap(port, baud, target, action, accept=None, chip_type='ESP82') -> int:
    dbg_print('======== RESET TO BOOTLOADER ========')
    dbg_print('\n--- Phase 2: Bootloader Trigger & Banner Verification ---')
    init_seq = bootloader.get_init_seq()
    with _open_serial(port=port, baudrate=baud, timeout=1, bytesize=8, parity='N', stopbits=1, xonxoff=0, rtscts=0) as handle:
        dbg_print('Checking for receiver already in ESP bootloader...')
        if _sync_esp_rom(handle):
            dbg_print(RECOVERY_UNSUPPORTED_MESSAGE)
            return ElrsUploadResult.ErrorGeneral

        for attempt in range(1, MAX_TRIGGER_ATTEMPTS + 1):
            dbg_print(f'  Sending reboot trigger (attempt {attempt}/{MAX_TRIGGER_ATTEMPTS})...')
            _clear_serial(handle)
            for _ in range(TRIGGER_BURST_COUNT):
                handle.write(init_seq)
                handle.flush()
                time.sleep(TRIGGER_BURST_DELAY_SECONDS)
            banner = bytearray()
            deadline = time.monotonic() + BANNER_READ_WINDOW_SECONDS
            while time.monotonic() < deadline:
                waiting = getattr(handle, 'in_waiting', 0)
                chunk = handle.read(waiting or 1)
                if chunk:
                    banner.extend(chunk)
                    match = BOOTLOADER_BANNER_REGEX.search(bytes(banner))
                    if match:
                        rx_target = match.group(0).replace(b'\x00', b'').strip().decode('ascii', 'ignore').upper()
                        dbg_print(f"Detected ESP bootloader banner: '{rx_target}'")
                        result = _verify_target(rx_target, target, action, accept)
                        if result != ElrsUploadResult.Success:
                            return result
                        if not _sync_esp_rom(handle):
                            dbg_print('ESP ROM sync did not acknowledge')
                            return ElrsUploadResult.ErrorGeneral
                        return ElrsUploadResult.Success
                else:
                    time.sleep(BANNER_POLL_SECONDS)
            dbg_print(f'    Banner match not found in {BANNER_READ_WINDOW_SECONDS:.1f}s window.')
    dbg_print(f'Failed to confirm bootloader banner after {MAX_TRIGGER_ATTEMPTS} attempts')
    return ElrsUploadResult.ErrorGeneral


def init_passthrough(source, target, env) -> int:
    env.AutodetectUploadPort([env])
    baud = ap_passthrough_init(env['UPLOAD_PORT'], int(env['UPLOAD_SPEED']))
    env.Replace(UPLOAD_SPEED=baud)
    return reset_to_bootloader_ap(env['UPLOAD_PORT'], baud, env['PIOENV'], str(source[0]))


def main(custom_args=None):
    parser = argparse.ArgumentParser(description='Initialize ArduPilot passthrough and reset ExpressLRS receiver to bootloader')
    parser.add_argument('-p', '--port', type=str, help='serial port of the ArduPilot flight controller')
    parser.add_argument('-b', '--baud', type=int, default=460800, help='baud rate for MAVLink to the flight controller')
    parser.add_argument('-r', '--rx', type=str, default=None, help='expected receiver target')
    parser.add_argument('-t', '--type', type=str, default='ESP82', help='flash target type')
    parser.add_argument('-a', '--action', type=str, default='upload', help='upload action')
    parser.add_argument('--accept', type=str, default=None, help='acceptable target to auto-overwrite')
    args = parser.parse_args(custom_args)

    if args.port is None:
        if serials_find is None:
            dbg_print('pyserial is required to autodetect the serial port')
            return ElrsUploadResult.ErrorGeneral
        args.port = serials_find.get_serial_port()
    try:
        upload_baud = ap_passthrough_init(args.port, args.baud)
        return reset_to_bootloader_ap(args.port, upload_baud, args.rx, args.action, args.accept, args.type)
    except PassthroughFailed as err:
        dbg_print(str(err))
        return ElrsUploadResult.ErrorGeneral


if __name__ == '__main__':
    exit(main())
