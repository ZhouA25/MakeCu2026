"""Find a SensX25 and tell which wire protocol it speaks (docs/tactile.md has the byte layouts).

Two protocols exist, and the bytes give them away:
  single  one sensor on a USB-TTL adapter (our setup): FF FF + 50 bytes + CRC8, after a start command
  hub     TouchTronix hub, up to 5 sensors:            AA AA..EE EE + 50 bytes, no CRC, always streaming
"""
import os
import re
import sys
import time

BAUD = 921600
ROWS = COLS = 5
PAYLOAD = ROWS * COLS * 2                     # 25 taxels x 16-bit big-endian
SINGLE_HEADER = b"\xff\xff"
HUB_HEADERS = {b"\xaa\xaa": "A", b"\xbb\xbb": "B", b"\xcc\xcc": "C", b"\xdd\xdd": "D", b"\xee\xee": "E"}
START_CMD = bytes.fromhex("01 06 60 10 00 01 57 CF")   # Modbus "write 1 to register 0x6010" = stream


def crc8_maxim(data: bytes) -> int:
    """Checksum the single sensor appends to each frame (same as sensx.sensor._crc8_maxim)."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def serial_baud(port: str, baud: int) -> int:
    """Virtual ports (pillbot.tools.fake_sensx) reject 921600 on macOS; baud means nothing there."""
    return 115200 if re.fullmatch(r"/dev/(ttys\d+|pts/\d+)", os.path.realpath(port)) else baud


def candidate_ports() -> list[str]:
    """USB serial ports, FTDI/CH340/CP210x-style (ttyUSB) before Arduino-style (ttyACM)."""
    from serial.tools import list_ports
    ports = []
    for p in list_ports.comports():
        if p.vid is None:
            continue                                    # Bluetooth, debug console, built-in UARTs
        dev = p.device
        if sys.platform == "darwin":
            dev = dev.replace("/dev/tty.", "/dev/cu.", 1)  # cu.* doesn't wait for carrier detect
        ports.append(dev)
    return sorted(set(ports), key=lambda d: ("ACM" in d or "usbmodem" in d, d))


def classify(raw: bytes) -> tuple[str | None, dict[str, int]]:
    """("single" | "hub" | None, {channel: frames seen}) for a chunk of captured bytes.

    Pixel high bytes are <= 0x0F (12-bit values), so header bytes can't occur inside a frame.
    """
    single, i = 0, raw.find(SINGLE_HEADER)
    while i != -1 and i + 2 + PAYLOAD + 1 <= len(raw):
        if crc8_maxim(raw[i + 2:i + 2 + PAYLOAD]) == raw[i + 2 + PAYLOAD]:
            single += 1
        i = raw.find(SINGLE_HEADER, i + 1)
    hub: dict[str, int] = {}
    n = 2 + PAYLOAD
    for i in range(len(raw) - n - 1):
        if raw[i:i + 2] in HUB_HEADERS and raw[i + n:i + n + 2] in HUB_HEADERS:
            ch = HUB_HEADERS[raw[i:i + 2]]
            hub[ch] = hub.get(ch, 0) + 1
    if single >= 5 and single >= sum(hub.values()):
        return "single", {"S": single}
    if sum(hub.values()) >= 5:
        return "hub", hub
    return None, {}


def probe(port: str, baud: int = BAUD, listen_s: float = 1.0, wake: bool = False):
    """Listen on `port`; if silent and `wake`, send the start command and listen again.

    Only `wake` ports you know are the sensor: the start command is a Modbus register write,
    and the arm's Dynamixel bus or another Modbus device could misread it.
    Returns (kind, {channel: frames}, bytes received).
    """
    import serial
    with serial.Serial(port, serial_baud(port, baud), timeout=0.05) as ser:
        ser.reset_input_buffer()
        raw = _capture(ser, listen_s)
        kind, counts = classify(raw)
        if kind is None and wake:
            ser.write(START_CMD)
            ser.flush()
            raw = _capture(ser, listen_s)
            kind, counts = classify(raw)
    return kind, counts, len(raw)


def _capture(ser, seconds: float) -> bytes:
    raw, end = bytearray(), time.time() + seconds
    while time.time() < end:
        raw += ser.read(4096)
    return bytes(raw)
