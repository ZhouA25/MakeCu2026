"""Serial port helpers: stable names, device guesses, and finding the PillBot hub automatically.

Port specs accepted anywhere a port is configured:
  /dev/ttyUSB0, /dev/cu.usbserial-0001          plain device (can change when you replug!)
  /dev/serial/by-id/usb-Silicon_Labs_CP2102...  stable Linux name (best for fixed setups)
  /dev/serial/by-id/*CP2102*                    glob: first match
  auto                                          (hub only) probe every USB serial port for hub messages
  socket://host:5555                            network hub (UNO Q relay)
"""
import glob
import os
import time

# USB VID:PID -> what it usually is in this project
KNOWN = {
    (0x10C4, 0xEA60): "CP210x USB-serial (ESP32 DevKit / ESP32 hub?)",
    (0x1A86, 0x7523): "CH340 USB-serial (ESP32 / ESP32-CAM-MB / adapters)",
    (0x1A86, 0x55D4): "CH9102 USB-serial (ESP32 DevKit)",
    (0x0403, 0x6014): "FTDI FT232H (ROBOTIS U2D2 — Dynamixel arm?)",
    (0x0403, 0x6001): "FTDI FT232R USB-serial",
    (0x2F5D, 0x2202): "ROBOTIS OpenRB / U2D2 PHB",
    (0x303A, 0x1001): "ESP32-S3 native USB",
    (0x2341, None): "Arduino",
}


def list_serial() -> list[dict]:
    from serial.tools import list_ports
    by_id = {}
    for p in glob.glob("/dev/serial/by-id/*"):
        by_id[os.path.realpath(p)] = p
    out = []
    for p in list_ports.comports():
        if p.vid is None and not p.device.startswith(("/dev/ttyUSB", "/dev/ttyACM", "/dev/cu.usb")):
            continue                                   # skip built-in UARTs / Bluetooth ports
        guess = KNOWN.get((p.vid, p.pid)) or KNOWN.get((p.vid, None)) or (p.description or "")
        out.append({"device": p.device, "by_id": by_id.get(os.path.realpath(p.device)),
                    "vid": p.vid, "pid": p.pid, "serial": p.serial_number, "guess": guess})
    return out


def stable_name(device: str) -> str:
    """The /dev/serial/by-id path for a device if there is one (Linux), else the device itself."""
    real = os.path.realpath(device)
    for p in glob.glob("/dev/serial/by-id/*"):
        if os.path.realpath(p) == real:
            return p
    return device


def probe_hub(port: str, timeout: float = 2.5, baud: int = 115200) -> bool:
    """True if `port` is streaming PillBot hub messages ('S ...' or 'READY pillbot-hub')."""
    import serial
    try:
        ser = serial.serial_for_url(port, baudrate=baud, timeout=0.2, do_not_open=True)
        try:
            ser.dtr = False
            ser.rts = False
        except (AttributeError, ValueError, NotImplementedError):
            pass
        ser.open()
    except Exception:
        return False
    try:
        t0 = time.time()
        while time.time() - t0 < timeout:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("READY pillbot-hub") or (line.startswith("S ") and len(line.split()) == 5):
                return True
        return False
    finally:
        ser.close()


def _same(a: str, b: str) -> bool:
    return os.path.realpath(a) == os.path.realpath(b)


def resolve_port(spec: str, kind: str = "device", exclude=(), probe=probe_hub, ports=None) -> str:
    """Turn a port spec into an openable port. Raises RuntimeError with a helpful message."""
    if not spec:
        raise RuntimeError(f"no port configured for {kind}")
    if spec.startswith(("socket://", "rfc2217://", "loop://")):
        return spec
    if any(ch in spec for ch in "*?["):
        matches = sorted(glob.glob(spec))
        if not matches:
            raise RuntimeError(f"{kind}: nothing matches {spec} — run: python -m pillbot.tools.ports")
        return matches[0]
    if spec != "auto":
        return spec
    if kind != "hub":
        raise RuntimeError(f"'auto' is only supported for the hub; set a port for {kind}")
    ports = ports if ports is not None else list_serial()
    cands = [p["device"] for p in ports
             if not any(e and e != "auto" and _same(p["device"], e) for e in exclude)]
    for dev in cands:
        if probe(dev):
            return dev
    raise RuntimeError("hub.port is 'auto' but no USB serial port is sending hub messages "
                       f"(tried {cands or 'none'}). Is the ESP32 plugged in and flashed with "
                       "firmware/esp32_hub? Run: python -m pillbot.tools.ports")
