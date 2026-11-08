"""Host side of the ESP32 PillBot Hub (firmware/esp32_hub). See the firmware header for the protocol."""
import logging
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger(__name__)

BTN_START, BTN_STOP, BTN_MODE = 1, 2, 4


@dataclass
class HubState:
    t_ms: int = 0
    clip_mm: int = -1        # HC-SR04 at the clip (-1 = no echo)
    zone_mm: int = -1        # HC-SR04 at the start zone
    buttons: int = 0
    stamp: float = field(default_factory=time.time)

    def pressed(self, mask: int) -> bool:
        return bool(self.buttons & mask)


class Hub:
    """Real hub. `port` is a serial device (ESP32 over USB, e.g. /dev/cu.usbserial-0001)
    or a network URL (UNO Q relay, e.g. socket://unoq.local:5555) — pyserial handles both."""

    def __init__(self, port: str, baud: int = 115200, exclude=()):
        self.port, self.baud = port, baud          # port may be 'auto', a glob, by-id, or socket://
        self._exclude = exclude
        self.state = HubState()
        self._ser = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def connect(self, timeout: float = 3.0):
        import serial
        from pillbot.utils.ports import resolve_port
        spec = self.port
        self.port = resolve_port(spec, "hub", exclude=self._exclude)
        if spec == "auto":
            log.info("hub found on %s", self.port)
        self._ser = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.1, do_not_open=True)
        # On many ESP32 boards DTR/RTS drive EN/IO0 (auto-reset). Some Linux drivers assert RTS on
        # open, which holds the chip in reset -> no data. Release both lines before opening.
        try:
            self._ser.dtr = False
            self._ser.rts = False
        except (AttributeError, ValueError, NotImplementedError):
            pass                                   # socket:// (UNO Q) has no control lines
        self._ser.open()
        threading.Thread(target=self._loop, daemon=True).start()
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.state.t_ms:
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"No data from hub on {self.port}. Check: is it really the ESP32 (ls -l /dev/serial/by-id/)? "
            f"Is the hub firmware on it (python -m serial.tools.miniterm --dtr 0 --rts 0 {self.port} 115200 "
            "should show 'S ...' lines)? UNO Q: is the app running?")

    def _loop(self):
        while not self._stop.is_set():
            line = self._ser.readline().decode(errors="ignore").strip()
            if line.startswith("S "):
                try:
                    _, t, c, z, b = line.split()
                    self.state = HubState(int(t), int(c), int(z), int(b))
                except ValueError:
                    pass
            elif line.startswith(("READY", "ERR")):
                log.info("hub: %s", line)

    def _send(self, cmd: str):
        with self._lock:
            self._ser.write((cmd + "\n").encode())

    def servo_us(self, ch: int, us: int):
        self._send(f"SV {ch} {int(us)}")

    def relay(self, on: bool):
        self._send(f"RL {1 if on else 0}")

    def close(self):
        self._stop.set()
        if self._ser:
            try:
                self.relay(False)
            finally:
                self._ser.close()


class MockHub:
    """Simulated hub for dry runs/tests. Set fields on .state to fake sensor readings."""

    def __init__(self, *_, **__):
        self.state = HubState(t_ms=1, clip_mm=200, zone_mm=200)
        self.servos = {}
        self.relay_on = False
        self.log = []

    def connect(self, timeout: float = 0):
        pass

    def servo_us(self, ch, us):
        self.servos[ch] = int(us)
        self.log.append(("SV", ch, int(us)))

    def relay(self, on):
        self.relay_on = bool(on)
        self.log.append(("RL", bool(on)))

    def close(self):
        self.relay_on = False


def make_hub(cfg: dict, dry_run: bool = False):
    h = cfg.get("hub", {}) or {}
    if dry_run or not h.get("enabled", False):
        return MockHub()
    # don't probe the ports the arm / tactile sensor use when looking for the hub
    exclude = [cfg.get("follower", {}).get("port"), (cfg.get("tactile") or {}).get("port")]
    return Hub(h.get("port", "auto"), h.get("baud", 115200), exclude=exclude)
