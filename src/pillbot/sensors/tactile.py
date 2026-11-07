"""Tactile sensing for the gripper pads.

Backends (configs/robot.yaml -> tactile.backend):
  none    no sensor; grasping falls back to gripper-stall detection
  mock    simulated readings driven by the dry-run robot (for testing without hardware)
  serial  any device streaming one CSV line per frame (our FSR firmware, or a TactX bridge)
  sensx   TouchTronix SensX25 (SNX0505-SNS-01) via their sensx Python API
  sensx2  two SensX25s, one per jaw
  net     frames from `python -m pillbot.sensors.tactile_server`, so several programs can share
          one sensor (only one program can open a serial port)

All backends return a 2D numpy array (rows x cols) of pressure readings.
How the sensor works, the Pi setup and a tuning guide: docs/tactile.md

Quickest way in from any script:

    from pillbot.sensors.tactile import open_tactile
    with open_tactile() as tac:              # configs/robot.yaml -> tactile:, tared on open
        s = tac.snapshot()                   # one consistent reading
        print(s.contact, s.pressure, s.centroid)
"""
import json
import logging
import socket
import threading
import time
from abc import ABC, abstractmethod
from contextlib import contextmanager
from dataclasses import dataclass

import numpy as np

log = logging.getLogger(__name__)


@dataclass
class TactileSnapshot:
    """Everything the gripper logic needs, computed from ONE frame so the values agree."""
    frame: np.ndarray | None                 # rows x cols, raw (untared); None = no fresh data
    pressure: float                          # sum of all taxels minus the tare baseline
    contact: bool                            # pressure >= contact_threshold
    peak: float                              # hottest single taxel (raw)
    centroid: tuple[float, float] | None     # pressure-weighted (row, col); moves when the cap slides


def _centroid(f: np.ndarray) -> tuple[float, float] | None:
    total = f.sum()
    if total <= 0:
        return None
    rows, cols = np.indices(f.shape)
    return float((rows * f).sum() / total), float((cols * f).sum() / total)


class TactileSensor(ABC):
    def __init__(self, contact_threshold: float = 50.0, slip_ratio: float = 0.5):
        self.contact_threshold = contact_threshold
        self.slip_ratio = slip_ratio
        self._baseline = 0.0

    available = True

    def connect(self): ...
    def close(self): ...

    @abstractmethod
    def read(self) -> np.ndarray | None:
        """Latest pressure frame, or None if no data."""

    def tare(self, n: int = 10):
        """Record the no-contact baseline. Call with the jaws open and empty."""
        vals = []
        for _ in range(n):
            f = self.read()
            if f is not None:
                vals.append(float(f.sum()))
            time.sleep(0.01)
        self._baseline = float(np.mean(vals)) if vals else 0.0

    def pressure(self) -> float:
        f = self.read()
        return 0.0 if f is None else max(0.0, float(f.sum()) - self._baseline)

    def in_contact(self) -> bool:
        return self.pressure() >= self.contact_threshold

    def is_slipping(self, grip_pressure: float) -> bool:
        """Pressure dropped well below what we had right after gripping."""
        return grip_pressure > 0 and self.pressure() < self.slip_ratio * grip_pressure

    def centroid(self) -> tuple[float, float] | None:
        """Pressure-weighted (row, col) contact center — shifts when the cap slides in the jaws."""
        f = self.read()
        return None if f is None else _centroid(f)

    def snapshot(self) -> TactileSnapshot:
        """pressure / contact / centroid from a single frame (separate calls may each see a new one)."""
        f = self.read()
        if f is None:
            return TactileSnapshot(None, 0.0, False, 0.0, None)
        p = max(0.0, float(f.sum()) - self._baseline)
        return TactileSnapshot(f, p, p >= self.contact_threshold, float(f.max()), _centroid(f))


class NullTactile(TactileSensor):
    available = False

    def read(self):
        return None

    def in_contact(self):
        return False

    def is_slipping(self, grip_pressure):
        return False


class MockTactile(TactileSensor):
    """Reports contact when the dry-run gripper is pressed against its simulated object."""

    def __init__(self, io, shape=(5, 5), **kw):
        super().__init__(**kw)
        self.io = io
        self.shape = tuple(shape)

    def read(self):
        block = self.io.sim_gripper_block
        cmd = self.io.target["gripper"]
        if block is None or cmd > block:
            return np.zeros(self.shape)
        return np.full(self.shape, 4.0 * (1 + block - cmd))  # harder squeeze = more pressure


class SerialTactile(TactileSensor):
    """Reads frames like `12,40,33,...` (row-major) from a serial port on a background thread."""

    def __init__(self, port: str, baud: int = 115200, shape=(1, 1), **kw):
        super().__init__(**kw)
        self.port, self.baud, self.shape = port, baud, tuple(shape)
        self._latest = None
        self._stop = threading.Event()
        self._ser = None

    def connect(self):
        import serial
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        threading.Thread(target=self._loop, daemon=True).start()
        time.sleep(0.3)

    def _loop(self):
        n = int(np.prod(self.shape))
        while not self._stop.is_set():
            line = self._ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            try:
                vals = [float(x) for x in line.split(",")]
            except ValueError:
                continue
            if len(vals) == n:
                self._latest = np.array(vals).reshape(self.shape)

    def read(self):
        return self._latest

    def close(self):
        self._stop.set()
        if self._ser:
            self._ser.close()


class _StaleGuard:
    """Turns a frame older than `max_age_s` into None, so an unplugged sensor reads as
    "no data" instead of repeating its last frame forever (e.g. "still gripping")."""

    def __init__(self, name: str, max_age_s: float):
        self.name, self.max_age_s, self._warned = name, max_age_s, False

    def check(self, frame, age_s: float):
        if frame is not None and age_s <= self.max_age_s:
            self._warned = False
            return frame
        if frame is not None and not self._warned:
            log.error("tactile %s: no new frame for %.2f s — cable/adapter unplugged?", self.name, age_s)
            self._warned = True
        return None


class SensXTactile(TactileSensor):
    """TouchTronix SensX 25 (SNX0505-SNS-01, 5x5) on a USB-TTL adapter, ~290 frames/s.

    Driver: the sensx_python submodule (pip install -e sensx_python).
    Values are kPa per taxel (0..534) in the sensor's factory "pressure" mode, 0 when untouched.
    The 5x5 sensor runs at 921600 baud (the library default is for the 16x12 model).
    """

    def __init__(self, port: str, baud: int = 921600, shape=(5, 5), max_age_s: float = 0.25, **kw):
        super().__init__(**kw)
        self.port, self.baud, self.shape = port, baud, tuple(shape)
        self.frames = 0                     # frames received since connect (rate = frames / elapsed)
        self._guard = _StaleGuard(port, max_age_s)
        self._dev = None

    def connect(self):
        from sensx import SensX
        from pillbot.sensors.sensx_probe import serial_baud
        # Opening the port also sends the start command; the sensor then streams on its own.
        self._dev = SensX(port=self.port, baud_rate=serial_baud(self.port, self.baud),
                          rows=self.shape[0], cols=self.shape[1])
        self._dev.on_frame = self._count
        self._dev.start()
        t0 = time.time()
        while self._dev.latest_timestamp == 0 and time.time() - t0 < 3:   # frame starts as zeros, not None
            time.sleep(0.05)
        if self._dev.latest_timestamp == 0:
            self._dev.close()
            raise RuntimeError(f"SensX on {self.port}: no frames in 3 s. "
                               "Run `python -m pillbot.tools.tactile_live doctor`")

    def _count(self, frame, ts):
        self.frames += 1

    def read(self):
        if self._dev is None:
            return None
        f = np.asarray(self._dev.latest_frame, dtype=float).reshape(self.shape)
        return self._guard.check(f, time.perf_counter() - self._dev.latest_timestamp)

    def close(self):
        if self._dev:
            self._dev.close()


class NetTactile(TactileSensor):
    """Frames streamed by `python -m pillbot.sensors.tactile_server` (JSON lines over TCP).

    Tares and thresholds locally, exactly like SensXTactile, so code can't tell the difference.
    """

    def __init__(self, url: str = "localhost:8765", shape=(5, 5), max_age_s: float = 0.25, **kw):
        super().__init__(**kw)
        host, _, port = url.removeprefix("tcp://").rpartition(":")
        self.addr, self.shape = (host or "localhost", int(port)), tuple(shape)
        self._latest: tuple[np.ndarray, float] | None = None
        self._guard = _StaleGuard(url, max_age_s)
        self._sock = None

    def connect(self):
        try:
            self._sock = socket.create_connection(self.addr, timeout=3)
        except OSError as e:
            raise RuntimeError(f"tactile server {self.addr[0]}:{self.addr[1]}: {e} — "
                               "start it with `python -m pillbot.sensors.tactile_server`") from e
        self._sock.settimeout(None)
        threading.Thread(target=self._loop, daemon=True).start()
        t0 = time.time()
        while self._latest is None and time.time() - t0 < 3:
            time.sleep(0.05)
        if self._latest is None:
            raise RuntimeError(f"tactile server {self.addr}: connected but no frames")

    def _loop(self):
        try:
            for line in self._sock.makefile("r"):
                try:
                    f = np.array(json.loads(line)["frame"], dtype=float).reshape(self.shape)
                except (ValueError, KeyError):
                    continue
                self._latest = (f, time.perf_counter())
        except OSError:
            pass                                   # closed; read() goes stale and returns None

    def read(self):
        if self._latest is None:
            return None
        f, stamp = self._latest
        return self._guard.check(f, time.perf_counter() - stamp)

    def close(self):
        if self._sock:
            self._sock.close()


class CombinedTactile(TactileSensor):
    """Two sensors (one per jaw): contact if EITHER feels it; pressure/slip from the primary."""

    def __init__(self, primary: TactileSensor, secondary: TactileSensor):
        super().__init__(primary.contact_threshold, primary.slip_ratio)
        self.primary, self.secondary = primary, secondary

    def connect(self):
        self.primary.connect(); self.secondary.connect()

    def close(self):
        self.primary.close(); self.secondary.close()

    def tare(self, n: int = 10):
        self.primary.tare(n); self.secondary.tare(n)

    def read(self):
        return self.primary.read()

    def pressure(self):
        return self.primary.pressure()

    def in_contact(self):
        return self.primary.in_contact() or self.secondary.in_contact()

    def is_slipping(self, grip_pressure):
        return self.primary.is_slipping(grip_pressure)


TactXTactile = SensXTactile   # old name


def make_tactile(cfg: dict, io=None, hub=None, dry_run: bool = False) -> TactileSensor:
    """configs/robot.yaml -> tactile.backend: sensx | sensx2 (one per jaw) | net | serial | mock | none"""
    t = cfg.get("tactile", {}) or {}
    backend = "mock" if dry_run else t.get("backend", "none")
    kw = dict(contact_threshold=t.get("contact_threshold", 50), slip_ratio=t.get("slip_ratio", 0.5))
    shape, baud, age = t.get("shape", (5, 5)), t.get("baud", 921600), t.get("max_age_s", 0.25)
    if backend == "mock":
        return MockTactile(io, shape=shape, **kw)
    if backend == "serial":
        return SerialTactile(t["port"], t.get("baud", 115200), t.get("shape", (1, 1)), **kw)
    if backend == "sensx":
        return SensXTactile(t["port"], baud, shape, max_age_s=age, **kw)
    if backend == "sensx2":
        return CombinedTactile(SensXTactile(t["port"], baud, shape, max_age_s=age, **kw),
                               SensXTactile(t["port2"], baud, shape, max_age_s=age, **kw))
    if backend == "net":
        return NetTactile(t.get("url", "localhost:8765"), shape, max_age_s=age, **kw)
    return NullTactile(**kw)


@contextmanager
def open_tactile(config: str = "configs/robot.yaml", tare: bool = True, **overrides):
    """make_tactile + connect + tare + close, for scripts outside the main pipeline.

        with open_tactile(port="/dev/ttyUSB0") as tac:     # overrides any tactile: key
            while not tac.in_contact(): ...
    """
    from pillbot.utils.config import load_yaml
    cfg = load_yaml(config)
    cfg["tactile"] = {**(cfg.get("tactile") or {}), **overrides}
    tac = make_tactile(cfg)
    tac.connect()
    try:
        if tare:
            tac.tare()
        yield tac
    finally:
        tac.close()
