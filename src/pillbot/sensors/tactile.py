"""Tactile sensing for the gripper pads.

Backends (configs/robot.yaml -> tactile.backend):
  none    no sensor; grasping falls back to gripper-stall detection
  mock    simulated readings driven by the dry-run robot (for testing without hardware)
  serial  any device streaming one CSV line per frame (our FSR firmware, or a TactX bridge)
  sensx   TouchTronix SensX25 (SNX0505-SNS-01) via their sensx Python API
  sensx2  two SensX25s, one per jaw

All backends return a 2D numpy array (rows x cols) of pressure readings.
"""
import logging
import threading
import time
from abc import ABC, abstractmethod

import numpy as np

log = logging.getLogger(__name__)


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
        if f is None or f.sum() <= 0:
            return None
        rows, cols = np.indices(f.shape)
        return float((rows * f).sum() / f.sum()), float((cols * f).sum() / f.sum())


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


class SensXTactile(TactileSensor):
    """TouchTronix SensX 25 (SNX0505-SNS-01, 5x5 FSR, 12-bit values 0..4095).

    Install their driver:  git clone https://github.com/TouchTronix-Robotics/sensx_python
                           pip install -e sensx_python
    The 5x5 sensor runs at 921600 baud (the library default is for the 16x12 model).
    """

    def __init__(self, port: str, baud: int = 921600, shape=(5, 5), **kw):
        super().__init__(**kw)
        self.port, self.baud, self.shape = port, baud, tuple(shape)
        self._dev = None

    def connect(self):
        from sensx import SensX
        self._dev = SensX(port=self.port, baud_rate=self.baud, rows=self.shape[0], cols=self.shape[1])
        self._dev.start()
        t0 = time.time()
        while self._dev.latest_frame is None and time.time() - t0 < 3:
            time.sleep(0.05)
        if self._dev.latest_frame is None:
            raise RuntimeError(f"SensX on {self.port}: no frames (check port/baud/cable)")

    def read(self):
        f = self._dev.latest_frame if self._dev else None
        return None if f is None else np.asarray(f, dtype=float).reshape(self.shape)

    def close(self):
        if self._dev:
            self._dev.close()


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
    """configs/robot.yaml -> tactile.backend: sensx | sensx2 (one per jaw) | serial | mock | none"""
    t = cfg.get("tactile", {}) or {}
    backend = "mock" if dry_run else t.get("backend", "none")
    kw = dict(contact_threshold=t.get("contact_threshold", 50), slip_ratio=t.get("slip_ratio", 0.5))
    shape, baud = t.get("shape", (5, 5)), t.get("baud", 921600)
    if backend == "mock":
        return MockTactile(io, shape=shape, **kw)
    if backend == "serial":
        return SerialTactile(t["port"], t.get("baud", 115200), t.get("shape", (1, 1)), **kw)
    if backend == "sensx":
        return SensXTactile(t["port"], baud, shape, **kw)
    if backend == "sensx2":
        return CombinedTactile(SensXTactile(t["port"], baud, shape, **kw),
                               SensXTactile(t["port2"], baud, shape, **kw))
    return NullTactile(**kw)
