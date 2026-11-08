"""One reader per camera, shared by everything in the process.

The ESP32-CAM serves ONE stream client. SharedCamera keeps that single connection open on a
background thread and hands the latest frame to whoever asks (bottle detection in run.py, the
live viewer, ...). It also means frames are always fresh — no stale buffer after the arm moves.
"""
import logging
import threading
import time

log = logging.getLogger(__name__)


class SharedCamera:
    _instances: dict[str, "SharedCamera"] = {}
    _lock = threading.Lock()

    def __init__(self, open_fn, name: str = "camera"):
        self._open_fn = open_fn          # () -> object with .read() -> (ok, frame) and .release()
        self.name = name
        self.frame = None
        self.stamp = 0.0
        self.fps = 0.0
        self.error = ""
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name=f"cam-{name}")
        self._thread.start()

    @classmethod
    def get(cls, cfg: dict, name: str | None = None) -> "SharedCamera":
        """Shared reader for a camera from robot.yaml (default: perception_camera)."""
        from pillbot.perception.cameras import open_camera
        name = name or cfg.get("perception_camera", "wrist")
        with cls._lock:
            cam = cls._instances.get(name)
            if cam is None or not cam._thread.is_alive():
                cam = cls(lambda: open_camera(cfg["cameras"][name]), name)
                cls._instances[name] = cam
            return cam

    def _loop(self):
        cap, last = None, time.time()
        while not self._stop.is_set():
            if cap is None:
                try:
                    cap = self._open_fn()
                    self.error = ""
                except Exception as e:          # camera off / wrong URL: keep retrying
                    self.error = str(e)
                    time.sleep(1.0)
                    continue
            ok, frame = cap.read()
            if not ok or frame is None:
                self.error = "read failed (camera busy or offline) — reconnecting"
                cap.release()
                cap = None
                time.sleep(0.5)
                continue
            now = time.time()
            self.fps = 0.9 * self.fps + 0.1 / max(now - last, 1e-3) if self.fps else 1 / max(now - last, 1e-3)
            last = now
            with self._cond:
                self.frame, self.stamp = frame, now
                self._cond.notify_all()
        if cap is not None:
            cap.release()

    def latest(self):
        """(frame copy or None, timestamp)."""
        with self._cond:
            return (None if self.frame is None else self.frame.copy()), self.stamp

    def wait_new(self, after: float | None = None, timeout: float = 3.0):
        """Block until a frame newer than `after` (default: now) arrives. Returns the frame or None."""
        after = time.time() if after is None else after
        deadline = time.time() + timeout
        with self._cond:
            while self.stamp <= after:
                left = deadline - time.time()
                if left <= 0:
                    return None
                self._cond.wait(left)
            return self.frame.copy()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        with SharedCamera._lock:
            for k, v in list(SharedCamera._instances.items()):
                if v is self:
                    del SharedCamera._instances[k]
