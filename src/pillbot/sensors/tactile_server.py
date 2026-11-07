"""Share one SensX25 with several programs: owns the serial port, streams JSON lines over TCP.

Only one program can open a serial port. Run this instead, and every other program connects to it:

    python -m pillbot.sensors.tactile_server                 # port from configs/robot.yaml
    python -m pillbot.sensors.tactile_server --port /dev/ttyUSB0 --listen 0.0.0.0:8765

Python clients: set `tactile.backend: net` (and `url:`) in configs/robot.local.yaml — make_tactile()
then returns a NetTactile that behaves exactly like the real sensor.
Any other language: open a TCP socket and read lines. Each line is one JSON object:

    {"t": 1730999999.123,               sender's time.time()
     "frame": [[0, 0, 12, ...], ...],   5x5 raw taxels, kPa (row 0 first)
     "pressure": 312.0,                 sum above the server's startup tare
     "contact": true,                   pressure >= contact_threshold
     "peak": 140.0,                     hottest taxel
     "centroid": [2.1, 1.8]}            (row, col) or null

Try it:  nc localhost 8765
"""
import argparse
import json
import logging
import socket
import threading
import time

from pillbot.sensors.tactile import SensXTactile
from pillbot.utils.config import load_yaml

log = logging.getLogger(__name__)


class TactileServer:
    def __init__(self, sensor, host: str = "127.0.0.1", port: int = 8765, hz: float = 100.0):
        self.sensor, self.hz = sensor, hz
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        self._srv = socket.create_server((host, port))
        self.address = self._srv.getsockname()[:2]

    def _accept(self):
        while True:
            try:
                c, addr = self._srv.accept()
            except OSError:
                return                                  # server closed
            c.settimeout(0.05)                          # a stuck client gets dropped, not waited on
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._clients.append(c)
            log.info("client %s:%d connected", *addr[:2])

    def serve_forever(self):
        threading.Thread(target=self._accept, daemon=True).start()
        period = 1.0 / self.hz
        while True:
            s = self.sensor.snapshot()
            if s.frame is not None:
                msg = {"t": time.time(), "frame": s.frame.tolist(), "pressure": s.pressure,
                       "contact": s.contact, "peak": s.peak, "centroid": s.centroid}
                self._broadcast((json.dumps(msg) + "\n").encode())
            time.sleep(period)

    def _broadcast(self, data: bytes):
        with self._lock:
            for c in list(self._clients):
                try:
                    c.sendall(data)
                except OSError:
                    self._clients.remove(c)
                    c.close()
                    log.info("client dropped")

    def close(self):
        self._srv.close()
        with self._lock:
            for c in self._clients:
                c.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/robot.yaml")
    ap.add_argument("--port", help="sensor serial port (default: tactile.port in the config)")
    ap.add_argument("--listen", default="127.0.0.1:8765",
                    help="host:port to serve on; 0.0.0.0:8765 lets other machines connect")
    ap.add_argument("--hz", type=float, default=100.0, help="lines per second sent to each client")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")

    t = load_yaml(args.config).get("tactile", {}) or {}
    sensor = SensXTactile(args.port or t["port"], t.get("baud", 921600), t.get("shape", (5, 5)),
                          max_age_s=t.get("max_age_s", 0.25),
                          contact_threshold=t.get("contact_threshold", 50), slip_ratio=t.get("slip_ratio", 0.5))
    sensor.connect()
    log.info("taring — don't touch the sensor")
    sensor.tare()
    host, _, port = args.listen.rpartition(":")
    srv = TactileServer(sensor, host or "127.0.0.1", int(port), args.hz)
    log.info("serving %s on %s:%d (Ctrl-C to stop)", sensor.port, *srv.address)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        sensor.close()


if __name__ == "__main__":
    main()
