"""A pretend SensX25 on a virtual serial port, byte-for-byte the real protocol (macOS / Linux).

  python -m pillbot.tools.fake_sensx            prints a port like /dev/ttys012 — use it as --port
  python -m pillbot.tools.fake_sensx --hub ABC  pretend to be the TouchTronix hub instead

Like the real single sensor it stays silent until it receives the start command, then streams
~290 frames/s. A pressure blob wanders over the pad and squeezes on and off.
"""
import argparse
import math
import os
import select
import time
import tty

import numpy as np

from pillbot.sensors.sensx_probe import COLS, HUB_HEADERS, ROWS, SINGLE_HEADER, crc8_maxim

HUB_BY_LETTER = {v: k for k, v in HUB_HEADERS.items()}


def blob(t: float, phase: float = 0.0) -> np.ndarray:
    r = 2 + 1.5 * math.sin(0.7 * t + phase)
    c = 2 + 1.5 * math.cos(0.5 * t + 2 * phase)
    strength = 250 * max(0.0, math.sin(0.8 * t + phase))
    rr, cc = np.indices((ROWS, COLS))
    f = strength * np.exp(-((rr - r) ** 2 + (cc - c) ** 2) / 1.2)
    return np.clip(f, 0, 534).astype(">u2")                    # pressure mode: 0..534 kPa, big-endian


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hub", metavar="LETTERS", help="emulate the hub with these channels, e.g. ABC")
    ap.add_argument("--hz", type=float, default=290)
    args = ap.parse_args()

    master, slave = os.openpty()
    tty.setraw(slave)                       # binary data: no newline translation
    os.set_blocking(master, False)
    print(f"fake {'hub ' + args.hub if args.hub else 'SensX25'} on {os.ttyname(slave)}   (Ctrl-C to stop)",
          flush=True)

    streaming, rx, t0 = bool(args.hub), b"", time.perf_counter()
    next_tick = t0
    try:
        while True:
            if select.select([master], [], [], 0)[0]:
                try:
                    rx += os.read(master, 4096)
                except OSError:
                    pass
                if b"\x01\x06\x60\x10" in rx:                    # the start command
                    streaming, rx = True, b""
                rx = rx[-16:]
            now = time.perf_counter()
            if streaming and now >= next_tick:
                next_tick = max(next_tick + 1 / args.hz, now - 0.1)
                if args.hub:
                    for i, ch in enumerate(args.hub.upper()):
                        _write(master, HUB_BY_LETTER[ch] + blob(now - t0, i * 1.3).tobytes())
                else:
                    payload = blob(now - t0).tobytes()
                    _write(master, SINGLE_HEADER + payload + bytes([crc8_maxim(payload)]))
            time.sleep(0.0005)
    except KeyboardInterrupt:
        pass


def _write(fd, data):
    try:
        os.write(fd, data)
    except OSError:
        pass                                # nobody reading: drop, like a real UART


if __name__ == "__main__":
    main()
