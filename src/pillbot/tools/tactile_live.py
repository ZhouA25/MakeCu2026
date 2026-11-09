"""SensX25 bench tool — find the sensor, watch it live, tune contact_threshold. No arm needed.

  python -m pillbot.tools.tactile_live doctor                 which port is the sensor? what's wrong?
  python -m pillbot.tools.tactile_live doctor --port X        also sends the start command if X is silent
  python -m pillbot.tools.tactile_live                        live numbers, exactly what the gripper code sees
  python -m pillbot.tools.tactile_live plot                   live heatmap window (needs matplotlib + a screen)

`live` and `plot` use configs/robot.yaml -> tactile: (port, contact_threshold, backend). Add --port to
override; with `backend: net` they read from the tactile server instead of the port.
No sensor yet?  python -m pillbot.tools.fake_sensx   then pass the port it prints.
"""
import argparse
import os
import sys
import time

import numpy as np

from pillbot.sensors.sensx_probe import BAUD, candidate_ports, probe
from pillbot.sensors.tactile import SensXTactile, make_tactile
from pillbot.utils.config import load_yaml


# ------------------------------------------------------------------ doctor
def doctor(port: str | None) -> int:
    from serial.tools import list_ports
    print("Serial ports:")
    for p in list_ports.comports():
        ids = f"{p.vid:04x}:{p.pid:04x}" if p.vid is not None else "    -    "
        print(f"  {p.device:<34} {ids}  {p.description}")
    ports = [port] if port else candidate_ports()
    if not ports:
        print("\nNo USB serial ports. Plug in the USB-TTL adapter; if its LED is on but nothing shows up,\n"
              "the USB cable is charge-only or the hub port isn't passing data — try another.")
        return 1
    found = False
    for p in ports:
        print(f"\n== {p}")
        if sys.platform.startswith("linux") and not os.access(p, os.R_OK | os.W_OK):
            print("   permission denied: sudo usermod -aG dialout $USER, then log out and back in")
            continue
        try:
            # Only wake (send the start command to) a port the user named: see sensx_probe.probe.
            kind, counts, n = probe(p, BAUD, wake=bool(port))
        except OSError as e:                                   # includes serial.SerialException
            print(f"   can't open: {e}  (another program has it open?)")
            continue
        if kind:
            found = True
            print(f"   OK: {'single SensX25' if kind == 'single' else 'TouchTronix hub'} at {BAUD} baud, "
                  f"{sum(counts.values())} good frames in ~1 s")
            if kind == "hub":
                print("   (pillbot's sensx backend expects a single sensor; the hub needs SensXHub)")
            print(f"   put it in configs/robot.local.yaml ->  tactile: {{port: {p}}}")
        elif n:
            print(f"   {n} bytes but no SensX frames: wrong baud, or TX/RX/GND wiring is off")
        elif port:
            print("   silent even after the start command: check VCC/GND, swap TX<->RX, check the "
                  "adapter's 3.3V/5V jumper")
        else:
            print(f"   silent. A single SensX25 waits for a start command — if this is it, run:\n"
                  f"     python -m pillbot.tools.tactile_live doctor --port {p}")
    return 0 if found else 1


# ------------------------------------------------------------------ live
def open_sensor(args):
    cfg = load_yaml(args.config)
    t = cfg.setdefault("tactile", {})
    if args.port:
        t.update(backend="sensx", port=args.port)
    elif t.get("backend") not in ("sensx", "net"):
        sys.exit(f"tactile.backend is '{t.get('backend')}' — pass --port or set backend: sensx")
    tac = make_tactile(cfg)
    tac.connect()
    print("Taring — don't touch the sensor...")
    tac.tare()
    return tac


def live(tac):
    t0, best = time.perf_counter(), 0.0   # perf_counter: time.time() ticks ~15 ms on Windows
    sys.stdout.write("\033[2J")
    while True:
        s = tac.snapshot()
        best = max(best, s.pressure)
        rate = f"{tac.frames / max(time.perf_counter() - t0, 1e-3):5.0f} frames/s" if isinstance(tac, SensXTactile) else ""
        bar_len = 40
        fill = min(bar_len, int(bar_len * s.pressure / max(tac.contact_threshold * 2, 1)))
        bar = "#" * fill + "-" * (bar_len - fill)
        bar = bar[:bar_len // 2] + "|" + bar[bar_len // 2:]       # '|' = contact_threshold
        out = ["\033[H" + f"{getattr(tac, 'port', 'net')}  {rate}   Ctrl-C to quit (restart to re-tare)\033[K",
               "\033[K"]
        if s.frame is None:
            out.append("NO DATA — sensor unplugged or stalled\033[K")
        else:
            for row in s.frame:
                out.append("   " + "".join(f"{v:6.0f}" for v in row) + "\033[K")
            c = f"({s.centroid[0]:.1f}, {s.centroid[1]:.1f})" if s.centroid else "-"
            out += ["\033[K",
                    f"pressure {s.pressure:7.0f}  [{bar}]  threshold {tac.contact_threshold:g}\033[K",
                    f"contact  {'YES' if s.contact else 'no '}      peak taxel {s.peak:4.0f}   centroid {c}\033[K",
                    f"max pressure so far {best:7.0f}  (set contact_threshold well below a light grip)\033[K"]
        sys.stdout.write("\n".join(out) + "\033[J\n")
        sys.stdout.flush()
        time.sleep(0.05)


def plot(tac, vmax: float):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots(figsize=(4.5, 5))
    fig.canvas.manager.set_window_title("SensX25")
    img = ax.imshow(np.zeros(tac.shape), vmin=0, vmax=vmax, cmap="inferno")
    ax.set_xticks([]); ax.set_yticks([])
    fig.colorbar(img, ax=ax, shrink=0.8, label="kPa")
    dot, = ax.plot([], [], "c+", ms=18, mew=2)                 # centroid

    def update(_):
        s = tac.snapshot()
        if s.frame is None:
            ax.set_title("NO DATA")
            return img, dot
        img.set_data(s.frame)
        if s.centroid:
            dot.set_data([s.centroid[1]], [s.centroid[0]])          # imshow: x = col, y = row
        else:
            dot.set_data([], [])
        ax.set_title(f"{'CONTACT' if s.contact else 'no contact'}   pressure {s.pressure:.0f}")
        return img, dot

    fig.canvas.mpl_connect("key_press_event", lambda e: e.key == "t" and tac.tare())
    _anim = FuncAnimation(fig, update, interval=40, cache_frame_data=False)
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", nargs="?", default="live", choices=["live", "plot", "doctor"])
    ap.add_argument("--port", help="sensor serial port (default: tactile.port in the config)")
    ap.add_argument("--config", default="configs/robot.yaml")
    ap.add_argument("--vmax", type=float, default=200, help="plot: top of the color scale, kPa")
    args = ap.parse_args()

    if args.cmd == "doctor":
        sys.exit(doctor(args.port))
    try:
        tac = open_sensor(args)
    except RuntimeError as e:
        sys.exit(str(e))
    try:
        live(tac) if args.cmd == "live" else plot(tac, args.vmax)
    except KeyboardInterrupt:
        pass
    finally:
        tac.close()


if __name__ == "__main__":
    main()
