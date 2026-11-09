"""Set up PillBot on a new computer: write the local config, check everything, move local files.

  python -m pillbot.tools.setup_local init                 detect the U2D2, hub and camera -> configs/robot.local.yaml
  python -m pillbot.tools.setup_local init --camera-ip 10.20.70.37
  python -m pillbot.tools.setup_local find-camera          scan the network for the ESP32-CAM (init does this too)
  python -m pillbot.tools.setup_local init --no-hub        no ESP32 hub on this setup: hub.enabled false (no clamp/buttons)
  python -m pillbot.tools.setup_local init --fresh         start the local file over (old one backed up)
  python -m pillbot.tools.setup_local check                what's installed, connected, calibrated and taught
  python -m pillbot.tools.setup_local export               pack this computer's local-only files into a .tgz
  python -m pillbot.tools.setup_local import pillbot-local-....tgz
                                                           unpack them on the new computer (backs up what's there)

Local-only files (NOT in git, so `git clone` doesn't bring them):
  configs/*.local.yaml            your ports / camera URL / tuned values, zone calibration
  configs/background.local.png    empty-zone background for bottle detection
  LeRobot motor calibration       ~/.cache/huggingface/lerobot/calibration/{robots,teleoperators}/pillbot_*
  firmware/esp32_cam/secrets.h    Wi-Fi password: only exported with --with-secrets

On a new computer:  bash scripts/setup_laptop.sh  (env + installs), then import, init, check.
"""
import argparse
import datetime as dt
import os
import platform
import shutil
import socket
import sys
import tarfile
import urllib.request
from pathlib import Path
from urllib.parse import urlparse

import yaml

from pillbot.utils.config import ROOT, deep_merge, load_yaml

LOCAL = ROOT / "configs/robot.local.yaml"
LOCAL_FILES = ["configs/*.local.yaml", "configs/*.local.png", "configs/*.local.json"]
SECRETS = "firmware/esp32_cam/secrets.h"
CAM_HOST = "pillbot-cam.local"
OK, BAD, WARN = "  ok ", "  !! ", "  -- "


# ------------------------------------------------------------------ helpers
def calibration_dir() -> Path:
    """Where LeRobot keeps motor calibration (same rule LeRobot uses)."""
    for mod in ("lerobot.utils.constants", "lerobot.constants"):
        try:
            return Path(__import__(mod, fromlist=["HF_LEROBOT_CALIBRATION"]).HF_LEROBOT_CALIBRATION)
        except (ImportError, AttributeError):
            continue
    if os.environ.get("HF_LEROBOT_CALIBRATION"):
        return Path(os.environ["HF_LEROBOT_CALIBRATION"]).expanduser()
    home = Path(os.environ.get("HF_LEROBOT_HOME") or
                Path(os.environ.get("HF_HOME", "~/.cache/huggingface")).expanduser() / "lerobot").expanduser()
    return home / "calibration"


def calibration_files() -> list[Path]:
    base = calibration_dir()
    return sorted(p for kind in ("robots", "teleoperators") for p in (base / kind).glob("pillbot_*/*.json"))


def camera_ok(url: str, timeout: float = 3.0) -> tuple[bool, str]:
    """Grab one JPEG from the ESP32-CAM (port 80 /capture, so the :81 stream stays free)."""
    u = urlparse(url)
    if not u.hostname:
        return False, "not a URL"
    try:
        socket.gethostbyname(u.hostname)
    except OSError:
        return False, f"can't resolve {u.hostname}"
    try:
        with urllib.request.urlopen(f"{u.scheme or 'http'}://{u.hostname}/capture", timeout=timeout) as r:
            data = r.read(4)
        return (data[:2] == b"\xff\xd8"), ("jpeg" if data[:2] == b"\xff\xd8" else "not a JPEG")
    except Exception as e:  # noqa: BLE001 — any network error = not reachable
        return False, str(getattr(e, "reason", e))


def local_ipv4s() -> list[str]:
    """This computer's LAN IPv4 addresses (no loopback / link-local)."""
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))                 # picks the default-route interface; sends nothing
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        ips.update(a[4][0] for a in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET))
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith(("127.", "169.254.")))


def find_camera(timeout: float = 0.4, workers: int = 64) -> str | None:
    """Scan this computer's /24 network(s) for an ESP32-CAM: port 81 open and a JPEG at :80/capture.
    For when pillbot-cam.local doesn't resolve (Windows apps often can't do .local names)."""
    from concurrent.futures import ThreadPoolExecutor

    def port_open(ip):
        try:
            with socket.create_connection((ip, 81), timeout=timeout):
                return ip
        except OSError:
            return None

    for me in local_ipv4s():
        base = me.rsplit(".", 1)[0]
        hosts = [f"{base}.{i}" for i in range(1, 255) if f"{base}.{i}" != me]
        print(f"   scanning {base}.0/24 for the camera...")
        with ThreadPoolExecutor(workers) as ex:
            candidates = [ip for ip in ex.map(port_open, hosts) if ip]
        for ip in candidates:
            if camera_ok(f"http://{ip}:81/stream")[0]:
                return ip
    return None


def ip_url(url: str) -> str:
    """http://name.local:81/stream -> http://<ip>:81/stream when the name resolves here: OpenCV/ffmpeg
    on Windows can't resolve .local names even when Python can."""
    u = urlparse(url)
    if not (u.hostname or "").endswith(".local"):
        return url
    try:
        ip = socket.gethostbyname(u.hostname)
    except OSError:
        return url
    return url.replace(u.hostname, ip, 1)


def find_ports(probe: bool = True) -> dict:
    """{'follower': port|None, 'hub': port|None, 'all': [...]}: U2D2 by its USB id, hub by its messages."""
    from pillbot.utils.ports import list_serial, probe_hub, stable_name
    ports = list_serial()
    u2d2 = [p for p in ports if (p["vid"], p["pid"]) == (0x0403, 0x6014) or "u2d2" in p["guess"].lower()]
    follower = stable_name(u2d2[0]["device"]) if len(u2d2) == 1 else None
    hub = None
    if probe:
        for p in ports:
            if p in u2d2:
                continue
            if probe_hub(p["device"]):
                hub = stable_name(p["device"])
                break
    return {"follower": follower, "hub": hub, "u2d2": u2d2, "all": ports}


def backup(path: Path) -> Path | None:
    if not path.exists():
        return None
    dest = path.with_name(path.name + ".bak-" + dt.datetime.now().strftime("%Y%m%d-%H%M%S"))
    shutil.copy2(path, dest)
    return dest


def ask(prompt: str, default: str, yes: bool) -> str:
    if yes:
        return default
    r = input(f"{prompt} [{default}] ").strip()
    return r or default


# ------------------------------------------------------------------ init
def cmd_init(args):
    first_time = not LOCAL.exists()
    current = {} if (first_time or args.fresh) else (yaml.safe_load(LOCAL.read_text()) or {})
    no_hub = getattr(args, "no_hub", False)
    probe = not (args.no_probe or no_hub)
    print("Looking for the U2D2 and the hub..." if probe else "Looking for the U2D2...")
    found = find_ports(probe=probe)
    for p in found["all"]:
        print(f"   {p['device']:32s} {p['guess']}")
    if not found["all"]:
        print("   no USB serial devices — plug in the U2D2 and the ESP32 (data cables)")
    if len(found["u2d2"]) > 1:
        print("   more than one U2D2-like device: unplug the extra one, or type the port below")

    follower = ask("U2D2 port (follower.port)", found["follower"] or
                   (current.get("follower") or {}).get("port", "") or "", args.yes)
    if no_hub:
        print("   --no-hub: hub.enabled false (no clamp, no START/STOP buttons; Ctrl-C stops a run)")
        hub_cfg = {"enabled": False}
    else:
        hub_port = "auto" if found["hub"] or args.no_probe else ask(
            "Hub not found (flashed? Serial Monitor closed?). hub.port", "auto", args.yes)
        if found["hub"]:
            print(f"   hub found on {found['hub']} -> hub.port: auto")
        hub_cfg = {"enabled": True, "port": hub_port}

    cams = []
    if args.camera_ip:
        cams.append(f"http://{args.camera_ip}:81/stream")
    old_url = (((current.get("cameras") or {}).get("wrist")) or {}).get("url")
    cams += [u for u in (f"http://{CAM_HOST}:81/stream", old_url) if u and u not in cams]
    url = None
    for u in cams:
        ok, why = camera_ok(u)
        print(f"   camera {u}: {'OK' if ok else why}")
        if ok:
            url = u
            break
    if url is None and not getattr(args, "no_scan", False):
        ip = find_camera()
        if ip:
            url = f"http://{ip}:81/stream"
            print(f"   camera found at {ip}")
        else:
            print("   no camera found on this network (same Wi-Fi? camera powered?)")
    if url is None:
        url = ask("Camera stream URL (http://<ip>:81/stream)", cams[0], args.yes)
    elif platform.system() == "Windows":
        url = ip_url(url)

    update = {"follower": {"port": follower} if follower else {},
              "hub": hub_cfg,
              "cameras": {"wrist": {"url": url}}}
    # Keep everything already in the local file (tuned values) and only set these keys.
    new = deep_merge(current, update)
    text = ("# Written by `python -m pillbot.tools.setup_local init` — only this computer's settings.\n"
            "# Everything else comes from configs/robot.yaml. Re-run init after moving computers.\n"
            + yaml.safe_dump(new, sort_keys=False))
    if args.dry_run:
        print("\n" + text)
        return
    b = backup(LOCAL)
    LOCAL.write_text(text)
    print(f"\nwrote {LOCAL.relative_to(ROOT)}" + (f" (old one saved as {b.name})" if b else ""))
    print("Next: python -m pillbot.tools.setup_local check")


def cmd_find_camera(args):
    ip = find_camera()
    if not ip:
        print("No camera found. Same Wi-Fi as the camera? Is it powered (LED / serial monitor)?")
        return 1
    print(f"Camera: http://{ip}:81/stream\nSave it:  python -m pillbot.tools.setup_local init --camera-ip {ip}")
    return 0


# ------------------------------------------------------------------ check
def _line(ok, what, fix=""):
    mark = OK if ok is True else (WARN if ok is None else BAD)
    print(f"{mark}{what}" + (f"\n        -> {fix}" if fix and ok is not True else ""))
    return ok is not False


def cmd_check(args):
    good = True
    print("Software")
    good &= _line(sys.version_info >= (3, 12), f"Python {platform.python_version()}",
                  "conda activate pillbot (needs Python 3.12+)")
    for mod, fix, needed in [
        ("lerobot", 'pip install -e ".[dev]"', True),
        ("lerobot_robot_pillbot", "pip install -e plugins/lerobot_robot_pillbot", True),
        ("serial", "pip install pyserial", True),
        ("cv2", "pip install opencv-python", True),
        ("sensx", "pip install -e sensx_python (only for the SensX pad)", False),
        ("mujoco", 'pip install -e ".[sim]" (only for the simulator)', False)]:
        try:
            __import__(mod)
            _line(True, mod)
        except Exception:  # noqa: BLE001
            good &= _line(False if needed else None, f"{mod} not importable", fix)

    print("\nConfig")
    cfg = load_yaml("configs/robot.yaml")
    good &= _line(LOCAL.exists(), "configs/robot.local.yaml", "python -m pillbot.tools.setup_local init")
    port = cfg["follower"].get("port", "")
    good &= _line(bool(port) and "XXXX" not in port and Path(port).exists() if port.startswith("/dev") else bool(port),
                  f"follower.port = {port}", "plug in the U2D2, then setup_local init")
    if (cfg.get("hub") or {}).get("enabled"):
        try:
            from pillbot.utils.ports import resolve_port
            hp = resolve_port(cfg["hub"]["port"], "hub", exclude=[port])
            _line(True, f"hub on {hp}")
        except Exception as e:  # noqa: BLE001
            good &= _line(False, f"hub: {e}", "flash firmware/esp32_hub, close the Serial Monitor, check the cable")
    else:
        _line(None, "hub disabled (no clamp)", "set hub.enabled: true")
    url = ((cfg.get("cameras") or {}).get("wrist") or {}).get("url")
    if url:
        ok, why = camera_ok(url)
        good &= _line(ok, f"camera {url} ({why})",
                      "same Wi-Fi as the camera? macOS: allow Terminal under Privacy & Security -> Local Network")

    print("\nCalibration and teaching")
    cal = calibration_files()
    fid = cfg["follower"].get("id", "pillbot_follower")
    good &= _line(any(p.stem == fid and "robots" in p.parts for p in cal),
                  f"LeRobot calibration for '{fid}' in {calibration_dir()}",
                  "bash scripts/calibrate.sh (or import it from the old computer)")
    _line((ROOT / "configs/zone_calib.local.yaml").exists() or None, "zone calibration",
          "python -m pillbot.perception.zone --calibrate --goto-scan")
    _line((ROOT / "configs/background.local.png").exists() or None, "detection background",
          "python -m pillbot.perception.detect_bottle --goto-scan, press b with the zone empty")
    from pillbot.control.poses import load_poses, untaught
    missing = untaught(load_poses())
    _line(not missing or None, f"poses taught ({len(missing)} still untaught)" if missing else "all poses taught",
          "python -m pillbot.control.teach --missing")
    print("\n" + ("Ready." if good else "Fix the !! lines above."))
    return 0 if good else 1


# ------------------------------------------------------------------ export / import
def cmd_export(args):
    out = Path(args.out or f"pillbot-local-{platform.node().split('.')[0]}-{dt.date.today()}.tgz")
    files = sorted({p for pat in LOCAL_FILES for p in ROOT.glob(pat)})
    if args.with_secrets and (ROOT / SECRETS).exists():
        files.append(ROOT / SECRETS)
    cal = calibration_files()
    with tarfile.open(out, "w:gz") as t:
        for p in files:
            t.add(p, arcname="repo/" + str(p.relative_to(ROOT)))
        for p in cal:
            t.add(p, arcname="calibration/" + str(p.relative_to(calibration_dir())))
    for p in files:
        print("  repo/" + str(p.relative_to(ROOT)))
    for p in cal:
        print("  calibration/" + str(p.relative_to(calibration_dir())))
    if not args.with_secrets and (ROOT / SECRETS).exists():
        print("  (camera Wi-Fi secrets.h left out — add --with-secrets only if you'll re-flash the camera)")
    print(f"wrote {out} ({len(files) + len(cal)} files). Copy it to the new computer, then:\n"
          f"  python -m pillbot.tools.setup_local import {out.name}")


def cmd_import(args):
    cal_base = calibration_dir()
    n = 0
    with tarfile.open(args.bundle, "r:gz") as t:
        for m in t.getmembers():
            if not m.isfile():
                continue
            parts = Path(m.name).parts
            if ".." in parts or Path(m.name).is_absolute() or parts[0] not in ("repo", "calibration"):
                print(f"  skipped {m.name}")
                continue
            dest = (ROOT if parts[0] == "repo" else cal_base).joinpath(*parts[1:])
            if args.dry_run:
                print(f"  would write {dest}")
                continue
            dest.parent.mkdir(parents=True, exist_ok=True)
            b = backup(dest)
            with t.extractfile(m) as src, open(dest, "wb") as f:
                shutil.copyfileobj(src, f)
            print(f"  {dest}" + (f"   (old: {b.name})" if b else ""))
            n += 1
    if not args.dry_run:
        print(f"imported {n} files. The ports differ on this computer: run "
              "python -m pillbot.tools.setup_local init, then check")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("init", help="detect ports + camera, write configs/robot.local.yaml")
    p.add_argument("--camera-ip", help="ESP32-CAM IP if pillbot-cam.local doesn't resolve")
    p.add_argument("--no-probe", action="store_true", help="don't open ports to look for the hub")
    p.add_argument("--no-hub", action="store_true",
                   help="this setup has no ESP32 hub: write hub.enabled false (no clamp, no buttons)")
    p.add_argument("-y", "--yes", action="store_true", help="accept detected values, no questions")
    p.add_argument("--dry-run", action="store_true", help="print the file instead of writing it")
    p.add_argument("--fresh", action="store_true",
                   help="start the local file from scratch (old one backed up) instead of updating it")
    p.add_argument("--no-scan", action="store_true", help="don't scan the network for the camera")
    sub.add_parser("find-camera", help="scan this computer's network for the ESP32-CAM and print its IP")
    sub.add_parser("check", help="installed / connected / calibrated / taught")
    p = sub.add_parser("export", help="pack local-only files + LeRobot calibration")
    p.add_argument("--out")
    p.add_argument("--with-secrets", action="store_true", help="include the camera Wi-Fi secrets.h")
    p = sub.add_parser("import", help="unpack a bundle from export")
    p.add_argument("bundle")
    p.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)
    return {"init": cmd_init, "check": cmd_check, "find-camera": cmd_find_camera, "export": cmd_export, "import": cmd_import}[args.cmd](args) or 0


if __name__ == "__main__":
    sys.exit(main())
