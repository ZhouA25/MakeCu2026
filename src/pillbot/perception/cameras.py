"""Open cameras with OpenCV for scripted perception: the ESP32-CAM, a webcam / USB camera, or a URL.

Choose with --camera on a tool, PILLBOT_CAMERA, or perception_camera (see resolve_camera).
List / preview:  python -m pillbot.perception.cameras [--show 0]

LeRobot opens cameras itself during record/rollout; this is only for our own scripts.
Don't run both at once — a camera can usually only be opened by one process.
"""
import os

import cv2

ALIASES = {"esp32": "wrist", "esp32cam": "wrist", "esp32-cam": "wrist", "cam": "wrist",
           "webcam": "0", "laptop_cam": "0"}


def camera_choices(cfg: dict) -> list[str]:
    return [n for n, c in (cfg.get("cameras") or {}).items() if c]


def resolve_camera(cfg: dict, spec=None) -> tuple[str, dict]:
    """Pick a camera -> (label, OpenCV camera config). `spec`, else $PILLBOT_CAMERA, else
    `perception_camera`, can be:
      a name from robot.yaml `cameras:` (wrist, phone, ...) or esp32 / webcam (aliases)
      a number: local webcam / USB camera index (0 = laptop's built-in, usually)
      a URL: an MJPEG stream, e.g. http://10.20.70.37:81/stream
    """
    spec = spec if spec not in (None, "") else (os.environ.get("PILLBOT_CAMERA") or cfg.get("perception_camera", "wrist"))
    spec = str(spec).strip()
    spec = ALIASES.get(spec.lower(), spec)
    cams = cfg.get("cameras") or {}
    if spec in cams:
        if not cams[spec]:
            raise ValueError(f"camera '{spec}' is disabled (null) in your config; choose one of "
                             f"{camera_choices(cfg)}, a number (webcam) or a stream URL")
        return spec, cams[spec]
    if spec.isdigit():
        return f"webcam {spec}", {"type": "opencv", "index_or_path": int(spec), "width": 640, "height": 480}
    if spec.startswith(("http://", "https://", "rtsp://")):
        return spec, {"type": "pillbot_ipcam", "url": spec}
    raise ValueError(f"unknown camera '{spec}': use one of {camera_choices(cfg)}, esp32, "
                     "a number (0 = laptop webcam) or a stream URL")


def camera_source(cam_cfg: dict):
    """Device index / path for local cameras, URL for network ones (ESP32-CAM)."""
    if cam_cfg.get("type") == "pillbot_ipcam":
        return cam_cfg["url"]
    return cam_cfg["index_or_path"]


def open_camera(cam_cfg: dict) -> cv2.VideoCapture:
    src = camera_source(cam_cfg)
    cap = cv2.VideoCapture(src)
    if not isinstance(src, str) or not src.startswith("http"):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg.get("width", 640))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg.get("height", 480))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {src} "
                           "(check `lerobot-find-cameras opencv`, camera permissions, or the ESP32-CAM URL)")
    for _ in range(5):          # let auto-exposure settle
        cap.read()
    return cap


def grab(cap: cv2.VideoCapture):
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("camera read failed")
    return frame


def perception_camera(cfg: dict, spec=None) -> cv2.VideoCapture:
    return open_camera(resolve_camera(cfg, spec)[1])


def probe_local(max_index: int = 5) -> list[int]:
    """Webcam / USB camera indices that open and return a frame."""
    found = []
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY   # DirectShow: fast to probe on Windows
    for i in range(max_index):
        cap = cv2.VideoCapture(i, backend)
        ok = cap.isOpened() and cap.read()[0]
        cap.release()
        if ok:
            found.append(i)
    return found


def main(argv=None):
    """python -m pillbot.perception.cameras            list cameras you can choose
       python -m pillbot.perception.cameras --show X   preview one (q quits)"""
    import argparse
    from pillbot.utils.config import load_yaml
    ap = argparse.ArgumentParser(description=main.__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--show", metavar="CAMERA", help="name, number or URL to preview")
    ap.add_argument("--config", default="configs/robot.yaml")
    args = ap.parse_args(argv)
    cfg = load_yaml(args.config)
    if args.show is not None:
        label, cam_cfg = resolve_camera(cfg, args.show)
        cap = open_camera(cam_cfg)
        print(f"showing {label} — q to quit")
        while True:
            ok, frame = cap.read()
            if ok:
                cv2.imshow(f"camera: {label}", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()
        return 0
    current = os.environ.get("PILLBOT_CAMERA") or cfg.get("perception_camera", "wrist")
    print(f"In use now: {current}   (change: --camera X on a tool, PILLBOT_CAMERA=X, or perception_camera: X)")
    print("Configured:")
    for n in camera_choices(cfg):
        c = cfg["cameras"][n]
        print(f"  {n:10s} {c.get('url') or c.get('index_or_path')}" + ("   (= esp32)" if n == "wrist" else ""))
    print("Local webcams / USB cameras (by number):")
    found = probe_local()
    print("  " + (", ".join(map(str, found)) if found else "none found"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
