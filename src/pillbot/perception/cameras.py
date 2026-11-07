"""Open the fixed cameras (phone + laptop) with OpenCV for scripted perception.

LeRobot opens cameras itself during record/rollout; this is only for our own scripts.
Don't run both at once — a camera can usually only be opened by one process.
"""
import cv2


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


def perception_camera(cfg: dict) -> cv2.VideoCapture:
    return open_camera(cfg["cameras"][cfg.get("perception_camera", "phone")])
