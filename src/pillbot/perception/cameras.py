"""Open the fixed cameras (phone + laptop) with OpenCV for scripted perception.

LeRobot opens cameras itself during record/rollout; this is only for our own scripts.
Don't run both at once — a camera can usually only be opened by one process.
"""
import cv2


def open_camera(cam_cfg: dict) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(cam_cfg["index_or_path"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg.get("width", 640))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg.get("height", 480))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {cam_cfg['index_or_path']} "
                           "(check `lerobot-find-cameras opencv` and camera permissions)")
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
