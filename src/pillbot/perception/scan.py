"""Looking with a camera that rides on the arm.

The wrist camera only gives a repeatable view if the arm is in the same place every time, so all
"look at the start zone" steps (zone calibration, background capture, finding the bottle) happen
from the taught `scan` pose.
"""
import logging
import time

log = logging.getLogger(__name__)


def camera_on_arm(cfg: dict) -> bool:
    return bool((cfg.get("perception") or {}).get("camera_on_arm"))


def goto_scan(io, poses: dict, cfg: dict, dry_run: bool = False):
    """Move to the scan pose (jaws open, out of the camera's view) and let it settle."""
    gripper_open = cfg["gripper"]["open"]
    io.move_to(poses["scan"] | {"gripper": gripper_open})
    if not dry_run:
        time.sleep(float((cfg.get("perception") or {}).get("scan_settle_s", 0.8)))


def flush(cap, seconds: float = 0.5):
    """Throw away buffered frames so the next read shows the arm where it is NOW
    (network streams can lag a few hundred ms)."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        cap.grab()


def connect_arm_at_scan(cfg: dict):
    """For the perception tools: connect the arm, go to `scan`, hold there. Returns the RobotIO
    (call .disconnect() when done)."""
    from pillbot.control.poses import load_poses
    from pillbot.control.robot_io import RobotIO
    poses = load_poses()
    if not any(poses.get("scan", {}).values()):
        raise SystemExit("`scan` pose not taught yet: python -m pillbot.control.teach --name scan")
    io = RobotIO.from_config(cfg)
    io.connect()
    goto_scan(io, poses, cfg)
    log.info("arm holding at scan pose")
    return io
