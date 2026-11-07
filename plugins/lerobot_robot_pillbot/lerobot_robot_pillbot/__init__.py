"""LeRobot plugin: Koch v1.1-geometry Dynamixel arms with whatever servos you actually have.

  robot type   pillbot_koch_follower   presets: koch     = 2x XL430-W250 + 4x XL330-M288 (stock Koch)
                                                all_m288 = 6x XL330-M288 (no XL430s needed)
                                                custom   = give `models: [...]` yourself
  teleop type  pillbot_koch_leader     presets: m077 = 6x XL330-M077 (stock, easy to back-drive)
                                                m288 = 6x XL330-M288 (works, but stiffer by hand)

Both accept `joints: [...]` (motor IDs 1..N in that order, last must be `gripper`) so you can build
a 5-servo arm (e.g. no wrist_flex on danslovich/5DOF_Robot_Arm), and `use_degrees`
(follower and leader MUST match). The follower also takes `gripper_goal_current` (mA) — the gripper
runs in current-limited mode, so this sets how hard it squeezes (it twists caps off by friction).

  camera type  pillbot_ipcam           network MJPEG camera (ESP32-CAM): `url: http://<ip>:81/stream`.
                                       Stock `opencv` can't take a URL (it turns it into a Path) and
                                       tries to set width/fps, which a stream refuses.

Install:  pip install -e plugins/lerobot_robot_pillbot      (LeRobot CLIs auto-discover it)
"""
from dataclasses import dataclass, field

from pathlib import Path

import cv2
from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.motors_bus import SerialMotorsBus
from lerobot.robots.config import RobotConfig
from lerobot.robots.koch_follower import KochFollower, KochFollowerConfig
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.koch_leader import KochLeader, KochLeaderConfig

DEFAULT_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

FOLLOWER_PRESETS = {
    "koch": ["xl430-w250", "xl430-w250", "xl330-m288", "xl330-m288", "xl330-m288", "xl330-m288"],
    "all_m288": ["xl330-m288"] * 6,
}
LEADER_PRESETS = {"m077": ["xl330-m077"] * 6, "m288": ["xl330-m288"] * 6}


def resolve_models(preset: str, models: list[str] | None, joints: list[str], presets: dict) -> list[str]:
    if models:
        chosen = list(models)
    elif preset in presets:
        full = presets[preset]
        # dropping joints keeps each remaining joint's preset model (gripper = last)
        idx = {j: i for i, j in enumerate(DEFAULT_JOINTS)}
        chosen = [full[idx[j]] if j in idx else full[-2] for j in joints]
    else:
        raise ValueError(f"unknown preset '{preset}' (choose {list(presets)} or give models)")
    if len(chosen) != len(joints):
        raise ValueError(f"{len(joints)} joints but {len(chosen)} motor models")
    if joints[-1] != "gripper":
        raise ValueError("last joint must be 'gripper'")
    return chosen


def build_motors(joints, models, use_degrees: bool) -> dict[str, Motor]:
    body = MotorNormMode.DEGREES if use_degrees else MotorNormMode.RANGE_M100_100
    return {j: Motor(i + 1, m, MotorNormMode.RANGE_0_100 if j == "gripper" else body)
            for i, (j, m) in enumerate(zip(joints, models))}


# ---------------------------------------------------------------- follower
@RobotConfig.register_subclass("pillbot_koch_follower")
@dataclass
class PillbotKochFollowerConfig(KochFollowerConfig):
    preset: str = "koch"
    models: list[str] | None = None
    joints: list[str] = field(default_factory=lambda: list(DEFAULT_JOINTS))
    gripper_goal_current: int | None = None   # mA; None = motor default


class PillbotKochFollower(KochFollower):
    config_class = PillbotKochFollowerConfig
    name = "pillbot_koch_follower"

    def __init__(self, config: PillbotKochFollowerConfig):
        super().__init__(config)
        models = resolve_models(config.preset, config.models, config.joints, FOLLOWER_PRESETS)
        self.bus = SerialMotorsBus(port=config.port, calibration=self.calibration,
                                   motors=build_motors(config.joints, models, config.use_degrees))

    def calibrate(self) -> None:
        """Koch calibration, made safe for geared joints and reduced joint sets.

        With a 4:1 planetary on the shoulder, the motor turns >1 rev across the joint's range, so the
        recorded raw min/max can fall outside 0..4095 — and Dynamixel rejects Position_Limit writes
        outside that. In extended-position mode the limits are ignored anyway, and with
        use_degrees=True only the range midpoint matters, so we clamp them into the register range.
        """
        if self._keep_calibration_file(self.bus):
            return
        full_turn = [m for m in ("shoulder_pan", "wrist_roll") if m in self.bus.motors]
        cal = self.bus.record_calibration("extended_position", full_turn_motors=full_turn)
        for name, c in cal.items():
            res = self.bus.resolution(self.bus.motors[name].model) - 1
            lo, hi = max(0, min(c.range_min, res)), max(0, min(c.range_max, res))
            if hi <= lo:                      # whole recorded range was off one end
                lo, hi = 0, res
            c.range_min, c.range_max = lo, hi
        self.calibration = cal
        self.bus.write_calibration(cal)
        self._save_calibration()

    def configure(self) -> None:
        # Koch behaviour: extended-position joints + current-limited gripper (can't crush the bottle)
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                if motor != "gripper":
                    self.bus.set_operating_mode("extended_position", motor)
            self.bus.set_operating_mode("current_based_position", "gripper")
            if "elbow_flex" in self.bus.motors:
                self.bus.write("Position_P_Gain", "elbow_flex", 1500)
                self.bus.write("Position_I_Gain", "elbow_flex", 0)
                self.bus.write("Position_D_Gain", "elbow_flex", 600)
            if self.config.gripper_goal_current is not None:
                # caps come off by friction: grip current sets the squeeze (and the max twist before slip)
                self.bus.write("Goal_Current", "gripper", int(self.config.gripper_goal_current))


# ---------------------------------------------------------------- leader
@TeleoperatorConfig.register_subclass("pillbot_koch_leader")
@dataclass
class PillbotKochLeaderConfig(KochLeaderConfig):
    preset: str = "m077"
    models: list[str] | None = None
    joints: list[str] = field(default_factory=lambda: list(DEFAULT_JOINTS))
    use_degrees: bool = False


class PillbotKochLeader(KochLeader):
    config_class = PillbotKochLeaderConfig
    name = "pillbot_koch_leader"

    def __init__(self, config: PillbotKochLeaderConfig):
        super().__init__(config)
        models = resolve_models(config.preset, config.models, config.joints, LEADER_PRESETS)
        self.bus = SerialMotorsBus(port=config.port, calibration=self.calibration,
                                   motors=build_motors(config.joints, models, config.use_degrees))


# ---------------------------------------------------------------- network camera
@CameraConfig.register_subclass("pillbot_ipcam")
@dataclass
class PillbotIPCameraConfig(OpenCVCameraConfig):
    url: str = ""
    index_or_path: int | Path = 0     # unused (url wins); kept so the parent dataclass is happy


class PillbotIPCamera(OpenCVCamera):
    """OpenCV camera that opens an MJPEG URL and accepts the stream's own size/fps."""

    def __init__(self, config: PillbotIPCameraConfig):
        if not config.url:
            raise ValueError("pillbot_ipcam needs `url`, e.g. http://192.168.1.50:81/stream")
        super().__init__(config)
        self.index_or_path = config.url          # a str is passed to cv2.VideoCapture untouched

    def __str__(self) -> str:
        return f"PillbotIPCamera({self.config.url})"

    def _configure_capture_settings(self) -> None:
        w = int(round(self.videocapture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        h = int(round(self.videocapture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if self.width is None or self.height is None:
            self.width, self.height = w, h
        elif w and h and (w, h) != (self.capture_width, self.capture_height):
            raise RuntimeError(f"{self}: stream is {w}x{h} but config says "
                               f"{self.capture_width}x{self.capture_height} — change FRAME in the "
                               "ESP32-CAM sketch or width/height in robot.yaml")
        self.capture_width, self.capture_height = w or self.capture_width, h or self.capture_height
        if self.fps is None:
            self.fps = self.videocapture.get(cv2.CAP_PROP_FPS) or 15
        # fps is not enforced: a Wi-Fi stream delivers what it can; LeRobot reuses the latest frame


__all__ = ["PillbotIPCamera", "PillbotIPCameraConfig", "PillbotKochFollower", "PillbotKochFollowerConfig", "PillbotKochLeader",
           "PillbotKochLeaderConfig", "resolve_models", "FOLLOWER_PRESETS", "LEADER_PRESETS"]
