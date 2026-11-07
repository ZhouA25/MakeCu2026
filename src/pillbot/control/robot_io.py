"""Thin wrapper around a LeRobot follower arm (LeRobot 0.7.x): Dynamixel Koch-style (default) or SO-101.

Keeps a full joint *target* and always sends all joints, so commanding the arm never
loosens a grip (the gripper target stays where we squeezed it, even if the servo stalled).

Joint keys: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
Arm joints are in degrees; gripper is 0 (closed) .. 100 (open).
"""
import logging
import time

log = logging.getLogger(__name__)

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ARM_JOINTS = JOINTS[:-1]


class Aborted(RuntimeError):
    """STOP was pressed (or should_stop() returned True) during a motion."""


def make_robot(fcfg: dict):
    """Build any LeRobot follower from a config dict like configs/robot.yaml `follower:`."""
    from lerobot.robots.config import RobotConfig
    from lerobot.robots.utils import make_robot_from_config
    if fcfg["type"].startswith("pillbot_"):
        import lerobot_robot_pillbot  # noqa: F401  (registers pillbot_koch_follower)
    else:
        import lerobot.robots  # noqa: F401
    cls = RobotConfig.get_choice_class(fcfg["type"])
    return make_robot_from_config(cls(**{k: v for k, v in fcfg.items() if k != "type"}))


class RobotIO:
    def __init__(self, port: str, robot_id: str, dry_run: bool = False,
                 max_step: float = 3.0, hz: float = 30.0, follower_cfg: dict | None = None):
        self.dry_run = dry_run
        self.joints = list((follower_cfg or {}).get("joints") or JOINTS)
        self.max_step = max_step
        self.hz = hz
        self.robot = None
        self._target = {j: 0.0 for j in self.joints}
        self._sim = {j: 0.0 for j in self.joints}
        # Dry-run only: pretend an object stops the gripper at this value (None = nothing in hand)
        self.sim_gripper_block: float | None = None
        # Optional callable checked every control tick, e.g. lambda: hub.state.pressed(BTN_STOP)
        self.should_stop = None
        if not dry_run:
            self.robot = make_robot(follower_cfg or {"type": "so101_follower", "port": port, "id": robot_id})

    @classmethod
    def from_config(cls, cfg: dict, dry_run: bool = False) -> "RobotIO":
        return cls(cfg["follower"]["port"], cfg["follower"]["id"], dry_run=dry_run,
                   max_step=cfg["safety"]["max_joint_step_deg"], hz=cfg["safety"]["control_hz"],
                   follower_cfg=cfg["follower"])

    # --- connection -------------------------------------------------------------
    def connect(self):
        if self.robot:
            self.robot.connect()
        self._target = self.read()

    def disconnect(self):
        if self.robot and self.robot.is_connected:
            self.robot.disconnect()

    def torque_off(self):
        if self.robot:
            self.robot.bus.disable_torque()

    # --- state ------------------------------------------------------------------
    def read(self) -> dict:
        if self.dry_run:
            return dict(self._sim)
        obs = self.robot.get_observation()
        return {j: float(obs[f"{j}.pos"]) for j in self.joints}

    def read_load(self) -> dict | None:
        """Per-joint motor current/load (Dynamixel: Present_Current, Feetech: Present_Load).
        On the Dynamixel gripper this is effectively grip force — a free tactile signal."""
        if self.dry_run:
            return None
        for reg in ("Present_Current", "Present_Load"):
            try:
                return {k: float(v) for k, v in self.robot.bus.sync_read(reg).items()}
            except Exception:
                continue
        return None

    @property
    def target(self) -> dict:
        return dict(self._target)

    # --- motion -----------------------------------------------------------------
    def _send_target(self):
        if self.dry_run:
            self._sim.update(self._target)
            if self.sim_gripper_block is not None:
                self._sim["gripper"] = max(self._sim["gripper"], self.sim_gripper_block)
        else:
            self.robot.send_action({f"{j}.pos": v for j, v in self._target.items()})

    def step_toward(self, goal: dict) -> bool:
        """Move the target at most max_step toward goal. Returns True if target == goal."""
        if self.should_stop is not None and self.should_stop():
            raise Aborted("STOP pressed")
        done = True
        for j, v in goal.items():
            delta = v - self._target[j]
            if abs(delta) > self.max_step:
                delta = self.max_step if delta > 0 else -self.max_step
                done = False
            self._target[j] += delta
        self._send_target()
        return done

    def move_to(self, goal: dict, tol: float = 1.5, timeout: float = 6.0,
                gripper_settle_s: float = 0.4) -> bool:
        """Interpolate to `goal` (partial dict ok). Waits for arm joints to arrive within tol deg.

        The gripper is never waited on (it legitimately stalls on objects); we just settle.
        """
        goal = {j: float(v) for j, v in goal.items() if j in self.joints}   # unknown joints ignored
        t0 = time.time()
        while not self.step_toward(goal):
            self._sleep()
            if time.time() - t0 > timeout:
                log.warning("move_to: timed out ramping toward %s", goal)
                return False
        arm_goal = {j: v for j, v in goal.items() if j != "gripper"}
        while arm_goal:
            cur = self.read()
            if all(abs(cur[j] - v) < tol for j, v in arm_goal.items()):
                break
            if time.time() - t0 > timeout:
                log.warning("move_to: arm did not reach goal (err %s)",
                            {j: round(cur[j] - v, 1) for j, v in arm_goal.items()})
                return False
            self._send_target()
            self._sleep()
        if "gripper" in goal and not self.dry_run:
            time.sleep(gripper_settle_s)
        return True

    def set_gripper(self, value: float, settle_s: float = 0.4):
        self.move_to({"gripper": value}, gripper_settle_s=settle_s)

    def _sleep(self):
        if not self.dry_run:
            time.sleep(1.0 / self.hz)

