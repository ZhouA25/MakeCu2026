"""Thin wrapper around a LeRobot follower arm (LeRobot 0.7.x): Dynamixel Koch-style (default) or SO-101.

Keeps a full joint *target* and always sends all joints, so commanding the arm never
loosens a grip (the gripper target stays where we squeezed it, even if the servo stalled).

Joint keys: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
Arm joints are in degrees; gripper is 0 (closed) .. 100 (open).

PWM joints (configs/robot.yaml `pwm_joints:`): a joint driven by a hobby servo on the ESP32 hub
instead of a Dynamixel (our elbow is a goBILDA 2000: the XL330 was too weak). Same degrees as the
rest; it's open-loop (no position feedback), so read() reports the commanded angle for it. The
first command after start-up snaps it to that angle: power up with the arm in its `rest` pose.
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
                 max_step: float = 3.0, hz: float = 30.0, follower_cfg: dict | None = None,
                 pwm_joints: dict | None = None, hub=None, start_pose: dict | None = None):
        self.dry_run = dry_run
        self.bus_joints = list((follower_cfg or {}).get("joints") or JOINTS)
        self.pwm = dict(pwm_joints or {})
        self.joints = self.bus_joints + [j for j in self.pwm if j not in self.bus_joints]
        self.hub = hub
        self._own_hub = False
        self._pwm_sent: dict[str, int] = {}
        self._start = {j: float(v) for j, v in (start_pose or {}).items() if j in self.pwm}
        self.max_step = max_step
        self.hz = hz
        self.robot = None
        self._target = {j: self._start.get(j, 0.0) for j in self.joints}
        self._sim = dict(self._target)
        # Dry-run only: pretend an object stops the gripper at this value (None = nothing in hand)
        self.sim_gripper_block: float | None = None
        # Optional callable checked every control tick, e.g. lambda: hub.state.pressed(BTN_STOP)
        self.should_stop = None
        self.limp: set[str] = set()              # joints currently torque-off (see torque_off/torque_on)
        if not dry_run:
            self.robot = make_robot(follower_cfg or {"type": "so101_follower", "port": port, "id": robot_id})

    @classmethod
    def from_config(cls, cfg: dict, dry_run: bool = False, hub=None) -> "RobotIO":
        """`hub`: pass the one you already opened (run.py); otherwise, if the arm has PWM joints,
        RobotIO opens the hub itself (only one program can hold its serial port)."""
        start = None
        if cfg.get("pwm_joints"):
            from pillbot.control.poses import load_poses
            from pillbot.control.idle import idle_cfg, rest_pose
            try:
                start = rest_pose(load_poses(), idle_cfg(cfg)["pose"])
            except (FileNotFoundError, KeyError):
                start = None
        io = cls(cfg["follower"]["port"], cfg["follower"]["id"], dry_run=dry_run,
                 max_step=cfg["safety"]["max_joint_step_deg"], hz=cfg["safety"]["control_hz"],
                 follower_cfg=cfg["follower"], pwm_joints=cfg.get("pwm_joints"), hub=hub, start_pose=start)
        if io.pwm and io.hub is None and not dry_run:
            from pillbot.sensors.hub import make_hub
            io.hub, io._own_hub = make_hub(cfg, dry_run=False), True
        return io

    # --- connection -------------------------------------------------------------
    def connect(self):
        if self._own_hub:
            self.hub.connect()
        if self.robot:
            self.robot.connect()
        self._target = self.read()
        if self.pwm:
            self._send_pwm()          # PWM joints start at the rest pose's angle (or 0)

    def disconnect(self):
        if self.robot and self.robot.is_connected:
            self.robot.disconnect()
        if self._own_hub:
            self.hub.close()

    # --- PWM joints (hobby servos on the hub) -----------------------------------
    def pwm_us(self, joint: str, deg: float) -> int:
        c = self.pwm[joint]
        us = c.get("center_us", 1500) + c.get("sign", 1) * c.get("us_per_deg", 6.67) * deg
        return int(round(min(max(us, c.get("min_us", 600)), c.get("max_us", 2400))))

    def _send_pwm(self):
        for j in self.pwm:
            if j in self.limp:
                continue
            us = self.pwm_us(j, self._target[j])
            if self.hub is not None and self._pwm_sent.get(j) != us:
                self.hub.servo_us(int(self.pwm[j]["channel"]), us)
            self._pwm_sent[j] = us

    def torque_off(self, joints: list[str] | None = None):
        """Let joints go limp (all if None): no holding current. Joints under gravity load will sag.
        A PWM joint goes limp too (pulse off) — it can't be read, so it comes back at its last angle."""
        joints = [j for j in (joints or self.joints) if j in self.joints]
        bus = [j for j in joints if j in self.bus_joints]
        if self.robot and bus:
            self.robot.bus.disable_torque(bus)
        for j in joints:
            if j in self.pwm and self.hub is not None:
                self.hub.servo_us(int(self.pwm[j]["channel"]), 0)
                self._pwm_sent.pop(j, None)
        self.limp |= set(joints)

    def torque_on(self, joints=None):
        """Re-energise limp joints (all limp ones if None) WHERE THEY ARE NOW: the goal is set to the
        present position first, so nothing jumps back to an old target when torque comes on."""
        wake = self.limp if joints is None else self.limp & set(joints)
        if not wake:
            return
        cur = self.read()
        for j in wake:
            if j in self.bus_joints:
                self._target[j] = cur[j]
        self.limp -= wake
        self._send_target()                      # goal = present (writing goals works with torque off)
        if self.robot and (bus := sorted(j for j in wake if j in self.bus_joints)):
            self.robot.bus.enable_torque(bus)

    # --- state ------------------------------------------------------------------
    def read(self) -> dict:
        if self.dry_run:
            return dict(self._sim)
        obs = self.robot.get_observation()
        out = {j: float(obs[f"{j}.pos"]) for j in self.bus_joints}
        for j in self.pwm:                         # open loop: what we last commanded
            out[j] = self._target.get(j, self._start.get(j, 0.0))
        return out

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
            self.robot.send_action({f"{j}.pos": v for j, v in self._target.items() if j in self.bus_joints})
        if self.pwm:
            self._send_pwm()

    def step_toward(self, goal: dict) -> bool:
        """Move the target at most max_step toward goal. Returns True if target == goal."""
        if self.limp & goal.keys():
            self.torque_on(goal.keys())          # moving a limp joint: wake it first
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

