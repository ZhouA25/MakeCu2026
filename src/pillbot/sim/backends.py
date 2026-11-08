"""Plug the simulation into the REAL control code.

SimRobotIO looks like control/robot_io.RobotIO and SimTactile like a SensX25, so grasp.py,
open_sequence.py and pick_place.py run unchanged against physics instead of hardware — the code
you test here is the code that runs on the prototype.

Time is simulated: commanding a move advances the physics instead of sleeping.
"""
import numpy as np

from pillbot.control.robot_io import Aborted, RobotIO
from pillbot.sensors.tactile import TactileSensor
from pillbot.sim.world import ARM, SimWorld


class SimRobotIO(RobotIO):
    """RobotIO whose servos are SimWorld joints. Angles are the real arm's LeRobot degrees,
    converted through configs/sim.yaml `joint_map`."""

    def __init__(self, world: SimWorld, cfg: dict | None = None):
        cfg = cfg or world.robot_cfg
        super().__init__("", "sim", dry_run=True, max_step=cfg["safety"]["max_joint_step_deg"],
                         hz=cfg["safety"]["control_hz"], follower_cfg=cfg["follower"],
                         pwm_joints=cfg.get("pwm_joints"))
        self.world = world
        self.jmap = world.sim.get("joint_map", {})

    # real <-> sim angle conventions
    def to_sim(self, real: dict) -> dict:
        out = {}
        for j, v in real.items():
            mp = self.jmap.get(j)
            out[j] = v if mp is None else (v - mp["offset_deg"]) / mp["sign"]
        return out

    def to_real(self, sim: dict) -> dict:
        out = {}
        for j, v in sim.items():
            mp = self.jmap.get(j)
            out[j] = v if mp is None else mp["sign"] * v + mp["offset_deg"]
        return out

    def connect(self):
        self._target = self.read()

    def read(self) -> dict:
        r = self.to_real(self.world.read())
        return {j: r[j] for j in self.joints if j in r}

    def read_load(self) -> dict:
        t = self.world.joint_torques()
        return {j: t.get(j, 0.0) for j in self.joints}

    def _send_target(self):
        self.world.set_targets(self.to_sim(self._target))

    def _sleep(self):
        self.world.tick()

    def move_to(self, goal: dict, tol: float = 1.5, timeout: float = 6.0,
                gripper_settle_s: float = 0.4) -> bool:
        """Same behaviour as RobotIO.move_to, but `timeout` counts SIMULATED seconds."""
        goal = {j: float(v) for j, v in goal.items() if j in self.joints}
        t0 = self.world.time
        while not self.step_toward(goal):
            self._sleep()
            if self.world.time - t0 > timeout:
                return False
        arm_goal = {j: v for j, v in goal.items() if j != "gripper"}
        while arm_goal:
            cur = self.read()
            if all(abs(cur[j] - v) < tol for j, v in arm_goal.items()):
                break
            if self.world.time - t0 > timeout:
                return False
            if self.should_stop is not None and self.should_stop():
                raise Aborted("STOP pressed")
            self._send_target()
            self._sleep()
        if "gripper" in goal:
            self.world.step_seconds(gripper_settle_s)
        return True


class SimTactile(TactileSensor):
    """The SensX25 on the sim's moving jaw: same 5x5 kPa frames, same tare/contact/slip logic."""

    def __init__(self, world: SimWorld, noise_kpa: float = 1.5, **kw):
        super().__init__(**kw)
        self.world, self.noise = world, noise_kpa
        self.shape = (5, 5)

    def read(self) -> np.ndarray:
        f = self.world.tactile_kpa()
        if self.noise:
            f = np.clip(f + self.world.rng.normal(0, self.noise, f.shape), 0, 534)
        return f


def make_sim(bottle: str = "standard_amber_40dr", seed: int | None = None, sim_cfg: dict | None = None,
             robot_cfg: dict | None = None, **reset_kw):
    """(world, io, tactile) wired like run.py wires the real robot. `sim_cfg` / `robot_cfg` replace
    configs/sim.yaml / robot.yaml (e.g. to try a design change)."""
    world = SimWorld(bottle, sim_cfg=sim_cfg, robot_cfg=robot_cfg, seed=seed)
    if reset_kw:
        world.reset(**reset_kw)
    t = world.robot_cfg.get("tactile", {}) or {}
    tactile = SimTactile(world, contact_threshold=t.get("contact_threshold", 50),
                         slip_ratio=t.get("slip_ratio", 0.5))
    io = SimRobotIO(world)
    io.connect()
    return world, io, tactile




class SimHub:
    """The ESP32 PillBot Hub in the sim: control/fixture.Fixture drives the sim's clip clamp through it,
    exactly like it drives the goBILDA servo through the real hub (servo_us on the clamp channel)."""

    def __init__(self, world: SimWorld, fcfg: dict | None = None):
        self.world = world
        self.fcfg = fcfg or world.robot_cfg["fixture"]
        self.servos, self.relay_on, self.log = {}, False, []

    @property
    def state(self):
        """Live readings: the clip ultrasonic sees the bottle once it's seated."""
        from pillbot.sensors.hub import HubState
        b, c = self.world.bottle_pos, self.world.clip_xy
        seated = np.hypot(*(b[:2] - c)) < 0.006 and abs(b[2] - self.world.sim["fixture"]["clip_floor"]) < 0.004
        return HubState(t_ms=int(self.world.time * 1000), clip_mm=40 if seated else 200, zone_mm=200)

    def connect(self, timeout: float = 0):
        pass

    def servo_us(self, ch: int, us: int):
        self.servos[ch] = int(us)
        self.log.append(("SV", ch, int(us)))
        if ch == self.fcfg["clamp_channel"] and us:
            lo, hi = self.fcfg["clamp_open_us"], self.fcfg["clamp_closed_us"]
            self.world.set_clamp(float(np.clip((us - lo) / (hi - lo), 0, 1)))
            self.world.step_seconds(self.fcfg.get("clamp_settle_s", 0.6))   # the servo moving

    def relay(self, on: bool):
        self.relay_on = bool(on)
        self.log.append(("RL", bool(on)))

    def close(self):
        self.relay_on = False


__all__ = ["ARM", "SimHub", "SimRobotIO", "SimTactile", "make_sim"]
