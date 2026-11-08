"""SimWorld — the physics simulation everything else drives (MuJoCo).

Angles here are SIM degrees (configs/sim.yaml convention); SimRobotIO converts to the real arm's
LeRobot degrees. The gripper is in LeRobot units: 0 = closed, 100 = open.

The cap is a little machine, simulated on top of the physics every few milliseconds:
  * thread    turning it needs `thread_torque` N·m (randomized per episode)
  * child-proof (cap_type push_and_turn): it won't turn at all until pushed down `push_unlock_mm`
  * free      after `twist_deg` of loosening it can be lifted off
"""
import math

import mujoco
import numpy as np

from pillbot.sim.scene import CLAMP_OPEN, TAXELS, build_xml
from pillbot.utils.config import get_bottle, load_yaml

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_roll")
REACH = ARM[:3]                      # the joints that move the grasp point (wrist_roll only spins it)
NECK_CLEAR = 0.008                   # m a loose cap must be lifted before it can move sideways
CAP_LOGIC_EVERY = 4                 # physics steps between cap-mechanism updates (8 ms)


class SimWorld:
    def __init__(self, bottle: str | dict = "standard_amber_40dr", sim_cfg: dict | None = None,
                 robot_cfg: dict | None = None, seed: int | None = None):
        self.sim = sim_cfg or load_yaml("configs/sim.yaml")
        self.robot_cfg = robot_cfg or load_yaml("configs/robot.yaml")
        self.bottle = get_bottle(bottle) if isinstance(bottle, str) else bottle
        current = self.robot_cfg["follower"].get("gripper_goal_current", 1000)
        self.model = mujoco.MjModel.from_xml_string(build_xml(self.sim, self.bottle, current))
        self.data = mujoco.MjData(self.model)
        self._scratch = mujoco.MjData(self.model)      # for FK/IK without disturbing the sim
        self.rng = np.random.default_rng(seed)
        self.hz = self.robot_cfg["safety"]["control_hz"]
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}
        self.on_tick = None                                # optional fn(world) after every control tick
        self.on_step = None                                # observation/recording, including settling

        m = self.model
        jid = {n: m.joint(n).id for n in (*ARM, "gripper", "clamp_l", "clamp_r", "cap_x", "cap_y", "cap_rot", "cap_lift", "cap_press")}
        self.qadr = {n: int(m.jnt_qposadr[i]) for n, i in jid.items()}
        self.jnt_lift = jid["cap_lift"]
        self.dadr = {n: int(m.jnt_dofadr[i]) for n, i in jid.items()}
        self.act = {n: m.actuator(n).id for n in (*ARM, "gripper")}
        self.clamp_act = [m.actuator("clamp_l").id, m.actuator("clamp_r").id]
        self.clip_body = m.body("clip").id
        self.free_q = int(m.jnt_qposadr[m.joint("bottle_free").id])
        self.eq_rot = m.equality("cap_rot_lock").id
        self.eq_thread = [m.equality(f"cap_{a}_lock").id for a in ("lift", "x", "y")]
        self.grasp_site = m.site("grasp").id
        self.bottle_body = m.body("bottle").id
        self.jaw_lo, self.jaw_hi = (math.radians(v) for v in self.sim["arm"]["jaw_range_deg"])
        self.taxel_area = self.sim["tactile"]["taxel_pitch"] ** 2
        self.taxel_sensors = [m.sensor(f"taxel_{r}_{c}").adr[0] for r in range(TAXELS) for c in range(TAXELS)]
        self.pad_geoms = [m.geom(f"taxel_{r}_{c}").id for r in range(TAXELS) for c in range(TAXELS)]
        self.pad_geoms.append(m.geom("fixed_pad").id)
        self.bottle_geoms = [m.geom(n).id for n in ("bottle_body", "cap", "cap_mark")]
        self._rgba = m.geom_rgba.copy()
        self.reset()

    # ------------------------------------------------------------------ episode setup
    def reset(self, clip_xy=None, randomize: bool = True, home: dict | None = None,
              start: str = "clip", pick_xy=None):
        """New episode: arm at `home` (sim deg), clip at `clip_xy`, fresh cap randomization.
        start="clip": the bottle is already clamped in the clip. start="zone": it stands on the table at
        `pick_xy` (default fixture.pick_xy) with the clamp open, waiting to be picked up."""
        m, d = self.model, self.data
        fx = self.sim["fixture"]
        mujoco.mj_resetData(m, d)
        xy = fx["clip_xy"] if clip_xy is None else clip_xy
        m.body_pos[self.clip_body][:2] = xy
        if start == "zone":
            bx, by = fx.get("pick_xy", (0.055, 0.095)) if pick_xy is None else pick_xy
            d.qpos[self.free_q:self.free_q + 3] = [bx, by, 0.0]
        else:
            d.qpos[self.free_q:self.free_q + 3] = [xy[0], xy[1], fx["clip_floor"]]
        d.qpos[self.free_q + 3:self.free_q + 7] = [1, 0, 0, 0]
        c = self.sim["cap"]
        pick = (lambda lo_hi: self.rng.uniform(*lo_hi)) if randomize else (lambda lo_hi: float(np.mean(lo_hi)))
        self.thread_torque = pick(c["thread_torque"])
        self.unlock_mm = pick(c["push_unlock_mm"])
        mu = pick(c["friction"])
        for g in self.pad_geoms:
            m.geom_friction[g][0] = mu
        self.child_proof = self.bottle["cap_type"] == "push_and_turn"
        self.twist_rad = math.radians(self.bottle["twist_deg"])
        self.cap_free = False
        self.time = 0.0
        q = {"shoulder_pan": 0, "shoulder_lift": 20, "elbow_flex": 70, "wrist_roll": 0} | (home or {})
        for j in ARM:
            d.qpos[self.qadr[j]] = math.radians(q[j])
        self.set_targets(q | {"gripper": 100})
        d.qpos[self.qadr["gripper"]] = self.jaw_hi
        self.set_clamp(start != "zone")
        if start != "zone":                               # start with the jaws already on the bottle
            for j in ("clamp_l", "clamp_r"):
                d.qpos[self.qadr[j]] = CLAMP_OPEN
        mujoco.mj_forward(m, d)
        self._cap_logic()
        if start != "zone":
            self.step_seconds(0.2)
            self.time = 0.0

    @property
    def clip_xy(self) -> np.ndarray:
        return self.model.body_pos[self.clip_body][:2].copy()

    @property
    def bottle_pos(self) -> np.ndarray:
        """Where the bottle's bottom centre really is (true value, for checking)."""
        return self.data.qpos[self.free_q:self.free_q + 3].copy()

    @property
    def cap_top_z(self) -> float:
        return float(self.bottle_pos[2]) + self.bottle["height_mm"] / 1000

    @property
    def cap_center(self) -> np.ndarray:
        """Where the jaws should close: middle of the cap's side wall (true value, for checking)."""
        x, y, _ = self.bottle_pos
        return np.array([x, y, self.cap_top_z - self.sim["fixture"]["cap_height"] / 2])

    def bottle_upright(self) -> float:
        """Tilt of the bottle from vertical, degrees."""
        _, x, y, _ = self.data.qpos[self.free_q + 3:self.free_q + 7]
        return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))

    def set_clamp(self, closed: bool | float):
        """Clip clamp: True/1 = squeeze the bottle body, False/0 = open (or anything in between)."""
        frac = float(closed)
        for a in self.clamp_act:
            self.data.ctrl[a] = frac * (CLAMP_OPEN + 0.004)

    def show_bottle(self, visible: bool):
        """Hide/show bottle + cap (rendering only) — to capture an 'empty clip' background image."""
        for g in self.bottle_geoms:
            self.model.geom_rgba[g][3] = self._rgba[g][3] if visible else 0.0

    # ------------------------------------------------------------------ stepping
    def _cap_logic(self):
        """Locks are hard equality constraints (friction-based locks creep in MuJoCo)."""
        m, d = self.model, self.data
        angle = d.qpos[self.qadr["cap_rot"]]
        if not self.cap_free and angle >= self.twist_rad:
            self.cap_free = True
        pressed = -d.qpos[self.qadr["cap_press"]] * 1000 >= self.unlock_mm
        locked = self.child_proof and not pressed and not self.cap_free
        if locked and not d.eq_active[self.eq_rot]:
            m.eq_data[self.eq_rot][0] = angle            # freeze it at its current angle
        d.eq_active[self.eq_rot] = locked
        lift_e, x_e, y_e = self.eq_thread
        d.eq_active[lift_e] = not self.cap_free
        # A loose cap still sits around the neck (its skirt stops it sliding sideways) until lifted clear.
        on_neck = d.qpos[self.qadr["cap_lift"]] < NECK_CLEAR
        if (not self.cap_free or on_neck) and not d.eq_active[x_e]:
            m.eq_data[x_e][0] = d.qpos[self.qadr["cap_x"]]
            m.eq_data[y_e][0] = d.qpos[self.qadr["cap_y"]]
        d.eq_active[x_e] = d.eq_active[y_e] = not self.cap_free or on_neck
        # Cap/bottle collisions are off (so the cap can be pushed down), so a loose cap still over the
        # neck needs a floor there; carried away from the bottle it can fall to the table.
        over_neck = math.hypot(d.qpos[self.qadr["cap_x"]], d.qpos[self.qadr["cap_y"]]) < 0.03
        m.jnt_range[self.jnt_lift][0] = 0.0 if over_neck else -0.2
        m.dof_frictionloss[self.dadr["cap_rot"]] = 0.0 if self.cap_free else self.thread_torque

    def step_seconds(self, seconds: float):
        n = max(1, round(seconds / self.model.opt.timestep))
        while n > 0:
            k = min(n, CAP_LOGIC_EVERY)
            self._cap_logic()
            mujoco.mj_step(self.model, self.data, nstep=k)
            n -= k
            self.time += k * self.model.opt.timestep
            if self.on_step is not None:
                mujoco.mj_kinematics(self.model, self.data)
                self.on_step(self)
        mujoco.mj_kinematics(self.model, self.data)      # positions match the new state

    def tick(self):
        """One control period (1 / control_hz), like one command to the real servos."""
        self.step_seconds(1.0 / self.hz)
        if self.on_tick is not None:
            self.on_tick(self)

    # ------------------------------------------------------------------ commands / readings
    def set_targets(self, deg: dict):
        """Servo targets: arm joints in sim degrees, gripper in 0..100 units. Partial dicts are fine."""
        for j, v in deg.items():
            if j == "gripper":
                self.data.ctrl[self.act["gripper"]] = self.units_to_jaw(v)
            elif j in self.act:
                self.data.ctrl[self.act[j]] = math.radians(v)

    def units_to_jaw(self, units: float) -> float:
        """LeRobot gripper units (0 closed .. 100 open) -> SO-101 jaw angle (rad), linear over its range."""
        return self.jaw_lo + float(np.clip(units, 0, 100)) / 100 * (self.jaw_hi - self.jaw_lo)

    def jaw_to_units(self, rad: float) -> float:
        return (rad - self.jaw_lo) / (self.jaw_hi - self.jaw_lo) * 100

    def targets(self) -> dict:
        t = {j: math.degrees(self.data.ctrl[self.act[j]]) for j in ARM}
        t["gripper"] = self.jaw_to_units(self.data.ctrl[self.act["gripper"]])
        return t

    def read(self) -> dict:
        """Joint positions: arm in sim degrees, gripper in 0..100 units."""
        d = self.data
        out = {j: math.degrees(d.qpos[self.qadr[j]]) for j in ARM}
        out["gripper"] = self.jaw_to_units(d.qpos[self.qadr["gripper"]])
        return out

    def joint_torques(self) -> dict:
        return {j: float(self.data.actuator_force[a]) for j, a in self.act.items()}

    def grasp_pos(self) -> np.ndarray:
        return self.data.site_xpos[self.grasp_site].copy()

    def tactile_kpa(self) -> np.ndarray:
        """5x5 frame like the real SensX25: kPa per taxel (normal force / taxel area), 0..534."""
        f = self.data.sensordata[self.taxel_sensors] / self.taxel_area / 1000.0
        return np.clip(f, 0, 534).reshape(TAXELS, TAXELS)

    def cap_state(self) -> dict:
        d = self.data
        moved = math.hypot(d.qpos[self.qadr["cap_x"]], d.qpos[self.qadr["cap_y"]])
        return {"turned_deg": math.degrees(d.qpos[self.qadr["cap_rot"]]),
                "pressed_mm": -d.qpos[self.qadr["cap_press"]] * 1000,
                "lifted_mm": d.qpos[self.qadr["cap_lift"]] * 1000,
                "moved_mm": moved * 1000,                  # sideways from the bottle: > ~30 = removed
                "free": self.cap_free}

    # ------------------------------------------------------------------ kinematics
    def site_pose(self, deg: dict, site: str) -> tuple[np.ndarray, np.ndarray]:
        """Position and hand-axis direction (fingers / camera view) of `site` for sim angles."""
        s = self._scratch
        s.qpos[:] = self.data.qpos
        for j, v in deg.items():
            if j in self.qadr:
                s.qpos[self.qadr[j]] = math.radians(v)
        mujoco.mj_kinematics(self.model, s)
        sid = self.model.site(site).id
        return s.site_xpos[sid].copy(), s.site_xmat[sid].reshape(3, 3)[:, 2].copy()

    def fk(self, deg: dict) -> tuple[np.ndarray, np.ndarray]:
        """Grasp point position and hand axis (unit vector, fingers direction) for sim angles."""
        s = self._scratch
        s.qpos[:] = self.data.qpos
        for j, v in deg.items():
            if j in self.qadr:
                s.qpos[self.qadr[j]] = math.radians(v)
        mujoco.mj_kinematics(self.model, s)
        return s.site_xpos[self.grasp_site].copy(), s.site_xmat[self.grasp_site].reshape(3, 3)[:, 2].copy()

    def ik(self, target, seeds: list[dict] | None = None, tol: float = 2e-4,
           site: str = "grasp", fixed: dict | None = None) -> tuple[dict, float]:
        """Sim angles (shoulder_pan, shoulder_lift, elbow_flex) putting `site` (default: the grasp
        point between the jaws; "wrist_cam" = the camera) at `target`.

        Damped least squares from several starting guesses; among exact-enough solutions picks the
        one whose fingers point most nearly straight down (best for grabbing a cap from above).
        `fixed`: other joints to hold at given sim angles while solving (e.g. a wrist_roll).
        Returns (angles, position error in m).
        """
        m, s = self.model, self._scratch
        sid = m.site(site).id
        target = np.asarray(target, float)
        lim = {j: np.radians(self.sim["arm"]["limits_deg"][j]) for j in REACH}
        cols = [self.dadr[j] for j in REACH]
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        pan = math.degrees(math.atan2(target[1], target[0]))
        seeds = seeds or [{"shoulder_pan": pan, "shoulder_lift": lift, "elbow_flex": elbow}
                          for lift in (-30, 10, 40) for elbow in (60, 100, 130)]
        best = None
        for seed in seeds:
            s.qpos[:] = self.data.qpos
            for j, v in (fixed or {}).items():
                s.qpos[self.qadr[j]] = math.radians(v)
            for j in REACH:
                s.qpos[self.qadr[j]] = math.radians(seed.get(j, 0))
            for _ in range(100):
                mujoco.mj_kinematics(m, s)
                mujoco.mj_comPos(m, s)
                err = target - s.site_xpos[sid]
                if np.linalg.norm(err) < tol:
                    break
                mujoco.mj_jacSite(m, s, jacp, jacr, sid)
                J = jacp[:, cols]
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), err)
                for k, j in enumerate(REACH):
                    a = self.qadr[j]
                    s.qpos[a] = np.clip(s.qpos[a] + np.clip(dq[k], -0.2, 0.2), *lim[j])
            mujoco.mj_kinematics(m, s)
            e = float(np.linalg.norm(target - s.site_xpos[sid]))
            down = -float(s.site_xmat[sid].reshape(3, 3)[2, 2])   # 1 = fingers straight down
            sol = {j: math.degrees(s.qpos[self.qadr[j]]) for j in REACH}
            score = (e > 1e-3, -down if e <= 1e-3 else e)                      # exact first, then most vertical
            if best is None or score < best[0]:
                best = (score, sol, e)
        return best[1], best[2]

    # ------------------------------------------------------------------ cameras
    def render(self, camera: str = "wrist", width: int | None = None, height: int | None = None) -> np.ndarray:
        """RGB image (H x W x 3, uint8). 'wrist' matches the ESP32-CAM resolution by default."""
        cam = self.sim["wrist_camera"]
        w, h = width or cam["width"], height or cam["height"]
        if (w, h) not in self._renderers:
            self._renderers[(w, h)] = mujoco.Renderer(self.model, h, w)
        r = self._renderers[(w, h)]
        r.update_scene(self.data, camera=camera)
        return r.render()

    def camera_ray(self, u: float, v: float, camera: str = "wrist", width: int | None = None,
                   height: int | None = None) -> tuple[np.ndarray, np.ndarray]:
        """World-space ray (origin, unit direction) through pixel (u, v) of `camera`."""
        cam = self.sim["wrist_camera"]
        w, h = width or cam["width"], height or cam["height"]
        c = self.model.camera(camera).id
        f = (h / 2) / math.tan(math.radians(self.model.cam_fovy[c]) / 2)
        d_cam = np.array([(u - w / 2) / f, -(v - h / 2) / f, -1.0])   # MuJoCo cameras look down -z
        d = self.data.cam_xmat[c].reshape(3, 3) @ d_cam
        return self.data.cam_xpos[c].copy(), d / np.linalg.norm(d)

    def close(self):
        for r in self._renderers.values():
            r.close()
        self._renderers.clear()
