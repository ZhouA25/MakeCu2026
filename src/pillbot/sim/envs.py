"""Gymnasium environments for training policies in the PillBot simulation.

ReachEnv — "move the grasp point to any point": the skill the approach to the cap is built from.

  observation (15 numbers): joint angles (3), commanded angles (3), grasp point xyz (3), goal xyz (3),
                            goal - grasp point in cm (3). Angles in radians, positions in meters.
  action (3 numbers in -1..1): change of the shoulder_pan / shoulder_lift / elbow_flex commands,
                            scaled to ±max_joint_step_deg per tick — exactly how RobotIO limits the
                            real arm, so a trained policy outputs commands the real servos can follow.
  reward: minus the distance to the goal, plus a bonus inside 5 mm.

Everything in the observation is available on the real robot: joint angles from the servos, the
grasp point from forward kinematics, the goal from the camera. That's what makes it transferable.
"""
import math

import gymnasium as gym
import numpy as np

from pillbot.sim.world import REACH, SimWorld

GOAL_RANGES = {"shoulder_pan": (-120, 120), "shoulder_lift": (-40, 80), "elbow_flex": (20, 150)}
START_RANGES = {"shoulder_pan": (-90, 90), "shoulder_lift": (-20, 60), "elbow_flex": (30, 140)}
SUCCESS_M = 0.005


def sample_reachable_goal(world, rng) -> np.ndarray:
    """A random grasp-point goal the arm can reach: random joint angles -> their grasp point, keeping
    points above the table and clear of the riser."""
    a = world.sim["arm"]
    while True:
        q = {j: rng.uniform(*GOAL_RANGES[j]) for j in REACH}
        p, _ = world.fk(q)
        near_riser = np.hypot(p[0], p[1]) < a.get("riser_radius", 0.072) + 0.04 \
            and p[2] < a["base_riser"] + 0.03
        if 0.03 < p[2] < 0.40 and not near_riser:
            return p


class ReachEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, max_steps: int = 100, seed: int | None = None, render_mode: str | None = None):
        self.world = SimWorld(seed=seed)
        self.max_steps = max_steps
        self.render_mode = render_mode
        self.step_deg = self.world.robot_cfg["safety"]["max_joint_step_deg"]
        self.lim = {j: self.world.sim["arm"]["limits_deg"][j] for j in REACH}
        self.action_space = gym.spaces.Box(-1.0, 1.0, (3,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (15,), np.float32)
        self.goal = np.zeros(3)
        self.cmd = {}

    def _sample_goal(self, rng) -> np.ndarray:
        return sample_reachable_goal(self.world, rng)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        start = {j: rng.uniform(*START_RANGES[j]) for j in REACH}
        self.world.reset(clip_xy=[0.45, 0.45], home=start)          # bottle out of the way
        self.goal = np.asarray(options["goal"], float) if options and "goal" in options else self._sample_goal(rng)
        self.cmd = dict(start)
        self.steps = 0
        return self._obs(), {}

    def set_goal(self, goal):
        self.goal = np.asarray(goal, float)

    def _obs(self) -> np.ndarray:
        q = self.world.read()
        tip = self.world.grasp_pos()
        return np.concatenate([
            [math.radians(q[j]) for j in REACH],
            [math.radians(self.cmd[j]) for j in REACH],
            tip, self.goal, (self.goal - tip) * 100,
        ]).astype(np.float32)

    def step(self, action):
        a = np.clip(np.asarray(action, float), -1, 1)
        for k, j in enumerate(REACH):
            self.cmd[j] = float(np.clip(self.cmd[j] + a[k] * self.step_deg, *self.lim[j]))
        self.world.set_targets(self.cmd)
        self.world.tick()
        self.steps += 1
        dist = float(np.linalg.norm(self.goal - self.world.grasp_pos()))
        reward = -10 * dist - 0.01 * float(a @ a) + (1.0 if dist < SUCCESS_M else 0.0)
        return self._obs(), reward, False, self.steps >= self.max_steps, {"dist": dist,
                                                                         "is_success": dist < SUCCESS_M}

    def render(self):
        return self.world.render("overview", 640, 480)

    def close(self):
        self.world.close()
