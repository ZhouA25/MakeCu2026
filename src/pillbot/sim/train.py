"""Train / evaluate the reach policy in simulation (PPO, stable-baselines3).

  python -m pillbot.sim.train reach                        train (~1M steps, ~5-10 min on a laptop CPU)
  python -m pillbot.sim.train reach --steps 3000000        longer = more precise
  python -m pillbot.sim.train eval                         accuracy of models/sim/reach_ppo.zip
  tensorboard --logdir outputs/sim_tb                      watch training curves (optional)

Needs:  pip install -e ".[sim]"
"""
import argparse
import time
from pathlib import Path

import numpy as np

from pillbot.utils.config import ROOT

DEFAULT_MODEL = ROOT / "models/sim/reach_ppo.zip"


class ReachPolicy:
    """A trained reach policy, used like a controller: policy.move(env_or_world, goal)."""

    def __init__(self, path: str | Path = DEFAULT_MODEL):
        from stable_baselines3 import PPO
        self.model = PPO.load(str(path), device="cpu")

    def act(self, obs: np.ndarray) -> np.ndarray:
        return self.model.predict(obs, deterministic=True)[0]

    def drive(self, world, goal, max_steps: int = 150, tol: float = 0.003) -> float:
        """Close the loop on a SimWorld (or, with the same observation, the real arm): returns the
        final distance to `goal` in meters."""
        from pillbot.sim.envs import ReachEnv
        env = ReachEnv.__new__(ReachEnv)                    # reuse obs/step logic on this world
        env.world, env.goal = world, np.asarray(goal, float)
        env.step_deg = world.robot_cfg["safety"]["max_joint_step_deg"]
        env.lim = {j: world.sim["arm"]["limits_deg"][j] for j in ("shoulder_pan", "shoulder_lift", "elbow_flex")}
        env.cmd = {j: v for j, v in world.targets().items() if j in env.lim}
        env.steps, env.max_steps = 0, max_steps
        dist = np.inf
        for _ in range(max_steps):
            _, _, _, _, info = env.step(self.act(env._obs()))
            dist = info["dist"]
            if dist < tol:
                break
        return dist


def train(steps: int, n_envs: int, out: Path, seed: int = 0):
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from pillbot.sim.envs import ReachEnv
    env = make_vec_env(ReachEnv, n_envs=n_envs, seed=seed, vec_env_cls=SubprocVecEnv)
    try:
        import tensorboard  # noqa: F401  (optional: training curves)
        tb = str(ROOT / "outputs/sim_tb")
    except ImportError:
        tb = None
    model = PPO("MlpPolicy", env, n_steps=1024, batch_size=1024, n_epochs=10, gamma=0.98,
                learning_rate=3e-4, policy_kwargs={"net_arch": [256, 256]}, seed=seed, verbose=1,
                tensorboard_log=tb)
    t0 = time.time()
    model.learn(total_timesteps=steps)
    out.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(out))
    env.close()
    print(f"saved {out}  ({(time.time() - t0) / 60:.1f} min)")


def evaluate(path: Path, episodes: int = 100, seed: int = 123) -> dict:
    from pillbot.sim.envs import ReachEnv
    pol, env = ReachPolicy(path), ReachEnv(seed=seed)
    final = []
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        for _ in range(env.max_steps):
            obs, _, _, trunc, info = env.step(pol.act(obs))
            if trunc:
                break
        final.append(info["dist"])
    f = np.array(final) * 1000
    res = {"episodes": episodes, "median_mm": float(np.median(f)), "mean_mm": float(f.mean()),
           "within_5mm": float((f < 5).mean()), "within_10mm": float((f < 10).mean()),
           "worst_mm": float(f.max())}
    print(f"{path.name}: median {res['median_mm']:.1f} mm, mean {res['mean_mm']:.1f} mm, "
          f"{100 * res['within_5mm']:.0f}% within 5 mm, {100 * res['within_10mm']:.0f}% within 10 mm, "
          f"worst {res['worst_mm']:.1f} mm  ({episodes} random goals)")
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["reach", "eval"])
    ap.add_argument("--steps", type=int, default=1_000_000)
    ap.add_argument("--envs", type=int, default=8, help="parallel simulations")
    ap.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    ap.add_argument("--episodes", type=int, default=100)
    args = ap.parse_args()
    if args.cmd == "reach":
        train(args.steps, args.envs, args.model)
        evaluate(args.model, args.episodes)
    else:
        evaluate(args.model, args.episodes)


if __name__ == "__main__":
    main()
