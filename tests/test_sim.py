"""Simulation checks (skipped if MuJoCo isn't installed: pip install -e ".[sim]")."""
import logging

import numpy as np
import pytest

pytest.importorskip("mujoco", reason='pip install -e ".[sim]"')

from pillbot.control.grasp import grip_until_contact  # noqa: E402
from pillbot.control.open_sequence import open_cap  # noqa: E402
from pillbot.sim.backends import make_sim  # noqa: E402
from pillbot.sim.demo import cap_poses  # noqa: E402
from pillbot.sim.perception import locate_cap  # noqa: E402
from pillbot.sim.scene import build_xml, load_sim_config  # noqa: E402
from pillbot.utils.config import ROOT, load_yaml  # noqa: E402


def _gripped(seed=0, bottle="standard_amber_40dr"):
    world, io, tac = make_sim(bottle, seed=seed, clip_xy=[0.11, 0.0])
    poses = cap_poses(world, io, world.cap_center)
    g = dict(world.robot_cfg["gripper"]) | {"open": world.sim["gripper_open"]}
    io.move_to(poses["cap_above"] | {"gripper": g["open"]}, timeout=10)
    tac.tare()
    io.move_to(poses["cap_grip"])
    assert grip_until_contact(io, tac, g).success
    return world, io, tac, poses


def test_every_bottle_builds():
    import mujoco
    sim = load_sim_config()
    for b in load_yaml("configs/bottles.yaml")["bottles"]:
        mujoco.MjModel.from_xml_string(build_xml(sim, b))


def test_ik_reaches_cap():
    world, _, _ = make_sim(seed=0, clip_xy=[0.11, 0.0])
    sol, err = world.ik(world.cap_center)
    assert err < 1e-3
    assert np.linalg.norm(world.fk(sol)[0] - world.cap_center) < 1e-3


def test_child_proof_needs_push():
    world, io, tac, poses = _gripped(seed=0)
    world.thread_torque = 0.05
    io.move_to({"wrist_roll": -30})
    assert abs(world.cap_state()["turned_deg"]) < 3          # locked: jaws slip, cap stays
    io.move_to({"wrist_roll": 0})
    io.move_to(poses["cap_press"])
    assert world.cap_state()["pressed_mm"] > world.unlock_mm
    io.move_to({"wrist_roll": -30})
    assert world.cap_state()["turned_deg"] > 20              # pushed: it turns


def test_wrist_camera_finds_cap():
    world, io, _ = make_sim(seed=3, clip_xy=[0.12, 0.01])
    found = locate_cap(world, io, nominal_xy=[0.11, 0.0])
    assert found is not None
    assert np.linalg.norm(found["cap_center"] - world.cap_center) < 0.005


def test_real_open_sequence_opens_easy_cap():
    """Recommended hardware: STS3215-class gripper servo + palm pusher, coordinated twist."""
    from pillbot.sim.kinematics import orbit_twist
    logging.disable(logging.WARNING)
    sim = load_sim_config()
    sim["arm"] |= {"palm_pusher": True, "grip_torque_per_ma": 0.00089}
    world, io, tac = make_sim(seed=0, sim_cfg=sim, clip_xy=[0.11, 0.0])
    world.thread_torque = 0.06
    poses = cap_poses(world, io, world.cap_center)
    g = dict(world.robot_cfg["gripper"]) | {"open": world.sim["gripper_open"]}
    io.move_to(poses["cap_above"] | {"gripper": g["open"]}, timeout=10)
    tac.tare()
    stats = open_cap(io, tac, world.bottle, poses, g, ocfg=world.robot_cfg.get("open"),
                     twist=orbit_twist(world, io))
    logging.disable(logging.NOTSET)
    assert stats["cap_lifted"] and world.cap_state()["free"]
    assert world.cap_state()["moved_mm"] > 30                # carried away from the bottle


def test_reach_env_contract():
    from gymnasium.utils.env_checker import check_env
    from pillbot.sim.envs import ReachEnv
    env = ReachEnv(seed=0)
    check_env(env, skip_render_check=True)
    obs, _ = env.reset(seed=1)
    _, _, _, _, info = env.step(np.zeros(3, np.float32))
    assert obs.shape == (15,) and "dist" in info


def test_urdf_matches_sim(tmp_path):
    from pillbot.sim.urdf import build_urdf, check
    from pillbot.utils.config import get_bottle
    sim, bottle = load_sim_config(), get_bottle("standard_amber_40dr")
    path = tmp_path / "hardware" / "urdf" / "pillbot.urdf"
    path.parent.mkdir(parents=True)
    (tmp_path / "hardware" / "stl").symlink_to(ROOT / "hardware" / "stl")
    path.write_text(build_urdf(sim, bottle))
    assert check(path, sim, bottle, n=20) < 1e-4
