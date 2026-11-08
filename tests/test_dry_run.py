import os
import subprocess
import sys

import pytest

from pillbot.control.grasp import grip_until_contact
from pillbot.control.open_sequence import open_cap
from pillbot.control.pick_place import pick_bottle, place_in_clip
from pillbot.control.poses import load_poses, zone_pose
from pillbot.control.robot_io import RobotIO
from pillbot.lerobot_args import cameras_arg, rollout_cmd
from pillbot.sensors.tactile import MockTactile, NullTactile
from pillbot.utils.config import ROOT, get_bottle, load_yaml

CFG = load_yaml("configs/robot.yaml")
G = CFG["gripper"]


@pytest.fixture
def io():
    r = RobotIO("", "test", dry_run=True, max_step=50)
    r.connect()
    r.set_gripper(G["open"])
    return r


def test_grasp_with_tactile(io):
    io.sim_gripper_block = 30
    res = grip_until_contact(io, MockTactile(io), G)
    assert res.success and res.gripper < 30


def test_grasp_miss_with_tactile(io):
    res = grip_until_contact(io, MockTactile(io), G)   # nothing in hand
    assert not res.success


def test_grasp_stall_detection_without_tactile(io):
    io.sim_gripper_block = 30
    assert grip_until_contact(io, NullTactile(), G).success
    io.sim_gripper_block = None
    io.set_gripper(G["open"])
    assert not grip_until_contact(io, NullTactile(), G).success


def test_zone_interpolation_hits_corners():
    poses = {f"zone_hover_{k}": {"a": v} for k, v in {"00": 0, "10": 10, "11": 30, "01": 20}.items()}
    assert zone_pose(poses, "hover", 0, 0)["a"] == 0
    assert zone_pose(poses, "hover", 1, 0)["a"] == 10
    assert zone_pose(poses, "hover", 1, 1)["a"] == 30
    assert zone_pose(poses, "hover", 0.5, 0.5)["a"] == 15


def test_pick_place_and_open(io):
    tac, poses = MockTactile(io), load_poses()
    io.sim_gripper_block = 35
    assert pick_bottle(io, tac, poses, G, 0.5, 0.5)
    place_in_clip(io, poses, G)
    io.sim_gripper_block = 15
    stats = open_cap(io, tac, get_bottle("standard_amber_40dr"), poses, G)
    assert stats["turned_deg"] == 270 and stats["cap_lifted"]


def test_zone_mapper():
    from pillbot.perception.zone import ZoneMapper
    m = ZoneMapper([(100, 100), (300, 100), (300, 300), (100, 300)])
    s, t = m.to_zone(200, 200)
    assert abs(s - 0.5) < 1e-3 and abs(t - 0.5) < 1e-3


def test_lerobot_args():
    assert "wrist: {type: pillbot_ipcam" in cameras_arg(CFG)
    cmd = rollout_cmd(CFG, load_yaml("configs/policies.yaml"), "pick_place")
    assert cmd[0] == "lerobot-rollout" and any(a.startswith("--task=") for a in cmd)


def test_full_pipeline_cli_dry_run():
    env = {**os.environ, "PYTHONPATH": str(ROOT / "src")}
    r = subprocess.run([sys.executable, "-m", "pillbot.run", "--dry-run"], cwd=ROOT,
                       capture_output=True, text=True, env=env)
    assert r.returncode == 0, r.stderr
    assert "opened!" in r.stderr


def test_esp32_cam_config():
    from pillbot.perception.cameras import camera_source
    wrist = CFG["cameras"]["wrist"]
    assert wrist["type"] == "pillbot_ipcam"
    assert camera_source(wrist).startswith("http://") and camera_source(wrist).endswith("/stream")
    assert camera_source({"index_or_path": 0}) == 0
    arg = cameras_arg(CFG)
    assert f'url: "{wrist["url"]}"' in arg            # URL quoted for LeRobot's parser


def test_press_modes():
    import pytest
    from pillbot.control.open_sequence import should_press
    child, plain = {"cap_type": "push_and_turn"}, {"cap_type": "twist"}
    assert should_press(child, "always") and should_press(plain, "always")
    assert should_press(child, "child_proof") and not should_press(plain, "child_proof")
    assert not should_press(child, "never")
    with pytest.raises(ValueError):
        should_press(plain, "sometimes")
