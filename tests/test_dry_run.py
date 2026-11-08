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


def test_place_modes():
    import pytest
    from pillbot.control.pick_place import blend, place_cfg
    assert place_cfg(None)["mode"] == "slide"
    assert place_cfg({"mode": "lift"})["mode"] == "lift"
    with pytest.raises(ValueError):
        place_cfg({"mode": "throw"})
    lo, hi = {"shoulder_pan": 0.0, "elbow_flex": 10.0, "gripper": 0.0}, {"shoulder_pan": 0.0, "elbow_flex": 60.0, "gripper": 90.0}
    assert blend(lo, hi, 0.1) == {"shoulder_pan": 0.0, "elbow_flex": 15.0}     # gripper untouched


def test_open_retry_and_reseat():
    import pytest
    from pillbot.control.open_sequence import OpenFailed, open_with_retry
    from pillbot.control.pick_place import clamp_and_check

    class IO:
        def __init__(self):
            self.moves = []
            self.target = {"gripper": 0.0, "shoulder_pan": 0.0}
        def move_to(self, g, **kw):
            self.moves.append(g); self.target |= g; return True
        def set_gripper(self, v, **kw):
            self.move_to({"gripper": v})
    G = {"open": 60, "closed": 0, "step": 5, "squeeze": 2, "stall_margin": 6}
    io, calls = IO(), []
    def open_fn(p):
        calls.append(p)
        if len(calls) == 1:
            raise OpenFailed("could not grip cap: missed")
        return {"turned_deg": 270}
    poses = {"cap_above": {"shoulder_pan": 1.0}}
    assert open_with_retry(open_fn, io, G, lambda: poses, retries=1)["open_attempts"] == 2
    with pytest.raises(OpenFailed):                       # not retryable
        open_with_retry(lambda p: (_ for _ in ()).throw(OpenFailed("bottle is spinning")), io, G, lambda: poses)

    class Fix:
        n = 0
        def clamp(self): self.n += 1
        def unclamp(self): pass
    class Tac:
        available = False
        def tare(self): pass
    fx, checks = Fix(), iter([False, True])
    clip = {"clip_above": {"shoulder_pan": 0.0}, "clip_insert": {"shoulder_pan": 0.0}}
    import pillbot.control.pick_place as pp
    pp_reseat = pp.reseat
    pp.reseat = lambda *a, **k: True
    try:
        assert clamp_and_check(io, Tac(), fx, clip, G, lambda: next(checks), retries=1) and fx.n == 2
        assert clamp_and_check(io, Tac(), fx, clip, G, lambda: None)        # can't tell -> accept
    finally:
        pp.reseat = pp_reseat


def test_idle_rests_and_wakes():
    from pillbot.control.idle import go_idle, rest_pose, wake
    from pillbot.control.poses import load_poses
    from pillbot.control.robot_io import RobotIO
    from pillbot.utils.config import load_yaml
    cfg = load_yaml("configs/robot.yaml")
    poses = load_poses()
    io = RobotIO.from_config(cfg, dry_run=True)
    io.connect()

    class Fix:
        log = []
        def release_servo(self): self.log.append("limp")
        def power(self, on): self.log.append(("power", on))
    fx = Fix()
    go_idle(io, poses, cfg, fx)
    assert io.limp == {"shoulder_pan", "wrist_roll", "gripper"}            # gravity-free joints relaxed
    assert fx.log == ["limp"]                       # goBILDA elbow still holds the arm: servo rail stays on
    assert "elbow_flex" in io.joints and "elbow_flex" not in io.limp
    assert rest_pose({"home": {"a": 1}, "rest": {"a": 0}}) == {"a": 1}       # untaught rest -> home
    io.move_to({"elbow_flex": 5.0})                    # a move wakes only the joints it moves
    assert io.limp == {"shoulder_pan", "wrist_roll", "gripper"}
    io.move_to({j: 0.0 for j in io.joints})
    assert not io.limp
    go_idle(io, poses, cfg | {"idle": {"limp": "all"}})
    assert io.limp == set(io.joints)
    wake(io, fx, cfg)
    assert not io.limp and fx.log[-1] == ("power", True)


def test_pwm_elbow_joint():
    from pillbot.control.robot_io import RobotIO
    from pillbot.sensors.hub import MockHub
    from pillbot.utils.config import load_yaml
    cfg = load_yaml("configs/robot.yaml")
    hub = MockHub()
    io = RobotIO.from_config(cfg, dry_run=True, hub=hub)
    io.connect()
    assert "elbow_flex" in io.joints and "elbow_flex" not in io.bus_joints
    c = cfg["pwm_joints"]["elbow_flex"]
    io.move_to({"elbow_flex": 30.0})
    assert hub.servos[c["channel"]] == io.pwm_us("elbow_flex", 30.0) == round(c["center_us"] + c["sign"] * c["us_per_deg"] * 30)
    assert io.pwm_us("elbow_flex", 1000) == c["max_us"]                      # clamped to the limits
    io.torque_off(["elbow_flex"])
    assert hub.servos[c["channel"]] == 0                                     # limp = pulse off
    io.move_to({"elbow_flex": 31.0})                                         # wakes it again
    assert hub.servos[c["channel"]] == io.pwm_us("elbow_flex", 31.0)
