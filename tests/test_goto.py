"""tools.goto: step parsing and a dry-run sequence; optional poses for runs without a tray."""
import pytest

from pillbot.control.poses import OPEN_ONLY, untaught
from pillbot.tools import goto

POSES = {"home": {"shoulder_pan": 1.0, "wrist_roll": 0.0},
         "cap_grip": {"shoulder_pan": 5.0, "wrist_roll": 10.0},
         "cap_drop": {"shoulder_pan": 0.0, "wrist_roll": 0.0}}


def test_parse_steps():
    assert goto.parse_step("home", POSES) == ("pose", "home")
    assert goto.parse_step("close", POSES) == ("grip", "close")
    assert goto.parse_step("grip=40", POSES) == ("grip", 40.0)
    assert goto.parse_step("twist-90", POSES) == ("twist", -90.0)
    assert goto.parse_step("wait=0.5", POSES) == ("wait", 0.5)
    with pytest.raises(ValueError, match="isn't taught"):
        goto.parse_step("cap_drop", POSES)
    with pytest.raises(ValueError, match="unknown step"):
        goto.parse_step("nope", POSES)


def test_run_steps_dry_run():
    from pillbot.control.robot_io import RobotIO
    io = RobotIO("x", "x", dry_run=True,
                 follower_cfg={"joints": ["shoulder_pan", "wrist_roll", "gripper"]})
    io.connect()
    steps = [goto.parse_step(s, POSES) for s in ["home", "cap_grip", "close", "twist-90", "open"]]
    goto.run_steps(io, steps, POSES, {"open": 60, "closed": 0})
    t = io.target
    assert t["shoulder_pan"] == pytest.approx(5.0)
    assert t["wrist_roll"] == pytest.approx(-80.0)
    assert t["gripper"] == pytest.approx(60.0)


def test_tray_and_rest_optional():
    poses = {n: {"a": 1.0} for n in OPEN_ONLY}
    assert untaught(poses, OPEN_ONLY) == []
    assert "cap_drop" not in untaught({}) and "rest" not in untaught({})
    assert "cap_grip" in untaught({})
