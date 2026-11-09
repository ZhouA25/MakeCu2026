"""tools.demo_moves: lift once, then claw + wrist twist at the top, never lowering."""
from pillbot.tools import demo_moves


def test_steps_and_dry_run():
    start = {"shoulder_pan": 0.0, "shoulder_lift": -10.0, "elbow_flex": 5.0, "wrist_roll": 20.0, "gripper": 0.0}
    assert demo_moves.lift_step(start, 60) == ("up", {"shoulder_lift": 50.0})
    assert demo_moves.lift_step(start, 0) is None
    steps = demo_moves.demo_steps(start, 45)
    assert [s[0] for s in steps] == ["claw OPEN", "claw SHUT", "claw OPEN", "claw SHUT",
                                     "twist +", "twist back", "twist -", "twist back"]
    assert steps[4][1] == {"wrist_roll": 65.0} and steps[6][1] == {"wrist_roll": -25.0}
    assert all("shoulder_lift" not in g and "elbow_flex" not in g for _, g in steps)   # never lowers
    assert demo_moves.main(["--dry-run"]) == 0
