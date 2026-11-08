import importlib.util
from pathlib import Path

import pytest

path = Path(__file__).resolve().parents[1] / 'plugins/lerobot_robot_pillbot/lerobot_robot_pillbot/motor_ids.py'
spec = importlib.util.spec_from_file_location('motor_ids_validation', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_roll', 'gripper']


def test_user_ids_preserve_other_joint_defaults():
    assert module.resolve_motor_ids(JOINTS, {'shoulder_lift':12, 'wrist_roll':15, 'gripper':10}) == dict(zip(JOINTS, [1,12,3,15,10]))


def test_duplicate_and_unknown_ids_rejected():
    with pytest.raises(ValueError, match='unique'):
        module.resolve_motor_ids(JOINTS, {'gripper':1})
    with pytest.raises(ValueError, match='unknown'):
        module.resolve_motor_ids(JOINTS, {'wrist_pitch':9})


def test_cli_preserves_explicit_motor_mapping():
    from pillbot.lerobot_args import robot_args
    from pillbot.utils.config import load_yaml
    import json
    arg = next(a for a in robot_args(load_yaml('configs/robot.yaml')) if a.startswith('--robot.motor_ids='))
    assert json.loads(arg.split('=',1)[1]) == {'shoulder_pan':11,'shoulder_lift':12,'elbow_flex':13,'wrist_roll':15,'gripper':10}
