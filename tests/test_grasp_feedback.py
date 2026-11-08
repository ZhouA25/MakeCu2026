"""Regression for transient contact being reported as a successful grasp."""
from pillbot.control.grasp import grip_until_contact


class IO:
    target = {'gripper': 4.}

    def set_gripper(self, target, settle_s=0):
        self.target = {'gripper': target}


class Tactile:
    available = True
    contact_threshold = 100.

    def __init__(self, pressure):
        self.value = pressure

    def in_contact(self):
        return True  # brief touch before squeeze

    def pressure(self):
        return self.value


def test_transient_contact_is_not_success():
    result = grip_until_contact(IO(), Tactile(20), {'closed':0, 'step':2, 'squeeze':1})
    assert not result.success


def test_insufficient_pressure_at_closed_limit_is_not_success():
    result = grip_until_contact(IO(), Tactile(120), {'closed':0, 'step':2, 'squeeze':1, 'target_pressure':200})
    assert not result.success
    assert result.gripper == 0


def test_sustained_pressure_is_success():
    result = grip_until_contact(IO(), Tactile(240), {'closed':0, 'step':2, 'squeeze':1, 'target_pressure':200})
    assert result.success
