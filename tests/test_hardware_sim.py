import os
import subprocess
import sys

import pytest

from pillbot.control.fixture import Fixture
from pillbot.control.open_sequence import open_cap
from pillbot.control.poses import load_poses
from pillbot.control.robot_io import Aborted, RobotIO
from pillbot.control.triggers import wait_for_trigger
from pillbot.sensors.hub import BTN_START, BTN_STOP, HubState, MockHub
from pillbot.sensors.tactile import CombinedTactile, MockTactile, NullTactile, make_tactile
from pillbot.utils.config import ROOT, get_bottle, load_yaml

CFG = load_yaml("configs/robot.yaml")
G = CFG["gripper"]


@pytest.fixture
def io():
    r = RobotIO("", "test", dry_run=True, max_step=50)
    r.connect()
    r.set_gripper(G["open"])
    return r


def test_fixture_clamp_unclamp():
    hub = MockHub()
    fx = Fixture(hub, CFG["fixture"], dry_run=True)
    fx.clamp()
    assert fx.clamped and hub.servos[CFG["fixture"]["clamp_channel"]] == CFG["fixture"]["clamp_closed_us"]
    fx.unclamp()
    assert not fx.clamped and hub.servos[CFG["fixture"]["clamp_channel"]] == CFG["fixture"]["clamp_open_us"]


def test_open_twists_with_wrist_in_configured_direction(io):
    io.sim_gripper_block = 15
    start = io.target["wrist_roll"]
    rolls = []
    orig = io.move_to
    def spy(goal, **kw):
        if "wrist_roll" in goal:
            rolls.append(goal["wrist_roll"] - start)
        return orig(goal, **kw)
    io.move_to = spy
    stats = open_cap(io, MockTactile(io), get_bottle("otc_vitamin"), load_poses(), G,
                     ocfg={"twist_per_grip_deg": 120, "loosen_sign": 1})
    assert stats["turned_deg"] == 360 and stats["cycles"] == 3 and stats["cap_lifted"]
    assert max(rolls) == 120                               # twisted +120 each grip, then unwound to 0


def test_stop_button_aborts_motion(io):
    hub = MockHub()
    io.should_stop = lambda: hub.state.pressed(BTN_STOP)
    io.move_to({"shoulder_pan": 10})
    hub.state = HubState(t_ms=2, buttons=BTN_STOP)
    with pytest.raises(Aborted):
        io.move_to({"shoulder_pan": 40})


def test_button_and_zone_triggers():
    hub = MockHub()
    hub.state = HubState(t_ms=1, buttons=BTN_START)
    assert wait_for_trigger("button", hub, {"hold_s": 0}, timeout=0.5)
    hub.state = HubState(t_ms=1)
    assert not wait_for_trigger("button", hub, {"hold_s": 0}, timeout=0.1)
    hub.state = HubState(t_ms=1, zone_mm=80)
    assert wait_for_trigger("zone", hub, {"hold_s": 0, "zone_present_mm": 120}, timeout=0.5)


def test_combined_tactile_either_jaw(io):
    io.sim_gripper_block = 30
    io.set_gripper(25)
    combo = CombinedTactile(NullTactile(), MockTactile(io))
    assert combo.in_contact()                            # second jaw alone counts


def test_make_tactile_backends(io):
    assert isinstance(make_tactile(CFG, io, dry_run=True), MockTactile)
    cfg = {**CFG, "tactile": {**CFG["tactile"], "backend": "none"}}
    assert isinstance(make_tactile(cfg, io), NullTactile)


def test_pipeline_cli_clamp_push_twist_dry_run():
    env = {**os.environ, "PYTHONPATH": str(ROOT / "src")}
    r = subprocess.run([sys.executable, "-m", "pillbot.run", "--dry-run", "--bottle", "otc_vitamin"],
                       cwd=ROOT, capture_output=True, text=True, env=env)
    assert r.returncode == 0, r.stderr
    assert "opened!" in r.stderr and "'turned_deg': 360.0" in r.stderr


def test_hub_over_tcp_like_uno_q():
    """Hub client against a fake UNO Q relay on localhost (same protocol over socket://)."""
    import socket
    import threading
    import time

    from pillbot.sensors.hub import Hub

    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    got = []

    def fake_unoq():
        conn, _ = srv.accept()
        conn.sendall(b"READY pillbot-hub-unoq 1\n")
        f = conn.makefile("r")
        for i in range(30):
            conn.sendall(f"S {i + 1} 45 300 1\n".encode())
            time.sleep(0.01)
        conn.settimeout(1)
        try:
            got.append(f.readline().strip())
        except OSError:
            pass
        conn.close()

    threading.Thread(target=fake_unoq, daemon=True).start()
    hub = Hub(f"socket://127.0.0.1:{port}")
    hub.connect(timeout=2)
    time.sleep(0.05)
    assert hub.state.clip_mm == 45 and hub.state.zone_mm == 300 and hub.state.pressed(BTN_START)
    hub.servo_us(0, 1500)
    time.sleep(0.3)
    assert got == ["SV 0 1500"]
    hub._stop.set()
    hub._ser.close()
    srv.close()
