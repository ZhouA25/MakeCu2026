"""SensX25 stack against the virtual sensor (pillbot.tools.fake_sensx) — real driver, real bytes."""
import os
import re
import subprocess
import sys
import time

import numpy as np
import pytest

from pillbot.sensors.sensx_probe import HUB_HEADERS, SINGLE_HEADER, classify, crc8_maxim, probe
from pillbot.sensors.tactile import NetTactile, SensXTactile, open_tactile
from pillbot.utils.config import ROOT

pytest.importorskip("sensx", reason="pip install -e sensx_python")
needs_pty = pytest.mark.skipif(sys.platform == "win32", reason="fake sensor uses a pseudo-terminal")


@pytest.fixture
def fake_port():
    env = {**os.environ, "PYTHONPATH": str(ROOT / "src")}
    p = subprocess.Popen([sys.executable, "-m", "pillbot.tools.fake_sensx"], cwd=ROOT, env=env,
                         stdout=subprocess.PIPE, text=True)
    port = re.search(r"/dev/\S+", p.stdout.readline()).group()
    yield port, p
    p.kill()
    p.wait()


def test_classify_single_and_hub_bytes():
    payload = np.arange(25, dtype=">u2").tobytes()
    single = (SINGLE_HEADER + payload + bytes([crc8_maxim(payload)])) * 6
    assert classify(single)[0] == "single"
    hub = b"".join(h + payload for h in list(HUB_HEADERS)[:3]) * 3
    kind, counts = classify(hub)
    assert kind == "hub" and set(counts) == {"A", "B", "C"}
    assert classify(bytes(range(256)) * 4)[0] is None
    corrupted = bytearray(single)
    for i in range(52, len(corrupted), 53):
        corrupted[i] ^= 1                    # break every CRC
    assert classify(bytes(corrupted))[0] is None


@needs_pty
def test_probe_wakes_single_sensor(fake_port):
    port, _ = fake_port
    assert probe(port, listen_s=0.3)[0] is None                 # silent until started
    assert probe(port, listen_s=0.3, wake=True)[0] == "single"


@needs_pty
def test_sensx_tactile_reads_and_goes_stale(fake_port):
    port, proc = fake_port
    tac = SensXTactile(port, max_age_s=0.2, contact_threshold=50)
    tac.connect()
    try:
        time.sleep(0.5)
        assert tac.frames > 50                                  # ~290/s
        s = tac.snapshot()
        assert s.frame.shape == (5, 5) and s.frame.max() <= 534
        assert s.contact == (s.pressure >= 50)
        proc.kill()                                             # "unplug" it
        time.sleep(0.4)
        assert tac.read() is None and not tac.in_contact()
    finally:
        tac.close()


@needs_pty
def test_server_shares_sensor_with_net_clients(fake_port):
    from pillbot.sensors.tactile_server import TactileServer
    port, _ = fake_port
    sensor = SensXTactile(port)
    sensor.connect()
    srv = TactileServer(sensor, "127.0.0.1", 0)                 # port 0 = any free port
    import threading
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    clients = [NetTactile(f"127.0.0.1:{srv.address[1]}") for _ in range(2)]
    try:
        for c in clients:
            c.connect()
        time.sleep(0.2)
        assert all(c.read() is not None and c.read().shape == (5, 5) for c in clients)
    finally:
        for c in clients:
            c.close()
        srv.close()
        sensor.close()


@needs_pty
def test_open_tactile_with_port_override(fake_port):
    port, _ = fake_port
    with open_tactile(backend="sensx", port=port) as tac:
        assert isinstance(tac, SensXTactile) and tac.read() is not None
