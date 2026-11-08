import pytest

from pillbot.utils.ports import resolve_port

PORTS = [{"device": "/dev/ttyUSB0"}, {"device": "/dev/ttyUSB1"}, {"device": "/dev/ttyACM0"}]


def test_plain_and_socket_ports_pass_through():
    assert resolve_port("/dev/ttyUSB3", "hub") == "/dev/ttyUSB3"
    assert resolve_port("socket://unoq.local:5555", "hub") == "socket://unoq.local:5555"


def test_auto_finds_the_port_that_talks_like_a_hub():
    probed = []
    def probe(dev):
        probed.append(dev)
        return dev == "/dev/ttyUSB1"
    assert resolve_port("auto", "hub", ports=PORTS, probe=probe) == "/dev/ttyUSB1"


def test_auto_skips_excluded_ports():
    probed = []
    def probe(dev):
        probed.append(dev)
        return dev == "/dev/ttyACM0"
    assert resolve_port("auto", "hub", exclude=["/dev/ttyUSB0", "auto", None], ports=PORTS,
                        probe=probe) == "/dev/ttyACM0"
    assert "/dev/ttyUSB0" not in probed                      # arm adapter never opened


def test_auto_with_no_hub_gives_clear_error():
    with pytest.raises(RuntimeError, match="no USB serial port is sending hub messages"):
        resolve_port("auto", "hub", ports=PORTS, probe=lambda d: False)


def test_glob(tmp_path):
    (tmp_path / "usb-Silicon_Labs_CP2102-if00").touch()
    assert resolve_port(str(tmp_path / "*CP2102*"), "hub").endswith("CP2102-if00")
    with pytest.raises(RuntimeError, match="nothing matches"):
        resolve_port(str(tmp_path / "*nope*"), "hub")
