"""setup_local: export -> import round trip and init without hardware."""
import tarfile

import yaml

from pillbot.tools import setup_local as sl


def _fake_root(tmp_path, monkeypatch):
    root = tmp_path / "repo"
    (root / "configs").mkdir(parents=True)
    (root / "configs/robot.local.yaml").write_text("follower:\n  port: /dev/old\n  gripper_goal_current: 1500\n")
    (root / "configs/zone_calib.local.yaml").write_text("corners: [[0, 0]]\n")
    (root / "configs/background.local.png").write_bytes(b"png")
    cal = tmp_path / "cal"
    (cal / "robots/pillbot_koch_follower").mkdir(parents=True)
    (cal / "robots/pillbot_koch_follower/pillbot_follower.json").write_text("{}")
    monkeypatch.setattr(sl, "ROOT", root)
    monkeypatch.setattr(sl, "LOCAL", root / "configs/robot.local.yaml")
    monkeypatch.setattr(sl, "calibration_dir", lambda: cal)
    return root, cal


def test_export_import_round_trip(tmp_path, monkeypatch):
    root, cal = _fake_root(tmp_path, monkeypatch)
    out = tmp_path / "b.tgz"
    sl.main(["export", "--out", str(out)])
    names = tarfile.open(out).getnames()
    assert "repo/configs/robot.local.yaml" in names and "repo/configs/background.local.png" in names
    assert "calibration/robots/pillbot_koch_follower/pillbot_follower.json" in names
    (root / "configs/robot.local.yaml").write_text("changed\n")
    (cal / "robots/pillbot_koch_follower/pillbot_follower.json").unlink()
    sl.main(["import", str(out)])
    assert "port: /dev/old" in (root / "configs/robot.local.yaml").read_text()
    assert (cal / "robots/pillbot_koch_follower/pillbot_follower.json").exists()
    assert list((root / "configs").glob("robot.local.yaml.bak-*"))           # old file kept


def test_import_rejects_paths_outside(tmp_path, monkeypatch):
    root, _ = _fake_root(tmp_path, monkeypatch)
    bad = tmp_path / "bad.tgz"
    evil = tmp_path / "evil.txt"
    evil.write_text("x")
    with tarfile.open(bad, "w:gz") as t:
        t.add(evil, arcname="repo/../../evil.txt")
    sl.main(["import", str(bad)])
    assert not (tmp_path / "evil.txt.bak").exists() and not (root.parent.parent / "evil.txt").exists()


def test_init_updates_only_machine_keys(tmp_path, monkeypatch):
    root, _ = _fake_root(tmp_path, monkeypatch)
    monkeypatch.setattr(sl, "find_ports", lambda probe=True: {"follower": "/dev/u2d2", "hub": "/dev/hub",
                                                             "u2d2": [{}], "all": []})
    monkeypatch.setattr(sl, "camera_ok", lambda url, timeout=3.0: (url.startswith("http://pillbot-cam"), "x"))
    sl.main(["init", "-y"])
    new = yaml.safe_load((root / "configs/robot.local.yaml").read_text())
    assert new["follower"] == {"port": "/dev/u2d2", "gripper_goal_current": 1500}   # tuned value kept
    assert new["hub"] == {"enabled": True, "port": "auto"}
    assert new["cameras"]["wrist"]["url"] == "http://pillbot-cam.local:81/stream"
