import shutil

import pytest

from pillbot.tools import fixture_bench as fb
from pillbot.utils.config import ROOT


@pytest.mark.parametrize("cmd", [["status"], ["servo", "1", "1600"], ["tune", "clamp"],
                                 ["seat"], ["cycle"], ["cycle", "--times", "1"], ["hand-open"],
                                 ["hand-open", "--bottle", "otc_vitamin"]])
def test_bench_commands_dry_run(cmd, capsys):
    fb.main(["--dry-run", *cmd])
    out = capsys.readouterr().out
    assert "Traceback" not in out


def test_cycle_clamps_and_unclamps(capsys):
    fb.main(["--dry-run", "cycle"])
    out = capsys.readouterr().out
    assert "cycle complete" in out and out.count("clamp + unclamp ok") == 3


def test_save_fixture_values_keeps_comments(tmp_path):
    p = tmp_path / "robot.yaml"
    shutil.copy(ROOT / "configs/robot.yaml", p)
    changed = fb.save_fixture_values({"clamp_open_us": 1234, "clip_seated_mm": 55}, p)
    text = p.read_text()
    assert set(changed) == {"clamp_open_us", "clip_seated_mm"}
    assert "clamp_open_us: 1234" in text and "clip_seated_mm: 55" in text
    assert "# tune: python -m pillbot.tools.fixture_bench tune clamp" in text   # comment preserved
    import yaml
    cfg = yaml.safe_load(text)
    assert cfg["fixture"]["clamp_open_us"] == 1234 and cfg["gripper"]["open"] == 60


def test_tune_elbow_dry_run(capsys):
    from pillbot.tools.fixture_bench import main
    main(["--dry-run", "tune", "elbow"])
    out = capsys.readouterr().out
    assert "min_us: 1340" in out and "max_us: 1660" in out and "sign: 1" in out   # 1500 +/- 4x50, -/+ 40 margin


def test_save_block_values_nested(tmp_path):
    from pillbot.tools.fixture_bench import save_block_values
    p = tmp_path / "r.yaml"
    p.write_text("fixture:\n  min_us: 1  # clamp\npwm_joints:\n  elbow_flex:\n    channel: 1\n"
                 "    min_us: 600    # keep\n    max_us: 2400\n    sign: 1\nidle:\n  max_us: 5\n")
    assert sorted(save_block_values(["pwm_joints", "elbow_flex"], {"min_us": 900, "max_us": 2100, "sign": -1}, p)) \
        == ["max_us", "min_us", "sign"]
    t = p.read_text()
    assert "    min_us: 900    # keep" in t and "    max_us: 2100" in t and "    sign: -1" in t
    assert "  min_us: 1  # clamp" in t and "  max_us: 5" in t                       # other blocks untouched
