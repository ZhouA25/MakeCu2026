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
