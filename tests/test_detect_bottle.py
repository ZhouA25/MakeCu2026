import shutil

import cv2
import numpy as np
import yaml

from pillbot.control.bottle_select import check_fits, match_bottle
from pillbot.control.poses import load_poses
from pillbot.perception.detect_bottle import (DEFAULTS, BackgroundDetector, ColorDetector, Detection,
                                              save_settings, settings, width_mm)
from pillbot.perception.zone import ZoneMapper
from pillbot.utils.config import ROOT, load_yaml

TABLE = 90


def scene(bottle_bgr=None, x=200, y=150, w=60, h=110, noise=0, seed=0):
    rng = np.random.default_rng(seed)
    img = np.full((480, 640, 3), TABLE, np.int16)
    if noise:
        img += rng.integers(-noise, noise + 1, img.shape)
    if bottle_bgr is not None:
        img[y:y + h, x:x + w] = bottle_bgr
    return np.clip(img, 0, 255).astype(np.uint8)


AMBER = cv2.cvtColor(np.uint8([[[15, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0].astype(np.int16)
WHITE = np.array([235, 235, 235], np.int16)          # white multivitamin bottle


def bg_detector():
    d = BackgroundDetector(settings(load_yaml("configs/robot.yaml")))
    d.capture([scene(noise=4, seed=i) for i in range(5)])
    return d


def test_background_finds_white_and_amber_bottles():
    d = bg_detector()
    for color in (WHITE, AMBER):
        det = d(scene(color, noise=4, seed=9))
        assert det is not None and abs(det.x - 200) <= 6 and abs(det.w - 60) <= 10


def test_background_ignores_empty_noisy_table():
    assert bg_detector()(scene(noise=4, seed=42)) is None


def test_background_without_snapshot_finds_nothing():
    assert BackgroundDetector(DEFAULTS)(scene(WHITE)) is None


def test_color_only_finds_amber():
    c = ColorDetector(DEFAULTS)
    assert c(scene(AMBER)) is not None
    assert c(scene(WHITE)) is None          # why we moved to background detection


def test_width_mm_from_zone():
    mapper = ZoneMapper([(100, 100), (500, 100), (500, 400), (100, 400)])   # 400 px = 200 mm
    det = Detection(x=200, y=150, w=80, h=100)
    assert abs(width_mm(det, mapper, 200) - 40) < 1


def test_match_bottle_and_fit_check():
    assert match_bottle(41)["name"] == "standard_amber_40dr"
    assert match_bottle(64)["name"] == "multivitamin_large"
    assert match_bottle(120) is None
    big = {"name": "x", "cap_diameter_mm": 70}
    assert check_fits(big, {"max_cap_mm": 50}) and not check_fits(big, {})


def test_per_bottle_pose_override(tmp_path, monkeypatch):
    import pillbot.control.poses as P
    raw = {"poses": {"cap_grip": {"a": 1.0}, "cap_grip@otc_vitamin": {"a": 9.0},
                     "cap_grip@empty": {"a": 0.0}, "home": {"a": 0.5}}}
    monkeypatch.setattr(P, "load_yaml", lambda path: raw)
    assert P.load_poses()["cap_grip"] == {"a": 1.0}
    assert P.load_poses(bottle="otc_vitamin")["cap_grip"] == {"a": 9.0}
    assert P.load_poses(bottle="empty")["cap_grip"] == {"a": 1.0}       # untaught override ignored
    assert "cap_grip@otc_vitamin" not in P.load_poses(bottle="otc_vitamin")


def test_save_settings_round_trip(tmp_path):
    p = tmp_path / "robot.yaml"
    shutil.copy(ROOT / "configs/robot.yaml", p)
    save_settings({"method": "background", "diff_threshold": 42, "min_area": 900}, p)
    save_settings({"min_area": 950}, p)                      # idempotent, merges
    cfg = yaml.safe_load(p.read_text())
    assert cfg["perception"]["diff_threshold"] == 42 and cfg["perception"]["min_area"] == 950
    assert p.read_text().count("\nperception:") == 1
    assert cfg["fixture"]["clamp_channel"] == 0 and cfg["perception_camera"]


def textured_table(shift=(0, 0), bottle=None, seed=3):
    """Table with texture (wood grain / paper fibres) so a camera shift is visible."""
    rng = np.random.default_rng(seed)
    base = cv2.GaussianBlur(rng.integers(0, 256, (520, 700, 3)).astype(np.uint8), (0, 0), 1.5)
    base = cv2.normalize(base, None, 0, 255, cv2.NORM_MINMAX)
    dx, dy = shift
    img = base[20 + dy:500 + dy, 30 + dx:670 + dx].copy()
    if bottle is not None:
        img[150:260, 200:260] = bottle
    return img


def test_alignment_absorbs_scan_pose_backlash():
    st = {**settings(load_yaml("configs/robot.yaml")), "align": True}
    d = BackgroundDetector(st)
    d.capture([textured_table()])
    assert d(textured_table(shift=(5, -4))) is None                       # empty, view nudged: no false bottle
    det = d(textured_table(shift=(5, -4), bottle=WHITE.astype(np.uint8)))
    assert det is not None and abs(det.x - 200) <= 8


def test_without_alignment_shift_causes_false_detections():
    st = {**settings(load_yaml("configs/robot.yaml")), "align": False}
    d = BackgroundDetector(st)
    d.capture([textured_table()])
    assert d(textured_table(shift=(5, -4))) is not None                   # why align is on by default


def test_config_is_single_wrist_camera():
    cfg = load_yaml("configs/robot.yaml")
    assert list(cfg["cameras"]) == ["wrist"] and cfg["perception_camera"] == "wrist"
    assert cfg["perception"]["camera_on_arm"] is True
    from pillbot.control.poses import REQUIRED
    assert "scan" in REQUIRED


def test_local_config_overlays_instead_of_replacing(tmp_path):
    from pillbot.utils.config import deep_merge
    base = {"cameras": {"wrist": {"url": "a", "width": 320}}, "perception": {"method": "background"}}
    local = {"cameras": {"wrist": {"url": "b"}, "phone": {"index_or_path": 1}}}
    m = deep_merge(base, local)
    assert m["cameras"]["wrist"] == {"url": "b", "width": 320}         # override one key, keep the rest
    assert m["perception"]["method"] == "background"                  # new repo settings still arrive
    assert "phone" not in deep_merge(m, {"cameras": {"phone": None}})["cameras"]   # null removes


def test_anchor_center_for_top_down_camera():
    st = settings(load_yaml("configs/robot.yaml"))
    assert st["anchor"] == "center"                       # camera_on_arm -> center
    d = BackgroundDetector(st)
    d.capture([scene(noise=4, seed=i) for i in range(3)])
    det = d(scene(WHITE, x=200, y=150, w=60, h=110, noise=4, seed=7))
    cx, cy = det.anchor_px("center")
    assert abs(cx - 230) <= 3 and abs(cy - 205) <= 3       # middle of the blob, not its bottom edge
    assert abs(det.anchor_px("base")[1] - 260) <= 3
    assert settings({"perception": {"camera_on_arm": False}})["anchor"] == "base"
