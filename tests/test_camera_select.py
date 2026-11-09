"""Camera selection: config name, esp32 alias, webcam number, URL, env var."""
import pytest

from pillbot.perception.cameras import resolve_camera

CFG = {"perception_camera": "wrist",
       "cameras": {"wrist": {"type": "pillbot_ipcam", "url": "http://10.20.70.37:81/stream"},
                   "phone": None}}


def test_default_is_perception_camera(monkeypatch):
    monkeypatch.delenv("PILLBOT_CAMERA", raising=False)
    assert resolve_camera(CFG) == ("wrist", CFG["cameras"]["wrist"])


def test_alias_number_url():
    assert resolve_camera(CFG, "esp32")[0] == "wrist"
    label, c = resolve_camera(CFG, "0")
    assert label == "webcam 0" and c["index_or_path"] == 0
    assert resolve_camera(CFG, "webcam")[1]["index_or_path"] == 0
    assert resolve_camera(CFG, "http://1.2.3.4:81/stream")[1]["url"] == "http://1.2.3.4:81/stream"


def test_env_var_and_errors(monkeypatch):
    monkeypatch.setenv("PILLBOT_CAMERA", "1")
    assert resolve_camera(CFG)[1]["index_or_path"] == 1
    with pytest.raises(ValueError, match="disabled"):
        resolve_camera(CFG, "phone")
    with pytest.raises(ValueError, match="unknown camera"):
        resolve_camera(CFG, "nope")
