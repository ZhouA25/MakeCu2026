import copy
import os
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively overlay `override` on `base`. A value of null in the override deletes the key."""
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if v is None:
            out.pop(k, None)
        elif isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_yaml(path: str | Path) -> dict:
    """Load a YAML config, overlaid with a sibling *.local.yaml if one exists.

    The local file only needs the keys you change (ports, URLs, tuned values); everything else
    comes from the main file, so new settings added to the repo reach you automatically.
    Set a key to null in the local file to remove it (e.g. a camera you don't have).
    Tests set PILLBOT_NO_LOCAL=1 to ignore personal overrides.
    """
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / path
    if path.name.endswith(".local.yaml"):
        with open(path) as f:
            return yaml.safe_load(f) or {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    local = path.with_suffix(".local.yaml")
    if local.exists() and not os.environ.get("PILLBOT_NO_LOCAL"):
        with open(local) as f:
            data = deep_merge(data, yaml.safe_load(f) or {})
    return data


def get_bottle(name: str) -> dict:
    for b in load_yaml("configs/bottles.yaml")["bottles"]:
        if b["name"] == name:
            return b
    raise KeyError(f"Unknown bottle '{name}'. Add it to configs/bottles.yaml")
