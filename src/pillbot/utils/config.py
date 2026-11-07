from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[3]


def load_yaml(path: str | Path) -> dict:
    """Load a YAML config; prefers a sibling *.local.yaml if one exists."""
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / path
    local = path.with_suffix(".local.yaml")
    with open(local if local.exists() else path) as f:
        return yaml.safe_load(f)


def get_bottle(name: str) -> dict:
    for b in load_yaml("configs/bottles.yaml")["bottles"]:
        if b["name"] == name:
            return b
    raise KeyError(f"Unknown bottle '{name}'. Add it to configs/bottles.yaml")
