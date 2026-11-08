"""Reject learned checkpoints without a matching physical-model fingerprint."""
import hashlib
import json
from pathlib import Path

from pillbot.utils.config import ROOT, load_yaml


def fingerprint(sim=None):
    sim = sim if sim is not None else load_yaml('configs/sim.yaml')
    payload = {'arm':sim['arm'], 'joint_map':sim.get('joint_map', {}),
               'assembly':hashlib.sha256((ROOT / 'src/pillbot/sim/assembly.py').read_bytes()).hexdigest()}
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def metadata_path(path):
    return Path(path).with_suffix('.metadata.json')


def check_checkpoint(path, sim=None):
    meta = metadata_path(path)
    if not meta.exists():
        raise ValueError('Reach checkpoint has no verified model metadata. Retrain for the approved assembly or use IK.')
    expected = json.loads(meta.read_text(encoding='utf-8')).get('model_fingerprint')
    if expected != fingerprint(sim):
        raise ValueError('Reach checkpoint model differs from this assembly. Retrain before using it.')
