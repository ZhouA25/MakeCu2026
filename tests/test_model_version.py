import json

import pytest

from pillbot.sim.model_version import check_checkpoint, fingerprint, metadata_path
from pillbot.utils.config import load_yaml


def test_checkpoint_without_geometry_metadata_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='no verified model metadata'):
        check_checkpoint(tmp_path / 'old.zip')


def test_geometry_change_invalidates_checkpoint(tmp_path):
    path = tmp_path / 'reach.zip'
    sim = load_yaml('configs/sim.yaml')
    metadata_path(path).write_text(json.dumps({'model_fingerprint': fingerprint(sim)}))
    check_checkpoint(path, sim)
    sim['arm']['upper_arm'] += .01
    with pytest.raises(ValueError, match='differs'):
        check_checkpoint(path, sim)
