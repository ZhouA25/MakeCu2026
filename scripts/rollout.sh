#!/usr/bin/env bash
# Run one trained policy on its own.  bash scripts/rollout.sh pick_place
source "$(dirname "$0")/_common.sh"
TASK=${1:-pick_place}
python -c "
from pillbot.lerobot_args import rollout_cmd; from pillbot.utils.config import load_yaml as l
import os; c=rollout_cmd(l('configs/robot.yaml'), l('configs/policies.yaml'), '$TASK'); os.execvp(c[0], c)"
