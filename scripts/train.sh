#!/usr/bin/env bash
# Train ACT.  bash scripts/train.sh pick_place   (DEVICE=cuda|mps|cpu, STEPS=...)
# Use a GPU (Colab/cloud) — on a laptop CPU this is very slow.
source "$(dirname "$0")/_common.sh"
TASK=${1:-pick_place}
HF_USER=$(python -m pillbot.lerobot_args hf-user)
DATASET=$(python -c "from pillbot.utils.config import load_yaml as l; print(l('configs/policies.yaml')['policies']['$TASK']['dataset'])")
lerobot-train \
  --dataset.repo_id="$HF_USER/$DATASET" \
  --policy.type=act \
  --output_dir="outputs/train/act_$TASK" \
  --job_name="act_$TASK" \
  --policy.device="${DEVICE:-cuda}" \
  --steps="${STEPS:-50000}" \
  --policy.push_to_hub=false
