#!/usr/bin/env bash
# Record teleop demos.  bash scripts/record.sh pick_place   |   bash scripts/record.sh open_cap
# Vary bottle position (inside the taped zone) and cap start angle every episode!
source "$(dirname "$0")/_common.sh"
TASK=${1:-pick_place}
collect robot; collect teleop; collect dataset "$TASK"
lerobot-record "${ARGS[@]}" \
  --dataset.num_episodes="${EPISODES:-50}" \
  --dataset.episode_time_s="${EPISODE_S:-30}" \
  --dataset.reset_time_s="${RESET_S:-10}" \
  --dataset.push_to_hub="${PUSH:-false}" \
  --display_data=true
