#!/usr/bin/env bash
# Calibrate follower + leader (types/ports/presets come from configs/robot.yaml).
source "$(dirname "$0")/_common.sh"
collect robot;  ROBOT=("${ARGS[@]}");  ARGS=()
collect teleop; TELEOP=("${ARGS[@]}")
# cameras aren't needed for calibration
lerobot-calibrate $(printf '%s\n' "${ROBOT[@]}" | grep -v -- '--robot.cameras' | tr '\n' ' ')
lerobot-calibrate "${TELEOP[@]}"
