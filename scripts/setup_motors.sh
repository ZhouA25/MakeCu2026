#!/usr/bin/env bash
# One-time: give each Dynamixel its ID (1..6) + baudrate. Connect ONE motor at a time when prompted.
source "$(dirname "$0")/_common.sh"
WHICH=${1:-follower}
if [ "$WHICH" = follower ]; then
  collect robot;  lerobot-setup-motors $(printf '%s\n' "${ARGS[@]}" | grep -v -- '--robot.cameras' | tr '\n' ' ')
else
  collect teleop; lerobot-setup-motors "${ARGS[@]}"
fi
