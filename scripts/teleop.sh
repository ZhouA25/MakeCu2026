#!/usr/bin/env bash
# Teleoperate with both camera feeds shown — use this to check camera placement/occlusion.
source "$(dirname "$0")/_common.sh"
collect robot; collect teleop
lerobot-teleoperate "${ARGS[@]}" --display_data=true
