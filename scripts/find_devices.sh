#!/usr/bin/env bash
# List serial ports and cameras. Put the results into configs/robot.local.yaml.
set -euo pipefail
echo "== Serial ports (unplug/replug when asked) =="; lerobot-find-port
echo "== Cameras (connect the phone first) ==";         lerobot-find-cameras opencv
