#!/usr/bin/env bash
# List serial ports and cameras. Put the results into configs/robot.local.yaml.
set -euo pipefail
echo "== Serial devices (stable names + hub probe) =="; python -m pillbot.tools.ports
echo "== Serial ports, LeRobot style (unplug/replug when asked) =="; lerobot-find-port
echo "== Cameras (connect the phone first) ==";         lerobot-find-cameras opencv
