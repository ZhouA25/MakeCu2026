#!/usr/bin/env bash
# One-shot setup of a computer to run PillBot (macOS or Linux). Safe to re-run.
#   bash scripts/setup_laptop.sh                     env + installs + check
#   bash scripts/setup_laptop.sh pillbot-local-....tgz   ...and import the old computer's local files
#   SIM=1 bash scripts/setup_laptop.sh               also install the simulator (MuJoCo, SB3, torch)
#   NO_HUB=1 bash scripts/setup_laptop.sh            no ESP32 hub: hub.enabled false (no clamp/buttons)
set -euo pipefail
cd "$(dirname "$0")/.."
ENV=${PILLBOT_ENV:-pillbot}
BUNDLE=${1:-}

if ! command -v conda >/dev/null; then
  echo "conda not found. Install Miniforge first: https://github.com/conda-forge/miniforge"; exit 1
fi
eval "$(conda shell.bash hook)"
if ! conda env list | awk '{print $1}' | grep -qx "$ENV"; then
  echo "== creating conda env '$ENV' (Python 3.12)"
  conda create -y -n "$ENV" python=3.12
fi
conda activate "$ENV"
conda install -y -q ffmpeg -c conda-forge >/dev/null

echo "== git submodules (SensX driver)"
git submodule update --init --recursive || echo "   (no submodule access — SensX driver skipped)"

echo "== installing pillbot + LeRobot 0.7 + our arm plugin"
pip install -q -e ".[dev]"
pip install -q -e plugins/lerobot_robot_pillbot
[ -f sensx_python/pyproject.toml ] || [ -f sensx_python/setup.py ] && pip install -q -e sensx_python || true
if [ "${SIM:-0}" = 1 ]; then echo "== simulator"; pip install -q -e ".[sim]"; fi

if [ -n "$BUNDLE" ]; then
  echo "== importing local files from $BUNDLE"
  python -m pillbot.tools.setup_local import "$BUNDLE"
fi

if [ "${NO_HUB:-0}" = 1 ]; then
  echo "== detecting the U2D2 + camera -> configs/robot.local.yaml (no hub)"
  python -m pillbot.tools.setup_local init --no-hub
else
  echo "== detecting ports + camera -> configs/robot.local.yaml (plug in the U2D2 and the hub first)"
  python -m pillbot.tools.setup_local init
fi
echo
python -m pillbot.tools.setup_local check || true
if [ "$(uname)" = Darwin ]; then
  echo
  echo "macOS: the first time Python opens the camera, allow Terminal under"
  echo "System Settings -> Privacy & Security -> Local Network."
fi
echo
echo "Done. Each new terminal:  conda activate $ENV"
