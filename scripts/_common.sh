# Sourced by the other scripts. Works with macOS's default bash 3.2.
set -euo pipefail
cd "$(dirname "$0")/.."
ARGS=()
# collect robot | teleop | dataset <name>  -> appends LeRobot flags (from configs/) to ARGS
collect() {
  local x
  while IFS= read -r -d '' x; do ARGS+=("$x"); done < <(python -m pillbot.lerobot_args --nul "$@")
}
