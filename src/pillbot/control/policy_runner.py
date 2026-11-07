"""Hand control to a trained LeRobot policy for one phase (e.g. pick & place), then take it back.

LeRobot needs exclusive access to the serial port and cameras, so we disconnect our own
RobotIO (and any OpenCV captures) before launching `lerobot-rollout`, and reconnect after.
"""
import logging
import subprocess

from pillbot.lerobot_args import rollout_cmd

log = logging.getLogger(__name__)


def run_policy_phase(io, cfg: dict, pcfg: dict, name: str, dry_run: bool = False) -> bool:
    cmd = rollout_cmd(cfg, pcfg, name)
    log.info("policy phase '%s': %s", name, " ".join(cmd))
    if dry_run:
        return True
    io.disconnect()
    try:
        ok = subprocess.run(cmd).returncode == 0
    finally:
        io.connect()
    return ok
