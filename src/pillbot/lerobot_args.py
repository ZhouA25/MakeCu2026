"""Single source of truth for LeRobot CLI arguments (ports, cameras, tasks) — read from configs/.

Used by scripts/*.sh, e.g.:  $(python -m pillbot.lerobot_args robot)
  robot            --robot.* flags for the follower (incl. both cameras)
  teleop           --teleop.* flags for the leader
  dataset <name>   --dataset.* flags for a policy in configs/policies.yaml
  policy-path <n>  checkpoint path
  hf-user          your HF username
"""
import shlex
import sys
import json

from pillbot.utils.config import load_yaml


def _cam_val(v) -> str:
    v = str(v)
    return f'"{v}"' if ":" in v or "/" in v else v      # URLs must be quoted inside the mapping


def cameras_arg(cfg: dict) -> str:
    parts = []
    for name, c in cfg["cameras"].items():
        inner = ", ".join(f"{k}: {_cam_val(v)}" for k, v in c.items())
        parts.append(f"{name}: {{{inner}}}")
    return "{ " + ", ".join(parts) + " }"


def _fmt(v) -> str:
    if isinstance(v, dict):
        return json.dumps(v, separators=(",", ":"))
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, list):
        return "[" + ",".join(str(x) for x in v) + "]"
    return str(v)


def _device_args(prefix: str, d: dict) -> list[str]:
    out = [f"--{prefix}.type={d['type']}"]
    out += [f"--{prefix}.{k}={_fmt(v)}" for k, v in d.items() if k != "type"]
    return out


def robot_args(cfg: dict) -> list[str]:
    return _device_args("robot", cfg["follower"]) + [f"--robot.cameras={cameras_arg(cfg)}"]


def teleop_args(cfg: dict) -> list[str]:
    return _device_args("teleop", cfg["leader"])


def dataset_args(pcfg: dict, name: str) -> list[str]:
    p = pcfg["policies"][name]
    return [f"--dataset.repo_id={pcfg['hf_user']}/{p['dataset']}", f"--dataset.single_task={p['task']}"]


def rollout_cmd(cfg: dict, pcfg: dict, name: str) -> list[str]:
    p = pcfg["policies"][name]
    return ["lerobot-rollout", "--strategy.type=base", f"--policy.path={p['path']}",
            *robot_args(cfg), f"--task={p['task']}", f"--duration={p['duration_s']}"]


def main(argv):
    nul = "--nul" in argv
    argv = [a for a in argv if a != "--nul"]
    cfg = load_yaml("configs/robot.yaml")
    pcfg = load_yaml("configs/policies.yaml")
    what = argv[0]
    if what == "robot":
        out = robot_args(cfg)
    elif what == "teleop":
        out = teleop_args(cfg)
    elif what == "dataset":
        out = dataset_args(pcfg, argv[1])
    elif what == "policy-path":
        print(pcfg["policies"][argv[1]]["path"]); return
    elif what == "hf-user":
        print(pcfg["hf_user"]); return
    else:
        raise SystemExit(__doc__)
    if nul:   # NUL-separated, for scripts/_common.sh
        sys.stdout.write("".join(a + "\0" for a in out))
    else:
        print(" ".join(shlex.quote(a) for a in out))


if __name__ == "__main__":
    main(sys.argv[1:])
