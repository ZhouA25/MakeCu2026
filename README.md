# PillBot — Assisted-Grip Pill Bottle Opener (LeRobot SO-101)

A hackathon project: a LeRobot SO-101 arm with a custom end-effector attachment that can
**pick up different pill bottles and open them** for people with limited grip strength or dexterity
(arthritis, Parkinson's, post-stroke, elderly users).

## The idea in one line
Bottle placed in a taped **start zone** → arm picks it up by the body → drops it into a **funnel clip** →
re-grips the cap → **push-down-and-twist** (child-proof caps) in a few grip cycles → cap dropped in a tray.

**Hardware assumptions:** SO-101 follower + leader, **no wrist camera** — two fixed cameras
(phone on a tripod opposite the arm + laptop webcam, both ~45° down), optional TouchTronix
TactX / FSR tactile pads, Raspberry Pi later (develop on a laptop first).

## Repo layout
```
configs/
  robot.yaml          ports, 2 fixed cameras, tactile backend, gripper + safety params
  poses.yaml          taught joint poses (zone corners, clip, cap, tray)
  bottles.yaml        bottle catalog: cap type, twist degrees
  policies.yaml       trained policy paths + task strings
src/pillbot/
  run.py              FULL PIPELINE (scripted or policy pick, scripted or policy open)
  lerobot_args.py     builds LeRobot CLI flags from configs (used by scripts/)
  control/            robot_io (safe stepping), grasp (feel-for-contact / missed-grasp),
                      pick_place, open_sequence, poses (zone interpolation), teach, policy_runner
  perception/         cameras, zone (pixel -> zone calibration), detect_bottle, spin_monitor
  sensors/tactile.py  none | mock | serial (FSR/CSV) | tactx (fill in with vendor SDK)
scripts/              find_devices, calibrate, teleop, record, train, rollout
hardware/ firmware/   CAD, BOM, wiring, FSR sketch
docs/                 architecture, safety, demo plan, task split
tests/                dry-run tests (no hardware needed)
```

## Quickstart
```bash
# 0. Environment — LeRobot 0.7 needs Python 3.12
conda create -y -n pillbot python=3.12 && conda activate pillbot
conda install -y ffmpeg -c conda-forge
pip install -e ".[dev]"
pytest -q                                   # everything runs in simulation

python -m pillbot.run --dry-run             # simulate the whole pipeline

# 1. Hardware setup
bash scripts/find_devices.sh                # -> put ports/camera indices in configs/robot.local.yaml
bash scripts/calibrate.sh
bash scripts/teleop.sh                      # check both camera views; move cameras if the cap is hidden in both

# 2. Teach poses + calibrate the zone (redo if cameras or clip move!)
python -m pillbot.control.teach --all
python -m pillbot.perception.zone --calibrate      # click the 4 tape corners in the same order
python -m pillbot.perception.detect_bottle          # live check: is the bottle found + s,t sensible?

# 3. Run it
python -m pillbot.run --pick skip                   # bottle already in the clip: test opening only
python -m pillbot.run                                # full scripted run

# 4. (Stretch) learned pick & place with ACT
bash scripts/record.sh pick_place                   # ~50+ demos, vary bottle position
DEVICE=cuda bash scripts/train.sh pick_place        # on a GPU (Colab/cloud)
python -m pillbot.run --pick policy                 # policy places, script opens
```

> LeRobot changes fast. This repo targets **lerobot 0.7.x** (`lerobot.robots.so_follower`,
> `lerobot-rollout`). If a flag errors, check `lerobot-record --help` / https://huggingface.co/docs/lerobot.

## Two approaches (pick one early, keep the other as a fallback)
| | Scripted primitives (`src/pillbot/control`) | Learned policy (ACT via LeRobot) |
|---|---|---|
| Time to first demo | Fast | Needs ~50+ teleop demos + training |
| Handles new bottles | Only with tuned params per bottle | Generalizes somewhat if demos are varied |
| Hackathon risk | Low | Medium–high |

Recommended: get the **scripted press-and-twist** working first, then record demos for the
pick-and-place part and train a policy if time allows.

## Team
See `docs/task-split.md`.
