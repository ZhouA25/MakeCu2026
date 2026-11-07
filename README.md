# PillBot — Assisted-Grip Pill Bottle Opener

MakeCU 2026 · Healthcare track. A small Dynamixel robot arm (LeRobot) plus a motorized bottle
fixture that **picks up a pill bottle and opens it** for people with limited grip strength or
dexterity (arthritis, Parkinson's, post-stroke, elderly users).

**How it works:** set a bottle in the taped start zone → the arm picks it up and drops it into a
funnel clip on a servo turntable → the clip clamps → the arm grips the cap (TouchTronix SensX25
tactile pads feel the contact and detect slipping) → the turntable spins the **bottle** under the held
cap to unscrew it → the arm lifts the cap off into a tray.

| Part | What we use |
|---|---|
| Arm | [danslovich/5DOF_Robot_Arm](https://github.com/danslovich/5DOF_Robot_Arm): 5× Dynamixel XL330-M288, 4:1 printed planetary shoulder — driven through LeRobot 0.7 by our plugin (see `hardware/arm.md`) |
| Twisting | goBILDA 2000 servo turntable + servo clamp, driven by an ESP32 "PillBot Hub" |
| Touch | TouchTronix SensX25 (one, on the gripper finger) |
| Vision | One fixed camera (webcam or phone on a tripod, ~45° down); optional endoscope as wrist cam |
| Controls | START / STOP / MODE push buttons on the hub; STOP cuts servo power in firmware |

Everything runs in **simulation** with `--dry-run`, so you can develop and test without hardware.

---

## 1. Install (laptop, ~10 min)

LeRobot 0.7 needs **Python 3.12+**.

```bash
# clone WITH the SensX driver submodule
git clone --recurse-submodules https://github.com/ZhouA25/MakeCu2026.git makecu
cd makecu
# (already cloned without it?)  git submodule update --init

# Python 3.12 environment (install Miniforge first if you don't have conda)
conda create -y -n pillbot python=3.12
conda activate pillbot
conda install -y ffmpeg -c conda-forge

pip install -e ".[dev]"                        # pillbot + lerobot[core-scripts,dynamixel] 0.7.x
pip install -e plugins/lerobot_robot_pillbot   # registers our XL330 arm with LeRobot's CLIs
pip install -e sensx_python                    # TouchTronix SensX25 driver
```

Check it:
```bash
python -c "import lerobot; print(lerobot.__version__)"   # 0.7.x
pytest -q                                               # all tests pass, no hardware needed
python -m pillbot.run --dry-run                          # full pick → place → open, simulated
```

GPU machine for training only: `pip install -e ".[train]"`.
Raspberry Pi 5: same steps, but Raspberry Pi OS may ship an older Python — use Miniforge (ARM64) to get 3.12.

---

## 2. Hardware setup (once per build)

Wiring and power: `hardware/wiring.md` · parts: `hardware/bom.md` · arm notes: `hardware/arm.md`.

**a. Find ports and cameras**, then put them in `configs/robot.local.yaml`
(copy of `configs/robot.yaml`, git-ignored — it overrides the defaults on your machine):
```bash
bash scripts/find_devices.sh        # Dynamixel adapter, SensX adapter(s), ESP32, camera indices
```

**b. Set the servo IDs** (one motor plugged in at a time, before assembly — order = `follower.joints`):
```bash
bash scripts/setup_motors.sh        # IDs 1-5: shoulder_pan, shoulder_lift, elbow_flex, wrist_roll, gripper
```

**c. Flash the ESP32 hub:** open `firmware/esp32_hub/esp32_hub.ino` in the Arduino IDE, board
"ESP32 Dev Module", install the **ESP32Servo** library, upload. The serial monitor (115200) should
print `READY pillbot-hub 2` then `S ...` lines; pressing a button changes the last number.

**d. Calibrate the arm:**
```bash
SKIP_LEADER=1 bash scripts/calibrate.sh   # no leader arm? skip it. Move each joint through its range when asked
```

**e. Teach poses** (torque off — move the arm by hand, press Enter at each prompt):
```bash
python -m pillbot.control.teach --all       # walks through every pose with a hint
python -m pillbot.control.teach --missing   # only the ones still unset
python -m pillbot.control.teach --name cap_press   # redo one
```
Keep the start zone, clip and cap tray **close to the base** and grasp **top-down with the forearm
vertical** — the XL330 elbow is the weakest joint (`hardware/arm.md`).

**f. Calibrate the camera to the start zone** (redo whenever the camera or tape moves):
```bash
python -m pillbot.perception.zone --calibrate     # click the 4 tape corners in the same order you taught them
python -m pillbot.perception.detect_bottle         # live check: bottle box + s,t position; q to quit
```

**g. Tune fixture servos** in `configs/robot.yaml` → `fixture:` (`clamp_open_us`, `clamp_closed_us`,
`loosen_direction`), and the tactile `contact_threshold` while watching live readings.

---

## 3. Run it

```bash
python -m pillbot.run                          # wait for Enter → find bottle → pick → place → open
python -m pillbot.run --trigger button         # start with the START button
python -m pillbot.run --trigger zone           # hands-free: starts when a bottle is set in the zone
python -m pillbot.run --pick skip              # bottle already in the clip — test opening only (do this first!)
python -m pillbot.run --zone-st 0.5,0.5        # skip vision, use a known spot in the zone
python -m pillbot.run --bottle otc_vitamin     # bottle profile from configs/bottles.yaml
python -m pillbot.run --loop                   # keep serving bottles until Ctrl-C
python -m pillbot.run --dry-run                # any of the above, simulated
```

**STOP**: the STOP button cuts servo power on the hub and aborts the run; the arm holds position
until you support it and press Enter. Ctrl-C also stops.

Suggested bring-up order: `--pick skip` (tune the clamp, turntable and grip on a clipped bottle) →
`--zone-st 0.5,0.5` (tune pick/place) → full run with vision → `--trigger zone` for the demo.

---

## 4. Learned policy (optional)

Needs a leader arm for teleop demos. Record → train on a GPU → run:
```bash
bash scripts/teleop.sh                         # check both camera views first
bash scripts/record.sh pick_place              # ~50+ demos, vary the bottle position (EPISODES=50)
DEVICE=cuda bash scripts/train.sh pick_place   # ACT, on Colab / a cloud GPU
python -m pillbot.run --pick policy            # policy picks & places, scripted turntable opens
```
Tasks, dataset names and checkpoint paths live in `configs/policies.yaml`.

---

## Configuration

| File | What's in it |
|---|---|
| `configs/robot.yaml` | arm type/ports/joints, cameras, hub, tactile backend, gripper, fixture, trigger, safety |
| `configs/robot.local.yaml` | your machine's overrides (git-ignored) |
| `configs/poses.yaml` | taught joint poses (written by `teach`) |
| `configs/bottles.yaml` | bottle profiles: cap type (`push_and_turn` / `twist`), twist degrees |
| `configs/policies.yaml` | trained policies for `--pick policy` / `--open policy` |
| `configs/zone_calib.local.yaml` | camera → zone calibration (written by `zone --calibrate`) |

Key switches in `robot.yaml`: `open_method: turntable | wrist`, `tactile.backend: sensx | sensx2 | mock | none`,
`hub.enabled`, `trigger.default`.

## Repo layout
```
src/pillbot/
  run.py              full pipeline CLI
  control/            robot_io (safe stepping, STOP), grasp (feel-for-contact, missed-grasp),
                      pick_place, open_sequence (wrist / turntable), fixture, poses, teach,
                      triggers, policy_runner
  perception/         cameras, zone calibration, bottle detection, spin monitor
  sensors/            tactile (SensX25), hub (ESP32 serial client)
  lerobot_args.py     builds LeRobot CLI flags from configs/ (used by scripts/)
plugins/lerobot_robot_pillbot/   LeRobot plugin: pillbot_koch_follower / pillbot_koch_leader
firmware/esp32_hub/   ESP32 hub firmware (ultrasonics, buttons, servos, relay)
sensx_python/         TouchTronix SensX driver (git submodule)
scripts/              find_devices, setup_motors, calibrate, teleop, record, train, rollout
hardware/             arm notes, BOM, wiring, CAD/STL
docs/                 architecture, safety, demo plan, task split
tests/                simulation tests
```

## Troubleshooting
| Problem | Fix |
|---|---|
| `No matching distribution for lerobot` | Python < 3.12 — `conda activate pillbot`, check `python --version` |
| `pillbot_koch_follower` unknown | `pip install -e plugins/lerobot_robot_pillbot` |
| `sensx_python` folder empty | `git submodule update --init` |
| Motors not found | Wrong port, motor power off, or IDs/baud not set — rerun `scripts/setup_motors.sh` |
| `No data from hub` | ESP32 not flashed / wrong port / `hub.enabled` — or set `hub.enabled: false` to run arm-only |
| Camera won't open | Index changed (phone reconnects reorder them) — rerun `find_devices.sh`; grant camera permission to Terminal on macOS |
| Arm sags / misses | Bring the zone & clip closer to the base, add an elbow rubber band, re-teach poses |
| Cap won't come off | Flip `fixture.loosen_direction`, raise `twist_deg` in `bottles.yaml`, check the clip isn't letting the bottle slip |

Safety: empty bottles only, keep hands clear, someone on STOP during every run (`docs/safety.md`).

Credits: arm design by Daniel Slovich ([5DOF_Robot_Arm](https://github.com/danslovich/5DOF_Robot_Arm));
[LeRobot](https://github.com/huggingface/lerobot) (Apache-2.0); TouchTronix [sensx_python](https://github.com/TouchTronix-Robotics/sensx_python).
