# PillBot — Assisted-Grip Pill Bottle Opener

MakeCU 2026 · Healthcare track. A small Dynamixel robot arm (LeRobot) plus a motorized bottle
fixture that **picks up a pill bottle and opens it** for people with limited grip strength or
dexterity (arthritis, Parkinson's, post-stroke, elderly users).

**How it works:** set a bottle in the taped start zone → the arm picks it up and drops it into a
funnel clip → a servo clamp locks the bottle body → the arm grips the cap (a TouchTronix SensX25
tactile pad feels the contact and detects slipping), **pushes down and twists** it off with its wrist →
and drops the cap in a tray. The arm itself stays general-purpose; only the clamp is task-specific.

| Part | What we use |
|---|---|
| Arm | [danslovich/5DOF_Robot_Arm](https://github.com/danslovich/5DOF_Robot_Arm): 5× Dynamixel XL330-M288, 4:1 printed planetary shoulder — driven through LeRobot 0.7 by our plugin (see `hardware/arm.md`) |
| Fixture | Funnel clip + goBILDA 2000 servo clamp, driven by the "PillBot Hub" — an ESP32 **or** an Arduino UNO Q |
| Touch | TouchTronix SensX25 (one, on the gripper finger) |
| Vision | **One camera:** an ESP32-CAM on the wrist, streaming over Wi-Fi. To find bottles the arm moves to a taught `scan` pose above the start zone and looks down |
| Controls | START / STOP / MODE push buttons on the hub; STOP cuts servo power in firmware |

Everything runs in **simulation** with `--dry-run`, so you can develop and test without hardware.
For physics (contacts, the child-proof cap, the SensX pad, the wrist camera, a trained reach policy)
see **§5 Physics simulation** and `docs/simulation.md`.

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

**c. Set up the hub** — pick one:

*Arduino UNO Q:* follow `firmware/unoq_hub/README.md` (App Lab app: MCU sketch + Python relay),
then set `hub.port: socket://<uno-q-address>:5555` in `configs/robot.local.yaml`.

*ESP32:* open `firmware/esp32_hub/esp32_hub.ino` in the Arduino IDE, board
"ESP32 Dev Module", install the **ESP32Servo** library, upload. The serial monitor (115200) should
print `READY pillbot-hub 2` then `S ...` lines; pressing a button changes the last number.

**c2. Flash the ESP32-CAM wrist camera:** open `firmware/esp32_cam/esp32_cam.ino`, set your Wi-Fi
name/password at the top, board **"AI Thinker ESP32-CAM"**, upload (ESP32-CAM-MB USB board, or an FTDI
adapter with IO0 → GND while resetting). The serial monitor (115200) prints
`stream: http://<ip>:81/stream` — open it in a browser to check, then put that URL in
`configs/robot.local.yaml` → `cameras.wrist.url`. Laptop/Pi and camera must be on the same Wi-Fi.

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

**f. Calibrate the camera to the start zone** (redo whenever the `scan` pose or the tape moves).
The wrist camera only sees the zone the same way every time from the `scan` pose
(~25–35 cm above the zone, looking straight down — teach it in step e), so `--goto-scan` moves the arm
there first:
```bash
python -m pillbot.perception.zone --calibrate --goto-scan    # click the 4 tape corners in the taught order
python -m pillbot.perception.detect_bottle --goto-scan       # CLEAR the zone, press b (background), add a bottle
python -m pillbot.perception.detect_bottle --goto-scan --tune   # sliders + mask if it sees shadows / misses bottles
```
Detection works for **any bottle** (amber, white multivitamin, …): it compares against a photo of the
empty zone, so re-take it (key `b`) if the camera, tape or lighting changes. Set
`perception.zone_size_mm` to your tape square's real size — the preview then shows the bottle width
in mm and which `bottles.yaml` profile it matches. Alternatives: `--method yolo` (pretrained,
`pip install -e ".[vision]"`) or `--method color` (amber only).

**Different bottles:** add each one to `configs/bottles.yaml` (body/cap diameter, cap type, twist
degrees), and teach its height-dependent poses once:
```bash
python -m pillbot.control.teach --bottle otc_vitamin --bottle-poses   # saved as cap_grip@otc_vitamin, ...
```
Bottles without their own poses use the generic ones. Measure the widest cap the jaws open around
and set `gripper.max_cap_mm` — runs refuse bottles whose cap won't fit.

**g. Bench-test and tune the fixture — no arm needed** (hub + clamp servo + clip):
```bash
python -m pillbot.tools.fixture_bench status            # live ultrasonics + buttons
python -m pillbot.tools.fixture_bench tune clamp        # find open/closed pulses -> saved to config
python -m pillbot.tools.fixture_bench seat              # "bottle seated" distance -> saved to config
python -m pillbot.tools.fixture_bench cycle             # clamp / unclamp a few times
python -m pillbot.tools.fixture_bench hand-open         # clamp, then YOU push + twist the cap off by hand
python -m pillbot.tools.fixture_bench servo 0 1500      # raw pulse on a channel (0 = limp)
```
Add `--dry-run` to try any of them without hardware. `hand-open` proves the clamp holds a real
child-proof bottle while the cap is twisted, and can save the measured `twist_deg` to `bottles.yaml`.
Then set `open.loosen_sign` (which way the wrist unscrews) and `follower.gripper_goal_current`
(grip strength) once the arm is up.
Then tune the tactile `contact_threshold` while watching live SensX readings:

**h. Tactile sensor (SensX25)** — how it works, Pi setup, using it from other programs: `docs/tactile.md`
```bash
python -m pillbot.tools.tactile_live doctor     # find the sensor's port / diagnose wiring
python -m pillbot.tools.tactile_live            # live pressure + contact = what the gripper sees; tune contact_threshold
python -m pillbot.tools.tactile_live plot       # heatmap window
python -m pillbot.tools.fake_sensx              # virtual sensor for testing without hardware
```

---

## 3. Run it

```bash
python -m pillbot.run                          # wait for Enter → find bottle → pick → place → open
python -m pillbot.run --trigger button         # start with the START button
python -m pillbot.run --trigger zone           # hands-free: starts when a bottle is set in the zone
python -m pillbot.run --pick skip              # bottle already in the clip — test opening only (do this first!)
python -m pillbot.run --zone-st 0.5,0.5        # skip vision, use a known spot in the zone
python -m pillbot.run --bottle otc_vitamin     # bottle profile from configs/bottles.yaml
python -m pillbot.run --bottle auto            # measure the bottle, pick the matching profile
python -m pillbot.run --loop                   # keep serving bottles until Ctrl-C
python -m pillbot.run --dry-run                # any of the above, simulated
```

**STOP**: the STOP button cuts servo power on the hub and aborts the run; the arm holds position
until you support it and press Enter. Ctrl-C also stops.

Suggested bring-up order: `--pick skip` (tune the clamp, grip, push and twist on a clipped bottle) →
`--zone-st 0.5,0.5` (tune pick/place) → full run with vision → `--trigger zone` for the demo.

---

## 4. Learned policy (optional)

Needs a leader arm for teleop demos. Record → train on a GPU → run:
```bash
bash scripts/teleop.sh                         # check both camera views first
bash scripts/record.sh pick_place              # ~50+ demos, vary the bottle position (EPISODES=50)
DEVICE=cuda bash scripts/train.sh pick_place   # ACT, on Colab / a cloud GPU
python -m pillbot.run --pick policy            # policy picks & places, scripted push-and-twist opens
```
Tasks, dataset names and checkpoint paths live in `configs/policies.yaml`.

---

## 5. Physics simulation (MuJoCo) — no hardware needed

The arm, clamped bottle, child-proof cap, SensX25 pad and wrist camera in MuJoCo. The real
`open_sequence.py` runs against it unchanged, and a reach policy is trained in it
(`models/sim/reach_ppo.zip`). Guide, results and design findings: **`docs/simulation.md`**.
```bash
# light setup, no LeRobot needed (or `pip install -e ".[sim]"` inside the full pillbot env)
python3 -m venv .venv && source .venv/bin/activate
pip install mujoco gymnasium stable-baselines3 imageio opencv-python pyyaml pyserial numpy -e sensx_python
export PYTHONPATH=src
python -m pillbot.sim.demo --video outputs/sim.gif   # reach -> find cap -> approach -> grip/push/untwist
python -m pillbot.sim.demo --episodes 20             # success rate over randomized caps
python -m pillbot.sim.workspace                      # where the arm can grab a cap from above
python -m pillbot.sim.train reach                    # retrain the policy after measuring the arm
```

---

## Configuration

| File | What's in it |
|---|---|
| `configs/robot.yaml` | arm type/ports/joints, cameras, hub, tactile backend, gripper, fixture, trigger, safety |
| `configs/robot.local.yaml` | your machine's overrides (git-ignored). Only put the keys you change (ports, URLs, tuned values) — everything else comes from `robot.yaml`. `key: null` removes a key (e.g. a camera you don't have) |
| `configs/poses.yaml` | taught joint poses (written by `teach`) |
| `configs/bottles.yaml` | bottle profiles: sizes, cap type (`push_and_turn` / `twist`), twist degrees |
| `configs/background.local.png` | empty-zone photo for bottle detection (written by `detect_bottle`, key b) |
| `configs/policies.yaml` | trained policies for `--pick policy` / `--open policy` |
| `configs/sim.yaml` | physics simulation: arm measurements, servo strength, cap randomization (`docs/simulation.md`) |
| `configs/zone_calib.local.yaml` | camera → zone calibration (written by `zone --calibrate`) |

Key switches in `robot.yaml`: `open:` (twist per grip, direction), `follower.gripper_goal_current`,
`tactile.backend: sensx | mock | none`,
`hub.enabled`, `trigger.default`.

## Repo layout
```
src/pillbot/
  run.py              full pipeline CLI
  control/            robot_io (safe stepping, STOP), grasp (feel-for-contact, missed-grasp),
                      pick_place, open_sequence (grip, push, twist), fixture (clamp), poses, teach,
                      triggers, policy_runner
  tools/fixture_bench.py  bench-test the hub + clip + clamp without the arm
  tools/tactile_live.py   find / watch / tune the SensX25;  tools/fake_sensx.py  virtual sensor
  perception/         cameras, zone calibration, bottle detection, spin monitor
  sensors/            tactile (SensX25 + backends), tactile_server (share it over TCP),
                      sensx_probe (port/protocol detection), hub (ESP32 serial client)
  sim/                MuJoCo physics: world, sim backends for the real control code, cap-finding,
                      training env + PPO, demo, workspace check
models/sim/           trained reach policy
  lerobot_args.py     builds LeRobot CLI flags from configs/ (used by scripts/)
plugins/lerobot_robot_pillbot/   LeRobot plugin: pillbot_koch_follower / pillbot_koch_leader
firmware/esp32_hub/   ESP32 hub firmware (ultrasonics, buttons, servos, relay)
firmware/unoq_hub/    same hub on an Arduino UNO Q (MCU sketch + Linux-side TCP relay)
firmware/esp32_cam/   ESP32-CAM wrist camera (fixed-size MJPEG stream over Wi-Fi)
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
| SensX: no frames / port busy | `python -m pillbot.tools.tactile_live doctor` — wiring, permissions and more in `docs/tactile.md` §7 |
| Motors not found | Wrong port, motor power off, or IDs/baud not set — rerun `scripts/setup_motors.sh` |
| `No data from hub` | ESP32 not flashed / wrong port / `hub.enabled` — or set `hub.enabled: false` to run arm-only |
| Bottle not detected / shadows detected | Re-take the background (`b`) with the zone empty; `--tune` the diff threshold and min area; avoid moving shadows |
| Wrist camera times out (`Stream timeout`) | The ESP32-CAM streams to **one viewer at a time** — close any browser tab on `:81/stream` (use `/capture` to peek instead), or press RST on the camera |
| Browser can see the camera but Python can't | macOS: allow Terminal under System Settings → Privacy & Security → Local Network |
| Wrist camera fails in LeRobot | Plugin not installed (`pillbot_ipcam` type), wrong URL, or stream size ≠ `width`/`height` — the error says which |
| Camera won't open | Index changed (phone reconnects reorder them) — rerun `find_devices.sh`; grant camera permission to Terminal on macOS |
| Arm sags / misses | Bring the zone & clip closer to the base, add an elbow rubber band, re-teach poses |
| Cap won't come off | Flip `fixture.loosen_direction`, raise `twist_deg` in `bottles.yaml`, check the clip isn't letting the bottle slip |

Safety: empty bottles only, keep hands clear, someone on STOP during every run (`docs/safety.md`).

Credits: arm design by Daniel Slovich ([5DOF_Robot_Arm](https://github.com/danslovich/5DOF_Robot_Arm));
[LeRobot](https://github.com/huggingface/lerobot) (Apache-2.0); TouchTronix [sensx_python](https://github.com/TouchTronix-Robotics/sensx_python).
