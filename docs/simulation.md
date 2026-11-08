# Simulation: train and test before the prototype exists

A MuJoCo physics model of the arm, the clamped pill bottle, its child-proof cap, the SensX25 pad and
the wrist camera. The **real control code** (`grasp.py`, `open_sequence.py`) runs against it unchanged,
and a policy for moving the arm is trained in it.

Setup, once, from the repo folder. The sim doesn't need LeRobot/PyTorch-for-the-robot, so this is a
light environment (any Python 3.10+). Already have the full `pillbot` env? `pip install -e ".[sim]"` there.
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install mujoco gymnasium stable-baselines3 imageio opencv-python pyyaml pyserial numpy -e sensx_python
export PYTHONPATH=src                            # run pillbot from the repo without installing it
```
Then:
```bash
python -m pillbot.sim.demo                       # the whole task once, stage by stage
python -m pillbot.sim.demo --pusher --video outputs/sim.gif   # with the recommended hand change, recorded
python -m pillbot.sim.demo --episodes 20         # success rate over 20 randomized bottles/caps
python -m pillbot.sim.workspace --risers 0 0.12  # where can the arm grab a cap from above?
python -m pillbot.sim.train reach                # retrain the reach policy (~5 min on a laptop CPU)
python -m pillbot.sim.train eval                 # how accurate it is
pytest tests/test_sim.py                         # 6 checks, ~5 s
```

## 1. The approach we recommend (and why)

**Engine: MuJoCo.** Free, `pip install`, runs ~160× faster than real time on a laptop, has good
contact/friction physics, built-in touch sensors (→ a simulated SensX25) and cameras (→ the ESP32-CAM).
It's also what LeRobot's own sim environments use. Isaac Sim needs an NVIDIA GPU and days of setup;
PyBullet's contacts are weaker for grasping; Gazebo drags in ROS. None of that fits a 24 h build.

**Don't train one network end to end.** "See the bottle → open the cap" with reinforcement learning
from scratch needs millions of contact-rich episodes and transfers poorly. Split the task, and learn
only the parts that benefit from learning:

| Stage | How | Why this way |
|---|---|---|
| 1. Move to any point | **Learned** (PPO policy, `sim/envs.py` ReachEnv) | General skill, cheap to train, uses only data the real arm has |
| 2. Find the cap | **Computed** — wrist camera + background subtraction + geometry (`sim/perception.py`) | No training data needed; works the same on the real camera |
| 3. Approach the cap | The stage-1 policy, aimed above the detected cap | Same skill, new goal |
| 4. Grip by feel, push, untwist | **Closed-loop control with the SensX25** — the team's `open_sequence.py`, run unchanged | Contact-rich; the tactile loop is robust and testable; RL here is the hard part with the least payoff in 24 h |

**Make transfer likely.** The policy sees only joint angles, the grasp point computed from them, and
the goal. All of those exist on the real robot. It outputs the same per-tick joint changes `RobotIO` sends to
the servos (±2°/tick at 30 Hz). Everything uncertain is **randomized** every episode: bottle position,
cap tightness, child-proof unlock force, pad friction, and start pose. A method that works across all
of those doesn't depend on the sim being exact.

**Later (needs the built arm): imitation learning.** The repo already has LeRobot ACT
(`scripts/record.sh`, `train.sh`). Record ~50 teleop demos on the real arm. The sim's scripted runs
could add demonstrations too, but sim images look nothing like the ESP32-CAM, so rely on real demos
for vision.

## 2. How it's built

```
configs/sim.yaml            arm measurements (MEASURE!), servo strength, cap randomization ranges, camera
src/pillbot/sim/
  scene.py                  generates the MuJoCo model from sim.yaml + bottles.yaml (no hand-written XML)
  world.py                  SimWorld: physics stepping, the cap mechanism, tactile frames, camera, FK/IK
  backends.py               SimRobotIO / SimTactile: the sim behind the SAME interfaces as the real arm + SensX
  perception.py             find the cap with the wrist camera -> 3D position
  envs.py                   Gymnasium ReachEnv (training)
  train.py                  PPO training / evaluation; ReachPolicy to use a trained model
  demo.py                   the four stages end to end
  workspace.py              reachability / finger-tilt table per bottle and riser height
models/sim/reach_ppo.zip    the trained reach policy (committed, so nobody has to retrain)
```

**The model:** table → riser → base `[shoulder_pan]` → upper arm `[shoulder_lift]` → forearm
`[elbow_flex]` → hand `[wrist_roll]` → two jaws `[slides]`. The moving jaw carries a **5×5 grid of
touch sensors** that outputs kPa per taxel like the SensX25 (force ÷ taxel area). The wrist camera is
320×240, like the ESP32-CAM QVGA stream. Servos are position-controlled with the XL330's real stall
torque (0.52 N·m; 4:1 planetary on the shoulder), so a joint that's too weak *is* too weak in the sim.

**The cap** is a small machine on top of the physics:
- **thread:** it takes `thread_torque` (0.08–0.30 N·m, randomized) to turn;
- **child-proof:** it's locked until pushed down `push_unlock_mm` (1.2–2.0 mm, against a 4 N/mm spring);
- **release:** after `twist_deg` it's free. It still sits around the neck until lifted 8 mm, then can be carried away.

## 3. Results

| Stage | Result in the sim |
|---|---|
| 1. Reach (trained policy, 100 random goals) | median **3.4 mm**, 76% within 5 mm, 94% within 10 mm |
| 2. Find the cap (wrist camera) | median **~1.5 mm** error; 2 looks: the 2nd centred on the 1st |
| 3. Approach | a few mm (policy), then IK poses from the detected cap for the precise part |
| 4. Open: amber 40 dr vial, today's hand | **24 / 32** opened (75%) |
| 4. Open: amber 40 dr vial, **with palm pusher** | **32 / 32** opened |
| 4. Open: small 20 dr vial, with pusher | 7 / 8 |
| 4. Open: tall OTC / multivitamin bottles | 0 / 8 — see finding 1 |

The precise final moves (grip, press) use inverse kinematics computed from the detected cap. That
replaces hand-teaching `cap_grip` / `cap_press` and is more exact than the policy's ~3 mm.

## 4. What the sim found (act on these)

1. **Raise the arm base: put it on a ~12 cm riser.** With no wrist pitch, the fingers point along the
   forearm. On the table, the arm can't reach any of the bottles from above with less than 41° of
   finger tilt. On a 12 cm riser, a vial clip at **10–12 cm** from the base axis is gripped within
   **1–2° of vertical**. Tall OTC bottles need about 12 cm and still tilt 12°+, and the multivitamin
   isn't reachable from above. Re-run `python -m pillbot.sim.workspace` with the real measurements.
2. **Add a palm pusher.** This is a printed block between the jaws whose bottom face meets the cap's
   top when the jaws are centred on it. Without it, the child-proof push goes through the jaws' side
   friction, the same friction the twist needs (the sim measured ~12 N of it used before twisting even
   starts). Result: 75% → 100% on vials. Then set `palm_pusher: true` in `configs/sim.yaml`.
3. **`cap_press` must go ~6 mm below `cap_grip`,** not "a few mm". A 4 mm push only compresses the
   child-proof spring ~2 mm (the arm and jaws give a little), which is borderline. Set in `sim.yaml`
   `press_depth_mm`.
4. **The opener used to report success without checking it.** It counted *commanded* wrist rotation
   (small slips add up: 270° commanded ≈ 230–260° real), and `cap_lifted` only meant "the last grip
   found contact". **Fixed in `open_sequence.py`:** after twisting, it lifts and checks that the arm got
   there *and* the pad still feels the cap. If not, it twists again (`open.max_extra_twists`, default 2).
5. **New option `gripper.target_pressure`.** After first contact, keep closing until the SensX reads this
   pressure, instead of a fixed extra squeeze. The sim demo uses 500. Tune it on the real pad
   (`tactile_live`). It's off (`null`) by default in `robot.yaml`.

## 5. Moving to the real robot (checklist)

- [ ] Measure the arm into `configs/sim.yaml` (lengths, riser, jaw gap, servo stiffness), then retrain:
      `python -m pillbot.sim.train reach` (~5 min) and re-run `workspace`.
- [ ] Fill in `joint_map` in `sim.yaml` (real LeRobot degrees ↔ sim degrees). Put the real arm in two
      poses, read the angles with `teach`, and match them. `SimRobotIO.to_real()` uses it.
- [ ] Calibrate the wrist camera (intrinsics + where it sits on the hand). `perception.py`'s geometry is
      exact in the sim; on the real arm it's only as good as this calibration.
- [ ] Compare the tactile scale: press the real pad with a known weight in `tactile_live` and compare with
      `sim.yaml tactile.taxel_pitch` (sim kPa = force / taxel area).
- [ ] Measure a real cap's opening torque and push force (a luggage scale on a lever works). If they're
      outside `sim.yaml cap:` ranges, widen the ranges and re-run `demo --episodes 32`.
- [ ] Use the trained policy on the real arm only after the above, at low speed, with someone on STOP.

## 6. Limitations (be honest about what the sim can't tell you)

- Pads are rigid boxes; real TPU/silicone pads conform. Real grip is probably *better* than simulated.
- The jaws are symmetric. The real gripper has one fixed and one moving jaw.
- The cap's thread is modelled as a friction torque, not geometry; cross-threading can't happen.
- No servo backlash (the planetary has some), no cable drag through the wrist, no camera blur or
  lighting changes.
- Getting friction right took care: MuJoCo's default friction "creeps" under steady load, which made
  caps slip out of the jaws or turn while locked. The model uses the no-slip solver with 100 iterations;
  fewer under-transmits grip torque by ~2×. Measured, see `physics:` in `sim.yaml`.
- Open: tall OTC bottles (0/8): the camera ends up too close to see the cap, and the fingers tilt
  12–25°. These need a different clip position / riser before they're worth tuning. Today's-hand failures
  (8/32) are mostly a missed regrip after an extra twist: not fully diagnosed.
