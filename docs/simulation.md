# Simulation: train and test before the prototype exists

A MuJoCo physics model of **your arm** (your STLs in `hardware/stl`, the SO-101 gripper), the clamped
pill bottle, its child-proof cap, the SensX25 pad and the wrist camera. The **real control code**
(`grasp.py`, `open_sequence.py`) runs against it unchanged, a policy for moving the arm is trained in
it, and the same model exports a URDF for ROS (§7).

Setup, once, from the repo folder. The sim doesn't need LeRobot/PyTorch-for-the-robot, so this is a
light environment (any Python 3.10+). Already have the full `pillbot` env? `pip install -e ".[sim]"` there.
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install mujoco gymnasium stable-baselines3 imageio opencv-python pyyaml pyserial numpy -e sensx_python
export PYTHONPATH=src                            # run pillbot from the repo without installing it
```
Then:
```bash
python -m pillbot.sim.demo                                   # today's hardware, stage by stage
python -m pillbot.sim.demo --pusher --strong-gripper --video outputs/sim.gif   # recommended hardware
python -m pillbot.sim.demo --episodes 16 --pusher --strong-gripper            # success rate
python -m pillbot.sim.workspace --risers 0 0.04 0.08         # where can the arm grab a cap from above?
python -m pillbot.sim.train reach                            # retrain the reach policy (~5 min)
python -m pillbot.sim.urdf --check                           # URDF for ROS, checked against the sim
pytest tests/test_sim.py                                     # 7 checks, ~10 s
```

## 1. The approach we recommend (and why)

**Engine: MuJoCo.** Free, `pip install`, runs ~160× faster than real time on a laptop, has good
contact/friction physics, built-in touch sensors (→ a simulated SensX25) and cameras (→ the ESP32-CAM).
It's also what LeRobot's own sim environments use. Isaac Sim needs an NVIDIA GPU and days of setup;
PyBullet's contacts are weaker for grasping; Gazebo needs ROS on Linux (§7). None of that fits a 24 h build.

**Don't train one network end to end.** "See the bottle → open the cap" with reinforcement learning
from scratch needs millions of contact-rich episodes and transfers poorly. Split the task, and learn
only the parts that benefit from learning:

| Stage | How | Why this way |
|---|---|---|
| 1. Move to any point | **Learned** (PPO policy, `sim/envs.py` ReachEnv) | General skill, cheap to train, uses only data the real arm has |
| 2. Find the cap | **Computed** — wrist camera + background subtraction + geometry (`sim/perception.py`) | No training data needed; works the same on the real camera |
| 3. Approach the cap | The stage-1 policy, aimed above the detected cap | Same skill, new goal |
| 4. Grip by feel, push, untwist | **Closed-loop control with the SensX25** — the team's `open_sequence.py`, plus a coordinated twist (`sim/kinematics.py`) | Contact-rich; the tactile loop is robust and testable; RL here is the hard part with the least payoff in 24 h |

**Make transfer likely.** The policy sees only joint angles, the grasp point computed from them, and
the goal. All of those exist on the real robot. It outputs the same per-tick joint changes `RobotIO` sends to
the servos (±2°/tick at 30 Hz). Everything uncertain is **randomized** every episode: bottle position,
cap tightness, child-proof unlock force, pad friction, and start pose. A method that works across all
of those doesn't depend on the sim being exact.

**Later (needs the built arm): imitation learning.** The repo already has LeRobot ACT
(`scripts/record.sh`, `train.sh`). Record ~50 teleop demos on the real arm; sim images look nothing
like the ESP32-CAM, so rely on real demos for vision.

## 2. How it's built

```
hardware/stl/               your STLs (see its README for what each part is)
hardware/urdf/pillbot.urdf  generated robot description for ROS (python -m pillbot.sim.urdf)
configs/sim.yaml            arm measurements (MEASURE!), servo strength, gripper, cap randomization, camera
src/pillbot/sim/
  scene.py                  generates the MuJoCo model from sim.yaml + bottles.yaml + your STLs
  world.py                  SimWorld: physics stepping, the cap mechanism, tactile frames, camera, FK/IK
  backends.py               SimRobotIO / SimTactile: the sim behind the SAME interfaces as the real arm + SensX
  kinematics.py             orbit_twist: turn the cap about ITS axis (wrist + arm together) — real robot too
  perception.py             find the cap with the wrist camera -> 3D position
  envs.py / train.py        Gymnasium ReachEnv; PPO training / evaluation; ReachPolicy
  demo.py                   the four stages end to end
  workspace.py              reachability / finger-tilt table per bottle and riser height
  urdf.py                   URDF export + check against the MuJoCo model
models/sim/reach_ppo.zip    the trained reach policy (committed, so nobody has to retrain)
```

**The model:** table → base `[shoulder_pan]` → upper arm `[shoulder_lift]` → forearm `[elbow_flex]` →
SO-101 gripper body + fixed jaw `[wrist_roll]` → hinged moving jaw `[gripper]`. Your STLs are the
**visuals**: `link_1`/`link_2` (150 / 151 mm), elbow, base ring, the SO-101 wrist-roll follower and moving
jaw. **Collisions** use boxes fitted to the real jaw faces (a mesh collides as its convex hull, which
would make a jaw a solid blob). The SO-101 jaw hinge comes from TheRobotStudio's SO-101 URDF; your STLs
are in the same frames, in mm. The moving jaw carries a **5×5 grid of touch sensors** that outputs kPa
per taxel like the SensX25 (force ÷ taxel area). Servos are position-controlled with the XL330's real stall
torque (0.52 N·m; 4:1 planetary on the shoulder), so a joint that's too weak *is* too weak in the sim.

**The cap** is a small machine on top of the physics:
- **thread:** it takes `thread_torque` (0.08–0.30 N·m, randomized) to turn;
- **child-proof:** it's locked until pushed down `push_unlock_mm` (1.2–2.0 mm, against a 4 N/mm spring);
- **release:** after `twist_deg` it's free. It still sits around the neck until lifted 8 mm, then can be carried away.

## 3. Results (SO-101 gripper, your arm's dimensions)

| Stage | Result in the sim |
|---|---|
| 1. Reach (trained policy, 100 random goals) | median **3.8 mm**, 71% within 5 mm, 93% within 10 mm |
| 2. Find the cap (wrist camera, dark mat under the clip) | median **1.9 mm** error |
| 3. Approach | a few mm (policy), then IK poses from the detected cap for the precise part |
| 4. Open, amber 40 dr vial, 16 randomized caps each: | |
| — today's plan: XL330 gripper servo + palm pusher | 3 / 16 (9 / 16 with `slip_ratio: 0.3`) |
| — **STS3215-class gripper servo + palm pusher** | **14 / 16** — both failures are clips < 10 cm from the base |
| — STS3215-class, no pusher | 10 / 16 |

All "open" runs use the coordinated twist. Wrist-only twisting fails even on easy caps (finding 1).

### Whole task: pick from a fixed spot → place in the clip → clamp → unscrew

`python -m pillbot.sim.demo --recommended --video outputs/sim.gif` runs it end to end, starting from the
pick spot (`fixture.pick_xy`). `--start clip` skips straight to a bottle already clamped. Stages 2–4
run the team's real `pick_bottle` / `place_in_clip` and `Fixture.clamp()`. A `SimHub` stands in for
the ESP32 hub: its clamp servo drives the sim clamp, and its clip ultrasonic reports "seated".

What it took to make picking and placing work at all:
- **Pick by the cap, not the body.** A palm pusher is in the way of a body grip, and a cap grip leaves
  room above the clamp jaws.
- **Base riser ~8 cm (now the default `arm.base_riser: 0.08`; the clip and pick spot stay on the table) and
  `tool_roll_deg: 90`.** Without the riser the arm can't get above the clip.
  Without the tool roll, the hinged jaw opens into the riser.
- **Aim the bottle's base, not the gripper.** A bottle held by its cap hangs tilted, so placement solves
  IK for a `held_base` site.
- **Keep the clamp jaws low (≤ ~38 mm above the clip floor).** The SO-101 fingertips reach ~23 mm below
  the cap grip. Taller jaws catch them, and opening fell to 0/8.
- **Couple the clamp jaws (rack and pinion)** so the clamp centres the bottle as it closes.

**Slide into a flush clip (current default, `place.mode: slide` in robot.yaml).** The real clip sits
flush with the table (`fixture.clip_floor: 0`), so the arm no longer carries the bottle up and over the
jaws. It grips the cap, lifts only a few mm (`place.skim`, 6% of the way to the hover pose), slides the
bottle across into the OPEN clamp, lets go 4 mm up (1 mm was worse); the coupled rack-and-pinion jaws
centre it. The pick spot and the clip are both 12 cm from the base axis, so the slide is mostly a
`shoulder_pan` sweep (no lifting load on shoulder_lift, no swinging bottle). `place.mode: lift` keeps
the old up-and-over placement.

**Results (10 randomized episodes each — small samples, treat ±2 as noise — amber 40 dr, pusher, 8 cm riser, gripper 1750 mA, slip_ratio 0.3):**

| Placement | Opened | Not seated | Other failures (after placing) |
|---|---|---|---|
| lift (old) | 2/10 | 4/10 | cap slipping / still threaded / not seen |
| slide (new) | **4/10** | **2/10** | cap slipping / missed / not seen |

Opening a bottle that's already clamped (`--start clip`, same settings) is 11/16. Things tried that did
**not** help: tighter side guides (1/16), and a 45° funnel around a raised clip floor (0/16: the held
bottle tips onto it). The funnel is still in the code as `fixture.pocket_slope_deg` (only used with
`clip_floor > 0`).

**Other sweeps (16 episodes, `--start clip`):** gripper current 1000 → 1750 mA: 10 → 11/16;
`slip_ratio` 0.3 vs 0.5: no difference. Cap/pad friction 0.8–1.2 vs 1.5–2.0: no difference (5/6 both),
so failures are the twist, not slip. Without a riser, the arm can't get above the clip at all.
The OTC and multivitamin bottles open 0/12 with every setting tried (press on/off, 3× gripper torque,
more friction): the large-cap model with the SO-101 jaw is probably wrong, so test those on the real arm.

## 4. What the sim found (act on these)

1. **The cap sits ~12 mm off the wrist axis: twist with the arm, not just the wrist.** The SO-101 grips
   against its fixed jaw, which is off-centre, so a 43 mm cap's centre is ~12 mm from the wrist_roll
   axis. Turning only the wrist swings the cap in a circle, so the jaws scrub instead of unscrewing.
   `sim/kinematics.py orbit_twist` turns the wrist in 3° steps and re-solves shoulder/elbow each step
   to keep the grasp point fixed (drift ~2 mm). `open_cap(..., twist=orbit_twist(world, io))` uses it.
   **The real robot needs this too:** the MuJoCo model is its kinematics (mujoco runs on the Pi). This
   needs `joint_map` calibrated (§5).
2. **Use an STS3215-class servo for the gripper, not an XL330.** The SO-101 jaw is a lever ~55 mm long
   from its hinge. An XL330 at 1000 mA gives ~0.3 N·m → ~5 N at the cap, which is not enough for caps
   above ~0.1 N·m: 3/16 opened. With ~3× the torque (the STS3215 the SO-101 is designed for): 14/16.
3. **Add a palm pusher.** This is a printed block between the jaws whose bottom face meets the cap's top.
   Without it the child-proof push rides on jaw friction, the same friction the twist uses. The push
   fades mid-twist and the cap re-locks at ~50°. With the pusher: 10/16 → 14/16.
4. **Base height: a ~4–8 cm riser.** With the base as estimated from your STLs (shoulder axis ~17 cm up),
   the vial at 12 cm tilts 15° and tall bottles 29–44° (`workspace`). On 4 cm, vials are within 1–4°
   at 10–12 cm. On 8 cm, every bottle profile works (OTC 2–4°, multivitamin 7–11°). Keep the clip ≥ 10.5 cm
   from the base axis, or there's no room to hover above the cap.
5. **Mount the wrist camera looking down the jaws, beside the gripper body.** A camera behind the jaws
   can't see the cap: with no wrist pitch, the elbow runs out of travel before it gets far enough back.
   The sim's camera is at `wrist_camera.offset` (beside the body, over the grip line).
6. **Put a dark mat under the clip.** A white cap on a light table fools the background subtraction
   (8 mm error → 1.9 mm with the mat).
7. **`cap_press` ~6 mm below `cap_grip`,** not "a few mm" (the arm and jaws give a little).
8. **The opener now checks its own success.** It used to count *commanded* wrist rotation and report
   `cap_lifted` without checking. After twisting, it lifts and checks that the arm got there *and* the
   pad still feels the cap. If not, it twists again (`open.max_extra_twists`).
9. **Slip alarm with the hinged jaw:** the pad pressure shifts a lot between grip and push, so
   `slip_ratio: 0.5` raises false alarms with a weak gripper. 0.3 was better (3 → 9/16); with a strong
   gripper it made no difference.

## 5. Moving to the real robot (checklist)

- [ ] Measure the arm into `configs/sim.yaml`: shoulder height, joint-to-joint lengths, riser, servo
      stiffness. Your STLs are an exploded assembly, so axis positions are estimates. Then retrain
      (`train reach`, ~5 min) and re-run `workspace`.
- [ ] Fill in `joint_map` in `sim.yaml` (real LeRobot degrees ↔ sim degrees). Put the real arm in two
      poses, read the angles with `teach`, and match them. `orbit_twist` and the trained policy both depend on it.
- [ ] Calibrate the wrist camera (intrinsics + where it sits on the hand).
- [ ] Compare the tactile scale: press the real pad with a known weight in `tactile_live` and compare with
      `sim.yaml tactile.taxel_pitch` (sim kPa = force / taxel area).
- [ ] Measure a real cap's opening torque and push force. If they're outside `sim.yaml cap:` ranges,
      widen them and re-run `demo --episodes 16`.
- [ ] First real runs: low speed, someone on STOP.

## 6. Limitations

- Pads are rigid boxes; real TPU/silicone pads conform (real grip is probably *better*).
- The thread is a friction torque, not geometry; the arm's servos have no backlash.
- The base/shoulder STLs are an exploded assembly and `BASE + MAINFRAME` is exported Y-up, so the
  shoulder height (0.17 m) is an estimate: measure it.
- Friction needed care: MuJoCo's default friction creeps under steady load. The model uses the no-slip
  solver with 100 iterations; fewer under-transmits grip torque ~2× (measured).
- Tall OTC / multivitamin bottles haven't been re-run with the SO-101 gripper; use an 8 cm riser first.

## 7. ROS

**Short answer: you don't need ROS for this, and it can't run on this Mac.** ROS 2 has no practical macOS
support (and there's no Docker here). LeRobot, which drives your arm, doesn't use ROS, and MuJoCo already
does the physics. Adding ROS mid-hackathon means Ubuntu, colcon packages and controller plugins, for no new
capability here.

If you want it anyway (e.g. RViz to visualize, MoveIt for motion planning), the robot is ready:
`hardware/urdf/pillbot.urdf` is generated from the same config and checked against the MuJoCo
model (grasp point within 0.04 mm over 50 random poses). Same joint names as LeRobot.
- **Where:** Ubuntu 24.04 + ROS 2 Jazzy, on a laptop/VM or **the Pi 5** (Ubuntu Server 24.04 for Pi).
- **See it:** make a ROS package, copy `hardware/stl` into it, change `../stl/` paths in the URDF to
  `package://<pkg>/stl/`, then `ros2 launch urdf_tutorial display.launch.py model:=<path>/pillbot.urdf`.
- **Plan motions:** MoveIt 2 Setup Assistant on the URDF (5-DOF: use position-only IK goals).
- **Physics:** Gazebo (gz sim) can load it with `ros2_control` tags added. Expect a day of setup, and
  contact/friction for the cap twist is weaker than MuJoCo's.
