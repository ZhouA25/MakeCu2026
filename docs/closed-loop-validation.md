# Recorded gripping validation and sim-to-real handoff

The user approved the revised shoulder/elbow assembly on 2026-11-08. Validation here is simulation-only. The current approved preview retains the SO-101 **hinged** gripper: `gripper` is an independently actuated revolute joint, with nominal -10 to +100 degree jaw travel (commanded as 0–100 gripper units). It is not a rigid end effector or a parallel slider.

## Run a recorded test

From the repository root in PowerShell:

```powershell
$env:PYTHONPATH='src'
.venv\Scripts\python.exe -m pillbot.sim.validate_grip --mode jaw
.venv\Scripts\python.exe -m pillbot.sim.validate_grip --mode grip --seed 0 --episodes 5
```

Every trial creates a unique directory under `D:/MakeCu2026/outputs/closed_loop/`. MP4 recording starts before motion and covers direct settling steps as well as control ticks. Failed trials are preserved. Completed MP4s are checked for readable first/last frames and frame count. Each directory includes synchronized joint/command/tactile/contact/bottle telemetry, result/configuration metadata, and phase images. Later runs also capture model XML and source snapshots for reproducibility. Early exploratory runs predate those additional snapshots; their videos and telemetry remain available.

The full `sim.demo` entry point and learned-policy evaluation also now create an MP4 for each episode. Optional `--video` in the old demo produces an additional first-episode GIF; it does not replace the mandatory MP4s. Those wider opening/policy workflows have not been dynamically revalidated in this session.

## Control and scoring

- Controller uses encoder-derived forward kinematics with a bounded correction for servo deflection, and the same `control/grasp.py` tactile close routine used by the hardware path.
- Uses a known bench position, approximately `(195, 65)` mm from the base, with ±1.5 mm object-position variation and ±0.6 mm targeting variation. Camera detection is **not** validated by this benchmark.
- Bottle body mass varies 25–45 g, plus the configured 10 g cap. Pad friction varies over the existing 0.8–1.2 range. This is a small local test envelope, not general robustness evidence.
- Approach follows the four-positioning-DOF arm's near-vertical path, leaves 5 mm clearance at the fixed finger, and seats below the cap rim so the moving sensor contacts the side rather than the lid top.
- Tactile threshold is 100 summed kPa and target pressure 200 in this **simulation benchmark only**; these values require real sensor calibration. They do not overwrite the hardware's default tactile settings.
- Lift request is 25 mm with a 3-second feedback hold; set-down releases near the table without forcing the position controller through the tabletop.
- Pass criteria: minimum hold height >15 mm, tilt <15 degrees, tactile contact for ≥90% of hold samples, relative bottle/grasp drift <5 mm, then upright release with bottom height within 4 mm of the table. Bottle ground truth is used for scoring, not motion control.

The original grasp code could report success after a transient touch followed by lost contact. It now checks the post-squeeze pressure and uses one consistent pressure sample for each decision. Regression tests cover lost contact, insufficient pressure and sustained contact.

## Motor IDs supplied by the user

| Joint | Motor ID | Status |
|---|---:|---|
| `wrist_roll` | 15 | User supplied |
| `gripper` (end effector) | 10 | User supplied |
| `shoulder_lift` | 12 | User supplied as “shoulder” |
| `shoulder_pan` | 1 | Previous default; not physically confirmed |
| `elbow_flex` | 3 | Previous default; not physically confirmed |

`configs/robot.yaml` stores the three explicit overrides. The LeRobot plugin accepts `motor_ids`, preserves unspecified defaults, and rejects duplicate/unknown mappings. CLI argument generation preserves this mapping. No motor IDs were written to devices and no hardware connection was opened. Plugin mapping logic is unit-tested; the complete LeRobot runtime is not installed/validated in this review environment.

## Remaining sim-to-real work

Calibrate joint signs/zeros, confirm remaining IDs and interfaces, measure motor load limits and tactile scale, and validate camera calibration. Collision geometry, inertias, drivetrain seating and wrist adapter are still simplified/unverified. A successful simulated grasp does not establish hardware readiness or child-resistant cap-opening success.

The old reach checkpoint was trained for different geometry. A model fingerprint now rejects checkpoints lacking matching geometry metadata; retraining writes that metadata. No new learned policy has been trained in this session. The validated grip controller uses feedback IK.

Latest run results and videos are indexed in `outputs/closed_loop/validation-summary.md`.
