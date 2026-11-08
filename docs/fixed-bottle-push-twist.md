# Fixed-ground bottle: push and twist — 2026-11-08

The user requested that the bottle be fixed relative to ground. `fixture.fixed_to_ground: true` now removes the bottle body's free joint entirely. Its world pose is constant, with zero translational/rotational DOFs; the cap retains its axial compression and rotation joints. The original free-bottle pickup scenario is unchanged by default.

Run (simulation only, always recorded):

```powershell
$env:PYTHONPATH='src'
.venv\Scripts\python.exe -m pillbot.sim.validate_twist --seed 0 --twist-degrees 90
```

The approved arm/gripper geometry, actuator limits and motor mapping are retained. No palm pusher, stronger servo or cap-motion animation was substituted. All motion is integrated through contact physics and actuator commands. The cap/thread mechanism remains the existing simplified spring/lock/friction model, not explicit screw threads.

## Recorded outcomes

Both attempts are in `outputs/closed_loop/validation-summary.md` and have verified local MP4s, telemetry, model XML, source snapshots and result metadata.

- Initial fixed-depth push: cap compressed 0.81 mm before twisting, below its 1.85 mm unlock threshold. Wrist rotated but cap turned only 0.009 degrees. This failure is retained.
- Compression-controlled push: cap compressed 1.857 mm before twist; remained unlocked for 100% of sampled twist steps; actual cap rotation 93.920 degrees for a 90-degree wrist command. Compression after twist 2.293 mm, peak during twist 2.642 mm. Bottle displacement exactly 0 mm; zero bottle-body DOFs. Grip was released and arm retreated. Cap remains threaded after this single stroke (catalog removal target is 270 degrees).

Successful video:
`D:/MakeCu2026/outputs/closed_loop/20261108T124616Z_fixed_push_twist_seed0_0a20aa/demo.mp4`

## Assumptions and feedback

Known bottle position `(180,65,0)` mm. Sampled cap friction torque 0.08364 N·m, unlock depth 1.8506 mm, axial spring 4000 N/m. These are model assumptions, not measured bottle loads. The controller regulates **simulated cap compression**, holds the grasp position laterally, and coordinates the arm with the off-axis wrist while twisting at approximately 15 degrees/second. Real transfer needs a calibrated compression/force observation; the simulation-only cap state is not currently available as a real sensor measurement.

Success means at least 70% of requested rotation occurred at the cap and compression stayed above the unlock threshold for at least 80% of twist samples. This is one successful 90-degree stroke, not proof of full unscrewing, broad robustness, real hardware operation, or motor thermal capability.

Validation: ten targeted tests passed, including static checks for zero bottle DOFs, retained cap slide/hinge joints, reset pose handling and unchanged free-bottle defaults. Every trial in this session has a local recording; the updated full index contains 25 videos.
