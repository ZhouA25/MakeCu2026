# Architecture

## Physical setup
```
     fixed camera on tripod (~45° down, opposite the arm)
                         \
  ------------------------------------------------------------------
  | taped START ZONE (~20x20cm) |  FUNNEL CLIP + CLAMP  |  cap tray |      all close to the base
  ------------------------------------------------------------------
        [ 5-DOF XL330 arm (danslovich/5DOF_Robot_Arm) — ESP32-CAM + SensX25 on the hand ]
```
- **Vision:** the fixed camera finds the bottle in the zone (pixel -> zone homography, then joint-space
  bilinear interpolation between 4 taught corner poses — no IK). The ESP32-CAM on the wrist looks down
  the jaws (cap alignment, grip check, demo footage).
- **Touch:** grasps are confirmed by the SensX25 pressure, or by gripper stall without it.

**Key design decision:** one arm can't hold the bottle body *and* twist the cap, so the fixture holds it:
a 3D-printed **funnel clip** (flared entry forgives 5–10 mm of placement error; a floor, so pushing the
cap pushes the bottle *in*) with a **goBILDA servo clamp** that locks the body. The arm stays
general-purpose: it grips the cap, pushes down and twists with `wrist_roll`. Blue tape on the bottle
+ `--spin-check` detects the bottle spinning in the clamp.

## Gripper
- Jaws shaped/lined (TPU, ridges) to grip the cap's side wall; SensX25 on the moving finger.
- Child-proof caps need **downward force while twisting** — the arm presses via the `cap_press` pose.
- Gripper runs current-limited (LeRobot `current_based_position`); `gripper_goal_current` sets the
  squeeze — the cap turns only as long as friction beats the cap's torque.

## Software pipeline
1. `perception/detect_bottle.py` — locate bottle in overhead cam.
2. Pick & place into fixture — scripted (taught poses) or learned (ACT policy via LeRobot).
3. Clamp the bottle (`control/fixture.py`).
4. `control/open_sequence.py` — grip → push down → twist → release → unwind wrist → … → lift cap.
5. Unclamp, drop cap in tray, return home. (Stretch: dispense one pill / hand bottle to user.)

## Cap types
| type | motion |
|---|---|
| push_and_turn (child-proof) | press down 2–4 mm while twisting CCW |
| twist | twist CCW only |
| snap / flip_top | pry up — probably out of scope |
