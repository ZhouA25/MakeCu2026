# Architecture

## Physical setup
```
     phone on tripod (~45° down, opposite the arm)      laptop webcam (different angle)
                         \                                   /
  ------------------------------------------------------------------
  | taped START ZONE (~20x20cm) |  FUNNEL CLIP  |  cap tray |
  ------------------------------------------------------------------
                         [ SO-101 follower + cap-gripper pads ]
```
No wrist camera: the clip fixes the cap position, so the cap re-grip is a taught pose.
Vision only finds the bottle in the zone (pixel -> zone homography, then joint-space
bilinear interpolation between 4 taught corner poses — no IK). Grasps are confirmed by
tactile pressure, or by gripper stall when no sensor is fitted.

**Key design decision:** one arm can't hold the bottle body *and* twist the cap.
Options (pick one):
1. **Funnel clip (chosen):** a 3D-printed cup with a flared entry (forgives 5–10 mm placement
   error), a floor (pushing the cap pushes the bottle *in*), and TPU ribs/foam so the body can't spin.
   Upgrade: a servo-actuated clamp if the bottle still spins. Blue tape on the bottle body +
   `--spin-check` detects spinning from the camera.
2. **Bimanual:** two SO-101 followers — one holds, one twists. More impressive, double the work.
3. **Motorized cap head:** a small extra servo/geared motor in the attachment spins the cap while
   the arm holds the bottle. Frees the wrist from the regrip loop.

## Cap-gripper attachment (hardware/cad)
- Replace/augment the stock jaw with **concave, TPU-lined pads** sized for ~35–55 mm caps
  (or swappable pad sets per bottle size).
- Child-proof caps need **downward force while twisting** — pads should grip the cap's side wall,
  and the arm presses down via elbow/wrist flex.
- Optional: FSR pressure sensor in the pad (firmware/force_sensor) so we know when we're gripping.

## Software pipeline
1. `perception/detect_bottle.py` — locate bottle in overhead cam.
2. Pick & place into fixture — scripted (taught poses) or learned (ACT policy via LeRobot).
3. `control/open_sequence.py` — grip → press → twist CCW → release → re-grip → … → lift cap.
4. Drop cap in tray, return home. (Stretch: dispense one pill / hand bottle to user.)

## Cap types
| type | motion |
|---|---|
| push_and_turn (child-proof) | press down 2–4 mm while twisting CCW |
| twist | twist CCW only |
| snap / flip_top | pry up — probably out of scope |
