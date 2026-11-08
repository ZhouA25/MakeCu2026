# STL files

All in **millimetres**. Renamed from the team's shared zip (original names in the last column).
The simulation loads some of them as visuals (`src/pillbot/sim/scene.py`, `docs/simulation.md`).

> Assembly correction (2026-11-08): see `docs/assembly-correction.md`. The original bounding-box lengths below are not joint-to-joint lengths. Shoulder brackets and drive plate are now used, and elbow halves belong to separate moving bodies.

## `arm/` — danslovich/5DOF_Robot_Arm parts
Exported in the arm's (exploded) assembly frame, standing straight up, so most of them line up as a
vertical arm when loaded together.

| File | Part | Size (mm) | Used in sim |
|---|---|---|---|
| `base.stl`, `base_plate.stl` | round base + bottom plate | Ø140 × 51, Ø140 × 5 | visual |
| `drive_plate.stl` | pan drive plate | 72 × 72 × 14 | |
| `planetary_gear_box.stl`, `sun_gear.stl`, `planet_gears.stl`, `planet_gear_5mm_hole.stl` | 4:1 planetary (shoulder) | Ø73 × 17 | |
| `shoulder_1.stl`, `shoulder_2.stl` | shoulder bracket halves | 73 × 73 × 85 | |
| `link_1.stl` | upper arm | 47 × 62 × **150** | visual |
| `elbow_1.stl`, `elbow_2.stl`, `elbow_coupler.stl` | elbow housing + coupler | 56 × 48 × 63 | visual (1, 2) |
| `link_2.stl` | forearm | 47 × 49 × **151** | visual |
| `wrist_gear_1.stl`, `wrist_gear_2.stl` | wrist-roll gear pair | Ø26, Ø30 | visual (1) |
| `hand_plate.stl`, `hand.stl`, `eef.stl` | original hand + finger | 46 × 46, 18 × 63 | — replaced by the SO-101 gripper |

## `gripper_so101/` — SO-101 gripper (TheRobotStudio/SO-ARM100)
Same frames as the SO-101 URDF meshes, in mm. The sim places them with the URDF's joint frames.

| File | Part | Size (mm) | Original name |
|---|---|---|---|
| `wrist_roll_follower_so101.stl` | gripper body + **fixed jaw** (fingers 6–105 mm out along the wrist axis) | 62 × 85 × 99 | `Wrist_Roll_Follower_SO101 Copy.stl` |
| `moving_jaw_so101.stl` | **hinged jaw** (83 mm from hinge to tip; opens −10°…+100°) | 23 × 91 × 47 | `Moving_Jaw_SO101.stl` |

## `base/`
| File | Part | Size (mm) | Original name |
|---|---|---|---|
| `base_mainframe.stl` | new base ring + mainframe, one piece (exported Y-up) | 140 × 59.5 × 194 | `BASE + MAINFRAME stuck together.STL` |

## `clamp/` — rack-and-pinion clamp (bottle fixture?)
| File | Qty | Size (mm) | Original name |
|---|---|---|---|
| `clamp_jaw_RB9.01.062.021_x2.stl` | **2** | 46 × 80 × 16 | `NEED 2 RB9.01.062.021 (D6.3) Clamp.STL` |
| `gear_rack_RB9.01.062.030.stl` | 1 | 4.5 × 9.4 × 61 | `RB9.01.062.030 Gear rack.STL` |
| `gear_rack_RB9.01.062.031.stl` | 1 | 4.5 × 9.4 × 61 | `RB9.01.062.031 Gear rack.STL` |
| `nail_RB9.01.062.100_x2.stl` | **2** | 4.2 × 5 × 9 | `NEED 2 RB9.01.062.100 Nail.STL` |
