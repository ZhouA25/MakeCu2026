# Arm: danslovich/5DOF_Robot_Arm (Dynamixel XL330-M288)

Source: https://github.com/danslovich/5DOF_Robot_Arm (Fusion 360 + STLs). We drive it with LeRobot
through our plugin (`plugins/lerobot_robot_pillbot`, type `pillbot_koch_follower`, preset `all_m288`).

## Joints (our motor IDs — set them with `bash scripts/setup_motors.sh`)
| ID | Joint (LeRobot name) | Drive | Notes |
|---|---|---|---|
| 1 | shoulder_pan  | base, direct | full-turn joint in calibration |
| 2 | shoulder_lift | **4:1 printed planetary** (`planetary_gear_box`, `Sun_gear`, `planet_gears`) | ~4x torque, 1/4 speed; motor turns >1 rev — plugin clamps calibration limits |
| 3 | elbow_flex    | direct (`elbow_coupler`) | **weakest joint under load** — see below |
| 4 | wrist_roll    | gear pair (`wrist_gear_1/2`) | no wrist pitch on this arm |
| 5 | gripper       | direct (`hand`, `eef`) | current-limited by LeRobot → can't crush |

The original repo's Arduino sketches number the servos differently (and run them at 57600 baud);
ignore those — `lerobot-setup-motors` assigns IDs in the order above and sets LeRobot's baud rate.

## Torque budget (rough, arm fully stretched out horizontally, empty bottle)
| Joint | Load moment | Available | Verdict |
|---|---|---|---|
| shoulder_lift | ~0.35–0.4 N·m | 0.52 N·m × 4 × ~0.8 gear eff. ≈ 1.6 N·m stall (≈0.3 continuous) | OK when bent; marginal fully stretched |
| elbow_flex | ~0.15–0.2 N·m with forearm horizontal | 0.52 N·m stall, ~0.10 continuous | **Overloaded if forearm is horizontal** |

Rules that keep it working:
1. **Put the start zone, clip and cap tray close to the base** (~10–18 cm), so the arm stays folded.
2. **Grasp top-down with the forearm near vertical.** With no wrist pitch the hand points along the
   forearm anyway, and a vertical forearm puts almost no gravity load on the elbow.
3. Rubber band / spring from the upper arm to the forearm if the elbow sags; tune with torque off
   until the forearm roughly floats.
4. Run the XL330s at 5–6 V (max 6 V!). Low-infill links. Empty bottles.
5. Twisting caps is done by the goBILDA turntable, not the wrist (XL330 can't hold cap torque).

## Wrist camera
ESP32-CAM (`firmware/esp32_cam`) on the `hand`, above the jaws, looking down them. ~10 g at the end of the
arm adds a little elbow load — another reason to keep the forearm vertical. Only 5 V + GND cross the
wrist-roll joint, so leave a loose loop of those two wires.

## Things to check on the bench
- **Gripper opening vs bottle size.** The stock `eef` finger is 18 mm wide × 63 mm long on a 46 mm
  hand — confirm it opens wider than your bottle body (40–55 mm). If not, grab the narrower cap /
  neck, or print longer fingers. Leave a flat 18 mm+ face for the SensX25.
- **Planetary backlash** on the shoulder: if poses don't repeat, re-teach approach poses from the
  same direction every time.
- **USB-to-Dynamixel interface:** LeRobot needs a USB serial Dynamixel adapter (U2D2 or a Waveshare
  serial bus servo board). The original MKR Dynamixel shield can work as a pass-through only if you
  flash it with a USB-to-Dynamixel bridge sketch.
