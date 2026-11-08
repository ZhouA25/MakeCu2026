# Pill Bottle Opener Robot: Design Review & Simulation Summary

**Project:** 5DOF Dynamixel XL-330 arm ([danslovich/5DOF_Robot_Arm](https://github.com/danslovich/5DOF_Robot_Arm)) + two Robonine parallel grippers ([SO-ARM100-101-102-Parallel-Gripper](https://github.com/roboninecom/SO-ARM100-101-102-Parallel-Gripper)). One gripper is the end effector. The other is fixed at the base and holds the bottle.

**Task:** Open a child-resistant (push-down-while-twisting) medicine bottle.

---

## 1. Hardware review

### Components
| Item | Key specs |
|---|---|
| Arm servos | 5× Dynamixel XL330-M288-T: 5 V, **0.52 N·m stall**, **~0.10 N·m rated continuous** (ROBOTIS: 20% of stall), 18 g, Protocol 2.0 |
| Arm controller (original) | Arduino MKR WiFi 1010 + MKR Dynamixel shield, 57600 baud |
| Gripper | Feetech STS3215 (12 V, 30 kg·cm), rack-and-pinion, 84 mm stroke, up to 120 N grip, ~170 g, ~$50 BOM |
| Gripper bus | Feetech protocol, RS485/TTL up to 1 Mbps |

### Findings from the CAD files
The STLs were exported in assembly coordinates, so the arm's layout could be reconstructed:
- **DXL_1 ("EEF") drives the original single jaw (`eef.stl`). It is not a wrist pitch.** With the Robonine gripper fitted, the arm has **4 positioning DOF**: base yaw, shoulder pitch, elbow pitch, and wrist roll.
- **The shoulder pitch is geared** through a planetary gearbox (`planetary_gear_box`, `Sun_gear`, `planet_gears`), estimated at **~4.3:1**.
- **The wrist roll** uses a crossed gear pair (`wrist_gear_1` → `wrist_gear_2`), about **1.14:1**.
- Base yaw and elbow are direct drive (1:1).
- Approximate geometry: shoulder axis ~188 mm above the table, upper arm ~160 mm, elbow to wrist face ~192 mm, gripper face to jaw centre ~100 mm (assumed).

### Integration issues
1. **Two separate buses.** The XL-330s (5 V, Dynamixel Protocol 2.0) and the STS3215s (12 V, Feetech protocol) can't be daisy-chained. You need a Feetech bus adapter, a 12 V supply, and a common ground. Simplest setup: one Python process on a PC, with a U2D2 for the Dynamixels and a Feetech USB board using the STServo SDK.
2. **No mounting interface.** Neither repo defines how the gripper attaches to the arm, so a custom adapter plate is needed between `hand_plate` and the gripper.
3. **Payload (revised).** I originally flagged the 170 g gripper as too heavy. That was wrong. Because the shoulder is geared, the gravity load on it is only ~0.07 N·m, and with the forearm vertical the elbow sees almost nothing. **The arm can carry the gripper.**

---

## 2. Simulation

### Method
- Analytical inverse kinematics (elbow-up) plus forward kinematics in Python.
- Quasi-static joint torques: gravity from point masses (PLA parts at 50% effective density, servos at 18 g, gripper at 170 g), plus the cap's reaction force and twist on the gripper.
- Each servo's load = joint torque ÷ gear ratio, compared with XL-330 rated and stall torque.
- Animated in 3D with matplotlib and exported as a GIF (160 frames).

### Scenario
- The bottle is fixed upright in the base gripper, 110 mm from the arm's yaw axis.
- Bottle body 34 mm Ø × 66 mm tall. Cap 38 mm Ø × 20 mm tall. Grasp point 76 mm above the table.
- **Assumed** cap loads: **20 N push-down** and **0.40 N·m unscrew torque**, with a 3.5 mm thread pitch. *These are not measured values.*
- Sequence: home → move above cap → descend → close gripper → **3 strokes of (push → push + twist 150° → release → open → unwind wrist → re-grip)** → threads free → lift 40 mm → swing aside with base yaw.

### Results

**Peak servo load during opening**

| Servo | Peak load | vs rated 0.10 | vs stall 0.52 |
|---|---|---|---|
| Base yaw (DXL5) | 0.40 N·m | ❌ 4× over | ⚠️ 77% |
| Shoulder (DXL4, 4.3:1) | 0.44 N·m | ❌ 4.4× over | ⚠️ 85% |
| Elbow (DXL3) | ~0.02 N·m (0.12 in transit) | ✅ | ✅ |
| Wrist roll (DXL2, 1.14:1) | 0.35 N·m | ❌ 3.5× over | ⚠️ 67% |

**What the arm can deliver at the grasp pose**

| Limit | Max push-down | Max twist |
|---|---|---|
| At stall (brief) | ~23 N | ~0.52 N·m |
| At rated (continuous) | ~7 N | ~0.10 N·m |

### Key conclusions
1. **The bottle can't sit on the same surface as the arm.** With no wrist pitch, twisting about the cap's vertical axis requires the forearm to point straight down. In that pose the gripper tip is only ~12 mm above the table (−5 to +28 mm across every geometry consistent with the CAD), while the cap is at ~76 mm. **Fix:** put the arm on a **~64 mm riser**, or set the bottle into a recessed fixture.
2. **The twist load ends up on the base yaw servo.** The cap's reaction is a twist about the vertical axis, which passes through the arm into the direct-drive base yaw servo. A stronger wrist servo alone doesn't fix this.
3. **Pushing down loads the shoulder**: push force × reach (0.11 m) ÷ 4.3. The elbow is barely loaded, because the push acts straight along the vertical forearm.
4. **It works briefly but not continuously.** With the assumed loads, the arm can open a cap near stall, but the yaw, shoulder, and wrist servos all run 3–4× over their rated torque and will overheat with repeated use.
5. **Twisting in strokes works kinematically.** While the cap climbs its thread, the forearm stays within ~1° of vertical, so the twist axis stays aligned with the cap. Strokes also keep the gripper cable from winding up.
6. **Choose the home pose carefully.** Holding the forearm far from vertical (e.g. 60° tilt) loads the elbow to ~0.39 N·m, close to stall. The simulation uses a home pose that hangs the forearm low (~0.12 N·m).

---

## 3. Recommendations
1. **Measure real cap loads** (push force with a luggage scale, twist with a torque wrench) and rerun the simulation with those numbers.
2. **Move the twist to the base:** put the base gripper on a rotary stage driven by an STS3215 in continuous-rotation mode (~2.9 N·m). The arm then only holds the cap, presses lightly, and lifts it off.
3. **Or upgrade the servos:** XC330-M288 (~0.93 N·m stall) as a drop-in for yaw and wrist, or gearing on the base yaw.
4. **Use the spare DXL_1** as a wrist pitch, mounted *before* the roll joint, so the arm isn't locked into the forearm-vertical pose and the riser is no longer needed.
5. **Build the riser or recessed fixture** (~64 mm for a bottle 110 mm from the base), plus the gripper adapter plate.
6. **Set up a dual-bus controller** on a PC (U2D2 + Feetech adapter, 12 V and 5 V supplies, common ground).

---

## 4. Deliverables
| File | Description |
|---|---|
| `pill_bottle_opening_sim.gif` | Animated simulation with a live per-servo load panel |
| `arm_model.py` | Geometry, masses, gear ratios, IK/FK, quasi-static joint loads |
| `simulate.py` | Scenario, trajectory, load summary, GIF rendering. Change `F_PUSH`, `T_TWIST`, `R_BOTTLE` at the top and rerun |

## 5. Limitations
- Joint positions are estimated from STL bounds (shoulder height ±30 mm). The gripper length and its centre of mass are assumed.
- The 4.3:1 shoulder ratio is inferred from gear dimensions, not tooth counts.
- The model is quasi-static: no acceleration, friction, link flex, or servo current-limit behaviour.
- The cap forces are assumptions until measured.

## Sources
- [danslovich/5DOF_Robot_Arm](https://github.com/danslovich/5DOF_Robot_Arm)
- [Robonine SO-ARM100/101/102 Parallel Gripper](https://github.com/roboninecom/SO-ARM100-101-102-Parallel-Gripper)
- [ROBOTIS XL330-M288 e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/)
- [XL330-M288-T rated torque note (OpenELAB)](https://openelab.io/a/s/products/dynamixel-xl330-m288-t)
- [Mecmesin: torque testing push-and-turn CRC closures](https://mecmesin.com/us/node/1725)
