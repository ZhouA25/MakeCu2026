# Wiring

## System
```
                         USB                      USB                 USB (UVC)
 Pi 5 / laptop ───────── SO-101 bus board    ──── SensX adapter       endoscope (wrist cam)
      │  USB                                                          phone / laptop cam
      └──────────── ESP32 #1 "PillBot Hub"
                        │ GPIO ─ 2× HC-SR04, 3× push buttons, relay
                        │ PWM ── turntable servo, clamp servo, tray servo, spare
```

## Power — three separate rails, ONE common ground
| Rail | Source | Feeds |
|---|---|---|
| Arm | Facmogu #1 (5V or 12V to match your STS3215s) | SO-101 bus board only |
| Hobby servos | Facmogu #2 at **6.0V** (7.4V for max torque on goBILDA/D646WP; NOT for SG90/MG90S) → **through the relay** | goBILDA ×2, D646WP, MG90S, SG90 — add 1000 µF across the rail |
| Logic | ELEGOO breadboard module (5V) | HC-SR04 ×2 |
| ESP32 | USB from Pi/laptop | ESP32 only |

Tie all grounds together (ESP32 GND, both supplies' negative, breadboard module GND).
Servo signal wires go to ESP32 pins; servo power does **not**.

## ESP32 #1 pin map (matches `firmware/esp32_hub/esp32_hub.ino`)
| Pin | Connects to | Notes |
|---|---|---|
| 4 | HC-SR04 #1 (clip) TRIG | |
| 34 | HC-SR04 #1 ECHO **via 1k/2k divider** | input-only pin, fine for echo |
| 16 | HC-SR04 #2 (start zone) TRIG | |
| 35 | HC-SR04 #2 ECHO **via divider** | input-only pin |
| 32 | START button → GND | internal pull-up, pressed = LOW |
| 33 | STOP button → GND | firmware cuts relay + servos instantly |
| 27 | MODE button → GND | |
| 25 | Servo ch0: **turntable** (goBILDA #1) | 500–2500 µs = 300° |
| 26 | Servo ch1: **clamp** (goBILDA #2 or D646WP) | |
| 13 | Servo ch2: cap tray / dispenser (MG90S) | |
| 14 | Servo ch3: spare | |
| 17 | Relay IN (hobby-servo power) | set RELAY_ACTIVE_LOW in firmware |
| 2 | Onboard LED (status) | |

Avoid GPIO 0, 2 (as input), 12, 15 for external parts — they're boot strapping pins.

## Gripper cable bundle (the wrist rolls a lot!)
SensX flex/USB, endoscope USB. Give them a loose service loop around the
wrist-roll joint, spiral wrap, and strain-relieve both ends. Test a full ±150° wrist roll before running.

## Raspberry Pi notes
- Pi 5 needs a 5V/5A supply or it throttles USB current (cameras + ESP32 + bus board are all USB).
- Use a powered USB hub if devices drop out.
- Phone camera on the Pi: Continuity Camera is Mac-only — use DroidCam/Iriun over USB, or just use the
  endoscope + a USB cam on the Pi. Simplest for the hackathon: run everything on the laptop.
