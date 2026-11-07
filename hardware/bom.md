# Bill of materials — what we have and what each part does

## Parts on hand
| Part | Qty | Role in PillBot | Notes |
|---|---|---|---|
| Raspberry Pi 5 (4GB) | 2 | **Pi A**: runs `pillbot.run` + LeRobot control. **Pi B**: spare / runs camera + UI, or a second station | Each needs a 5V **5A** USB-C supply (not in list). Develop on a laptop first. Training stays on a GPU (Colab/cloud). |
| ESP32 ESP-WROOM-32 | 3 | **#1 = PillBot Hub** (ultrasonics, buttons, servos, relay — `firmware/esp32_hub`). #2, #3 = spares | Talks to Pi/laptop over USB serial |
| TouchTronix SNX0505-SNS-01 (SensX 25, 5×5 FSR) | 1 | **Tactile pad on the gripper finger (`eef`) that presses the cap** — grip-by-feel + slip detection while the wrist twists | UART 921600 baud via its USB adapter, `sensx` Python API (git submodule). Other jaw: plain TPU pad |
| **ESP32-CAM** (AI-Thinker, OV2640) | 1 | **Wrist camera** — streams over Wi-Fi, so only 5 V + GND run through the wrist; flash LED lights the cap. `firmware/esp32_cam` | ~10 g. Needs the ESP32-CAM-MB USB board or an FTDI adapter to flash. 5 V supply that can do ~300 mA peaks |
| Endoscope inspection camera (optional) | 1 | **Wrist camera!** Tiny + light + built-in LEDs; mount on the gripper looking down the jaws | Must be a **USB (UVC)** model, not Wi-Fi-only. Usually fixed focus ~3–8 cm: mount so the cap is sharp at grasp |
| goBILDA 2000 Series Dual Mode servo (25-2) | 2 | **#1 clamp on the clip** — locks the bottle body while the arm twists the cap. #2: spare | 25 kg·cm @7.4V, stall 2.5–3 A. Position mode (default) |
| Hitec D646WP servo | 1 | Clip clamp (if goBILDA #2 is used elsewhere) or spare | ~12 kg·cm @7.4V, IP67 |
| MG90S | 3 | Cap tray door / pill dispenser gate / status flag | Metal gear micro servos |
| SG90 | 2 | Light duty: camera tilt, indicator flag | |
| S3004 | 1 | Spare | |
| HC-SR04 ultrasonic | 2 | #1 **bottle seated in clip**, #2 **bottle placed in start zone** (auto-start) | 5V part — ECHO needs a 1k/2k divider to 3.3V for the ESP32 |
| DC 5V relay module | 1 | **Cuts power to the hobby-servo rail** on STOP/fault (hardware kill) | Check if it's active-LOW and triggers from 3.3V |
| Facmogu adjustable DC power supply | 2 | **#1 → SO-101 servo bus** (5V or 12V, match your STS3215 version). **#2 → hobby servo rail at 6.0–7.4V** | Set current limit; goBILDA stall = 2.5–3 A each |
| ELEGOO breadboard power module | 2 | 5V logic rail for the HC-SR04s | ~700 mA max — **never** power servos from these |

## Out of stock — replaced
| Wanted | Replacement |
|---|---|
| Touch buttons ×3 | **Tactile push buttons** (CURC "Tactile Push Buttons (Assorted)") or Grove Buttons — wired pin → button → GND. Use the biggest you can find (or a 3D-printed large cap) for STOP |
| Muscle sensor (EMG trigger) | Hands-free start with the **start-zone ultrasonic** (`--trigger zone`): just set the bottle down |
| Thin film pressure sensor | Not replaced — one SensX25 on the moving finger is enough; the other jaw gets a grippy TPU pad. The Dynamixel gripper is current-limited, so it can't crush even without sensing |
| Pseudo-LiDAR, CN0023 | Not needed |

## Not in the list — confirm or get
- **SO-101 follower + leader arms** (with their Feetech bus driver boards + PSU) — the whole project depends on them
- Pi 5 27 W USB-C supplies ×2, microSD cards ×2
- **SensX USB/UART adapter** (connector module — check it came with the sensor)
- 1000 µF capacitor across the hobby-servo rail, resistors (1k/2k dividers ×2 for the HC-SR04 echoes)
- Servo extension leads, jumper wires, Wago/terminal blocks, zip ties, spiral wrap for the wrist cable bundle
- PLA + TPU filament (clip + clamp jaws, gripper pads, camera mount)
- Empty pill bottles (amber child-proof + OTC) — no real meds
