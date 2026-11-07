# Bill of materials — what we have and what each part does

## Parts on hand
| Part | Qty | Role in PillBot | Notes |
|---|---|---|---|
| Raspberry Pi 5 (4GB) | 2 | **Pi A**: runs `pillbot.run` + LeRobot control. **Pi B**: spare / runs camera + UI, or a second station | Each needs a 5V **5A** USB-C supply (not in list). Develop on a laptop first. Training stays on a GPU (Colab/cloud). |
| ESP32 ESP-WROOM-32 | 3 | **#1 = PillBot Hub** (all sensors, servos, relay — `firmware/esp32_hub`). #2 = pseudo-LiDAR / spare. #3 = spare | Talks to Pi/laptop over USB serial |
| TouchTronix SNX0505-SNS-01 (SensX 25, 5×5 FSR) | 1 | **Tactile pad on the moving gripper jaw** — grip-by-feel + slip detection | UART 921600 baud via its USB adapter, `sensx` Python API. Only one, so the other jaw gets the film sensor |
| Thin film pressure sensor | 1 | **Pad on the fixed jaw** (backup/second contact signal) | Voltage divider → ADS1115 A0 |
| ADS1115 16-bit ADC | 2 | #1: film pressure (A0) + muscle sensor (A1). #2: spare channels | ESP32's own ADC is noisy/non-linear — use these instead |
| Muscle sensor (EMG) | 1 | **Accessibility trigger**: user flexes forearm → robot opens the bottle | ADS1115 A1. Tune threshold per person |
| Endoscope inspection camera | 1 | **Wrist camera!** Tiny + light + built-in LEDs; mount on the gripper looking down the jaws | Must be a **USB (UVC)** model, not Wi-Fi-only. Usually fixed focus ~3–8 cm: mount so the cap is sharp at grasp |
| goBILDA 2000 Series Dual Mode servo (25-2) | 2 | **#1 turntable under the clip — twists the BOTTLE** while the arm holds the cap. #2: clamp jaws on the clip | 25 kg·cm @7.4V, 300° range, stall 2.5–3 A each. Continuous mode needs the goBILDA programmer (we use position mode) |
| Hitec D646WP servo | 1 | Clip clamp (if goBILDA #2 is used elsewhere) or spare | ~12 kg·cm @7.4V, IP67 |
| CN0023 | 1 | Assumed **SunFounder 15 kg servo** (DigiKey CN0023) → spare clamp/tray servo | If it's actually the Chip Quik microSD adapter, ignore |
| MG90S | 3 | Cap tray door / pill dispenser gate / status flag | Metal gear micro servos |
| SG90 | 2 | Light duty: camera tilt, indicator flag | |
| S3004 | 1 | Spare | |
| HC-SR04 ultrasonic | 2 | #1 **bottle seated in clip**, #2 **bottle placed in start zone** (auto-start) | 5V part — ECHO needs a 1k/2k divider to 3.3V for the ESP32 |
| Pseudo-LiDAR sensor | 1 | Mounted above the clip: measure bottle/cap height → auto-pick bottle profile; or zone presence | Model unknown — tell us the part # (likely ToF, UART/I2C) |
| Touch buttons (TTP223-style) | 3 | **START**, **STOP**, **MODE/bottle select** — big, easy-press targets for low-dexterity users | 3.3V OK |
| DC 5V relay module | 1 | **Cuts power to the hobby-servo rail** on STOP/fault (hardware kill) | Check if it's active-LOW and triggers from 3.3V |
| Facmogu adjustable DC power supply | 2 | **#1 → SO-101 servo bus** (5V or 12V, match your STS3215 version). **#2 → hobby servo rail at 6.0–7.4V** | Set current limit; goBILDA stall = 2.5–3 A each |
| ELEGOO breadboard power module | 2 | 3.3V/5V logic rails for sensors (HC-SR04 5V, ADS1115 3.3V) | ~700 mA max — **never** power servos from these |

## Not in the list — confirm or get
- **SO-101 follower + leader arms** (with their Feetech bus driver boards + PSU) — the whole project depends on them
- Pi 5 27 W USB-C supplies ×2, microSD cards ×2
- **SensX USB/UART adapter** (connector module — check it came with the sensor)
- 1000 µF capacitor across the hobby-servo rail, resistors (1k/2k dividers ×2, 10k for film sensor)
- Servo extension leads, jumper wires, Wago/terminal blocks, zip ties, spiral wrap for the wrist cable bundle
- PLA + TPU filament (clip, turntable, pads, endoscope mount)
- Empty pill bottles (amber child-proof + OTC) — no real meds
