# PillBot Hub on an Arduino UNO Q

Drop-in alternative to `firmware/esp32_hub`. The UNO Q's MCU can't stream serial to your laptop
over USB-C (USB-C goes to its Linux side), so this is an **App Lab app** with two halves:

- `sketch/sketch.ino` — MCU: reads buttons + ultrasonics, drives servos + relay, handles STOP.
- `python/main.py` — UNO Q Linux: relays it to TCP port **5555** using the ESP32 hub's text protocol.

The PillBot code on the laptop / Raspberry Pi then uses `hub.port: socket://<uno-q-address>:5555`.

## Install
1. Set up the UNO Q in **Arduino App Lab** (connect to Wi-Fi during setup; note its hostname / IP).
2. **Create new app** → name it `pillbot-hub`.
3. **Add Library** → add **Servo** and **Arduino_RouterBridge** (App Lab writes `sketch/sketch.yaml`).
4. Paste `sketch/sketch.ino` into the sketch tab and `python/main.py` into the Python tab.
5. **Run**. The Python console should print `PillBot hub listening on :5555`.
   (To start it automatically on boot, set it as the default app in App Lab.)

## Test from the laptop
```bash
nc <uno-q-address> 5555          # should print READY ... then S lines; type PING -> PONG
```
Then in `configs/robot.local.yaml`:
```yaml
hub:
  enabled: true
  port: socket://<uno-q-address>:5555     # e.g. socket://unoq.local:5555 or socket://192.168.1.42:5555
```
and run `python -m pillbot.run --pick skip --trigger button`.

## Pins (UNO header)
| Pin | Connects to | Notes |
|---|---|---|
| D2 / D4 | HC-SR04 #1 (clip) TRIG / ECHO | digital pins are 5 V tolerant → **no divider needed** |
| D7 / D8 | HC-SR04 #2 (start zone) TRIG / ECHO | |
| A0 / A1 / A2 | START / STOP / MODE buttons → GND | internal pull-ups; analog pins are NOT 5 V tolerant |
| D9 | Servo ch0: turntable (goBILDA #1) | signal only — servo power from the 6 V rail via the relay |
| D10 | Servo ch1: clamp | |
| D11 | Servo ch2: cap tray (optional) | |
| D6 | Servo ch3: spare | |
| D12 | Relay IN (hobby-servo power) | `RELAY_ACTIVE_LOW` in the sketch |

Common ground between the UNO Q, the servo supply and the HC-SR04 5 V supply.
Power the UNO Q from a 5 V / 3 A USB-C supply.

## Latency / safety
Each command goes laptop → Wi-Fi → Linux → Bridge → MCU (tens of ms). That's fine for the
turntable and clamp. STOP is handled on the MCU directly, and the relay switches off if the
laptop disconnects.
