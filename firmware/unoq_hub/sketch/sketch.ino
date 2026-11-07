// PillBot Hub — Arduino UNO Q, MCU side (STM32U585, Zephyr core)
// Same job as firmware/esp32_hub: 2x HC-SR04, START/STOP/MODE buttons, 4 servos, relay.
// The MCU can't talk to your laptop directly — it talks to the UNO Q's Linux side through
// Bridge RPC, and python/main.py relays that to the network (see firmware/unoq_hub/README.md).
//
// RPCs provided (all provide_safe: they run in loop() context, required for GPIO/servo calls):
//   sv(ch, us)  -> bool   servo channel 0..3 pulse 500..2500 us, 0 = detach (limp)
//   rl(on)      -> bool   relay (hobby-servo power) 0/1
//   state()     -> String "<ms> <clip_mm> <zone_mm> <buttons>"   buttons: 1=START 2=STOP 4=MODE
// STOP cuts the relay and detaches all servos right here on the MCU, even if Linux is busy.
//
// UNO Q header pins are 3.3 V logic but the DIGITAL pins are 5 V tolerant, so the HC-SR04
// ECHO can connect directly (no divider). Analog pins A0-A5 are NOT 5 V tolerant — only use
// them for the buttons (wired pin -> button -> GND).

#include <Arduino_RouterBridge.h>
#include <Servo.h>

const int PIN_TRIG_CLIP = 2, PIN_ECHO_CLIP = 4;
const int PIN_TRIG_ZONE = 7, PIN_ECHO_ZONE = 8;
const int PIN_BTN_START = A0, PIN_BTN_STOP = A1, PIN_BTN_MODE = A2;
const int SERVO_PINS[4] = {9, 10, 11, 6};   // ch0 clamp, ch1 cap tray (optional), ch2-3 spare (PWM pins)
const int PIN_RELAY = 12;
const bool RELAY_ACTIVE_LOW = true;

Servo servos[4];
bool attached_[4] = {false, false, false, false};
bool relayOn = false;
long clipMm = -1, zoneMm = -1;
bool readClipNext = true;

void setRelay(bool on) {
  relayOn = on;
  digitalWrite(PIN_RELAY, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void setServo(int ch, int us) {
  if (ch < 0 || ch > 3) return;
  if (us == 0) { if (attached_[ch]) { servos[ch].detach(); attached_[ch] = false; } return; }
  us = constrain(us, 500, 2500);
  if (!attached_[ch]) { servos[ch].attach(SERVO_PINS[ch], 500, 2500); attached_[ch] = true; }
  servos[ch].writeMicroseconds(us);
}

void emergencyStop() {
  setRelay(false);
  for (int i = 0; i < 4; i++) setServo(i, 0);
}

// Manual pulse timing (doesn't depend on pulseIn support in the Zephyr core)
long readUltrasonicMm(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  const unsigned long timeout = 12000;      // ~2 m
  unsigned long t0 = micros();
  while (digitalRead(echo) == LOW) { if (micros() - t0 > timeout) return -1; }
  unsigned long start = micros();
  while (digitalRead(echo) == HIGH) { if (micros() - start > timeout) return -1; }
  return (long)((micros() - start) * 0.343 / 2.0);
}

uint8_t buttons() {   // pull-ups: pressed = LOW, unplugged = not pressed
  return (digitalRead(PIN_BTN_START) ? 0 : 1) | (digitalRead(PIN_BTN_STOP) ? 0 : 2) |
         (digitalRead(PIN_BTN_MODE) ? 0 : 4);
}

// ---- RPC handlers ----
bool rpc_sv(int ch, int us) { setServo(ch, us); return true; }
bool rpc_rl(int on) { setRelay(on != 0); return relayOn; }
String rpc_state() {
  return String(millis()) + " " + String(clipMm) + " " + String(zoneMm) + " " + String(buttons());
}

void setup() {
  pinMode(PIN_TRIG_CLIP, OUTPUT); pinMode(PIN_ECHO_CLIP, INPUT);
  pinMode(PIN_TRIG_ZONE, OUTPUT); pinMode(PIN_ECHO_ZONE, INPUT);
  pinMode(PIN_BTN_START, INPUT_PULLUP); pinMode(PIN_BTN_STOP, INPUT_PULLUP); pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false);                       // servos unpowered until the host enables them

  Bridge.begin();
  Bridge.provide_safe("sv", rpc_sv);
  Bridge.provide_safe("rl", rpc_rl);
  Bridge.provide_safe("state", rpc_state);
}

void loop() {
  if (buttons() & 2) emergencyStop();    // STOP wins, every loop
  if (readClipNext) clipMm = readUltrasonicMm(PIN_TRIG_CLIP, PIN_ECHO_CLIP);
  else              zoneMm = readUltrasonicMm(PIN_TRIG_ZONE, PIN_ECHO_ZONE);
  readClipNext = !readClipNext;
  delay(5);                              // keep loop short: provide_safe RPCs are served between iterations
}
