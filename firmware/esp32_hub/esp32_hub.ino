// PillBot Hub — ESP32 (ESP-WROOM-32 DevKit)
// Reads: 2x HC-SR04, 3x push buttons (START / STOP / MODE)
// Drives: 4 servo channels (ch0 clamp, ch1 ELBOW goBILDA, ch2-3 spare), 1 relay (hobby-servo power)
//
// Library (Arduino Library Manager): "ESP32Servo"
// Board: "ESP32 Dev Module". Pin map: hardware/wiring.md
//
// Serial protocol @115200, one line per message:
//   ESP32 -> host  (every 20 ms):  S <ms> <clip_mm> <zone_mm> <buttons>
//                                  buttons bitmask: 1=START 2=STOP 4=MODE ; distances -1 = no echo
//                  at boot:        READY pillbot-hub 3
//   host -> ESP32:  SV <ch> <us>   set servo channel 0..3 pulse (500..2500), 0 = detach (go limp)
//                   RL <0|1>       relay off/on (hobby-servo power)
//                   PING           -> PONG
// Safety: STOP button immediately cuts the relay and detaches the servos (firmware-level) — EXCEPT
// channels in HOLD_ON_STOP: the elbow holds its angle, like the Dynamixels do, so the arm doesn't drop.
// Wire the elbow servo's power straight from the supply, NOT through the relay.

#include <ESP32Servo.h>

// ---- pins ----
const int PIN_TRIG_CLIP = 4,  PIN_ECHO_CLIP = 34;
const int PIN_TRIG_ZONE = 16, PIN_ECHO_ZONE = 35;
const int PIN_BTN_START = 32, PIN_BTN_STOP = 33, PIN_BTN_MODE = 27;
const int SERVO_PINS[4] = {25, 26, 13, 14};
const bool HOLD_ON_STOP[4] = {false, true, false, false};   // ch1 = elbow joint
const int PIN_RELAY = 17;
const int PIN_LED = 2;
const bool RELAY_ACTIVE_LOW = true;   // most cheap relay modules switch ON when IN is LOW

Servo servos[4];
int servoUs[4] = {0, 0, 0, 0};
bool relayOn = false;

long clipMm = -1, zoneMm = -1;
bool readClipNext = true;
unsigned long lastReport = 0;
String rx;

void setRelay(bool on) {
  relayOn = on;
  digitalWrite(PIN_RELAY, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void setServo(int ch, int us) {
  if (ch < 0 || ch > 3) return;
  if (us == 0) { if (servos[ch].attached()) servos[ch].detach(); servoUs[ch] = 0; return; }
  us = constrain(us, 500, 2500);
  if (!servos[ch].attached()) { servos[ch].setPeriodHertz(50); servos[ch].attach(SERVO_PINS[ch], 500, 2500); }
  servos[ch].writeMicroseconds(us);
  servoUs[ch] = us;
}

void emergencyStop() {
  setRelay(false);
  for (int i = 0; i < 4; i++) if (!HOLD_ON_STOP[i]) setServo(i, 0);
}

long readUltrasonicMm(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 12000);   // ~2 m max; keeps the loop fast
  if (us == 0) return -1;
  return (long)(us * 0.343 / 2.0);                 // speed of sound 343 m/s
}

// Push buttons wired pin -> button -> GND, internal pull-ups: pressed = LOW, unplugged = not pressed
uint8_t buttons() {
  return (digitalRead(PIN_BTN_START) ? 0 : 1) | (digitalRead(PIN_BTN_STOP) ? 0 : 2) |
         (digitalRead(PIN_BTN_MODE) ? 0 : 4);
}

void handleLine(const String& line) {
  if (line == "PING") { Serial.println("PONG"); return; }
  int ch, us, v;
  if (sscanf(line.c_str(), "SV %d %d", &ch, &us) == 2) { setServo(ch, us); return; }
  if (sscanf(line.c_str(), "RL %d", &v) == 1) { setRelay(v != 0); return; }
  Serial.print("ERR "); Serial.println(line);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG_CLIP, OUTPUT); pinMode(PIN_ECHO_CLIP, INPUT);
  pinMode(PIN_TRIG_ZONE, OUTPUT); pinMode(PIN_ECHO_ZONE, INPUT);
  pinMode(PIN_BTN_START, INPUT_PULLUP); pinMode(PIN_BTN_STOP, INPUT_PULLUP); pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT); pinMode(PIN_LED, OUTPUT);
  setRelay(false);                                   // servos unpowered until host enables
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  Serial.println("READY pillbot-hub 3");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { rx.trim(); if (rx.length()) handleLine(rx); rx = ""; }
    else if (rx.length() < 64) rx += c;
  }

  uint8_t b = buttons();
  if (b & 2) emergencyStop();                        // STOP wins, every loop

  // alternate the two ultrasonic sensors so they don't hear each other
  if (readClipNext) clipMm = readUltrasonicMm(PIN_TRIG_CLIP, PIN_ECHO_CLIP);
  else              zoneMm = readUltrasonicMm(PIN_TRIG_ZONE, PIN_ECHO_ZONE);
  readClipNext = !readClipNext;

  unsigned long now = millis();
  if (now - lastReport >= 20) {
    lastReport = now;
    Serial.printf("S %lu %ld %ld %u\n", now, clipMm, zoneMm, b);
    digitalWrite(PIN_LED, relayOn ? HIGH : (now / 500) % 2);   // solid = servos powered, blink = idle
  }
}
