/*
 * nano_motor_safety.ino -- Arduino Nano (ATmega328P, 5 V) low-level controller for the AGV
 *
 * HARDWARE
 *   Ultrasonic HC-SR04 (front)  Trig = D4, Echo = D2 (INT0, hardware interrupt, 5 V is fine on a Nano)
 *   Steering servo (Ackermann)  Signal = D9  (Servo library, Timer1)
 *   L298N (2 drive motors, 4 wheels): ONE shared PWM pin
 *       ENA + ENB jumpers REMOVED and both EN pins wired together to D3 (Timer2 PWM, set to ~3.9 kHz)
 *       IN1 = D7, IN2 = D8   (motor A)      IN3 = A0, IN4 = A1   (motor B)
 *   Link to Pi: USB cable (Serial @ 115200). Do NOT wire Nano TX (5 V) to Pi GPIO: it would damage the Pi.
 *   Power: servo from its own 5-6 V BEC (stall current resets the Nano otherwise), common GND with L298N + Nano.
 *
 * SAFETY CONTRACT
 *   - Echo <= 15 cm (or a pulse too short to measure) trips ESTOP on ONE reading, motors are braked inside the
 *     same loop pass, and every Pi command is ignored while tripped.
 *   - Release needs 3 consecutive readings > 20 cm (or no echo): hysteresis against chatter.
 *   - Boot / sonar stalled > 200 ms => treated as tripped (fail-safe).
 *   - No valid command for 250 ms => ramp to stop and servo to centre.
 *   - loop() never blocks (only the 10 us trigger pulse), so latency from echo to brake is < 1 ms.
 *     The sensing period (30 ms) is the dominant latency: include it in the stopping-distance budget.
 *
 * PROTOCOL
 *   Pi -> Nano : $CMD,<seq16>,<speed -100..100>,<steer -100..100>*HH\n     (HH = XOR of bytes between $ and *)
 *   Nano -> Pi : $TEL,<ms>,<dist_mm 9999=none>,<flags>,<bad>,<dropped>*HH\n
 *                flags: bit0 ESTOP, bit1 CMD_TIMEOUT, bit2 SONAR_STALE/not primed
 *   steer +100 = full right.
 *
 * RAM is tight (2 KB): no String objects, no dynamic allocation.
 */

#include <Arduino.h>
#include <Servo.h>

// ============================== CONFIG ====================================
const uint8_t PIN_TRIG = 4;
const uint8_t PIN_ECHO = 2;                 // INT0 -- must stay on D2 (or D3) for attachInterrupt
const uint8_t PIN_SERVO = 9;
const uint8_t PIN_EN = 3;                   // shared PWM for both L298N channels (OC2B)
const uint8_t PIN_IN1 = 7, PIN_IN2 = 8, PIN_IN3 = A0, PIN_IN4 = A1;

// Safety
const uint16_t STOP_MM        = 150;        // 15 cm hard stop
const uint16_t RELEASE_MM     = 200;
const uint8_t  CLEAR_COUNT    = 3;
const uint16_t MIN_PULSE_US   = 116;        // < 20 mm
const uint32_t SONAR_PERIOD_US = 30000UL;   // trigger every 30 ms (~33 Hz)
const uint32_t ECHO_TIMEOUT_US = 18000UL;   // ~3 m
const uint32_t STALE_MS       = 200;
const uint32_t CMD_TIMEOUT_MS = 250;
const bool     SAFETY_ACTIVE_BRAKE = true;  // short motor terminals (EN high, IN pairs equal)

// Motors (open loop; both motors share one PWM so they get the same duty)
const uint8_t  MIN_DUTY       = 70;         // deadband: duty where the car just moves (TUNE)
const bool     INVERT_A = false, INVERT_B = false;   // flip if a motor spins backwards
const bool     ZERO_CMD_BRAKES = true;      // hold position at speed 0
const float    ACCEL_PCT_PER_S = 160.0f;
const float    DECEL_PCT_PER_S = 500.0f;

// Servo (CALIBRATE with wheels off the ground)
const int      SERVO_CENTER_US    = 1500;
const int      SERVO_HALF_SPAN_US = 350;    // keep below the linkage limit
const bool     SERVO_INVERT       = false;  // flip if +steer turns left
const float    SERVO_US_PER_S     = 2500.0f;

const uint32_t CONTROL_PERIOD_MS   = 5;
const uint32_t TELEMETRY_PERIOD_MS = 50;
// ==========================================================================

Servo steerServo;

// ------------------------------ Sonar state -------------------------------
volatile uint32_t gRiseUs = 0, gFallUs = 0;
volatile bool gGotRise = false, gGotFall = false;

bool     gEstop = false;
bool     gPrimed = false;                   // has at least one completed measurement
uint32_t gLastDoneMs = 0;
uint16_t gDistMm = 9999;
uint8_t  gClearCnt = 0;
bool     gBlocked = false;
uint32_t gLastTrigUs = 0;
bool     gWaiting = false;

void echoISR() {
  uint32_t t = micros();                    // AVR micros() is correct inside ISRs (handles pending overflow)
  if (PIND & _BV(PD2)) { gRiseUs = t; gGotRise = true; }
  else if (gGotRise && !gGotFall) { gFallUs = t; gGotFall = true; }
}

inline bool echoHigh() { return (PIND & _BV(PD2)) != 0; }
inline bool safetyTripped() { return gEstop || !gPrimed || (millis() - gLastDoneMs) > STALE_MS; }

// ------------------------------ Motor layer -------------------------------
void setDir(bool aFwd, bool bFwd) {
  if (INVERT_A) aFwd = !aFwd;
  if (INVERT_B) bFwd = !bFwd;
  digitalWrite(PIN_IN1, aFwd ? HIGH : LOW);  digitalWrite(PIN_IN2, aFwd ? LOW : HIGH);
  digitalWrite(PIN_IN3, bFwd ? HIGH : LOW);  digitalWrite(PIN_IN4, bFwd ? LOW : HIGH);
}
void motorCoast() {
  analogWrite(PIN_EN, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}
void motorBrake() {                         // L298N fast brake: inputs equal while enabled
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_EN, 255);
}
void motorDrive(float pct) {                // -100..100
  if (fabs(pct) < 0.5f) { if (ZERO_CMD_BRAKES) motorBrake(); else motorCoast(); return; }
  bool fwd = pct > 0;
  float mag = fabs(pct); if (mag > 100.0f) mag = 100.0f;
  setDir(fwd, fwd);
  analogWrite(PIN_EN, (uint8_t)(MIN_DUTY + (mag / 100.0f) * (255 - MIN_DUTY)));
}
void safeStop() { if (SAFETY_ACTIVE_BRAKE) motorBrake(); else motorCoast(); }

// ------------------------------ Sonar logic -------------------------------
void finalizeReading(bool haveEcho, uint32_t pulseUs) {
  bool blockedNow = false;
  if (haveEcho) {
    if (pulseUs < MIN_PULSE_US) { gDistMm = 0; blockedNow = true; }
    else { gDistMm = (uint16_t)((pulseUs * 343UL) / 2000UL); blockedNow = (gDistMm <= STOP_MM); }
  } else {
    gDistMm = 9999;
  }
  gLastDoneMs = millis();
  gPrimed = true;

  if (blockedNow) {                         // a single reading trips immediately
    gClearCnt = 0;
    gBlocked = true;
    if (!gEstop) { gEstop = true; safeStop(); }
  } else if (gBlocked) {
    bool beyond = (!haveEcho) || (gDistMm > RELEASE_MM);
    if (beyond) { if (++gClearCnt >= CLEAR_COUNT) { gBlocked = false; gEstop = false; } }
    else gClearCnt = 0;
  }
}

void serviceSonar() {
  uint32_t nowUs = micros();
  if (!gWaiting) {
    // Only trigger when the echo line is low (a no-echo pulse from the last cycle can last ~38 ms).
    if ((uint32_t)(nowUs - gLastTrigUs) >= SONAR_PERIOD_US && !echoHigh()) {
      noInterrupts(); gGotRise = false; gGotFall = false; interrupts();
      digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10); digitalWrite(PIN_TRIG, LOW);
      gLastTrigUs = micros();
      gWaiting = true;
    }
  } else {
    bool fell; uint32_t r, f;
    noInterrupts(); fell = gGotFall; r = gRiseUs; f = gFallUs; interrupts();
    if (fell) { finalizeReading(true, f - r); gWaiting = false; }
    else if ((uint32_t)(nowUs - gLastTrigUs) > ECHO_TIMEOUT_US) { finalizeReading(false, 0); gWaiting = false; }
  }
}

// -------------------------------- Command state ---------------------------
struct Cmd { int8_t speed; int8_t steer; uint32_t rxMs; bool valid; };
Cmd gCmd = {0, 0, 0, false};
uint32_t gBadFrames = 0, gDropped = 0;
bool gCmdTimedOut = true;

float gRamp = 0.0f;
float gServoUs = SERVO_CENTER_US;

float slewTo(float cur, float tgt, float up, float down) {
  float step = (fabs(tgt) < fabs(cur)) ? down : up;
  float d = tgt - cur;
  if (d > step) d = step; else if (d < -step) d = -step;
  return cur + d;
}

float steerToUs(int steer) {
  float s = steer / 100.0f;
  if (SERVO_INVERT) s = -s;
  return SERVO_CENTER_US + s * SERVO_HALF_SPAN_US;
}

void controlUpdate() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < CONTROL_PERIOD_MS) return;
  float dt = (now - last) * 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  last = now;

  if (safetyTripped()) {                    // hard override: Pi commands are not even read
    gRamp = 0.0f;
    safeStop();
    return;                                 // servo holds its angle
  }

  bool fresh = gCmd.valid && (now - gCmd.rxMs) < CMD_TIMEOUT_MS;
  gCmdTimedOut = !fresh;

  float tgtSpeed = fresh ? gCmd.speed : 0.0f;
  float tgtUs = fresh ? steerToUs(gCmd.steer) : (float)SERVO_CENTER_US;

  gRamp = slewTo(gRamp, tgtSpeed, ACCEL_PCT_PER_S * dt, DECEL_PCT_PER_S * dt);
  motorDrive(gRamp);

  float d = tgtUs - gServoUs, maxStep = SERVO_US_PER_S * dt;
  if (d > maxStep) d = maxStep; else if (d < -maxStep) d = -maxStep;
  gServoUs += d;
  int us = (int)gServoUs;
  if (us < SERVO_CENTER_US - SERVO_HALF_SPAN_US) us = SERVO_CENTER_US - SERVO_HALF_SPAN_US;   // hard clamp
  if (us > SERVO_CENTER_US + SERVO_HALF_SPAN_US) us = SERVO_CENTER_US + SERVO_HALF_SPAN_US;
  steerServo.writeMicroseconds(us);
}

// ------------------------- Serial parsing state machine -------------------
enum ParseState : uint8_t { P_IDLE, P_BODY, P_CS_HI, P_CS_LO, P_EOL };
ParseState pState = P_IDLE;
char    pBuf[32];
uint8_t pLen = 0, pCs = 0, pCsRx = 0;
uint16_t gLastSeq = 0;
bool gHaveSeq = false;

int hexVal(uint8_t b) {
  if (b >= '0' && b <= '9') return b - '0';
  if (b >= 'A' && b <= 'F') return b - 'A' + 10;
  if (b >= 'a' && b <= 'f') return b - 'a' + 10;
  return -1;
}

bool parseCmd(const char *s, long *out) {   // "CMD,a,b,c" -> out[0..2]
  if (strncmp(s, "CMD,", 4) != 0) return false;
  const char *p = s + 4;
  for (uint8_t i = 0; i < 3; i++) {
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) return false;
    out[i] = v; p = end;
    if (i < 2) { if (*p != ',') return false; p++; }
  }
  return *p == '\0';
}

void handleFrame(const char *s) {
  long v[3];
  if (!parseCmd(s, v) || v[0] < 0 || v[0] > 65535 || v[1] < -100 || v[1] > 100 || v[2] < -100 || v[2] > 100) {
    gBadFrames++; return;
  }
  uint16_t seq = (uint16_t)v[0];
  if (gHaveSeq) {
    int16_t diff = (int16_t)(seq - gLastSeq);
    if (diff <= 0) return;                  // duplicate / out of order
    if (diff > 1) gDropped += (uint32_t)(diff - 1);
  }
  gLastSeq = seq; gHaveSeq = true;
  gCmd.speed = (int8_t)v[1]; gCmd.steer = (int8_t)v[2]; gCmd.rxMs = millis(); gCmd.valid = true;
}

void feed(uint8_t b) {
  if (b == '$') { pLen = 0; pCs = 0; pState = P_BODY; return; }       // '$' always resynchronises
  switch (pState) {
    case P_IDLE: break;
    case P_BODY:
      if (b == '*') pState = P_CS_HI;
      else if (b < 32 || b > 126 || pLen >= sizeof(pBuf) - 1) { gBadFrames++; pState = P_IDLE; }
      else { pBuf[pLen++] = (char)b; pCs ^= b; }
      break;
    case P_CS_HI: { int h = hexVal(b); if (h < 0) { gBadFrames++; pState = P_IDLE; } else { pCsRx = (uint8_t)(h << 4); pState = P_CS_LO; } break; }
    case P_CS_LO: { int h = hexVal(b); if (h < 0) { gBadFrames++; pState = P_IDLE; } else { pCsRx |= (uint8_t)h; pState = P_EOL; } break; }
    case P_EOL:
      if (b == '\n' || b == '\r') { pBuf[pLen] = 0; if (pCsRx == pCs) handleFrame(pBuf); else gBadFrames++; }
      else gBadFrames++;
      pState = P_IDLE;
      break;
  }
}

// -------------------------------- Telemetry -------------------------------
void sendTelemetry() {
  uint8_t flags = (gEstop ? 1 : 0) | (gCmdTimedOut ? 2 : 0) |
                  ((!gPrimed || (millis() - gLastDoneMs) > STALE_MS) ? 4 : 0);
  char payload[40];
  int n = snprintf(payload, sizeof(payload), "TEL,%lu,%u,%u,%lu,%lu",
                   (unsigned long)millis(), gDistMm, flags, (unsigned long)gBadFrames, (unsigned long)gDropped);
  if (n <= 0 || n >= (int)sizeof(payload)) return;
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)payload[i];
  char out[52];
  int m = snprintf(out, sizeof(out), "$%s*%02X\n", payload, cs);
  if (m > 0 && Serial.availableForWrite() >= m) Serial.write((const uint8_t *)out, m);   // never block
}

// ---------------------------------- Arduino --------------------------------
void setup() {
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  motorCoast();
  TCCR2B = (TCCR2B & 0b11111000) | 0x02;    // Timer2 prescaler 8 -> ~3.9 kHz PWM on D3 (D5/D6/millis unaffected)

  pinMode(PIN_TRIG, OUTPUT); digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), echoISR, CHANGE);

  steerServo.attach(PIN_SERVO, SERVO_CENTER_US - SERVO_HALF_SPAN_US - 100, SERVO_CENTER_US + SERVO_HALF_SPAN_US + 100);
  steerServo.writeMicroseconds(SERVO_CENTER_US);

  Serial.begin(115200);
  gLastTrigUs = micros() - SONAR_PERIOD_US;   // first ping immediately
}

void loop() {
  while (Serial.available()) feed((uint8_t)Serial.read());    // drain everything every pass
  serviceSonar();
  controlUpdate();

  static uint32_t tTel = 0;
  uint32_t now = millis();
  if (now - tTel >= TELEMETRY_PERIOD_MS) { tTel = now; sendTelemetry(); }
}
