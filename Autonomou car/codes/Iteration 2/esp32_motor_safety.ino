/*
 * esp32_motor_safety.ino -- Low-level safety + motor controller (ESP32-S3, Arduino-ESP32 core 2.x or 3.x)
 *
 * TASK / PRIORITY MAP (all on core 1)
 *   sensorTask  prio 5  1 ms tick. Round-robin HC-SR04 scheduler, echo timed by GPIO ISR (no pulseIn,
 *                       no blocking). Evaluates the 15 cm rule the instant a reading completes and
 *                       brakes the motors DIRECTLY from this task.
 *   motorTask   prio 4  5 ms tick. Command watchdog, mixing, slew-rate limiting, output.
 *   loop()      prio 1  UART parsing state machine + telemetry + 1 Hz USB debug.
 *
 * SAFETY CONTRACT
 *   - Any front sensor (FL/FC/FR) <= STOP_MM  -> gEstop latched, motors braked, all Pi commands ignored.
 *   - Release needs CLEAR_COUNT consecutive readings > RELEASE_MM (hysteresis, prevents chatter).
 *   - Front sensor not updated for STALE_MS, or never updated since boot -> treated as tripped (fail-safe).
 *   - No valid command for CMD_TIMEOUT_MS -> ramp to stop.
 *   - Every motor write goes through applyDrive(), which re-checks safetyTripped() inside a mutex, so a
 *     command can never overwrite an emergency brake.
 *
 * WIRING NOTES (important)
 *   - HC-SR04 Echo is 5 V.  ESP32-S3 GPIO is NOT 5 V tolerant -> divider on EVERY echo pin
 *     (e.g. 1 kOhm in series from Echo, 2 kOhm from the GPIO node to GND  => ~3.3 V).
 *   - Power sensors from 5 V, L298N logic ENA/ENB jumpers REMOVED (PWM drives them), common GND everywhere.
 *   - STEERING: single Ackermann servo on PIN_SERVO (LEDC, 50 Hz). Power it from its own 5-6 V BEC/regulator
 *     (NOT the ESP32 5 V pin or the L298N 5 V output: stall current causes brownouts), signal wire to the GPIO,
 *     common GND. Calibrate SERVO_CENTER_US / SERVO_HALF_SPAN_US with the wheels off the ground first.
 *   - L298N drives the two rear motors with identical commands. No differential mixing.
 *   - The motor and servo pins below were NOT in the spec: they are assumptions on safe GPIOs. Change as needed.
 *     Avoid GPIO 0,3,45,46 (strapping), 19,20 (USB), 26-32 (flash), 35-37 (octal PSRAM boards).
 *   - Add 10k pull-downs on IN1..IN4 / ENA / ENB so the bridge is inert during boot.
 */

#include <Arduino.h>
#include <HardwareSerial.h>

// ============================== CONFIG ====================================
// ---- Ultrasonic pins (index order = SensorId) ----
static const uint8_t PIN_TRIG[5] = {4, 5, 6, 7, 15};
static const uint8_t PIN_ECHO[5] = {16, 8, 9, 10, 11};
enum SensorId : uint8_t { S_LEFT = 0, S_FRONT_LEFT, S_FRONT_CENTER, S_FRONT_RIGHT, S_RIGHT, S_COUNT };
static const uint8_t SCHEDULE[S_COUNT] = {S_FRONT_LEFT, S_FRONT_CENTER, S_FRONT_RIGHT, S_LEFT, S_RIGHT};

// ---- L298N pins (ASSUMED): drive motors only. Both channels get the SAME command (rear axle). ----
static const uint8_t PIN_ENA = 12, PIN_IN1 = 13, PIN_IN2 = 14;   // rear-left drive motor
static const uint8_t PIN_ENB = 21, PIN_IN3 = 47, PIN_IN4 = 42;   // rear-right drive motor

// ---- Ackermann steering servo (ASSUMED pin) ----
static const uint8_t PIN_SERVO = 1;

// ---- UART link to Pi ----
static const uint8_t  PIN_LINK_RX = 18, PIN_LINK_TX = 17;
static const uint32_t LINK_BAUD = 115200;
HardwareSerial LinkSerial(2);

// ---- Safety thresholds ----
static const uint16_t STOP_MM       = 150;    // 15 cm hard stop
static const uint16_t RELEASE_MM    = 200;    // must exceed this to release
static const uint8_t  CLEAR_COUNT   = 3;      // consecutive clear readings
static const uint16_t MIN_PULSE_US  = 116;    // < 20 mm: too close for HC-SR04 -> treat as blocked
static const uint32_t SLOT_US       = 9000;   // per-sensor slot  (cycle = 5 * 9 ms = 45 ms)
static const uint32_t ECHO_TIMEOUT_US = 6000; // ~1.0 m max range (shorter = faster cycle)
static const uint32_t STALE_MS      = 200;    // front sensor data too old -> fail-safe
static const uint32_t CMD_TIMEOUT_MS = 250;   // Pi command watchdog
static const bool     SAFETY_ACTIVE_BRAKE = true; // true: short motor terminals (EN high, IN1=IN2); false: coast

// ---- Motor mapping ----
static const uint32_t PWM_FREQ_HZ = 5000;
static const uint8_t  PWM_RES_BITS = 8;
static const uint32_t PWM_MAX = 255;
static const uint32_t MIN_DUTY = 70;          // deadband: duty at which motors just move (TUNE)

// ---- Ackermann servo (50 Hz, pulse width in microseconds) ----
// steer +100 = full right.  CALIBRATE: find the pulse where wheels are straight, then the
// largest pulse offset each way that does NOT bind the steering linkage, and put that in SERVO_HALF_SPAN_US.
static const uint32_t SERVO_FREQ_HZ   = 50;
static const uint8_t  SERVO_RES_BITS  = 14;
static const float    SERVO_CENTER_US = 1500.0f;   // wheels straight (TUNE)
static const float    SERVO_HALF_SPAN_US = 350.0f; // full-lock travel each side (TUNE, keep below linkage limit)
static const bool     SERVO_INVERT    = false;     // flip if +steer turns left
static const float    SERVO_US_PER_S  = 2500.0f;   // slew limit: protects linkage, smooths steering
static const bool     ZERO_CMD_BRAKES = true; // hold position at 0 command
static const bool     INVERT_LEFT = false, INVERT_RIGHT = false;
static const float    ACCEL_PCT_PER_S = 180.0f;   // speed ramp up
static const float    DECEL_PCT_PER_S = 600.0f;   // speed ramp down (normal stop)

// ---- Protocol / telemetry ----
static const uint32_t TELEMETRY_PERIOD_MS = 50;   // 20 Hz
#define DEBUG_PRINT 1
// ==========================================================================

// ------------------------ PWM abstraction (core 2.x / 3.x) ----------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static inline void pwmInit(uint8_t pin, uint8_t) { ledcAttach(pin, PWM_FREQ_HZ, PWM_RES_BITS); }
static inline void pwmSet(uint8_t pin, uint8_t, uint32_t duty) { ledcWrite(pin, duty); }
#else
static inline void pwmInit(uint8_t pin, uint8_t ch) { ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(pin, ch); }
static inline void pwmSet(uint8_t, uint8_t ch, uint32_t duty) { ledcWrite(ch, duty); }
#endif

// Servo on its own LEDC channel (50 Hz, 14-bit). Core 2.x channel 2 keeps it off the motor timer.
static const uint8_t SERVO_CH = 2;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static inline void servoInit() { ledcAttach(PIN_SERVO, SERVO_FREQ_HZ, SERVO_RES_BITS); }
static inline void servoDuty(uint32_t d) { ledcWrite(PIN_SERVO, d); }
#else
static inline void servoInit() { ledcSetup(SERVO_CH, SERVO_FREQ_HZ, SERVO_RES_BITS); ledcAttachPin(PIN_SERVO, SERVO_CH); }
static inline void servoDuty(uint32_t d) { ledcWrite(SERVO_CH, d); }
#endif

static void servoWriteUs(float us) {
  float lo = SERVO_CENTER_US - SERVO_HALF_SPAN_US, hi = SERVO_CENTER_US + SERVO_HALF_SPAN_US;
  us = fminf(fmaxf(us, lo), hi);                                   // hard clamp: never exceed linkage limit
  uint32_t period_us = 1000000UL / SERVO_FREQ_HZ;                  // 20000 us
  servoDuty((uint32_t)(us * (float)(1UL << SERVO_RES_BITS) / (float)period_us));
}

static float steerToServoUs(int steer) {                           // -100..100 -> microseconds
  float s = (float)steer / 100.0f;
  if (SERVO_INVERT) s = -s;
  return SERVO_CENTER_US + s * SERVO_HALF_SPAN_US;
}

// ------------------------------ Shared state ------------------------------
static const uint16_t DIST_NONE = 0xFFFF;

static volatile uint16_t gDistMm[S_COUNT];
static volatile uint32_t gLastUpdateMs[S_COUNT];   // 0 = never updated
static volatile bool     gBlocked[S_COUNT];
static uint8_t           gClearCnt[S_COUNT];       // sensor task only
static volatile bool     gEstop = false;
static volatile bool     gSensorStale = true;      // fail-safe until proven alive
static volatile bool     gCmdTimedOut = true;
static volatile uint32_t gEstopCount = 0;

struct Cmd { int8_t speed; int8_t steer; uint32_t rxMs; bool valid; };
static Cmd gCmd = {0, 0, 0, false};
static portMUX_TYPE gCmdMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t gBadFrames = 0;
static volatile uint32_t gDropped = 0;
static SemaphoreHandle_t gMotorMutex;

static inline bool safetyTripped() { return gEstop || gSensorStale; }
static inline bool isFront(uint8_t id) { return id == S_FRONT_LEFT || id == S_FRONT_CENTER || id == S_FRONT_RIGHT; }

// ------------------------------ Motor layer -------------------------------
struct MotorPins { uint8_t en, in1, in2, ch; bool invert; };
static const MotorPins MOTOR_L = {PIN_ENA, PIN_IN1, PIN_IN2, 0, INVERT_LEFT};
static const MotorPins MOTOR_R = {PIN_ENB, PIN_IN3, PIN_IN4, 1, INVERT_RIGHT};

static inline void motorCoast(const MotorPins &m) {
  pwmSet(m.en, m.ch, 0);
  digitalWrite(m.in1, LOW); digitalWrite(m.in2, LOW);
}
static inline void motorBrake(const MotorPins &m) {
  // L298N fast-brake: both inputs equal while the enable is HIGH shorts the motor terminals.
  digitalWrite(m.in1, HIGH); digitalWrite(m.in2, HIGH);
  pwmSet(m.en, m.ch, PWM_MAX);
}
static inline void motorDrive(const MotorPins &m, float pct) {   // -100..100
  if (m.invert) pct = -pct;
  if (fabsf(pct) < 0.5f) { ZERO_CMD_BRAKES ? motorBrake(m) : motorCoast(m); return; }
  bool fwd = pct > 0;
  float mag = fminf(fabsf(pct), 100.0f);
  uint32_t duty = MIN_DUTY + (uint32_t)((mag / 100.0f) * (float)(PWM_MAX - MIN_DUTY));
  digitalWrite(m.in1, fwd ? HIGH : LOW);
  digitalWrite(m.in2, fwd ? LOW : HIGH);
  pwmSet(m.en, m.ch, duty);
}

// Single choke point for ALL motor writes.  Safety is re-checked INSIDE the lock.
static void applyDrive(float l, float r) {
  xSemaphoreTake(gMotorMutex, portMAX_DELAY);
  if (safetyTripped()) {
    if (SAFETY_ACTIVE_BRAKE) { motorBrake(MOTOR_L); motorBrake(MOTOR_R); }
    else                     { motorCoast(MOTOR_L); motorCoast(MOTOR_R); }
  } else {
    motorDrive(MOTOR_L, l);
    motorDrive(MOTOR_R, r);
  }
  xSemaphoreGive(gMotorMutex);
}

// ------------------------------ Ultrasonics -------------------------------
struct EchoState { volatile uint32_t tRise, tFall; volatile bool gotRise, gotFall; };
static EchoState gEcho[S_COUNT];

static void IRAM_ATTR echoISR(void *arg) {
  uint8_t i = (uint8_t)(uintptr_t)arg;
  uint32_t t = micros();
  if (digitalRead(PIN_ECHO[i])) { gEcho[i].tRise = t; gEcho[i].gotRise = true; }
  else if (gEcho[i].gotRise && !gEcho[i].gotFall) { gEcho[i].tFall = t; gEcho[i].gotFall = true; }
}

static void updateEstop() {
  bool any = gBlocked[S_FRONT_LEFT] || gBlocked[S_FRONT_CENTER] || gBlocked[S_FRONT_RIGHT];
  if (any && !gEstop) {
    gEstop = true;
    gEstopCount++;
    applyDrive(0, 0);            // brake NOW, from the sensor task, before anything else runs
  } else if (!any && gEstop) {
    gEstop = false;
  }
}

static void finalizeReading(uint8_t id, bool haveEcho, uint32_t pulseUs) {
  uint16_t mm = DIST_NONE;
  bool blockedNow = false;
  if (haveEcho) {
    if (pulseUs < MIN_PULSE_US) { mm = 0; blockedNow = true; }
    else { mm = (uint16_t)((pulseUs * 343UL) / 2000UL); blockedNow = (mm <= STOP_MM); }  // 343 m/s @ ~20 C
  }
  gDistMm[id] = mm;
  gLastUpdateMs[id] = millis();
  if (!isFront(id)) return;

  if (blockedNow) {                       // single reading trips immediately
    gClearCnt[id] = 0;
    gBlocked[id] = true;
    updateEstop();
  } else if (gBlocked[id]) {              // hysteresis + consecutive-clear release
    bool beyond = (!haveEcho) || (mm > RELEASE_MM);
    if (beyond) { if (++gClearCnt[id] >= CLEAR_COUNT) { gBlocked[id] = false; updateEstop(); } }
    else gClearCnt[id] = 0;
  }
}

static void sensorTask(void *) {
  TickType_t last = xTaskGetTickCount();
  uint8_t slot = 0, cur = SCHEDULE[0];
  uint32_t slotStartUs = micros() - SLOT_US;     // fire first ping immediately
  bool pending = false;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    uint32_t now = micros();

    if (pending) {                               // collect result as early as possible
      EchoState &e = gEcho[cur];
      if (e.gotFall) { finalizeReading(cur, true, e.tFall - e.tRise); pending = false; }
      else if ((uint32_t)(now - slotStartUs) > ECHO_TIMEOUT_US) { finalizeReading(cur, false, 0); pending = false; }
    }

    if (!pending && (uint32_t)(now - slotStartUs) >= SLOT_US) {   // next slot (spacing kills crosstalk/ringing)
      cur = SCHEDULE[slot];
      slot = (slot + 1) % S_COUNT;
      EchoState &e = gEcho[cur];
      e.gotRise = false; e.gotFall = false;      // clear BEFORE triggering
      digitalWrite(PIN_TRIG[cur], HIGH);
      delayMicroseconds(10);                     // only blocking call: 10 us, datasheet-mandated
      digitalWrite(PIN_TRIG[cur], LOW);
      slotStartUs = micros();
      pending = true;
    }
  }
}

// -------------------------------- Motor task ------------------------------
static float slew(float cur, float tgt, float stepUp, float stepDown) {
  float step = (fabsf(tgt) < fabsf(cur)) ? stepDown : stepUp;
  float d = tgt - cur;
  if (d > step) d = step; else if (d < -step) d = -step;
  return cur + d;
}

// Ackermann: steering is the servo's job, so both rear motors get the same speed.
// (Optional upgrade: electronic differential on the rear axle, scaling the inner wheel by
//  (1 - (track/2)*tan(delta)/wheelbase) -- usually unnecessary at competition speeds.)
static void mixDrive(int speed, int /*steer*/, float &l, float &r) {
  l = r = (float)speed;
}

/*  Open-loop speed: the L298N setup in the spec has no encoders, so there is no velocity feedback to
 *  close a PID around.  When wheel encoders are added, run a PID per wheel here on (target - measured)
 *  and feed its output into motorDrive().  Slew limiting below already gives smooth acceleration. */
static void motorTask(void *) {
  TickType_t last = xTaskGetTickCount();
  float rampL = 0, rampR = 0;
  float servoUs = SERVO_CENTER_US;
  const float dt = 0.005f;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
    uint32_t nowMs = millis();

    // Front-sensor liveness (also fails safe at boot until every front sensor has reported once)
    bool stale = false;
    for (uint8_t id = S_FRONT_LEFT; id <= S_FRONT_RIGHT; id++) {
      uint32_t u = gLastUpdateMs[id];
      if (u == 0 || (nowMs - u) > STALE_MS) stale = true;
    }
    gSensorStale = stale;

    if (safetyTripped()) { rampL = rampR = 0; applyDrive(0, 0); continue; }

    Cmd c;
    portENTER_CRITICAL(&gCmdMux); c = gCmd; portEXIT_CRITICAL(&gCmdMux);
    bool fresh = c.valid && (nowMs - c.rxMs) < CMD_TIMEOUT_MS;
    gCmdTimedOut = !fresh;

    float tl = 0, tr = 0;
    float servoTarget = SERVO_CENTER_US;               // no valid command -> wheels straight
    if (fresh) { mixDrive(c.speed, c.steer, tl, tr); servoTarget = steerToServoUs(c.steer); }
    float maxStep = SERVO_US_PER_S * dt;               // slew-limited servo (not safety critical:
    float d = servoTarget - servoUs;                   //  it keeps its position during an ESTOP)
    if (d > maxStep) d = maxStep; else if (d < -maxStep) d = -maxStep;
    servoUs += d;
    servoWriteUs(servoUs);
    rampL = slew(rampL, tl, ACCEL_PCT_PER_S * dt, DECEL_PCT_PER_S * dt);
    rampR = slew(rampR, tr, ACCEL_PCT_PER_S * dt, DECEL_PCT_PER_S * dt);
    applyDrive(rampL, rampR);
  }
}

// ------------------------- Serial parsing state machine -------------------
// Frame:  $CMD,<seq>,<speed>,<steer>*HH\n      HH = XOR of bytes between '$' and '*' (hex)
enum ParseState : uint8_t { P_IDLE, P_BODY, P_CS_HI, P_CS_LO, P_EOL };
static ParseState pState = P_IDLE;
static char    pBuf[48];
static uint8_t pLen = 0, pCs = 0, pCsRx = 0;
static uint16_t gLastSeq = 0;
static bool gHaveSeq = false;

static int hexVal(uint8_t b) {
  if (b >= '0' && b <= '9') return b - '0';
  if (b >= 'A' && b <= 'F') return b - 'A' + 10;
  if (b >= 'a' && b <= 'f') return b - 'a' + 10;
  return -1;
}

static void handleFrame(const char *s) {
  int seq, sp, st;
  if (sscanf(s, "CMD,%d,%d,%d", &seq, &sp, &st) != 3 ||
      seq < 0 || seq > 65535 || sp < -100 || sp > 100 || st < -100 || st > 100) {
    gBadFrames++;
    return;
  }
  uint16_t s16 = (uint16_t)seq;
  if (gHaveSeq) {
    int16_t diff = (int16_t)(s16 - gLastSeq);
    if (diff <= 0) return;                     // duplicate / out-of-order: ignore, not an error
    if (diff > 1) gDropped += (uint32_t)(diff - 1);
  }
  gLastSeq = s16; gHaveSeq = true;
  portENTER_CRITICAL(&gCmdMux);
  gCmd.speed = (int8_t)sp; gCmd.steer = (int8_t)st; gCmd.rxMs = millis(); gCmd.valid = true;
  portEXIT_CRITICAL(&gCmdMux);
}

static void feed(uint8_t b) {
  if (b == '$') { pLen = 0; pCs = 0; pState = P_BODY; return; }   // '$' always resynchronises
  switch (pState) {
    case P_IDLE: break;
    case P_BODY:
      if (b == '*') { pState = P_CS_HI; }
      else if (b < 32 || b > 126 || pLen >= sizeof(pBuf) - 1) { gBadFrames++; pState = P_IDLE; }
      else { pBuf[pLen++] = (char)b; pCs ^= b; }
      break;
    case P_CS_HI: { int h = hexVal(b); if (h < 0) { gBadFrames++; pState = P_IDLE; } else { pCsRx = (uint8_t)(h << 4); pState = P_CS_LO; } break; }
    case P_CS_LO: { int h = hexVal(b); if (h < 0) { gBadFrames++; pState = P_IDLE; } else { pCsRx |= (uint8_t)h; pState = P_EOL; } break; }
    case P_EOL:
      if (b == '\n' || b == '\r') {
        pBuf[pLen] = 0;
        if (pCsRx == pCs) handleFrame(pBuf); else gBadFrames++;
      } else gBadFrames++;
      pState = P_IDLE;
      break;
  }
}

// -------------------------------- Telemetry -------------------------------
static void sendTelemetry() {
  uint16_t d[S_COUNT];
  for (uint8_t i = 0; i < S_COUNT; i++) { uint16_t v = gDistMm[i]; d[i] = (v == DIST_NONE) ? 9999 : v; }
  uint8_t flags = (gEstop ? 1 : 0) | (gCmdTimedOut ? 2 : 0) | (gSensorStale ? 4 : 0);
  char payload[96];
  int n = snprintf(payload, sizeof(payload), "TEL,%lu,%u,%u,%u,%u,%u,%u,%lu,%lu",
                   (unsigned long)millis(), d[S_LEFT], d[S_FRONT_LEFT], d[S_FRONT_CENTER], d[S_FRONT_RIGHT],
                   d[S_RIGHT], flags, (unsigned long)gBadFrames, (unsigned long)gDropped);
  if (n <= 0 || n >= (int)sizeof(payload)) return;
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)payload[i];
  char out[112];
  int m = snprintf(out, sizeof(out), "$%s*%02X\n", payload, cs);
  if (m > 0 && LinkSerial.availableForWrite() >= m) LinkSerial.write((const uint8_t *)out, m);  // never block
}

// ---------------------------------- Arduino --------------------------------
void setup() {
#if DEBUG_PRINT
  Serial.begin(115200);
#endif
  // Motors first, inert
  const uint8_t mpins[] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};
  for (uint8_t p : mpins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pwmInit(PIN_ENA, 0); pwmInit(PIN_ENB, 1);
  motorCoast(MOTOR_L); motorCoast(MOTOR_R);
  servoInit();
  servoWriteUs(SERVO_CENTER_US);                      // wheels straight at boot
  gMotorMutex = xSemaphoreCreateMutex();

  // Sensors
  for (uint8_t i = 0; i < S_COUNT; i++) {
    pinMode(PIN_TRIG[i], OUTPUT); digitalWrite(PIN_TRIG[i], LOW);
    pinMode(PIN_ECHO[i], INPUT);
    gDistMm[i] = DIST_NONE;
    attachInterruptArg(digitalPinToInterrupt(PIN_ECHO[i]), echoISR, (void *)(uintptr_t)i, CHANGE);
  }

  // Link UART (RX buffer sized before begin; parser drains it every ms)
  LinkSerial.setRxBufferSize(256);
  LinkSerial.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);

  xTaskCreatePinnedToCore(sensorTask, "sensor", 3072, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(motorTask,  "motor",  3072, NULL, 4, NULL, 1);
}

void loop() {
  static uint32_t tTel = 0, tDbg = 0;

  int budget = 128;                                   // bounded work per pass
  while (budget-- > 0 && LinkSerial.available()) feed((uint8_t)LinkSerial.read());

  uint32_t now = millis();
  if (now - tTel >= TELEMETRY_PERIOD_MS) { tTel = now; sendTelemetry(); }

#if DEBUG_PRINT
  if (now - tDbg >= 1000) {
    tDbg = now;
    if (Serial.availableForWrite() > 100)
      Serial.printf("L=%u FL=%u FC=%u FR=%u R=%u estop=%d stale=%d cmdTO=%d trips=%lu bad=%lu drop=%lu\n",
                    gDistMm[S_LEFT], gDistMm[S_FRONT_LEFT], gDistMm[S_FRONT_CENTER], gDistMm[S_FRONT_RIGHT],
                    gDistMm[S_RIGHT], gEstop, gSensorStale, gCmdTimedOut, (unsigned long)gEstopCount,
                    (unsigned long)gBadFrames, (unsigned long)gDropped);
  }
#endif
  delay(1);                                           // yields; safety lives in higher-priority tasks
}
