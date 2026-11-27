/*
  ============================================================================
  MESHMERIZE  -  Techfest 2026-27, IIT Bombay      Arduino Nano (ATmega328P)
  Team: RoboTech Innovator          14-channel LFR (CD74HC4067 mux -> A7)
  ============================================================================
  Hardware : NO gyro, NO encoders.  Sensors + junction counting only.
  Display  : SSD1306 128x64 OLED (U8glib 1.x, same constructor as your old code)

  STATE MACHINE
    STATE_IDLE ---SEL---> STATE_CALIBRATE  -> back to IDLE
               ---SEL---> STATE_DRY_RUN    -> STATE_FINISHED (END box, red LED)
               ---SEL---> STATE_ACTUAL_RUN -> STATE_FINISHED (END box, red LED)
               ---SEL---> STATE_SENSOR_VIEW / STATE_TEST (bench tools)
    Inside a run (runMaze):
       FOLLOW (centroid PID, boost gating)
         |-- no signal ..........> VOID/DEAD-END : brake, gap probe, log 'B', U-turn
         |-- END box (wide) .....> LED ON, brake, FINISHED
         '-- outer sensors+wide -> NODE : creep (axle centring), latch L/S/R,
                                   dry: pick + log + loop guard
                                   act: replay compressed path
                                   sensor-locked pivot, cooldown, back to FOLLOW

  BUTTONS : UP (A2) = toggle BOOST ON/OFF     DOWN (A1) = next menu item
            SELECT (A0) = start item / lock calibration step
            Hold SELECT >0.4 s during a run = abort to menu

  MEMORY LAYOUT (SRAM, approx. - see compile report in the chat reply)
    sensors : rawV[14] u16, norm[14] u8, minV/maxV/baseV/scl [14] u16 each
    path    : char path[80]  (raw log, simplified online + final pass)
    EEPROM  : @0   CalData (60 B)  magic, polarity, refWidth, boost, min/max
              @64  PathData (66 B) magic, len, compressed path (<=64)
    No String class, no heap, no Serial buffers (DEBUG_SERIAL = 0).
  ============================================================================
*/

#include "U8glib.h"
#include <EEPROM.h>
#include <avr/pgmspace.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);

// ============================================================================
// 1. PIN DEFINITIONS  (identical to your existing board)  +  NEW END LED
// ============================================================================
// motor pins
#define right_motor_forward   10   // IN1(L298) || TB6612 AIN2
#define right_motor_backward  11   // IN2(L298) || TB6612 AIN1
#define left_motor_forward     3   // IN3(L298) || TB6612 BIN1
#define left_motor_backward    9   // IN4(L298) || TB6612 BIN2
// speed (PWM) pins
#define right_motor_speed      6   // EnA / PWMA
#define left_motor_speed       5   // EnB / PWMB
// buttons (INPUT_PULLUP, active LOW)
#define BUTTON_UP_PIN         A2
#define BUTTON_SELECT_PIN     A0
#define BUTTON_DOWN_PIN       A1
// mux
#define S0   4
#define S1   7
#define S2   8
#define S3  12
#define SIG_PIN A7
// status LED (existing, D13)
#define led 13
// >>> NEW: competition RED LED.  D2 was unused on your board.  <<<
// Wire:  D2 --[220..330 ohm]--> red LED anode, cathode --> GND.
// (Alternative free pin: A3.  Just change this one define.)
#define END_LED_PIN 2

// ============================================================================
// 2. TUNABLE CONSTANTS
// ============================================================================
#define DEBUG_SERIAL        0
#define N_SENS             14

// ---- ADC / mux
#define MUX_SETTLE_US      12      // settle after channel switch
#define ADC_PRESCALER_BITS 0x05    // 0x05=/32 (~28us/conv). Use 0x07 (/128) if readings are noisy.

// ---- calibration
#define FORCE_POLARITY     (-1)    // -1 auto, 0 = white reads LOW, 1 = white reads HIGH
#define CAL_WINDOW_MS      60000UL // 1-minute calibration window (auto-locks at the end)
#define CAL_SAMPLES        32      // baseline averaging (<=64)
#define CAL_MIN_SPAN       100     // min (white-dark) ADC span for a sensor to be "good"
#define CAL_MIN_GOOD       12      // min good sensors for valid calibration
#define ACTIVE_NORM        128     // norm >= this => active (centre sensors)
#define WING_ACTIVE_NORM   100     // wings (0..2, 11..13) are a bit more sensitive
#define NOISE_NORM         40      // norm below this = noise floor (no line)

// ---- sensor geometry (from your photo). Lateral position, centre pitch = 10 units.
// Wings are swept BACK behind the centre row by ~1.3 / 2.6 / 4.5 pitches (sensors 2/1/0).
// Measure your real pitch & adjust if you like - only the PID scale depends on it.
const int16_t SENSOR_POS[N_SENS] PROGMEM = {
  -68, -61, -50,  -35, -25, -15, -5, 5, 15, 25, 35,  50, 61, 68
};

// ---- partition masks (bit i = sensor i)
#define MASK_PEAK_L   0x0001u      // sensor 0
#define MASK_EDGE_L   0x0006u      // sensors 1..2
#define MASK_CENTER   0x07F8u      // sensors 3..10
#define MASK_EDGE_R   0x1800u      // sensors 11..12
#define MASK_PEAK_R   0x2000u      // sensor 13
#define MASK_WING_L   0x0007u      // 0..2
#define MASK_WING_R   0x3800u      // 11..13
#define MASK_OUTER    0x3807u      // 0..2 and 11..13
#define MASK_CORE     0x01E0u      // sensors 5..8 (turn-lock window)

// ---- speeds (PWM 0..255)
#define DRY_SPEED        110
#define ACT_SPEED        140
#define BOOST_SPEED      190
#define APPROACH_SPEED    80       // when edge/peak cue seen
#define CREEP_PWM         70       // node axle-centring step
#define PIVOT_PWM        110
#define PIVOT_SLOW_PWM    70
#define PIVOT_BRAKE_PWM   90
#define BRAKE_REV_PWM    120
#define MAX_REV_PWM       80       // PID may reverse a wheel by this much (sharp bends)
#define ACCEL_STEP         2       // PWM added per control cycle when speeding up

// ---- PID (position units: 10 = one sensor pitch)
#define KP     3.5f
#define KD    25.0f
#define CONTROL_PERIOD_US 2000UL

// ---- boost mode
#define BOOST_IN_DRY        0      // 1 = also allow boost in dry run
#define BOOST_ERR_MAX      15      // |error| must be below this (1.5 pitch)
#define BOOST_THICK_MAX_X10 13     // centre thickness <= 1.3 x refWidth
#define BOOST_ARM_MS      120      // clean tracking time before boost engages
#define BOOST_CUT_MIN_DROP 40      // speed drop that triggers a brake pulse
#define BOOST_CUT_BRAKE_MS 10

// ---- junction / node logic (all relative to refWidth where possible)
#define WIDE_CUE_X10       15      // centre thickness >= 1.5 x refWidth = wide cue
#define ENDBOX_RATIO_X10   20      // END: active >= 2.0 x refWidth ...
#define ENDBOX_CONFIRM_MS 150      // ... AND wide continuously this long  (must be < NODE_CREEP_MS)
#define END_EXTRA_MS      300      // extra creep allowed while still wide
// NODE_CREEP_MS ~= (D_sensor_to_axle_mm + 15 - 1.3*pitch_mm) / creep_speed_mm_per_ms
#define NODE_CREEP_MS     200
#define NODE_BRAKE_REV_MS   8
#define NODE_BRAKE_HOLD_MS 40
#define NODE_SETTLE_MS     15
#define JUNCTION_COOLDOWN_MS 80    // post-junction debounce
#define REARM_QUIET_MS     20      // outer sensors must be quiet this long to re-arm

// ---- void / dead end
#define VOID_CONFIRM_MS    25
#define GAP_PROBE_MS       60      // creep forward to bridge a tiny gap (0 = off)
#define UTURN_DIR          1       // +1 pivot right, -1 pivot left
#define BRAKE_REV_MS       10
#define BRAKE_HOLD_MS      40

// ---- pivots
#define PIVOT_LEAVE_MAX_MS  600
#define PIVOT_TIMEOUT_MS   2200
#define PIVOT_STABLE_SAMPLES  2
#define PIVOT_BRAKE_MS      14
#define PIVOT_SETTLE_MS     30
#define PIVOT_RECOVER_MS   350

// ---- dry-run exploration
#define DRY_PRIORITY    "LSR"      // left-hand rule. Try "RSL" for right-hand.
#define LOOP_MIN_PERIOD     4
#define LOOP_MAX_PERIOD    10
#define RAW_MAX            80
#define EE_PATH_MAX        64

// ---- timing / UI
#define DRY_TIME_LIMIT_MS  180000UL
#define ACT_TIME_LIMIT_MS  150000UL
#define START_DELAY_MS      1500
#define ABORT_HOLD_MS        400
#define DISPLAY_PERIOD_MS    200
#define DISPLAY_IN_RUN         0   // 1 = incremental OLED refresh while driving (costs ~3 ms jitter)

// ---- EEPROM
#define EE_CAL_ADDR    0
#define EE_PATH_ADDR  64
#define EE_CAL_MAGIC  0xA7
#define EE_PATH_MAGIC 0x5C

// ============================================================================
// types (kept above the first function so Arduino's auto-prototypes work)
// ============================================================================
enum State : uint8_t { STATE_IDLE, STATE_CALIBRATE, STATE_DRY_RUN, STATE_ACTUAL_RUN,
                       STATE_FINISHED, STATE_SENSOR_VIEW, STATE_TEST };
enum Screen : uint8_t { SCR_MENU, SCR_CAL1, SCR_CAL2, SCR_RUN, SCR_DONE, SCR_VIEW, SCR_MSG };
enum MsgId : uint8_t { MSG_NONE, MSG_CAL_OK, MSG_CAL_BAD, MSG_CAL_CANCEL, MSG_NO_CAL,
                       MSG_NO_PATH, MSG_TURN_FAIL, MSG_ABORT, MSG_STARTING, MSG_MISMATCH };
enum RunResult : uint8_t { RES_END, RES_ABORT, RES_FAIL };

struct Exits { bool L, S, R, endBox; };

struct CalData {                       // EEPROM @ EE_CAL_ADDR
  uint8_t  magic, whiteHigh, refWidth, boost;
  uint16_t minV[N_SENS];
  uint16_t maxV[N_SENS];
};
struct PathData {                      // EEPROM @ EE_PATH_ADDR
  uint8_t magic, len;
  char    p[EE_PATH_MAX];
};

// ============================================================================
// globals
// ============================================================================
// ---- sensing / calibration
uint16_t rawV[N_SENS];
uint8_t  norm[N_SENS];
uint8_t  actThr[N_SENS];
uint16_t minV[N_SENS], maxV[N_SENS], baseV[N_SENS], scl[N_SENS];  // baseV = step-1 baseline
uint16_t goodMask = 0;                 // sensors with usable span
bool     whiteHigh = true;             // true: white reads high ADC
uint8_t  refWidth = 3;                 // straight-line active-sensor count (1x reference)
uint8_t  goodCnt = 0, endMinActive = 12;
bool     calValid = false;

uint16_t actMask = 0;                  // bit i = sensor i active
uint8_t  actCnt = 0, ctrCnt = 0, noiseCnt = 0;

// ---- UI / state
State    state = STATE_IDLE;
Screen   scr = SCR_MENU;
MsgId    msgId = MSG_NONE;
uint8_t  menuItem = 0;
#define  MENU_ITEMS 5
bool     boostMode = false;
bool     endLedOn = false;
bool     btnPrev[3] = {false, false, false};
uint32_t btnT[3] = {0, 0, 0};
bool     dispPaging = false;
uint32_t dispLast = 0;

// calibration display
uint16_t calBits = 0;
uint8_t  calLive = 0, calGoodLive = 0;
uint16_t calSecLeft = 60;

// ---- run state
bool     isDry = true;
uint8_t  junctionCount = 0;
uint16_t nodeCount = 0;
bool     mismatchFlag = false;
uint32_t runStartMs = 0, runFinalMs = 0;
char     path[RAW_MAX];
uint8_t  pathLen = 0;
uint8_t  replayIdx = 0;
bool     escapeFlag = false;
char     forbidTok = 0;
bool     nodeArmed = true, quietFlag = false;
uint32_t quietSince = 0, rearmAt = 0;
float    prevErr = 0;
int16_t  curSpeed = APPROACH_SPEED;
uint32_t cleanSince = 0;

// ============================================================================
// 3. LOW-LEVEL HELPERS
// ============================================================================
static inline uint8_t popcount16(uint16_t v) {
  uint8_t c = 0;
  while (v) { v &= (v - 1); c++; }
  return c;
}

void selectChannel(uint8_t ch) {
  digitalWrite(S0, bitRead(ch, 0));
  digitalWrite(S1, bitRead(ch, 1));
  digitalWrite(S2, bitRead(ch, 2));
  digitalWrite(S3, bitRead(ch, 3));
}

void setEndLed(bool on) {
  endLedOn = on;
  digitalWrite(END_LED_PIN, on ? HIGH : LOW);   // competition red LED
  digitalWrite(led, on ? HIGH : LOW);           // board LED mirrors it
}

// debounced press edge
bool pressedEdge(uint8_t pin) {
  uint8_t id = (pin == BUTTON_UP_PIN) ? 0 : (pin == BUTTON_SELECT_PIN) ? 1 : 2;
  bool down = (digitalRead(pin) == LOW);
  uint32_t now = millis();
  if (down && !btnPrev[id] && (now - btnT[id] > 40)) { btnPrev[id] = true; btnT[id] = now; return true; }
  if (!down && btnPrev[id] && (now - btnT[id] > 40)) { btnPrev[id] = false; btnT[id] = now; }
  return false;
}

// SELECT held for ABORT_HOLD_MS during a run
bool abortHeld() {
  static uint32_t t0 = 0;
  if (digitalRead(BUTTON_SELECT_PIN) == LOW) {
    if (t0 == 0) t0 = millis() | 1;
    else if (millis() - t0 > ABORT_HOLD_MS) { t0 = 0; return true; }
  } else t0 = 0;
  return false;
}

// ============================================================================
// 4. MOTOR DRIVE  (direction pins digital, speed pins PWM - same as your wiring)
// ============================================================================
static void driveOne(uint8_t fwd, uint8_t bwd, uint8_t pwm, int16_t v) {
  if (v > 0)      { digitalWrite(fwd, HIGH); digitalWrite(bwd, LOW); }
  else if (v < 0) { digitalWrite(fwd, LOW);  digitalWrite(bwd, HIGH); v = -v; }
  else            { digitalWrite(fwd, LOW);  digitalWrite(bwd, LOW); }
  if (v > 255) v = 255;
  analogWrite(pwm, v);
}
void setMotors(int16_t l, int16_t r) {
  driveOne(left_motor_forward,  left_motor_backward,  left_motor_speed,  l);
  driveOne(right_motor_forward, right_motor_backward, right_motor_speed, r);
}
void motorsStop() { setMotors(0, 0); }
// active short-brake: both inputs HIGH, PWM full (works for TB6612 and L298)
void motorsShortBrake() {
  digitalWrite(left_motor_forward, HIGH);  digitalWrite(left_motor_backward, HIGH);
  digitalWrite(right_motor_forward, HIGH); digitalWrite(right_motor_backward, HIGH);
  analogWrite(left_motor_speed, 255);      analogWrite(right_motor_speed, 255);
}
// straight-line stop: short reverse pulse, then short-brake, then release
void brakeStop(uint8_t revMs, uint8_t holdMs) {
  if (revMs) { setMotors(-BRAKE_REV_PWM, -BRAKE_REV_PWM); delay(revMs); }
  motorsShortBrake();
  delay(holdMs);
  motorsStop();
}

// ============================================================================
// 5. SENSING  (calibrated per-sensor normalisation + partition features)
// ============================================================================
void computeScales() {
  goodMask = 0;
  for (uint8_t i = 0; i < N_SENS; i++) {
    uint16_t span = (maxV[i] > minV[i]) ? (maxV[i] - minV[i]) : 0;
    if (span >= CAL_MIN_SPAN) { goodMask |= (1u << i); scl[i] = (uint16_t)(65280UL / span); }
    else scl[i] = 0;
    actThr[i] = (i <= 2 || i >= 11) ? WING_ACTIVE_NORM : ACTIVE_NORM;
  }
  goodCnt = popcount16(goodMask);
  int8_t e = (int8_t)goodCnt - 1;               // END box needs (almost) every sensor
  if (e < 8) e = 8;
  if (e > 13) e = 13;
  endMinActive = (uint8_t)e;
}

// raw -> 0..255 relative activation (1 = this sensor's own calibrated white peak)
uint8_t normOf(uint8_t i, uint16_t v) {
  if (!(goodMask & (1u << i))) return 0;
  uint16_t d;
  if (whiteHigh) d = (v > minV[i]) ? (v - minV[i]) : 0;
  else           d = (maxV[i] > v) ? (maxV[i] - v) : 0;
  uint32_t n = ((uint32_t)d * scl[i]) >> 8;
  return (n > 255) ? 255 : (uint8_t)n;
}

void readRawAll() {
  for (uint8_t i = 0; i < N_SENS; i++) {
    selectChannel(i);
    delayMicroseconds(MUX_SETTLE_US);
    rawV[i] = analogRead(SIG_PIN);
  }
}

void readSensors() {
  uint16_t m = 0;
  uint8_t nz = 0;
  for (uint8_t i = 0; i < N_SENS; i++) {
    selectChannel(i);
    delayMicroseconds(MUX_SETTLE_US);
    rawV[i] = analogRead(SIG_PIN);
    uint8_t n = normOf(i, rawV[i]);
    norm[i] = n;
    if (n >= actThr[i]) m |= (1u << i);
    if (n >= NOISE_NORM) nz++;
  }
  actMask  = m;
  actCnt   = popcount16(m);
  ctrCnt   = popcount16(m & MASK_CENTER);
  noiseCnt = nz;
}

// relative thickness x10 (1.0x == refWidth)
static inline uint16_t thick10(uint8_t cnt) {
  return (uint16_t)cnt * 10u / (refWidth ? refWidth : 1);
}
bool isEndWide() {
  return (actCnt >= endMinActive) && ((uint16_t)actCnt * 10u >= (uint16_t)refWidth * ENDBOX_RATIO_X10);
}

// weighted line position.  Centre partition first; falls back to the whole
// array (wings give large leverage) for sharp bends. Narrow lines just work:
// the centroid does not care how many sensors are active.
int16_t linePosition() {
  int32_t num = 0;
  uint16_t den = 0;
  for (uint8_t i = 3; i <= 10; i++) {
    uint8_t w = (norm[i] >= NOISE_NORM) ? norm[i] : 0;
    num += (int32_t)w * (int16_t)pgm_read_word(&SENSOR_POS[i]);
    den += w;
  }
  if (den < 60) {                                // centre empty -> use all 14
    num = 0; den = 0;
    for (uint8_t i = 0; i < N_SENS; i++) {
      uint8_t w = (norm[i] >= NOISE_NORM) ? norm[i] : 0;
      num += (int32_t)w * (int16_t)pgm_read_word(&SENSOR_POS[i]);
      den += w;
    }
  }
  if (den == 0) return 0;
  return (int16_t)(num / (int32_t)den);
}

// ============================================================================
// 6. EEPROM  (calibration, boost flag, compressed path)
// ============================================================================
void loadCalibration() {
  CalData cd;
  EEPROM.get(EE_CAL_ADDR, cd);
  if (cd.magic == EE_CAL_MAGIC) {
    whiteHigh = cd.whiteHigh;
    refWidth  = cd.refWidth;
    boostMode = cd.boost ? true : false;
    for (uint8_t i = 0; i < N_SENS; i++) { minV[i] = cd.minV[i]; maxV[i] = cd.maxV[i]; }
    computeScales();
    calValid = (goodCnt >= CAL_MIN_GOOD) && refWidth >= 1 && refWidth <= 8;
  } else {
    whiteHigh = true; refWidth = 3; calValid = false;
    for (uint8_t i = 0; i < N_SENS; i++) { minV[i] = 0; maxV[i] = 1023; }
    computeScales();
  }
}
void saveCalibration() {
  CalData cd;
  cd.magic = EE_CAL_MAGIC; cd.whiteHigh = whiteHigh; cd.refWidth = refWidth; cd.boost = boostMode;
  for (uint8_t i = 0; i < N_SENS; i++) { cd.minV[i] = minV[i]; cd.maxV[i] = maxV[i]; }
  EEPROM.put(EE_CAL_ADDR, cd);
}
void saveBoost() { EEPROM.update(EE_CAL_ADDR + 3, boostMode ? 1 : 0); }   // 'boost' = 4th byte

void savePath() {
  PathData pd;
  uint8_t n = (pathLen > EE_PATH_MAX) ? EE_PATH_MAX : pathLen;
  pd.magic = EE_PATH_MAGIC; pd.len = n;
  for (uint8_t i = 0; i < EE_PATH_MAX; i++) pd.p[i] = (i < n) ? path[i] : 0;
  EEPROM.put(EE_PATH_ADDR, pd);
}
void loadPath() {
  PathData pd;
  EEPROM.get(EE_PATH_ADDR, pd);
  pathLen = 0;
  if (pd.magic == EE_PATH_MAGIC && pd.len <= EE_PATH_MAX) {
    for (uint8_t i = 0; i < pd.len; i++) path[i] = pd.p[i];
    pathLen = pd.len;
  }
}

// ============================================================================
// 7. DISPLAY  (OLED; incremental paging so it never blocks the PID for long)
// ============================================================================
#define TXT(x, y, s)  u8g.drawStrP((x), (y), U8G_PSTR(s))
static void num(uint8_t x, uint8_t y, long v) { u8g.setPrintPos(x, y); u8g.print(v); }

static void bitsStr(char *b, uint16_t m) {           // sensor 0 .. 13 left->right
  for (uint8_t i = 0; i < N_SENS; i++) b[i] = (m & (1u << i)) ? '1' : '0';
  b[N_SENS] = 0;
}

static void drawMenu() {
  u8g.setFont(u8g_font_6x10);
  TXT(0, 10, "RoboTech Innovator");
  u8g.setFont(u8g_font_7x14B);
  switch (menuItem) {
    case 0: TXT(4, 28, ">CALIBRATE");  break;
    case 1: TXT(4, 28, ">DRY RUN");    break;
    case 2: TXT(4, 28, ">ACTUAL RUN"); break;
    case 3: TXT(4, 28, ">SENSOR VIEW"); break;
    default: TXT(4, 28, ">LED/MOTOR TEST"); break;
  }
  u8g.setFont(u8g_font_6x10);
  if (boostMode) { TXT(0, 40, "BOOST ON"); } else { TXT(0, 40, "BOOST OFF"); }
  if (calValid)  { TXT(66, 40, "CAL OK"); } else { TXT(66, 40, "CAL ??"); }
  TXT(0, 51, "REF:"); num(24, 51, refWidth);
  TXT(48, 51, "PATH:"); num(78, 51, pathLen);
  TXT(0, 62, "U:boost D:next S:go");
}

static void drawCal1() {
  char b[N_SENS + 1];
  u8g.setFont(u8g_font_6x10);
  TXT(0, 10, "CAL 1/2 STRAIGHT");
  num(104, 10, calSecLeft); TXT(116, 10, "s");
  TXT(0, 22, "Active:"); num(48, 22, calLive);
  bitsStr(b, calBits); u8g.drawStr(0, 34, b);
  TXT(0, 46, "Centre bot on line");
  TXT(0, 58, "SEL=capture DN=exit");
}

static void drawCal2() {
  u8g.setFont(u8g_font_6x10);
  TXT(0, 9, "CAL 2/2 SWEEP L/R");
  num(104, 9, calSecLeft); TXT(116, 9, "s");
  for (uint8_t i = 0; i < N_SENS; i++) {
    uint16_t span = (maxV[i] > minV[i]) ? (maxV[i] - minV[i]) : 0;
    uint16_t h = span / 20; if (h > 34) h = 34;
    uint8_t x = i * 9;
    if (span >= CAL_MIN_SPAN) u8g.drawBox(x, 48 - h, 7, h > 0 ? h : 1);
    else u8g.drawFrame(x, 46, 7, 2);
  }
  TXT(0, 62, "Good:"); num(32, 62, calGoodLive); TXT(46, 62, "/14 SEL=lock");
}

static void drawRun() {
  char b[N_SENS + 1];
  u8g.setFont(u8g_font_6x10);
  if (isDry) { TXT(0, 9, "DRY"); } else { TXT(0, 9, "ACT"); }
  if (boostMode) { TXT(40, 9, "BOOST ON"); } else { TXT(40, 9, "BOOST OFF"); }
  TXT(0, 21, "J:"); num(12, 21, junctionCount);
  TXT(42, 21, "N:"); num(54, 21, nodeCount);
  uint32_t el = millis() - runStartMs;
  uint32_t lim = isDry ? DRY_TIME_LIMIT_MS : ACT_TIME_LIMIT_MS;
  TXT(84, 21, "L:"); num(96, 21, (el >= lim) ? 0 : (long)((lim - el) / 1000UL));
  TXT(0, 33, "REF:"); num(24, 33, refWidth);
  TXT(48, 33, "ACT:"); num(72, 33, actCnt);
  if (mismatchFlag) { TXT(96, 33, "MISM"); }
  bitsStr(b, actMask); u8g.drawStr(0, 46, b);
  if (endLedOn) { TXT(0, 60, "END: REACHED  LED ON"); } else { TXT(0, 60, "END: --"); }
}

static void drawDone() {
  u8g.setFont(u8g_font_7x14B);
  TXT(4, 14, "END REACHED!");
  u8g.setFont(u8g_font_6x10);
  TXT(0, 27, "RED LED ON  t="); num(86, 27, (long)(runFinalMs / 1000UL)); TXT(104, 27, "s");
  if (isDry) { TXT(0, 39, "DRY DONE. Path:"); } else { TXT(0, 39, "ACTUAL DONE. Path:"); }
  num(100, 39, pathLen);
  char b[22]; uint8_t n = (pathLen > 20) ? 20 : pathLen;
  for (uint8_t i = 0; i < n; i++) b[i] = path[i];
  b[n] = 0; u8g.drawStr(0, 51, b);
  TXT(0, 62, "SEL = menu");
}

static void drawView() {
  char b[N_SENS + 1];
  u8g.setFont(u8g_font_6x10);
  TXT(0, 10, "SENSOR VIEW");
  bitsStr(b, actMask); u8g.setFont(u8g_font_7x14B); u8g.drawStr(0, 28, b);
  u8g.setFont(u8g_font_6x10);
  TXT(0, 40, "Act:"); num(30, 40, actCnt); TXT(54, 40, "Ctr:"); num(84, 40, ctrCnt);
  TXT(0, 51, "REF:"); num(30, 51, refWidth); TXT(54, 51, "Good:"); num(90, 51, goodCnt);
  TXT(0, 62, "SEL/DN = exit");
}

static void drawMsg() {
  u8g.setFont(u8g_font_7x14B);
  switch (msgId) {
    case MSG_CAL_OK:     TXT(8, 28, "CAL LOCKED"); u8g.setFont(u8g_font_6x10); TXT(8, 44, "Saved to EEPROM"); break;
    case MSG_CAL_BAD:    TXT(8, 28, "CAL BAD!");   u8g.setFont(u8g_font_6x10); TXT(8, 44, "Check sensors/line"); break;
    case MSG_CAL_CANCEL: TXT(8, 28, "CAL CANCEL");  u8g.setFont(u8g_font_6x10); TXT(8, 44, "Old data restored"); break;
    case MSG_NO_CAL:     TXT(8, 28, "CALIBRATE!");  u8g.setFont(u8g_font_6x10); TXT(8, 44, "No valid calibration"); break;
    case MSG_NO_PATH:    TXT(8, 28, "NO PATH");     u8g.setFont(u8g_font_6x10); TXT(8, 44, "Do a DRY RUN first"); break;
    case MSG_TURN_FAIL:  TXT(8, 28, "TURN FAIL");   u8g.setFont(u8g_font_6x10); TXT(8, 44, "Line not re-acquired"); break;
    case MSG_ABORT:      TXT(8, 28, "ABORTED");     break;
    case MSG_MISMATCH:   TXT(8, 28, "PATH MISMATCH"); break;
    case MSG_STARTING:   TXT(8, 28, "STARTING...");  u8g.setFont(u8g_font_6x10); TXT(8, 44, "Hands off the bot"); break;
    default: break;
  }
}

void drawScreen() {
  switch (scr) {
    case SCR_MENU: drawMenu(); break;
    case SCR_CAL1: drawCal1(); break;
    case SCR_CAL2: drawCal2(); break;
    case SCR_RUN:  drawRun();  break;
    case SCR_DONE: drawDone(); break;
    case SCR_VIEW: drawView(); break;
    case SCR_MSG:  drawMsg();  break;
  }
}

// one OLED page per call; whole frame at most every DISPLAY_PERIOD_MS
void displayService() {
  if (!dispPaging) {
    if (millis() - dispLast < DISPLAY_PERIOD_MS) return;
    dispLast = millis();
    u8g.firstPage();
    dispPaging = true;
  }
  drawScreen();
  dispPaging = u8g.nextPage();
}
// full frame now (state changes, stopped robot)
void displayBlocking() {
  dispPaging = false;
  u8g.firstPage();
  do { drawScreen(); } while (u8g.nextPage());
  dispLast = millis();
}
void showMsg(MsgId id, uint16_t ms) {
  msgId = id; scr = SCR_MSG;
  displayBlocking();
  delay(ms);
}

// ============================================================================
// 8. TAHMID CALIBRATION  (1-minute window, 2 steps)
//    Step 1: bot centred on the 30 mm line -> baseline + refWidth
//    Step 2: operator rocks the bot left/right so wings/edges sweep the stripe
//            -> per-sensor min/max (peakVal = white extreme) -> thresholds
// ============================================================================
static bool polarityGuess() {
  if (FORCE_POLARITY >= 0) return FORCE_POLARITY ? true : false;
  uint16_t c = ((uint16_t)rawV[5] + rawV[6] + rawV[7] + rawV[8]) / 4;
  uint16_t o = ((uint16_t)rawV[0] + rawV[1] + rawV[12] + rawV[13]) / 4;
  return c >= o;
}

bool runTahmidCalibration() {
  uint32_t t0 = millis();
  uint32_t tDraw = 0;
  setEndLed(false);

  // ---------------- step 1 : straight baseline ----------------
  scr = SCR_CAL1;
  for (;;) {
    readRawAll();
    uint16_t lo = 1023, hi = 0;
    for (uint8_t i = 0; i < N_SENS; i++) { if (rawV[i] < lo) lo = rawV[i]; if (rawV[i] > hi) hi = rawV[i]; }
    uint16_t mid = (lo + hi) / 2;
    bool wh = polarityGuess();
    calBits = 0;
    for (uint8_t i = 0; i < N_SENS; i++) if (wh ? (rawV[i] > mid) : (rawV[i] < mid)) calBits |= (1u << i);
    if ((hi - lo) < CAL_MIN_SPAN) calBits = 0;
    calLive = popcount16(calBits);
    uint32_t el = millis() - t0;
    calSecLeft = (el >= CAL_WINDOW_MS) ? 0 : (uint16_t)((CAL_WINDOW_MS - el) / 1000UL);
    if (millis() - tDraw > DISPLAY_PERIOD_MS) { tDraw = millis(); displayBlocking(); }
    digitalWrite(led, (millis() / 250) & 1);
    if (pressedEdge(BUTTON_SELECT_PIN) && calLive > 0) break;
    if (pressedEdge(BUTTON_DOWN_PIN) || el >= CAL_WINDOW_MS) { digitalWrite(led, LOW); showMsg(MSG_CAL_CANCEL, 800); return false; }
  }
  // capture baseline (average) + polarity
  uint16_t sum[N_SENS];
  for (uint8_t i = 0; i < N_SENS; i++) sum[i] = 0;
  for (uint8_t s = 0; s < CAL_SAMPLES; s++) {
    readRawAll();
    for (uint8_t i = 0; i < N_SENS; i++) sum[i] += rawV[i];
    delay(2);
  }
  for (uint8_t i = 0; i < N_SENS; i++) { baseV[i] = sum[i] / CAL_SAMPLES; rawV[i] = baseV[i]; }
  whiteHigh = polarityGuess();

  // ---------------- step 2 : umbrella sweep ----------------
  for (uint8_t i = 0; i < N_SENS; i++) { minV[i] = baseV[i]; maxV[i] = baseV[i]; }
  scr = SCR_CAL2;
  for (;;) {
    readRawAll();
    calGoodLive = 0;
    for (uint8_t i = 0; i < N_SENS; i++) {
      if (rawV[i] < minV[i]) minV[i] = rawV[i];
      if (rawV[i] > maxV[i]) maxV[i] = rawV[i];
      if ((maxV[i] - minV[i]) >= CAL_MIN_SPAN) calGoodLive++;
    }
    uint32_t el = millis() - t0;
    calSecLeft = (el >= CAL_WINDOW_MS) ? 0 : (uint16_t)((CAL_WINDOW_MS - el) / 1000UL);
    if (millis() - tDraw > DISPLAY_PERIOD_MS) { tDraw = millis(); displayBlocking(); }
    digitalWrite(led, (millis() / 120) & 1);
    if (pressedEdge(BUTTON_SELECT_PIN) && calGoodLive >= CAL_MIN_GOOD) break;
    if (el >= CAL_WINDOW_MS) { if (calGoodLive >= CAL_MIN_GOOD) break; loadCalibration(); digitalWrite(led, LOW); showMsg(MSG_CAL_CANCEL, 800); return false; }
    if (pressedEdge(BUTTON_DOWN_PIN)) { loadCalibration(); digitalWrite(led, LOW); showMsg(MSG_CAL_CANCEL, 800); return false; }
  }
  digitalWrite(led, LOW);

  // ---------------- lock ----------------
  computeScales();
  uint8_t rw = 0;                                   // relative straight-line thickness reference
  for (uint8_t i = 0; i < N_SENS; i++) if (normOf(i, baseV[i]) >= actThr[i]) rw++;
  refWidth = rw;
  calValid = (goodCnt >= CAL_MIN_GOOD) && refWidth >= 1 && refWidth <= 8;
  saveCalibration();
  showMsg(calValid ? MSG_CAL_OK : MSG_CAL_BAD, 1200);
  return calValid;
}

// ============================================================================
// 9. PATH LOGGER, LOOP GUARD, 'B' COMPRESSION
//    Angle algebra: L=270, R=90, S=0, B=180 (sum mod 360).
//    LBL->S  SBL->R  SBR->L  LBR->B  RBL->B  SBS->B  (B results are re-reduced)
// ============================================================================
static uint16_t angleOf(char t) {
  switch (t) { case 'L': return 270; case 'R': return 90; case 'B': return 180; default: return 0; }
}
static char turnSum(char a, char b, char c) {
  uint16_t s = (angleOf(a) + angleOf(b) + angleOf(c)) % 360;
  switch (s) { case 90: return 'R'; case 180: return 'B'; case 270: return 'L'; default: return 'S'; }
}
// online: X B Y at the tail -> single token (keeps the buffer small)
static void simplifyTail() {
  while (pathLen >= 3 && path[pathLen - 2] == 'B') {
    char z = turnSum(path[pathLen - 3], 'B', path[pathLen - 1]);
    pathLen -= 2;
    path[pathLen - 1] = z;
  }
}
// full pass (run at END): repeat until no 'B' can be removed
void compressPath() {
  bool changed;
  do {
    changed = false;
    for (uint8_t i = 1; (uint8_t)(i + 1) < pathLen; i++) {
      if (path[i] == 'B') {
        path[i - 1] = turnSum(path[i - 1], 'B', path[i + 1]);
        for (uint8_t k = i; (uint8_t)(k + 2) < pathLen; k++) path[k] = path[k + 2];
        pathLen -= 2;
        changed = true;
        break;
      }
    }
  } while (changed);
}

// closed-cycle detector: the last 2p tokens are two identical blocks whose net
// rotation is +/-360 deg -> the bot has gone round a loop twice.
static uint8_t loopPeriod() {
  for (uint8_t p = LOOP_MIN_PERIOD; p <= LOOP_MAX_PERIOD; p++) {
    if (pathLen < 2 * p) break;
    bool same = true;
    int8_t rot = 0;
    for (uint8_t j = 0; j < p; j++) {
      char a = path[pathLen - 1 - j];
      char b = path[pathLen - 1 - j - p];
      if (a != b || a == 'B') { same = false; break; }
      rot += (a == 'L') ? -1 : (a == 'R') ? 1 : 0;
    }
    if (same && (rot == 4 || rot == -4)) return p;
  }
  return 0;
}

void logToken(char t) {
  if (pathLen >= RAW_MAX) return;
  path[pathLen++] = t;
  simplifyTail();
  if (t != 'B') {
    uint8_t p = loopPeriod();
    if (p) {                                   // loop trap: drop both laps, force a different exit
      forbidTok = path[pathLen - 2 * p];
      pathLen -= 2 * p;
      escapeFlag = true;
    }
  }
}

static bool exitAvail(char t, const Exits &e) {
  return (t == 'L' && e.L) || (t == 'S' && e.S) || (t == 'R' && e.R);
}
static char pickExit(const Exits &e, bool useEscape) {
  uint8_t nAvail = (e.L ? 1 : 0) + (e.S ? 1 : 0) + (e.R ? 1 : 0);
  const char *order = DRY_PRIORITY;
  for (uint8_t k = 0; k < 3; k++) {
    char t = order[k];
    if (!exitAvail(t, e)) continue;
    if (useEscape && escapeFlag && nAvail > 1 && t == forbidTok) continue;
    return t;
  }
  return 'B';
}

// ============================================================================
// 10. SENSOR-LOCKED PIVOT / U-TURN
//     dir: +1 = pivot right, -1 = pivot left
//     anyLine=false : stop when the CORE (5..8) acquires the target line
//     anyLine=true  : stop as soon as ANY sensor sees a line (dead-end U-turn)
// ============================================================================
bool pivotToLine(int8_t dir, bool anyLine) {
  uint32_t t0 = millis();
  uint8_t stable = 0;
  setMotors(dir * PIVOT_PWM, -dir * PIVOT_PWM);

  // phase 1: leave whatever line is under us
  for (;;) {
    readSensors();
    bool gone = anyLine ? (noiseCnt == 0) : (ctrCnt == 0);
    stable = gone ? (stable + 1) : 0;
    if (stable >= 2) break;
    if (millis() - t0 > PIVOT_LEAVE_MAX_MS) break;
  }
  // phase 2: acquire the target
  stable = 0;
  for (;;) {
    readSensors();
    bool got = anyLine ? (actCnt > 0) : ((actMask & MASK_CORE) != 0);
    stable = got ? (stable + 1) : 0;
    if (stable >= PIVOT_STABLE_SAMPLES) break;
    if (millis() - t0 > PIVOT_TIMEOUT_MS || abortHeld()) { motorsStop(); return false; }
  }
  // brake settle (never a fixed-delay-only turn: the stop is sensor triggered)
  setMotors(-dir * PIVOT_BRAKE_PWM, dir * PIVOT_BRAKE_PWM);
  delay(PIVOT_BRAKE_MS);
  motorsShortBrake();
  delay(PIVOT_SETTLE_MS);
  motorsStop();
  // overshoot recovery: if the line slipped away, creep back until any line
  readSensors();
  if (noiseCnt == 0) {
    setMotors(-dir * PIVOT_SLOW_PWM, dir * PIVOT_SLOW_PWM);
    uint32_t t1 = millis();
    while (millis() - t1 < PIVOT_RECOVER_MS) { readSensors(); if (actCnt > 0) break; }
    motorsStop();
  }
  return true;
}

// ============================================================================
// 11. NODE HANDLING  (junction counting without encoders)
// ============================================================================
// creep forward (axle centring), latch L/R wing sightings, confirm END, then
// classify the straight exit once the bot has settled.
Exits scanNode() {
  Exits ex = { false, false, false, false };
  uint16_t latchL = 0, latchR = 0, prev = actMask;
  uint32_t wideStart = isEndWide() ? millis() : 0;
  uint32_t t0 = millis();
  setMotors(CREEP_PWM, CREEP_PWM);
  for (;;) {
    bool w = isEndWide();
    if (w) {
      if (!wideStart) wideStart = millis();
      if (millis() - wideStart >= ENDBOX_CONFIRM_MS) {   // white END box confirmed
        setEndLed(true);                                  // RED LED immediately
        ex.endBox = true;
        return ex;
      }
    } else wideStart = 0;
    uint32_t el = millis() - t0;
    if (el >= NODE_CREEP_MS && !(w && el < (uint32_t)NODE_CREEP_MS + END_EXTRA_MS)) break;
    readSensors();
    uint16_t both = prev & actMask;                      // seen on 2 consecutive samples
    latchL |= both & MASK_WING_L;
    latchR |= both & MASK_WING_R;
    prev = actMask;
  }
  brakeStop(NODE_BRAKE_REV_MS, NODE_BRAKE_HOLD_MS);
  delay(NODE_SETTLE_MS);
  uint8_t votes = 0;
  for (uint8_t k = 0; k < 8; k++) {
    readSensors();
    if (ctrCnt >= 1 && thick10(ctrCnt) < WIDE_CUE_X10) votes++;
    delay(1);
  }
  ex.S = (votes >= 5);
  ex.L = (popcount16(latchL) >= 2);
  ex.R = (popcount16(latchR) >= 2);
  return ex;
}

void finishAtEnd(bool dry) {
  setEndLed(true);
  motorsShortBrake();
  delay(60);
  motorsStop();
  runFinalMs = millis() - runStartMs;
  if (dry) {
    compressPath();       // dead-end ('B') pruning pass
    savePath();           // restart-safe: survives reset / power cycle
  }
  isDry = dry;
  scr = SCR_DONE;
  displayBlocking();
}

static bool execTurn(char t, RunResult *res) {
  bool ok = true;
  if (t == 'L')      ok = pivotToLine(-1, false);
  else if (t == 'R') ok = pivotToLine(+1, false);
  else if (t == 'B') ok = pivotToLine(UTURN_DIR, true);
  if (!ok) { motorsStop(); *res = RES_FAIL; return false; }
  return true;
}

// returns true when the run is over (END / fail); *res then holds the result
bool handleNode(bool dry, RunResult *res) {
  Exits ex = scanNode();
  if (ex.endBox) { finishAtEnd(dry); *res = RES_END; return true; }
  nodeCount++;
  uint8_t n = (ex.L ? 1 : 0) + (ex.S ? 1 : 0) + (ex.R ? 1 : 0);
  char tok;
  if (n == 0) {                                   // nothing to follow -> dead end
    if (dry) logToken('B');
    tok = 'B';
  } else if (n == 1) {                            // corner: forced, not a decision
    tok = ex.L ? 'L' : (ex.R ? 'R' : 'S');
  } else {                                        // real junction: junctionCount++
    junctionCount++;
    if (dry) {
      tok = pickExit(ex, true);
      escapeFlag = false;                       // escape applies to one decision only
      logToken(tok);                              // may set escapeFlag for the NEXT decision
    } else {
      char want = (replayIdx < pathLen) ? path[replayIdx] : 0;
      replayIdx++;
      if (want && exitAvail(want, ex)) tok = want;
      else { mismatchFlag = true; tok = pickExit(ex, false); }   // fallback: stay alive
    }
  }
  if (!execTurn(tok, res)) return true;
  return false;
}

// no signal anywhere: bridge tiny gaps, otherwise dead-end U-turn
bool handleVoid(bool dry, RunResult *res) {
  brakeStop(BRAKE_REV_MS, BRAKE_HOLD_MS);
  if (GAP_PROBE_MS) {
    setMotors(CREEP_PWM, CREEP_PWM);
    uint32_t t0 = millis();
    while (millis() - t0 < GAP_PROBE_MS) {
      readSensors();
      if (noiseCnt > 0) { motorsStop(); return false; }   // it was just a gap
    }
    motorsStop();
  }
  nodeCount++;
  if (dry) logToken('B');
  if (!execTurn('B', res)) return true;
  return false;
}

// ============================================================================
// 12. MAZE RUN  (dry explorer / actual replayer share one engine)
// ============================================================================
RunResult runMaze(bool dry) {
  isDry = dry;
  junctionCount = 0; nodeCount = 0; replayIdx = 0;
  mismatchFlag = false; escapeFlag = false; forbidTok = 0;
  if (dry) pathLen = 0;
  setEndLed(false);
  prevErr = 0; curSpeed = APPROACH_SPEED; cleanSince = 0;
  nodeArmed = true; quietFlag = false; rearmAt = 0;
  bool voidActive = false;
  uint32_t voidSince = 0;
  bool boostAllowed = boostMode && (!dry || BOOST_IN_DRY);
  int16_t baseSpd = dry ? DRY_SPEED : ACT_SPEED;

  runStartMs = millis();
  scr = SCR_RUN;
  displayBlocking();
  uint32_t tLoop = micros();

  for (;;) {
    while ((uint32_t)(micros() - tLoop) < CONTROL_PERIOD_US) { }
    tLoop = micros();
    readSensors();
    if (abortHeld()) { motorsStop(); return RES_ABORT; }
#if DISPLAY_IN_RUN
    displayService();
#endif
    uint32_t now = millis();

    // ---- re-arm node detector (cooldown + outer sensors quiet)
    if (!nodeArmed) {
      if ((actMask & MASK_OUTER) == 0) {
        if (!quietFlag) { quietFlag = true; quietSince = now; }
        else if (now - quietSince >= REARM_QUIET_MS && now >= rearmAt) nodeArmed = true;
      } else quietFlag = false;
    }

    RunResult res = RES_END;

    // ---- VOID : absolutely no signal across the array (a narrow line is NOT void)
    if (noiseCnt == 0 && !isEndWide()) {
      if (!voidActive) { voidActive = true; voidSince = now; }
      if (now - voidSince >= VOID_CONFIRM_MS) {
        voidActive = false;
        if (handleVoid(dry, &res)) return res;
        nodeArmed = false; quietFlag = false; rearmAt = millis() + JUNCTION_COOLDOWN_MS;
        prevErr = 0; curSpeed = APPROACH_SPEED; cleanSince = 0;
        tLoop = micros();
      }
      continue;                                   // coast on last command meanwhile
    }
    voidActive = false;

    // ---- features
    bool outer   = (actMask & MASK_OUTER) != 0;
    bool peaks   = (actMask & (MASK_PEAK_L | MASK_PEAK_R)) != 0;
    uint16_t ct  = thick10(ctrCnt);
    bool wideCue = (ct >= WIDE_CUE_X10);
    bool endWide = isEndWide();

    // ---- NODE / END
    if (endWide || (nodeArmed && outer && (wideCue || peaks))) {
      if (handleNode(dry, &res)) return res;
      nodeArmed = false; quietFlag = false; rearmAt = millis() + JUNCTION_COOLDOWN_MS;
      prevErr = 0; curSpeed = APPROACH_SPEED; cleanSince = 0;
      tLoop = micros();
      continue;
    }

    // ---- steering
    bool approach = nodeArmed && (outer || wideCue);   // early junction cue (Priority-2 partitions)
    int16_t pos = linePosition();
    float err = (float)pos;
    if (approach && wideCue) err = 0;                  // perpendicular bar under centre: hold straight
    float d = err - prevErr;
    prevErr = err;
    float corr = KP * err + KD * d;

    // ---- speed gating (boost only on clean, low-error, quiet-edge tracking)
    int16_t target = baseSpd;
    if (boostAllowed) {
      int16_t ae = (pos < 0) ? -pos : pos;
      bool clean = (ae <= BOOST_ERR_MAX) && ((actMask & MASK_OUTER) == 0) && (ct <= BOOST_THICK_MAX_X10);
      if (clean) {
        if (!cleanSince) cleanSince = now | 1;
        if (now - cleanSince >= BOOST_ARM_MS) target = BOOST_SPEED;
      } else cleanSince = 0;
    }
    if (approach) target = APPROACH_SPEED;
    if (target < curSpeed) {                           // cut boost immediately
      if (curSpeed - target >= BOOST_CUT_MIN_DROP) { motorsShortBrake(); delay(BOOST_CUT_BRAKE_MS); tLoop = micros(); }
      curSpeed = target;
    } else {
      curSpeed += ACCEL_STEP;
      if (curSpeed > target) curSpeed = target;
    }

    int16_t l = (int16_t)(curSpeed + corr);
    int16_t r = (int16_t)(curSpeed - corr);
    if (l < -MAX_REV_PWM) l = -MAX_REV_PWM;
    if (r < -MAX_REV_PWM) r = -MAX_REV_PWM;
    setMotors(l, r);
  }
}

// ============================================================================
// 13. STATE HANDLERS
// ============================================================================
void startRun(bool dry) {
  if (!calValid) { showMsg(MSG_NO_CAL, 1200); state = STATE_IDLE; scr = SCR_MENU; return; }
  if (!dry && pathLen == 0) { showMsg(MSG_NO_PATH, 1200); state = STATE_IDLE; scr = SCR_MENU; return; }
  showMsg(MSG_STARTING, START_DELAY_MS);        // hands off, single-switch start
  RunResult r = runMaze(dry);
  if (r == RES_END) { state = STATE_FINISHED; return; }
  motorsStop();
  showMsg(r == RES_FAIL ? MSG_TURN_FAIL : MSG_ABORT, 1500);
  state = STATE_IDLE; scr = SCR_MENU;
}

void menuStep() {
  if (pressedEdge(BUTTON_UP_PIN))   { boostMode = !boostMode; saveBoost(); }
  if (pressedEdge(BUTTON_DOWN_PIN)) { menuItem = (menuItem + 1) % MENU_ITEMS; }
  if (pressedEdge(BUTTON_SELECT_PIN)) {
    switch (menuItem) {
      case 0: state = STATE_CALIBRATE;  break;
      case 1: state = STATE_DRY_RUN;    break;
      case 2: state = STATE_ACTUAL_RUN; break;
      case 3: state = STATE_SENSOR_VIEW; break;
      default: state = STATE_TEST;      break;
    }
    return;
  }
  scr = SCR_MENU;
  displayService();
}

void finishedStep() {                           // LED stays ON until the operator leaves
  scr = SCR_DONE;
  displayService();
  if (pressedEdge(BUTTON_SELECT_PIN) || pressedEdge(BUTTON_DOWN_PIN)) {
    setEndLed(false);
    state = STATE_IDLE; scr = SCR_MENU;
  }
}

void viewStep() {
  readSensors();
  scr = SCR_VIEW;
  displayService();
  if (pressedEdge(BUTTON_SELECT_PIN) || pressedEdge(BUTTON_DOWN_PIN)) { state = STATE_IDLE; scr = SCR_MENU; }
}

void testStep() {                               // bench check: red LED + each wheel briefly
  setEndLed(true); delay(800); setEndLed(false);
  setMotors(100, 0);  delay(350); motorsStop(); delay(150);
  setMotors(0, 100);  delay(350); motorsStop();
  state = STATE_IDLE; scr = SCR_MENU;
}

// ============================================================================
// 14. ARDUINO ENTRY POINTS
// ============================================================================
void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT); pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  pinMode(END_LED_PIN, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT); pinMode(right_motor_backward, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);    pinMode(right_motor_speed, OUTPUT);   // (old sketch set left twice)
  motorsStop();
  setEndLed(false);
  ADCSRA = (ADCSRA & ~0x07) | ADC_PRESCALER_BITS;
  u8g.setColorIndex(1);
  loadCalibration();
  loadPath();
  state = STATE_IDLE; scr = SCR_MENU;
}

void loop() {
  switch (state) {
    case STATE_IDLE:        menuStep();              break;
    case STATE_CALIBRATE:   runTahmidCalibration(); state = STATE_IDLE; scr = SCR_MENU; break;
    case STATE_DRY_RUN:     startRun(true);          break;
    case STATE_ACTUAL_RUN:  startRun(false);         break;
    case STATE_FINISHED:    finishedStep();          break;
    case STATE_SENSOR_VIEW: viewStep();              break;
    case STATE_TEST:        testStep();              break;
  }
}
