/*
 * Teensy 4.1 — Competition-oriented hybrid line follower (single-file draft)
 *
 * Architecture choice (brief):
 * 1) Raw PID-on-position-only: fast but brittle (sensor dropout, thickness, junctions).
 * 2) Pure behavior tree: clear but heavy for 1 kHz loops and harder to tune live.
 * 3) CHOSEN — Layered hybrid: (A) rich per-frame sensor snapshot, (B) continuous
 *    steering from weighted centroid + light balance assist, (C) binary pattern
 *    layer for discrete events, (D) explicit FSM with hysteresis/counters for
 *    corners, black patches, loss, and stop zones. This balances speed, robustness,
 *    and maintainability on Teensy-class hardware.
 *
 * Motors: signed command [-MOTOR_CMD_MAX .. +MOTOR_CMD_MAX] → PWM + direction.
 *         Replace motorApplyHalfBridge() internals for your TB9051FTG wiring.
 */

#include <Arduino.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Build-time options
// ---------------------------------------------------------------------------
static constexpr uint8_t NUM_SENSORS = 14;

// Sensor order: index 0 = leftmost physical sensor, index 13 = rightmost.
static const uint8_t SENSOR_PINS[NUM_SENSORS] = {
  21, 20, 19, 18, 17, 16, 15, 14, 13, 41, 40, 39, 38, 37};

// Symmetric weights; line center between sensors 6 and 7 (indices 6,7).
static const float SENSOR_WEIGHTS[NUM_SENSORS] = {
  -6.5f, -5.5f, -4.5f, -3.5f, -2.5f, -1.5f, -0.5f,
  0.5f, 1.5f, 2.5f, 3.5f, 4.5f, 5.5f, 6.5f};

// ---------------------------------------------------------------------------
// Motor pins — PLACEHOLDERS: wire to your driver (e.g. TB9051 PWM + DIR or dual PWM).
// Left motor: PWM on MOT_L_PWM, direction on MOT_L_DIR (HIGH = positive command).
// Right motor: same convention.
// ---------------------------------------------------------------------------
static const uint8_t MOT_L_PWM = 6;
static const uint8_t MOT_L_DIR = 7;
static const uint8_t MOT_R_PWM = 8;
static const uint8_t MOT_R_DIR = 9;

static constexpr uint32_t MOTOR_PWM_HZ = 20000;
static constexpr int16_t MOTOR_CMD_MAX = 1000;  // abstract full-scale command

// ---------------------------------------------------------------------------
// Calibration — per-sensor white/black ADC; black maps HIGH after normalize.
// Replace with EEPROM or calibration routine later.
// ---------------------------------------------------------------------------
static uint16_t CAL_WHITE[NUM_SENSORS] = {
  3800, 3780, 3760, 3740, 3720, 3700, 3680, 3660,
  3640, 3620, 3600, 3580, 3560, 3540};
static uint16_t CAL_BLACK[NUM_SENSORS] = {
  200, 220, 240, 260, 280, 300, 320, 340,
  360, 380, 400, 420, 440, 460};

// Normalized full scale (0 = white, 1000 = black target)
static constexpr uint16_t NORM_MAX = 1000;

// Binary thresholds on normalized scale
static constexpr uint16_t BIN_ON_THRESHOLD = 520;   // "see line"
static constexpr uint16_t BIN_EDGE_THRESHOLD = 650; // stronger "definitely black"

// Pattern / hysteresis
static constexpr uint8_t PATTERN_CONFIRM_FRAMES = 3;
static constexpr uint8_t LOST_CONFIRM_FRAMES = 8;
static constexpr uint8_t FOUND_CONFIRM_FRAMES = 2;

// Timing (ms)
static constexpr uint32_t CORNER_ENTRY_SETTLE_MS = 12;
static constexpr uint32_t CORNER_BRAKE_MS = 35;
static constexpr uint32_t INTERSECTION_STRAIGHT_HOLD_MS = 220;
static constexpr uint32_t FULLBLACK_STOP_PATCH_MS = 2000;
static constexpr uint32_t LINE_LOST_SEARCH_MAX_MS = 900;

// Loop target (informational; loop runs as fast as analogRead allows)
static constexpr uint32_t LOOP_PERIOD_US = 800;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------
enum class RunMode : uint8_t {
  Boot,
  Idle,
  Follow,
  CornerPrep,   // settle + brake before aggressive turn
  CornerTurn,   // reacquire with biased rotation
  Intersection, // brief straight through ambiguous black
  FullBlack,    // sustained all-black: crossing vs stop patch
  LineLost,
  StopPatch,
};

enum class PatternId : uint8_t {
  None,
  NormalLine,
  SharpLeft,
  SharpRight,
  TJunction,
  Crossroad,
  AllBlack,
  AllWhite,
  LineLost,
  Ambiguous,
};

enum class LineSide : int8_t {
  Left = -1,
  Center = 0,
  Right = 1,
};

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------
struct SensorFrame {
  uint16_t raw[NUM_SENSORS];
  uint16_t norm[NUM_SENSORS];
  uint8_t  bin[NUM_SENSORS];      // 0/1 from BIN_ON_THRESHOLD
  uint8_t  edge[NUM_SENSORS];     // 0/1 from BIN_EDGE_THRESHOLD
  uint16_t mask;                  // bit i = bin[i]
  uint16_t edgeMask;

  uint8_t  activeCount;
  uint8_t  edgeActiveCount;

  float    analogSum;       // sum of normalized values
  float    confidence;      // 0..1 analog "how much line material"
  float    weightedPos;     // continuous line estimate (weighted centroid)
  float    leftMass;        // sum norm[0..6]
  float    rightMass;       // sum norm[7..13]
  float    balance;         // rightMass - leftMass (secondary thickness/asymmetry)
  float    symmetry;        // 1 - |balance|/(leftMass+rightMass+1e-3), higher = more symmetric
};

struct PidState {
  float kp;
  float ki;
  float kd;
  float iTerm;
  float prevError;
  float prevMeasurement; // for derivative-on-measurement option
};

struct MotionCommand {
  int16_t left;
  int16_t right;
};

struct RobotState {
  RunMode mode;
  RunMode prevMode;

  PatternId lastPattern;
  uint8_t   patternStableCount;

  LineSide  lastLineSide;
  int8_t    cornerDir;        // +1 = turn right, -1 = turn left (robot frame)
  uint32_t  modeSinceMs;

  uint32_t  fullBlackStartMs;
  uint32_t  intersectionHoldUntilMs;

  uint8_t   lostStreak;
  uint8_t   foundStreak;

  float     lastWeightedPos;
  float     posDerivative;    // filtered d(pos)/dt

  int16_t   baseSpeed;        // scheduled forward command magnitude
  uint8_t   stopPatchLatched;
};

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
SensorFrame g_frame;
PidState    g_steerPid{
  .kp = 2.1f,
  .ki = 0.0f,  // start with I=0; enable only after validation
  .kd = 0.18f,
  .iTerm = 0,
  .prevError = 0,
  .prevMeasurement = 0,
};

RobotState g_robot{
  .mode = RunMode::Boot,
  .prevMode = RunMode::Boot,
  .lastPattern = PatternId::None,
  .patternStableCount = 0,
  .lastLineSide = LineSide::Center,
  .cornerDir = 0,
  .modeSinceMs = 0,
  .fullBlackStartMs = 0,
  .intersectionHoldUntilMs = 0,
  .lostStreak = 0,
  .foundStreak = 0,
  .lastWeightedPos = 0,
  .posDerivative = 0,
  .baseSpeed = 520,
  .stopPatchLatched = 0,
};

// Tunable: balance assist on steering (NOT sole steering variable)
static constexpr float BALANCE_ASSIST_GAIN = 0.035f;

// Gain schedule: scale Kp/Kd by error magnitude and confidence
static constexpr float KP_SCALE_LOW_ERR = 1.0f;
static constexpr float KP_SCALE_HIGH_ERR = 0.72f;
static constexpr float KD_SCALE_HIGH_DPOS = 1.35f;

// Corner turn: differential bias while reacquiring
static constexpr int16_t CORNER_TURN_BIAS = 380;

// Line lost: search spin intensity scales with last |weightedPos|
static constexpr float LOST_SEARCH_GAIN = 220.0f;

// Derivative filter for position
static constexpr float POS_D_ALPHA = 0.35f;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
static void   sensorsInit();
static void   motorsInit();
static void   frameCapture(SensorFrame& f);
static void   normalizeAndFeatures(SensorFrame& f);
static PatternId classifyPattern(const SensorFrame& f);
static float    scheduledSteerPid(PidState& pid, float error, float measurement, float dtSec,
                                  float conf, float absErr);
static void     updateLineSideEstimate(const SensorFrame& f, RobotState& r);
static void     transitionMode(RunMode m);
static void     fsmUpdate(uint32_t nowMs, float dtSec);
static MotionCommand makeFollowCommand(float steer, int16_t baseSpeed, const SensorFrame& f);
static MotionCommand makeCornerCommand(int8_t dir, const SensorFrame& f);
static MotionCommand makeLostCommand(const SensorFrame& f, uint32_t elapsedLostMs);
static MotionCommand makeIntersectionCommand();
static void     motionApply(const MotionCommand& cmd);
static void     hardBrakePlaceholder();
static void     debugPrintState(uint32_t nowMs);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);
  analogReadAveraging(4);

  sensorsInit();
  motorsInit();

  g_robot.mode = RunMode::Idle;
  g_robot.prevMode = RunMode::Idle;
  g_robot.modeSinceMs = millis();
  transitionMode(RunMode::Follow);  // start following — change to Idle if you want a start button

  Serial.println(F("Teensy41_LineFollower_Competition: ready"));
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop() {
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  float dtSec = (nowUs - lastUs) * 1.0e-6f;
  if (dtSec < 1.0e-4f) {
    dtSec = 1.0e-4f;
  }
  lastUs = nowUs;
  uint32_t nowMs = millis();

  frameCapture(g_frame);
  normalizeAndFeatures(g_frame);

  // Smooth derivative of weighted position (rad/s in "sensor units per second")
  float dPosRaw = (g_frame.weightedPos - g_robot.lastWeightedPos) / dtSec;
  g_robot.posDerivative = POS_D_ALPHA * dPosRaw + (1.0f - POS_D_ALPHA) * g_robot.posDerivative;
  g_robot.lastWeightedPos = g_frame.weightedPos;

  updateLineSideEstimate(g_frame, g_robot);

  fsmUpdate(nowMs, dtSec);

  static uint32_t lastDbg = 0;
  if (nowMs - lastDbg > 120) {
    debugPrintState(nowMs);
    lastDbg = nowMs;
  }

  // Soft loop pacing (optional — comment out for max rate)
  delayMicroseconds(LOOP_PERIOD_US);
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
static void sensorsInit() {
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
}

static uint16_t normalizeOne(uint8_t idx, uint16_t raw) {
  int32_t w = (int32_t)CAL_WHITE[idx];
  int32_t b = (int32_t)CAL_BLACK[idx];
  if (b <= w + 10) {
    b = w + 200;  // guard bad cal
  }
  int32_t x = ((int32_t)raw - w) * (int32_t)NORM_MAX / (b - w);
  if (x < 0) x = 0;
  if (x > (int32_t)NORM_MAX) x = NORM_MAX;
  return (uint16_t)x;
}

static void frameCapture(SensorFrame& f) {
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    f.raw[i] = analogRead(SENSOR_PINS[i]);
  }
}

static void normalizeAndFeatures(SensorFrame& f) {
  f.mask = 0;
  f.edgeMask = 0;
  f.activeCount = 0;
  f.edgeActiveCount = 0;
  f.analogSum = 0;
  float wNumer = 0;
  float wDenom = 0;

  f.leftMass = 0;
  f.rightMass = 0;

  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    f.norm[i] = normalizeOne(i, f.raw[i]);
    f.analogSum += f.norm[i];

    if (i <= 6) {
      f.leftMass += f.norm[i];
    } else {
      f.rightMass += f.norm[i];
    }

    f.bin[i] = (f.norm[i] >= BIN_ON_THRESHOLD) ? 1 : 0;
    f.edge[i] = (f.norm[i] >= BIN_EDGE_THRESHOLD) ? 1 : 0;
    if (f.bin[i]) {
      f.mask |= (1u << i);
      f.activeCount++;
    }
    if (f.edge[i]) {
      f.edgeMask |= (1u << i);
      f.edgeActiveCount++;
    }

    float wi = (float)f.norm[i];
    wNumer += wi * SENSOR_WEIGHTS[i];
    wDenom += wi;
  }

  if (wDenom > 8.0f) {
    f.weightedPos = wNumer / wDenom;
  } else {
    f.weightedPos = g_robot.lastWeightedPos;
  }

  f.balance = f.rightMass - f.leftMass;
  float massSum = f.leftMass + f.rightMass + 1.0e-3f;
  f.symmetry = 1.0f - fabsf(f.balance) / massSum;
  if (f.symmetry < 0) f.symmetry = 0;
  if (f.symmetry > 1) f.symmetry = 1;

  // Confidence: analog mass + spread (not only count)
  float confMag = f.analogSum / (float)(NUM_SENSORS * NORM_MAX);
  if (confMag > 1.0f) confMag = 1.0f;
  f.confidence = confMag * (0.55f + 0.45f * f.symmetry);
}

// ---------------------------------------------------------------------------
// Binary / mask pattern classification
// ---------------------------------------------------------------------------
static uint8_t popcount16(uint16_t m) {
  uint8_t c = 0;
  while (m) {
    m &= (uint16_t)(m - 1);
    c++;
  }
  return c;
}

static bool maskSharpLeft(const SensorFrame& f) {
  // Outer left sees strong black; center weak
  uint16_t leftOuter = f.edgeMask & ((1u << 0) | (1u << 1) | (1u << 2));
  uint16_t centerWeak = f.edgeMask & ((1u << 6) | (1u << 7));
  return (leftOuter != 0) && (popcount16(centerWeak) <= 1) && (f.rightMass + 200.0f < f.leftMass);
}

static bool maskSharpRight(const SensorFrame& f) {
  uint16_t rightOuter = f.edgeMask & ((1u << 11) | (1u << 12) | (1u << 13));
  uint16_t centerWeak = f.edgeMask & ((1u << 6) | (1u << 7));
  return (rightOuter != 0) && (popcount16(centerWeak) <= 1) && (f.leftMass + 200.0f < f.rightMass);
}

static bool maskTJunction(const SensorFrame& f) {
  // Wide activation with center present — group-based
  if (f.activeCount < 10) return false;
  uint16_t leftGrp = f.mask & 0x007Fu;   // 0..6
  uint16_t rightGrp = f.mask & 0x3F80u;  // 7..13
  return (popcount16(leftGrp) >= 3) && (popcount16(rightGrp) >= 3);
}

static bool maskCrossroad(const SensorFrame& f) {
  if (f.activeCount < 12) return false;
  return (f.edgeActiveCount >= 10);
}

static PatternId classifyPattern(const SensorFrame& f) {
  if (f.activeCount == 0) {
    return PatternId::AllWhite;
  }
  if (f.activeCount >= NUM_SENSORS - 1 && f.edgeActiveCount >= NUM_SENSORS - 2) {
    return PatternId::AllBlack;
  }
  if (maskCrossroad(f)) {
    return PatternId::Crossroad;
  }
  if (maskTJunction(f)) {
    return PatternId::TJunction;
  }
  if (maskSharpLeft(f)) {
    return PatternId::SharpLeft;
  }
  if (maskSharpRight(f)) {
    return PatternId::SharpRight;
  }
  if (f.confidence > 0.18f && fabsf(f.weightedPos) < 5.5f) {
    return PatternId::NormalLine;
  }
  return PatternId::Ambiguous;
}

static void patternStabilize(PatternId p, RobotState& r) {
  if (p == r.lastPattern) {
    if (r.patternStableCount < 250) {
      r.patternStableCount++;
    }
  } else {
    r.lastPattern = p;
    r.patternStableCount = 1;
  }
}

// ---------------------------------------------------------------------------
// Side estimate (for recovery)
// ---------------------------------------------------------------------------
static void updateLineSideEstimate(const SensorFrame& f, RobotState& r) {
  if (f.confidence < 0.12f) {
    return;
  }
  if (f.weightedPos < -0.85f) {
    r.lastLineSide = LineSide::Left;
  } else if (f.weightedPos > 0.85f) {
    r.lastLineSide = LineSide::Right;
  } else {
    r.lastLineSide = LineSide::Center;
  }
}

// ---------------------------------------------------------------------------
// PID with gain scheduling + optional I
// ---------------------------------------------------------------------------
static float scheduledSteerPid(PidState& pid, float error, float measurement, float dtSec,
                               float conf, float absErr) {
  float kp = pid.kp;
  float kd = pid.kd;

  // Schedule: large error → slightly lower Kp to avoid rail, higher Kd damping
  float errNorm = absErr / 6.5f;
  if (errNorm > 1.0f) errNorm = 1.0f;
  kp *= KP_SCALE_LOW_ERR * (1.0f - errNorm) + KP_SCALE_HIGH_ERR * errNorm;

  float dMeas = -(measurement - pid.prevMeasurement) / dtSec; // derivative on measurement
  pid.prevMeasurement = measurement;

  float kdBoost = 1.0f + KD_SCALE_HIGH_DPOS * (fabsf(g_robot.posDerivative) / 4000.0f);
  if (kdBoost > 1.8f) kdBoost = 1.8f;
  kd *= kdBoost;

  // Confidence: reduce aggression when sensors are weak
  float confScale = 0.65f + 0.55f * conf;
  if (confScale > 1.2f) confScale = 1.2f;
  kp *= confScale;

  float pTerm = kp * error;
  pid.iTerm += pid.ki * error * dtSec;
  float iClamp = 180.0f;
  if (pid.iTerm > iClamp) pid.iTerm = iClamp;
  if (pid.iTerm < -iClamp) pid.iTerm = -iClamp;

  float dTerm = kd * dMeas;

  float u = pTerm + pid.iTerm + dTerm;
  pid.prevError = error;

  // Output limit (steering abstract units)
  const float uMax = 850.0f;
  if (u > uMax) u = uMax;
  if (u < -uMax) u = -uMax;
  return u;
}

// ---------------------------------------------------------------------------
// Motion mixing: primary PID steer + secondary balance assist
// ---------------------------------------------------------------------------
static MotionCommand makeFollowCommand(float steer, int16_t baseSpeed, const SensorFrame& f) {
  MotionCommand m;
  float balanceCorr = BALANCE_ASSIST_GAIN * f.balance * (0.4f + 0.6f * f.symmetry);

  float steerTotal = steer + balanceCorr;

  // Adaptive base speed: slow on high error or low confidence
  float errMag = fabsf(f.weightedPos);
  int16_t spd = baseSpeed;
  spd = (int16_t)((float)spd * (0.55f + 0.45f * f.confidence));
  int16_t slow = (int16_t)((float)baseSpeed * (0.42f + 0.58f * (1.0f - errMag / 7.0f)));
  if (slow < spd) {
    spd = slow;
  }
  if (spd < 160) spd = 160;

  float left = (float)spd - steerTotal;
  float right = (float)spd + steerTotal;

  m.left = (int16_t)constrain((int)lroundf(left), -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
  m.right = (int16_t)constrain((int)lroundf(right), -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
  return m;
}

static MotionCommand makeCornerCommand(int8_t dir, const SensorFrame& f) {
  MotionCommand m;
  int16_t fwd = (int16_t)((float)g_robot.baseSpeed * 0.38f);
  int16_t bias = CORNER_TURN_BIAS;
  if (f.confidence > 0.25f && fabsf(f.weightedPos) < 1.2f) {
    // reacquired — soften
    bias = (int16_t)(bias * 0.35f);
  }
  if (dir < 0) {
    m.left = fwd - bias;
    m.right = fwd + bias;
  } else {
    m.left = fwd + bias;
    m.right = fwd - bias;
  }
  m.left = (int16_t)constrain((int)m.left, -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
  m.right = (int16_t)constrain((int)m.right, -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
  return m;
}

static MotionCommand makeLostCommand(const SensorFrame& f, uint32_t elapsedLostMs) {
  MotionCommand m;
  float mem = (g_robot.lastLineSide == LineSide::Left) ? -1.0f :
              (g_robot.lastLineSide == LineSide::Right) ? 1.0f : 0.0f;
  float steerSearch = LOST_SEARCH_GAIN * mem;
  // Spiral in slightly increasing aggression over time
  float t = (float)elapsedLostMs / 1000.0f;
  steerSearch *= (1.0f + 0.35f * t);

  int16_t spd = (int16_t)((float)g_robot.baseSpeed * 0.35f);
  float left = (float)spd - steerSearch;
  float right = (float)spd + steerSearch;
  m.left = (int16_t)constrain((int)lroundf(left), -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
  m.right = (int16_t)constrain((int)lroundf(right), -MOTOR_CMD_MAX, MOTOR_CMD_MAX);
  return m;
}

static MotionCommand makeIntersectionCommand() {
  MotionCommand m;
  int16_t v = (int16_t)((float)g_robot.baseSpeed * 0.62f);
  m.left = v;
  m.right = v;
  return m;
}

// ---------------------------------------------------------------------------
// Motor output — generic signed command; adapt to TB9051
// ---------------------------------------------------------------------------
static void motorApplyHalfBridge(int16_t cmd, uint8_t pinPwm, uint8_t pinDir) {
  int16_t mag = abs(cmd);
  if (mag > MOTOR_CMD_MAX) mag = MOTOR_CMD_MAX;
  uint8_t duty = (uint8_t)((mag * 255) / MOTOR_CMD_MAX);
  bool forward = cmd >= 0;
  digitalWriteFast(pinDir, forward ? HIGH : LOW);
  analogWrite(pinPwm, duty);
}

static void motionApply(const MotionCommand& cmd) {
  motorApplyHalfBridge(cmd.left, MOT_L_PWM, MOT_L_DIR);
  motorApplyHalfBridge(cmd.right, MOT_R_PWM, MOT_R_DIR);
}

static void motorsInit() {
  pinMode(MOT_L_PWM, OUTPUT);
  pinMode(MOT_L_DIR, OUTPUT);
  pinMode(MOT_R_PWM, OUTPUT);
  pinMode(MOT_R_DIR, OUTPUT);
  analogWriteFrequency(MOT_L_PWM, MOTOR_PWM_HZ);
  analogWriteFrequency(MOT_R_PWM, MOTOR_PWM_HZ);
  analogWriteResolution(8);
  motionApply({0, 0});
}

static void hardBrakePlaceholder() {
  // Placeholder: coast to zero duty. Replace with fast decay, reverse pulse, or dynamic brake.
  motionApply({0, 0});
  delayMicroseconds(800);
}

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------
static void transitionMode(RunMode m) {
  if (g_robot.mode != m) {
    g_robot.prevMode = g_robot.mode;
    g_robot.mode = m;
    g_robot.modeSinceMs = millis();

    if (m == RunMode::CornerPrep || m == RunMode::CornerTurn) {
      g_steerPid.iTerm = 0;
    }
    if (m == RunMode::Follow) {
      g_steerPid.prevMeasurement = g_frame.weightedPos;
    }
  }
}

static bool centerReacquired(const SensorFrame& f) {
  return (f.confidence > 0.22f) && (fabsf(f.weightedPos) < 1.1f) && (f.activeCount >= 2);
}

static void fsmUpdate(uint32_t nowMs, float dtSec) {
  PatternId p = classifyPattern(g_frame);
  patternStabilize(p, g_robot);

  // Line lost streak vs recovery streak (binary layer)
  if (g_frame.activeCount == 0 || g_frame.confidence < 0.08f) {
    if (g_robot.foundStreak) g_robot.foundStreak--;
    if (g_robot.lostStreak < 250) g_robot.lostStreak++;
  } else {
    if (g_robot.lostStreak) g_robot.lostStreak--;
    if (g_robot.foundStreak < 250) g_robot.foundStreak++;
  }

  bool lineLostConfirmed = (g_robot.lostStreak >= LOST_CONFIRM_FRAMES);
  bool lineFoundConfirmed = (g_robot.foundStreak >= FOUND_CONFIRM_FRAMES);

  switch (g_robot.mode) {
    case RunMode::Boot:
    case RunMode::Idle:
      motionApply({0, 0});
      break;

    case RunMode::Follow: {
      // Priority: sharp corner intent
      if (g_robot.patternStableCount >= PATTERN_CONFIRM_FRAMES) {
        if (p == PatternId::SharpLeft) {
          g_robot.cornerDir = -1;
          transitionMode(RunMode::CornerPrep);
          break;
        }
        if (p == PatternId::SharpRight) {
          g_robot.cornerDir = +1;
          transitionMode(RunMode::CornerPrep);
          break;
        }
      }

      if (p == PatternId::AllBlack) {
        g_robot.fullBlackStartMs = nowMs;
        transitionMode(RunMode::FullBlack);
        break;
      }

      if (lineLostConfirmed) {
        transitionMode(RunMode::LineLost);
        break;
      }

      if (p == PatternId::Crossroad || p == PatternId::TJunction) {
        g_robot.intersectionHoldUntilMs = nowMs + INTERSECTION_STRAIGHT_HOLD_MS;
        transitionMode(RunMode::Intersection);
        break;
      }

      float target = 0.0f;
      float err = target - g_frame.weightedPos;
      float absErr = fabsf(err);
      float steer = scheduledSteerPid(g_steerPid, err, g_frame.weightedPos, dtSec,
                                      g_frame.confidence, absErr);
      MotionCommand mc = makeFollowCommand(steer, g_robot.baseSpeed, g_frame);
      motionApply(mc);
      break;
    }

    case RunMode::CornerPrep: {
      if (nowMs - g_robot.modeSinceMs < CORNER_ENTRY_SETTLE_MS) {
        MotionCommand mc = makeFollowCommand(0, (int16_t)(g_robot.baseSpeed / 2), g_frame);
        motionApply(mc);
      } else if (nowMs - g_robot.modeSinceMs < CORNER_ENTRY_SETTLE_MS + CORNER_BRAKE_MS) {
        hardBrakePlaceholder();
      } else {
        transitionMode(RunMode::CornerTurn);
      }
      break;
    }

    case RunMode::CornerTurn: {
      MotionCommand mc = makeCornerCommand(g_robot.cornerDir, g_frame);
      motionApply(mc);
      if (centerReacquired(g_frame) && lineFoundConfirmed) {
        g_steerPid.prevError = 0;
        g_steerPid.iTerm = 0;
        g_steerPid.prevMeasurement = g_frame.weightedPos;
        transitionMode(RunMode::Follow);
      }
      // Safety timeout
      if (nowMs - g_robot.modeSinceMs > 450) {
        transitionMode(RunMode::Follow);
      }
      break;
    }

    case RunMode::Intersection: {
      if (nowMs > g_robot.intersectionHoldUntilMs) {
        transitionMode(RunMode::Follow);
        break;
      }
      if (p == PatternId::AllBlack) {
        g_robot.fullBlackStartMs = nowMs;
        transitionMode(RunMode::FullBlack);
        break;
      }
      motionApply(makeIntersectionCommand());
      break;
    }

    case RunMode::FullBlack: {
      // Briefly drive straight (intersection crossing); if stuck, escalate
      if (nowMs - g_robot.fullBlackStartMs < INTERSECTION_STRAIGHT_HOLD_MS) {
        motionApply(makeIntersectionCommand());
        if (p != PatternId::AllBlack && lineFoundConfirmed) {
          transitionMode(RunMode::Follow);
        }
        break;
      }
      if (nowMs - g_robot.fullBlackStartMs > FULLBLACK_STOP_PATCH_MS) {
        transitionMode(RunMode::StopPatch);
        break;
      }
      // Continue slow straight with last remembered steer bias faded to zero
      MotionCommand mc = makeIntersectionCommand();
      mc.left = (int16_t)(mc.left * 0.85f);
      mc.right = (int16_t)(mc.right * 0.85f);
      motionApply(mc);
      if (p != PatternId::AllBlack && lineFoundConfirmed) {
        transitionMode(RunMode::Follow);
      }
      break;
    }

    case RunMode::LineLost: {
      uint32_t lostFor = nowMs - g_robot.modeSinceMs;
      if (lineFoundConfirmed && g_frame.confidence > 0.15f) {
        g_steerPid.iTerm = 0;
        g_steerPid.prevMeasurement = g_frame.weightedPos;
        transitionMode(RunMode::Follow);
        break;
      }
      if (lostFor > LINE_LOST_SEARCH_MAX_MS) {
        hardBrakePlaceholder();
        transitionMode(RunMode::Idle);
        break;
      }
      motionApply(makeLostCommand(g_frame, lostFor));
      break;
    }

    case RunMode::StopPatch:
      hardBrakePlaceholder();
      motionApply({0, 0});
      g_robot.stopPatchLatched = 1;
      break;
  }
}

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------
static const char* modeName(RunMode m) {
  switch (m) {
    case RunMode::Boot: return "BOOT";
    case RunMode::Idle: return "IDLE";
    case RunMode::Follow: return "FOLLOW";
    case RunMode::CornerPrep: return "CORNER_PREP";
    case RunMode::CornerTurn: return "CORNER_TURN";
    case RunMode::Intersection: return "INTERSECTION";
    case RunMode::FullBlack: return "FULLBLACK";
    case RunMode::LineLost: return "LINE_LOST";
    case RunMode::StopPatch: return "STOP_PATCH";
    default: return "?";
  }
}

static const char* patternName(PatternId p) {
  switch (p) {
    case PatternId::None: return "NONE";
    case PatternId::NormalLine: return "NORMAL";
    case PatternId::SharpLeft: return "SHARP_L";
    case PatternId::SharpRight: return "SHARP_R";
    case PatternId::TJunction: return "T_JUNCT"; 
    case PatternId::Crossroad: return "CROSS";
    case PatternId::AllBlack: return "ALL_BLACK";
    case PatternId::AllWhite: return "ALL_WHITE";
    case PatternId::LineLost: return "LOST";
    case PatternId::Ambiguous: return "AMBIG";
    default: return "?";
  }
}

static void debugPrintState(uint32_t nowMs) {
  Serial.print(F("t=")); Serial.print(nowMs);
  Serial.print(F(" mode=")); Serial.print(modeName(g_robot.mode));
  Serial.print(F(" pat=")); Serial.print(patternName(g_robot.lastPattern));
  Serial.print(F(" stab=")); Serial.print(g_robot.patternStableCount);
  Serial.print(F(" pos=")); Serial.print(g_frame.weightedPos, 3);
  Serial.print(F(" conf=")); Serial.print(g_frame.confidence, 3);
  Serial.print(F(" act=")); Serial.print(g_frame.activeCount);
  Serial.print(F(" edge=")); Serial.print(g_frame.edgeActiveCount);
  Serial.print(F(" sym=")); Serial.print(g_frame.symmetry, 3);
  Serial.print(F(" bal=")); Serial.print(g_frame.balance, 1);
  Serial.print(F(" lost=")); Serial.print(g_robot.lostStreak);
  Serial.print(F(" found=")); Serial.print(g_robot.foundStreak);
  Serial.print(F(" dpos=")); Serial.print(g_robot.posDerivative, 1);
  Serial.print(F(" corner=")); Serial.print((int)g_robot.cornerDir);
  Serial.print(F(" stop=")); Serial.println((int)g_robot.stopPatchLatched);
}

this is cursors code
cursors response
Innovations baked in (beyond bare PID): confidence-weighted steering gain, derivative-on-measurement with filtered (\dot{p}) for damping spikes, dual thresholds (bin vs edge) for sharper geometry calls, mask-based T/cross heuristics, pattern stability counting before corner commits, and line-loss recovery biased by lastLineSide with ramping search aggression.

If you want next steps on the bench: tune BIN_* thresholds after calibration, then g_steerPid and g_robot.baseSpeed, then corner timings (CORNER_*) against your wheelbase and traction.

compare with above three yours, copilots, and claudes