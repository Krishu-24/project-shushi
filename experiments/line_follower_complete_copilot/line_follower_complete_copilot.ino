/*
  Teensy 4.1 Competition Line Follower Firmware
  Single-file, competition-oriented, engineered for speed and robustness.

  Key design points:
  - 14 analog reflective IR sensors (left-to-right)
  - Per-sensor normalization (0..1000), black -> high, white -> low
  - Parallel analog (continuous) + binary (event) processing
  - Weighted analog position is primary steering signal
  - Thickness/balance used as secondary assist
  - PD(I)-style controller with derivative-based caution and adaptive speed scheduling
  - State machine for FOLLOW, PREPARE_TURN, EXECUTE_TURN, INTERSECTION, RECOVER, STOPPED
  - Confidence scoring, hysteresis, consecutive-frame confirmation for robust transitions
  - Motor abstraction with signed commands ready for TB9051FTG adaptation
  - Debug printing for tuning and telemetry

  Hardware placeholders:
  - Sensor pins: {21,20,19,18,17,16,15,14,13,41,40,39,38,37}
  - Motor pins: LEFT_PWM_PIN, LEFT_DIR_PIN, RIGHT_PWM_PIN, RIGHT_DIR_PIN (replace as needed)
  - ADC: 12-bit, averaging 4
  - PWM: attempt 20 kHz, 8-bit resolution (adjustable)

  Author: Competitive-minded embedded engineer (single-file draft)
*/

#include <Arduino.h>

/* =========================
   Tunable Constants
   ========================= */

// Sensors
constexpr uint8_t SENSOR_COUNT = 14;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {21,20,19,18,17,16,15,14,13,41,40,39,38,37};

// ADC
constexpr int ADC_BITS = 12;
constexpr int ADC_MAX = (1 << ADC_BITS) - 1;
constexpr int ADC_AVERAGING = 4;

// Normalization
constexpr int NORM_MIN = 0;
constexpr int NORM_MAX = 1000;

// Binary thresholds (on normalized scale)
constexpr int BINARY_TH = 420;   // normal black detection
constexpr int EDGE_TH   = 720;   // strong edge detection for sharp turns

// Sensor weights (center between sensors 6 and 7)
const float SENSOR_WEIGHTS[SENSOR_COUNT] = {
  -6.5f, -5.5f, -4.5f, -3.5f, -2.5f, -1.5f, -0.5f,
   0.5f,  1.5f,  2.5f,  3.5f,  4.5f,  5.5f,  6.5f
};

// Motor pins (placeholders - adapt to wiring)
constexpr uint8_t LEFT_PWM_PIN  = 3;
constexpr uint8_t LEFT_DIR_PIN  = 2;
constexpr uint8_t RIGHT_PWM_PIN = 5;
constexpr uint8_t RIGHT_DIR_PIN = 4;

// PWM
constexpr int PWM_FREQ = 20000; // 20 kHz
constexpr int PWM_BITS = 8;
constexpr int PWM_MAX = (1 << PWM_BITS) - 1;

// Motor command limits
constexpr int MOTOR_MAX = PWM_MAX;
constexpr int MOTOR_MIN = -PWM_MAX;

// Base speed and adaptive scaling
constexpr int BASE_SPEED = 220; // nominal forward speed (0..PWM_MAX)
constexpr int MIN_SPEED  = 120; // minimum safe speed
constexpr int MAX_SPEED  = 255; // absolute max (constrained by PWM)
constexpr float STEER_SCALE = 1.0f; // steering influence on differential

// PID / Control
constexpr float KP = 1.05f;
constexpr float KI = 0.0006f; // small integral
constexpr float KD = 0.20f;
constexpr float THICKNESS_K = 0.07f; // secondary assist from thickness balance
constexpr float DERIVATIVE_SMOOTH = 0.6f; // smoothing for derivative to reduce noise
constexpr float INTEGRAL_LIMIT = 3000.0f;

// Loop timing
constexpr unsigned long LOOP_TARGET_US = 3500; // ~285 Hz loop (tune for your robot)

// State machine timing and hysteresis
constexpr unsigned long PREPARE_TURN_DELAY_MS = 25;
constexpr unsigned long EXECUTE_TURN_TIMEOUT_MS = 1200;
constexpr unsigned long INTERSECTION_CONFIRM_MS = 2000;
constexpr unsigned long LOST_CONFIRM_MS = 60;
constexpr uint8_t INTERSECTION_COUNT_THRESHOLD = 12; // >= this many sensors black => intersection candidate
constexpr uint8_t T_JUNCTION_MIN_GROUPS = 3; // left/center/right groups active -> junction

// Sharp turn detection parameters
constexpr uint8_t OUTER_STRONG_COUNT = 2; // how many outer sensors must be strong
constexpr uint8_t CENTER_WEAK_MAX = 1;   // how many center sensors can be strong for a sharp turn

// Confirmation frames (consecutive frames) for robust decisions
constexpr uint8_t CONFIRM_FRAMES = 3;

/* =========================
   Types and Data Structures
   ========================= */

enum RobotState {
  STATE_FOLLOW,
  STATE_PREPARE_TURN,
  STATE_EXECUTE_TURN,
  STATE_INTERSECTION,
  STATE_RECOVER,
  STATE_STOPPED
};

enum TurnDir {
  TURN_NONE = 0,
  TURN_LEFT = -1,
  TURN_RIGHT = 1
};

struct SensorFrame {
  uint16_t raw[SENSOR_COUNT];     // raw ADC
  int norm[SENSOR_COUNT];         // normalized 0..1000 (black high)
  bool bin[SENSOR_COUNT];         // binary threshold
  bool edge[SENSOR_COUNT];        // strong edge threshold
  uint16_t mask;                  // binary mask (LSB sensor 0)
  uint8_t activeCount;            // number of binary active sensors
  long analogSum;                 // sum of normalized values
  float weightedPos;              // weighted position (weights units)
  float leftMass;                 // analog mass left
  float rightMass;                // analog mass right
  float thickness;                // rightMass - leftMass
  float confidence;               // 0..1 confidence of analog estimate
};

struct ControllerContext {
  // Sensor frame
  SensorFrame frame;

  // Calibration (placeholders)
  uint16_t calibWhite[SENSOR_COUNT];
  uint16_t calibBlack[SENSOR_COUNT];

  // PID state
  float integral;
  float lastError;
  float lastDerivative;
  unsigned long lastLoopUs;

  // State machine
  RobotState state;
  unsigned long stateEnterMs;
  TurnDir pendingTurn;
  int lastKnownSide; // -1 left, 0 center, 1 right

  // Counters and hysteresis
  uint8_t consecutiveAllBlack;
  uint8_t consecutiveAllWhite;
  uint8_t consecutiveTurnDetect;
  uint8_t consecutiveRecoverFound;

  // Debug
  bool debug;
};

static ControllerContext ctx;

/* =========================
   Forward Declarations
   ========================= */

// Setup
void setupHardware();
void setupSensors();
void setupMotors();
void configurePWM();

// Sensor processing
void readSensors(SensorFrame &f);
void normalizeSensors(SensorFrame &f);
void computeMetrics(SensorFrame &f);

// Control & state machine
void updateStateMachine();
void enterState(RobotState s);
void handleFollow();
void handlePrepareTurn();
void handleExecuteTurn();
void handleIntersection();
void handleRecover();
void handleStopped();

// Motor abstraction
void motorDrive(int leftCmd, int rightCmd);
void setMotorChannel(uint8_t pwmPin, uint8_t dirPin, int cmd);
void hardBrake(unsigned long ms = 30);
void rampBrake(int fromLeft, int fromRight, unsigned long ms);

// Steering & PID
int computeSteerCommand(const SensorFrame &f);

// Decision helpers
TurnDir detectSharpTurn(const SensorFrame &f);
bool anyCenterActive(const SensorFrame &f);
bool allWhite(const SensorFrame &f);
bool allBlack(const SensorFrame &f);
bool detectTJunction(const SensorFrame &f);

// Utilities
void debugPrint(const SensorFrame &f);
inline unsigned long microsSafe();
inline unsigned long millisSafe();

/* =========================
   Setup
   ========================= */

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) ; // wait briefly for serial on some hosts

  // Initialize context
  memset(&ctx, 0, sizeof(ctx));
  ctx.state = STATE_FOLLOW;
  ctx.lastKnownSide = 0;
  ctx.debug = true;
  ctx.lastLoopUs = microsSafe();

  // Placeholder calibration values (raw ADC 0..4095)
  // Replace with real calibration later. Values chosen to be plausible.
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    ctx.calibWhite[i] = 300;   // white reflectance (low)
    ctx.calibBlack[i] = 3600;  // black reflectance (high)
  }

  // Hardware init
  setupHardware();
  setupSensors();
  setupMotors();
  configurePWM();

  // ADC config
  analogReadResolution(ADC_BITS);
  analogReadAveraging(ADC_AVERAGING);

  // Enter initial state
  enterState(ctx.state);
  Serial.println("Competition line follower ready.");
}

/* =========================
   Main loop
   ========================= */

void loop() {
  unsigned long loopStartUs = microsSafe();

  // Read and process sensors
  readSensors(ctx.frame);
  normalizeSensors(ctx.frame);
  computeMetrics(ctx.frame);

  // Update state machine and control
  updateStateMachine();

  // Debug telemetry at ~50-200ms depending on need
  static unsigned long lastDbg = 0;
  if (ctx.debug && (millisSafe() - lastDbg >= 120)) {
    debugPrint(ctx.frame);
    lastDbg = millisSafe();
  }

  // Loop timing: aim for LOOP_TARGET_US
  unsigned long elapsed = microsSafe() - loopStartUs;
  if (elapsed < LOOP_TARGET_US) {
    // busy-waiting small amount is fine; use delayMicroseconds for cooperative timing
    delayMicroseconds(LOOP_TARGET_US - elapsed);
  }
}

/* =========================
   Hardware setup helpers
   ========================= */

void setupHardware() {
  // Nothing special here; pinMode set in respective setup functions
}

void setupSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
}

void setupMotors() {
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  // Ensure motors stopped
  motorDrive(0, 0);
}

void configurePWM() {
  // Attempt to set PWM frequency and resolution
  analogWriteFrequency(LEFT_PWM_PIN, PWM_FREQ);
  analogWriteFrequency(RIGHT_PWM_PIN, PWM_FREQ);
  analogWriteResolution(PWM_BITS);
}

/* =========================
   Sensor reading & normalization
   ========================= */

void readSensors(SensorFrame &f) {
  // Read raw ADC values
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    f.raw[i] = analogRead(SENSOR_PINS[i]);
  }
}

void normalizeSensors(SensorFrame &f) {
  f.analogSum = 0;
  f.mask = 0;
  f.activeCount = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    uint16_t raw = f.raw[i];
    uint16_t w = ctx.calibWhite[i];
    uint16_t b = ctx.calibBlack[i];

    int denom = int(b) - int(w);
    int n;
    if (denom == 0) {
      // fallback linear mapping
      n = map(raw, 0, ADC_MAX, NORM_MIN, NORM_MAX);
    } else {
      float t = float(raw - w) / float(denom);
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;
      n = int(t * float(NORM_MAX));
    }

    // store normalized
    f.norm[i] = n;
    f.analogSum += n;

    // binary thresholds
    f.bin[i] = (n >= BINARY_TH);
    f.edge[i] = (n >= EDGE_TH);

    if (f.bin[i]) {
      f.mask |= (1u << i);
      f.activeCount++;
    }
  }
}

void computeMetrics(SensorFrame &f) {
  // Weighted position and mass metrics
  float weightedSum = 0.0f;
  float weightTotal = 0.0f;
  float leftMass = 0.0f;
  float rightMass = 0.0f;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    float val = float(f.norm[i]); // 0..1000
    float w = SENSOR_WEIGHTS[i];
    weightedSum += w * val;
    weightTotal += val;
    if (w < 0.0f) leftMass += (-w) * val;
    else rightMass += w * val;
  }

  if (weightTotal > 0.0f) {
    f.weightedPos = weightedSum / weightTotal;
    // Confidence: how much analog sum relative to max possible
    float maxPossible = float(NORM_MAX) * float(SENSOR_COUNT);
    f.confidence = constrain(float(f.analogSum) / maxPossible, 0.0f, 1.0f);
  } else {
    // No signal: keep weightedPos at 0 but confidence 0
    f.weightedPos = 0.0f;
    f.confidence = 0.0f;
  }

  f.leftMass = leftMass;
  f.rightMass = rightMass;
  f.thickness = rightMass - leftMass;
}

/* =========================
   State machine & behaviors
   ========================= */

void enterState(RobotState s) {
  ctx.state = s;
  ctx.stateEnterMs = millisSafe();

  // Reset or prepare state-specific variables
  switch (s) {
    case STATE_FOLLOW:
      ctx.integral = 0.0f;
      ctx.lastError = 0.0f;
      ctx.lastDerivative = 0.0f;
      ctx.consecutiveAllBlack = 0;
      ctx.consecutiveAllWhite = 0;
      ctx.consecutiveTurnDetect = 0;
      ctx.consecutiveRecoverFound = 0;
      break;
    case STATE_PREPARE_TURN:
      // small settle delay before executing turn
      break;
    case STATE_EXECUTE_TURN:
      // timeout will be enforced in handler
      break;
    case STATE_INTERSECTION:
      ctx.consecutiveAllBlack = 0;
      break;
    case STATE_RECOVER:
      ctx.consecutiveRecoverFound = 0;
      break;
    case STATE_STOPPED:
      motorDrive(0, 0);
      break;
  }

  if (ctx.debug) {
    Serial.print("ENTER_STATE: ");
    switch (s) {
      case STATE_FOLLOW: Serial.println("FOLLOW"); break;
      case STATE_PREPARE_TURN: Serial.println("PREPARE_TURN"); break;
      case STATE_EXECUTE_TURN: Serial.println("EXECUTE_TURN"); break;
      case STATE_INTERSECTION: Serial.println("INTERSECTION"); break;
      case STATE_RECOVER: Serial.println("RECOVER"); break;
      case STATE_STOPPED: Serial.println("STOPPED"); break;
    }
  }
}

void updateStateMachine() {
  // Evaluate high-level conditions and dispatch to state handlers
  switch (ctx.state) {
    case STATE_FOLLOW: handleFollow(); break;
    case STATE_PREPARE_TURN: handlePrepareTurn(); break;
    case STATE_EXECUTE_TURN: handleExecuteTurn(); break;
    case STATE_INTERSECTION: handleIntersection(); break;
    case STATE_RECOVER: handleRecover(); break;
    case STATE_STOPPED: handleStopped(); break;
  }
}

/* FOLLOW: main continuous control with event detection */
void handleFollow() {
  SensorFrame &f = ctx.frame;

  // 1) Intersection / all-black detection with hysteresis
  if (f.activeCount >= INTERSECTION_COUNT_THRESHOLD) {
    ctx.consecutiveAllBlack++;
    if (ctx.consecutiveAllBlack >= CONFIRM_FRAMES) {
      // candidate intersection
      enterState(STATE_INTERSECTION);
      return;
    }
  } else {
    ctx.consecutiveAllBlack = 0;
  }

  // 2) Lost line detection (all white)
  if (f.activeCount == 0) {
    ctx.consecutiveAllWhite++;
    if (ctx.consecutiveAllWhite >= (LOST_CONFIRM_MS / (LOOP_TARGET_US / 1000) + 1)) {
      // lost line confirmed
      enterState(STATE_RECOVER);
      return;
    }
  } else {
    ctx.consecutiveAllWhite = 0;
  }

  // 3) Sharp turn detection using edge sensors and center weakness
  TurnDir td = detectSharpTurn(f);
  if (td != TURN_NONE) {
    ctx.consecutiveTurnDetect++;
    if (ctx.consecutiveTurnDetect >= CONFIRM_FRAMES) {
      ctx.pendingTurn = td;
      enterState(STATE_PREPARE_TURN);
      return;
    }
  } else {
    ctx.consecutiveTurnDetect = 0;
  }

  // 4) Normal follow: compute steering and adaptive speed
  int steer = computeSteerCommand(f); // signed steering in PWM units

  // Adaptive speed scheduling:
  // - reduce speed when large error magnitude or high curvature (derivative)
  float absError = fabsf(ctx.lastError);
  float deriv = fabsf(ctx.lastDerivative);
  // confidence reduces speed if low
  float confFactor = f.confidence;
  // compute speed scale: base -> reduce with error and derivative
  float speedScale = 1.0f;
  // penalize large error
  speedScale -= constrain(absError * 0.06f, 0.0f, 0.45f);
  // penalize high derivative (curvature)
  speedScale -= constrain(deriv * 0.5f, 0.0f, 0.35f);
  // penalize low confidence
  speedScale *= (0.5f + 0.5f * confFactor); // 0.5..1.0
  // clamp
  speedScale = constrain(speedScale, 0.35f, 1.0f);

  int targetSpeed = int(BASE_SPEED * speedScale);
  targetSpeed = constrain(targetSpeed, MIN_SPEED, MAX_SPEED);

  // Map steering to motor commands (differential)
  int leftCmd = targetSpeed - int(steer * STEER_SCALE);
  int rightCmd = targetSpeed + int(steer * STEER_SCALE);

  // Constrain final commands
  leftCmd = constrain(leftCmd, MOTOR_MIN, MOTOR_MAX);
  rightCmd = constrain(rightCmd, MOTOR_MIN, MOTOR_MAX);

  // Update last known side for recovery heuristics
  if (f.weightedPos < -0.6f) ctx.lastKnownSide = -1;
  else if (f.weightedPos > 0.6f) ctx.lastKnownSide = 1;
  else ctx.lastKnownSide = 0;

  motorDrive(leftCmd, rightCmd);
}

/* PREPARE_TURN: short settle, brake, then execute */
void handlePrepareTurn() {
  // small settle delay to let robot enter corner
  if (millisSafe() - ctx.stateEnterMs < PREPARE_TURN_DELAY_MS) return;

  // aggressive brake to stabilize heading
  rampBrake(BASE_SPEED, BASE_SPEED, 30); // quick ramp down
  hardBrake(20);

  // enter execute
  enterState(STATE_EXECUTE_TURN);
}

/* EXECUTE_TURN: rotate until center sensors reacquire line or timeout */
void handleExecuteTurn() {
  SensorFrame &f = ctx.frame;

  // If center sensors detect line, finish turn
  if (anyCenterActive(f)) {
    // small forward burst to settle on line
    motorDrive(BASE_SPEED / 2, BASE_SPEED / 2);
    delay(40);
    ctx.integral = 0.0f;
    ctx.lastError = 0.0f;
    ctx.lastDerivative = 0.0f;
    enterState(STATE_FOLLOW);
    return;
  }

  // Timeout safety
  if (millisSafe() - ctx.stateEnterMs > EXECUTE_TURN_TIMEOUT_MS) {
    // fallback to recover
    enterState(STATE_RECOVER);
    return;
  }

  // Execute in-place or near in-place turn
  int turnPower = 200; // tune for robot's turning capability
  if (ctx.pendingTurn == TURN_LEFT) {
    motorDrive(-turnPower, turnPower);
  } else if (ctx.pendingTurn == TURN_RIGHT) {
    motorDrive(turnPower, -turnPower);
  } else {
    // safety fallback
    motorDrive(0, 0);
  }
}

/* INTERSECTION: handle T/cross or full-black patch */
void handleIntersection() {
  SensorFrame &f = ctx.frame;

  // If still mostly black, increment confirmation
  if (f.activeCount >= INTERSECTION_COUNT_THRESHOLD) {
    ctx.consecutiveAllBlack++;
    if (ctx.consecutiveAllBlack * (LOOP_TARGET_US / 1000) >= INTERSECTION_CONFIRM_MS) {
      // confirmed black stop patch -> STOPPED
      motorDrive(0, 0);
      enterState(STATE_STOPPED);
      return;
    } else {
      // drive straight slowly to cross intersection
      motorDrive(MIN_SPEED, MIN_SPEED);
      return;
    }
  } else {
    // Not all black anymore: decide junction type
    // If left/center/right groups all active -> T or cross
    if (detectTJunction(f)) {
      // default: go straight for now (could be changed to turn preference)
      motorDrive(BASE_SPEED, BASE_SPEED);
      delay(80);
      ctx.integral = 0.0f;
      ctx.lastError = 0.0f;
      enterState(STATE_FOLLOW);
      return;
    } else {
      // Otherwise, resume follow
      ctx.integral = 0.0f;
      ctx.lastError = 0.0f;
      enterState(STATE_FOLLOW);
      return;
    }
  }
}

/* RECOVER: search for line using last-known heuristics */
void handleRecover() {
  SensorFrame &f = ctx.frame;

  // If we detect line again, resume
  if (f.activeCount > 0) {
    ctx.consecutiveRecoverFound++;
    if (ctx.consecutiveRecoverFound >= CONFIRM_FRAMES) {
      // small forward to stabilize
      motorDrive(BASE_SPEED / 2, BASE_SPEED / 2);
      delay(40);
      ctx.integral = 0.0f;
      ctx.lastError = 0.0f;
      ctx.lastDerivative = 0.0f;
      enterState(STATE_FOLLOW);
      return;
    }
  } else {
    ctx.consecutiveRecoverFound = 0;
  }

  // Search pattern: rotate toward last known side with gentle oscillation
  int searchPower = 150;
  if (ctx.lastKnownSide <= 0) {
    // rotate left
    motorDrive(-searchPower, searchPower);
  } else {
    // rotate right
    motorDrive(searchPower, -searchPower);
  }
}

/* STOPPED: remain stopped; could add LED blink or remote resume */
void handleStopped() {
  motorDrive(0, 0);
  // remain stopped until external reset or track change
}

/* =========================
   Steering & PID
   ========================= */

/*
  computeSteerCommand:
  - Primary: weighted position (continuous)
  - Secondary: thickness/balance assist
  - PD(I) controller with derivative smoothing
  - Returns signed steering command in PWM units
*/
int computeSteerCommand(const SensorFrame &f) {
  // Desired position is 0 (center). Error = 0 - measured
  float error = -f.weightedPos; // if weightedPos positive (line to right), error negative -> steer right

  // thickness assist scaled down
  float thicknessAssist = (f.thickness / 1000.0f) * THICKNESS_K;

  // Time delta
  unsigned long nowUs = microsSafe();
  float dt = float(nowUs - ctx.lastLoopUs) / 1e6f;
  if (dt <= 0.0f) dt = 0.001f;
  ctx.lastLoopUs = nowUs;

  // Integral with anti-windup
  ctx.integral += error * dt;
  if (ctx.integral > INTEGRAL_LIMIT) ctx.integral = INTEGRAL_LIMIT;
  if (ctx.integral < -INTEGRAL_LIMIT) ctx.integral = -INTEGRAL_LIMIT;

  // Derivative (smoothed)
  float rawDeriv = (error - ctx.lastError) / dt;
  float deriv = (DERIVATIVE_SMOOTH * ctx.lastDerivative) + ((1.0f - DERIVATIVE_SMOOTH) * rawDeriv);
  ctx.lastDerivative = deriv;

  // PID output
  float out = KP * error + KI * ctx.integral + KD * deriv + thicknessAssist;

  // Save last error
  ctx.lastError = error;

  // Map to PWM steering units: scale factor chosen empirically
  const float SCALE = 30.0f; // tune to map weight units to PWM
  int steer = int(out * SCALE);

  // Constrain
  steer = constrain(steer, -MOTOR_MAX, MOTOR_MAX);
  return steer;
}

/* =========================
   Decision helpers & patterns
   ========================= */

TurnDir detectSharpTurn(const SensorFrame &f) {
  // Outer groups: left 0..2, right 11..13
  uint8_t leftOuter = 0, rightOuter = 0;
  for (uint8_t i = 0; i <= 2; ++i) if (f.edge[i]) leftOuter++;
  for (uint8_t i = 11; i <= 13; ++i) if (f.edge[i]) rightOuter++;

  // Center strong count (sensors 5..8)
  uint8_t centerStrong = 0;
  for (uint8_t i = 5; i <= 8; ++i) if (f.edge[i]) centerStrong++;

  // If outer side strongly sees black and center is weak -> sharp turn
  if (leftOuter >= OUTER_STRONG_COUNT && centerStrong <= CENTER_WEAK_MAX) return TURN_LEFT;
  if (rightOuter >= OUTER_STRONG_COUNT && centerStrong <= CENTER_WEAK_MAX) return TURN_RIGHT;

  // Additional heuristic: if outer binary (not edge) is strong and center very weak
  uint8_t leftOuterBin = 0, rightOuterBin = 0;
  for (uint8_t i = 0; i <= 2; ++i) if (f.bin[i]) leftOuterBin++;
  for (uint8_t i = 11; i <= 13; ++i) if (f.bin[i]) rightOuterBin++;
  uint8_t centerBin = 0;
  for (uint8_t i = 5; i <= 8; ++i) if (f.bin[i]) centerBin++;

  if (leftOuterBin >= 3 && centerBin <= 1) return TURN_LEFT;
  if (rightOuterBin >= 3 && centerBin <= 1) return TURN_RIGHT;

  return TURN_NONE;
}

bool anyCenterActive(const SensorFrame &f) {
  // center sensors indices 6 and 7 (0-based)
  return f.bin[6] || f.bin[7] || f.norm[6] >= BINARY_TH || f.norm[7] >= BINARY_TH;
}

bool allWhite(const SensorFrame &f) {
  return f.activeCount == 0;
}

bool allBlack(const SensorFrame &f) {
  return f.activeCount >= SENSOR_COUNT;
}

bool detectTJunction(const SensorFrame &f) {
  // Group left (0..4), center (5..8), right (9..13)
  uint8_t leftG = 0, centerG = 0, rightG = 0;
  for (uint8_t i = 0; i <= 4; ++i) if (f.bin[i]) leftG++;
  for (uint8_t i = 5; i <= 8; ++i) if (f.bin[i]) centerG++;
  for (uint8_t i = 9; i <= 13; ++i) if (f.bin[i]) rightG++;

  uint8_t groups = 0;
  if (leftG >= 2) groups++;
  if (centerG >= 2) groups++;
  if (rightG >= 2) groups++;

  return (groups >= T_JUNCTION_MIN_GROUPS);
}

/* =========================
   Motor abstraction & braking
   ========================= */

void motorDrive(int leftCmd, int rightCmd) {
  leftCmd = constrain(leftCmd, MOTOR_MIN, MOTOR_MAX);
  rightCmd = constrain(rightCmd, MOTOR_MIN, MOTOR_MAX);
  setMotorChannel(LEFT_PWM_PIN, LEFT_DIR_PIN, leftCmd);
  setMotorChannel(RIGHT_PWM_PIN, RIGHT_DIR_PIN, rightCmd);
}

void setMotorChannel(uint8_t pwmPin, uint8_t dirPin, int cmd) {
  if (cmd == 0) {
    // coast / stop placeholder
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, 0);
    return;
  }
  if (cmd > 0) {
    digitalWrite(dirPin, HIGH); // forward
    analogWrite(pwmPin, cmd);
  } else {
    digitalWrite(dirPin, LOW); // reverse
    analogWrite(pwmPin, -cmd);
  }
}

// Hard brake: immediate stop placeholder; ready for electrical braking later
void hardBrake(unsigned long ms) {
  motorDrive(0, 0);
  delay(ms);
}

// Ramp brake: simple deceleration ramp from current speed to zero over ms
void rampBrake(int fromLeft, int fromRight, unsigned long ms) {
  // fromLeft/fromRight are nominal starting speeds (positive)
  // We'll step down in a few increments
  const uint8_t steps = 4;
  unsigned long stepMs = (ms / steps) + 1;
  for (uint8_t s = 0; s < steps; ++s) {
    float t = float(s) / float(steps);
    int l = int(fromLeft * (1.0f - t));
    int r = int(fromRight * (1.0f - t));
    motorDrive(l, r);
    delay(stepMs);
  }
  motorDrive(0, 0);
}

/* =========================
   Debug & Utilities
   ========================= */

void debugPrint(const SensorFrame &f) {
  Serial.print("N:");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(f.norm[i]);
    if (i < SENSOR_COUNT - 1) Serial.print(',');
  }
  Serial.print(" | B:");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) Serial.print(f.bin[i] ? '1' : '0');
  Serial.print(" | E:");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) Serial.print(f.edge[i] ? '1' : '0');
  Serial.print(" | mask:0x");
  Serial.print(f.mask, HEX);
  Serial.print(" | active:");
  Serial.print(f.activeCount);
  Serial.print(" | pos:");
  Serial.print(f.weightedPos, 3);
  Serial.print(" | thick:");
  Serial.print(f.thickness, 1);
  Serial.print(" | conf:");
  Serial.print(f.confidence, 2);
  Serial.print(" | state:");
  switch (ctx.state) {
    case STATE_FOLLOW: Serial.print("FOLLOW"); break;
    case STATE_PREPARE_TURN: Serial.print("PREPARE_TURN"); break;
    case STATE_EXECUTE_TURN: Serial.print("EXECUTE_TURN"); break;
    case STATE_INTERSECTION: Serial.print("INTERSECTION"); break;
    case STATE_RECOVER: Serial.print("RECOVER"); break;
    case STATE_STOPPED: Serial.print("STOPPED"); break;
  }
  Serial.print(" | pidI:");
  Serial.print(ctx.integral, 1);
  Serial.print(" | err:");
  Serial.print(ctx.lastError, 3);
  Serial.print(" | deriv:");
  Serial.print(ctx.lastDerivative, 2);
  Serial.print(" | lastSide:");
  if (ctx.lastKnownSide < 0) Serial.print("L");
  else if (ctx.lastKnownSide > 0) Serial.print("R");
  else Serial.print("C");
  Serial.println();
}

inline unsigned long microsSafe() {
  return micros();
}
inline unsigned long millisSafe() {
  return millis();
}

/* =========================
   End of file
   ========================= */
