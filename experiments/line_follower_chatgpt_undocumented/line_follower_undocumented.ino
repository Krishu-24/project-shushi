/*LineFollowerFinal/
├── LineFollowerFinal.ino
├── config.h
├── types.h
├── calibration.h
├── calibration.cpp
├── normalization.h
├── normalization.cpp
├── sensors.h
├── sensors.cpp
├── pid_control.h
├── pid_control.cpp
├── patterns.h
├── patterns.cpp
├── motors.h
├── motors.cpp
├── hard_brake.h
├── hard_brake.cpp
├── recovery.h
├── recovery.cpp
├── state_machine.h
├── state_machine.cpp
├── debug.h
└── debug.cpp

Role of each:
	•	config.h → constants, pins, thresholds
	•	types.h → structs/enums
	•	calibration.* → white/black capture, storage
	•	normalization.* → raw → normalized
	•	sensors.* → read and process all sensors
	•	pid_control.* → PID only
	•	patterns.* → binary logic and turn detection
	•	motors.* → TB9051 driver interface
	•	hard_brake.* → brake behavior
	•	recovery.* → lost-line recovery strategy
	•	state_machine.* → overall control flow
	•	debug.* → serial output / plot formatting
*/

// Teensy 4.1 - 14 channel analog line follower
// Modular state-machine based controller
// Designed for: normalized analog PID + binary pattern detection + hard-brake turn handling
//
// IMPORTANT:
// 1) Update MOTOR_DRIVER section to match your exact TB9051 wiring.
// 2) Update SENSOR_PINS order so index 0 is leftmost and index 13 is rightmost.
// 3) Tune thresholds, speeds, and gains on track.

#include <Arduino.h>

// ========================= USER CONFIG =========================
namespace Config {
  constexpr uint8_t SENSOR_COUNT = 14;

  // Leftmost -> rightmost
  constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {
    21, 20, 19, 18, 17, 16, 15, 14, 13, 41, 40, 39, 38, 37
  };

  // Symmetric weights for analog position estimate.
  // Center lies between indices 6 and 7 => position target = 0.
  constexpr float SENSOR_WEIGHTS[SENSOR_COUNT] = {
    -6.5f, -5.5f, -4.5f, -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f,  4.5f,  5.5f,  6.5f
  };

  // ADC / normalization
  constexpr int ADC_MAX = 4095;              // Teensy 12-bit
  constexpr int ANALOG_ON_THRESHOLD = 600;   // normalized black threshold (0..1000)
  constexpr int EDGE_ON_THRESHOLD   = 750;   // stronger threshold for turn-edge detection
  constexpr float LOST_LINE_MIN_SUM = 0.10f; // low analog confidence after normalization

  // PID
  constexpr float KP = 22.0f;
  constexpr float KI = 0.0f;     // start at 0
  constexpr float KD = 110.0f;
  constexpr float I_LIMIT = 120.0f;

  // Speeds
  constexpr int PWM_MAX = 4095;              // if using analogWriteResolution(12)
  constexpr int BASE_SPEED = 2200;
  constexpr int MIN_FOLLOW_SPEED = 0;
  constexpr int MAX_FOLLOW_SPEED = 3600;
  constexpr int TURN_SPEED_INNER = 1200;
  constexpr int TURN_SPEED_OUTER = 2200;
  constexpr int SEARCH_SPEED = 1200;

  // Timing
  constexpr uint32_t LOOP_US = 1000;               // 1 kHz control loop target
  constexpr uint32_t TURN_BRAKE_DELAY_MS = 12;     // small delay before aggressive brake
  constexpr uint32_t BRAKE_PULSE_MS = 18;          // electrical braking pulse
  constexpr uint32_t INTERSECTION_STRAIGHT_MS = 50;
  constexpr uint32_t LOST_LINE_STOP_MS = 250;
  constexpr uint32_t BLACK_PATCH_STOP_MS = 2000;
  constexpr uint32_t TURN_TIMEOUT_MS = 450;

  // Pattern thresholds using binary count
  constexpr uint8_t ALL_BLACK_MIN_COUNT = 13;
  constexpr uint8_t T_CROSS_MIN_COUNT = 10;
  constexpr uint8_t LOST_LINE_MAX_COUNT = 0;

  // Driver wiring placeholder for two brushed channels.
  // Replace with your actual TB9051 control pins / semantics.
  namespace MotorPins {
    constexpr uint8_t LEFT_PWM  = 2;
    constexpr uint8_t LEFT_IN1  = 3;
    constexpr uint8_t LEFT_IN2  = 4;
    constexpr uint8_t RIGHT_PWM = 5;
    constexpr uint8_t RIGHT_IN1 = 6;
    constexpr uint8_t RIGHT_IN2 = 7;
    constexpr uint8_t STBY_OR_EN = 8; // optional; set HIGH if used
  }
}

// ========================= TYPES =========================
enum class RunState : uint8_t {
  FOLLOW_LINE,
  PREPARE_TURN,
  EXECUTE_TURN,
  HANDLE_INTERSECTION,
  RECOVER_LINE,
  STOPPED
};

enum class TurnDir : int8_t {
  NONE = 0,
  LEFT = -1,
  RIGHT = 1
};

struct SensorFrame {
  int raw[Config::SENSOR_COUNT];
  int norm[Config::SENSOR_COUNT];        // 0..1000 (black high)
  uint8_t bin[Config::SENSOR_COUNT];     // thresholded for logic
  uint8_t edgeBin[Config::SENSOR_COUNT]; // stronger threshold for sharp-turn detection
  uint8_t activeCount = 0;
  uint16_t binaryMask = 0;
  float analogSum = 0.0f;                // sum of norm/1000
  float position = 0.0f;                 // weighted center estimate
  float leftMass = 0.0f;
  float rightMass = 0.0f;
  float thicknessBalance = 0.0f;         // rightMass - leftMass
  bool allWhite = false;
  bool allBlack = false;
  bool centerSeen = false;
  bool leftEdgeHit = false;
  bool rightEdgeHit = false;
};

struct CalibrationData {
  int white[Config::SENSOR_COUNT];
  int black[Config::SENSOR_COUNT];
  bool valid = false;
};

struct PIDData {
  float error = 0.0f;
  float prevError = 0.0f;
  float integral = 0.0f;
  float derivative = 0.0f;
  float output = 0.0f;
};

struct ControllerContext {
  RunState state = RunState::FOLLOW_LINE;
  TurnDir pendingTurn = TurnDir::NONE;
  TurnDir lastKnownLineSide = TurnDir::NONE;
  CalibrationData cal{};
  SensorFrame sensors{};
  PIDData pid{};
  uint32_t stateEnteredMs = 0;
  uint32_t allBlackSinceMs = 0;
  uint32_t lostLineSinceMs = 0;
};

ControllerContext ctx;

// ========================= FORWARD DECL =========================
void setupPins();
void setupPWM();
void captureWhiteCalibration();
void captureBlackCalibration();
void readSensors(SensorFrame &f);
int normalizeSensor(int rawValue, int whiteValue, int blackValue);
float computeLinePosition(const SensorFrame &f);
float computeThicknessBalance(const SensorFrame &f);
void updateStateMachine();
void followLine();
void prepareTurn();
void executeTurn();
void handleIntersection();
void recoverLine();
void stopRobot();
void motorDrive(int leftCmd, int rightCmd);
void motorBrake(uint32_t brakeMs);
void setMotorChannel(bool left, int cmd);
int clampPwm(int v);
TurnDir detectSharpTurnDirection(const SensorFrame &f);
TurnDir inferLostLineDirection(const SensorFrame &f);
bool detectTJunctionOrCross(const SensorFrame &f);
void enterState(RunState next);
const char* stateName(RunState s);
void printDebugFrame(const SensorFrame &f);

// ========================= SETUP =========================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogReadAveraging(4);
  setupPins();
  setupPWM();

  // Safe defaults in case you already measured and want to hand-edit later.
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    ctx.cal.white[i] = 3000;
    ctx.cal.black[i] = 800;
  }
  ctx.cal.valid = true;

  enterState(RunState::FOLLOW_LINE);
}

void loop() {
  static uint32_t lastLoopUs = 0;
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < Config::LOOP_US) return;
  lastLoopUs = nowUs;

  readSensors(ctx.sensors);
  updateStateMachine();

  // Uncomment for tuning
  // printDebugFrame(ctx.sensors);
}

// ========================= IO SETUP =========================
void setupPins() {
  using namespace Config;
  using namespace Config::MotorPins;

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(STBY_OR_EN, OUTPUT);
  digitalWrite(STBY_OR_EN, HIGH);

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
}

void setupPWM() {
  analogWriteResolution(12);
  analogWriteFrequency(Config::MotorPins::LEFT_PWM, 20000);
  analogWriteFrequency(Config::MotorPins::RIGHT_PWM, 20000);
}

// ========================= CALIBRATION =========================
void captureWhiteCalibration() {
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    ctx.cal.white[i] = analogRead(Config::SENSOR_PINS[i]);
  }
}

void captureBlackCalibration() {
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    ctx.cal.black[i] = analogRead(Config::SENSOR_PINS[i]);
  }
  ctx.cal.valid = true;
}

int normalizeSensor(int rawValue, int whiteValue, int blackValue) {
  // Expected for your reflective setup: white raw > black raw.
  int denom = whiteValue - blackValue;
  if (abs(denom) < 20) return 0;

  float n = (float)(whiteValue - rawValue) / (float)denom; // black -> ~1
  int out = (int)lroundf(n * 1000.0f);
  if (out < 0) out = 0;
  if (out > 1000) out = 1000;
  return out;
}

// ========================= SENSOR PIPELINE =========================
void readSensors(SensorFrame &f) {
  memset(&f, 0, sizeof(f));

  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    f.raw[i] = analogRead(Config::SENSOR_PINS[i]);
    f.norm[i] = normalizeSensor(f.raw[i], ctx.cal.white[i], ctx.cal.black[i]);

    f.bin[i] = (f.norm[i] >= Config::ANALOG_ON_THRESHOLD) ? 1 : 0;
    f.edgeBin[i] = (f.norm[i] >= Config::EDGE_ON_THRESHOLD) ? 1 : 0;

    f.activeCount += f.bin[i];
    f.binaryMask = (uint16_t)((f.binaryMask << 1) | f.bin[i]);

    const float a = f.norm[i] * 0.001f;
    f.analogSum += a;
    if (Config::SENSOR_WEIGHTS[i] < 0) f.leftMass += a;
    if (Config::SENSOR_WEIGHTS[i] > 0) f.rightMass += a;
  }

  f.position = computeLinePosition(f);
  f.thicknessBalance = computeThicknessBalance(f);
  f.allWhite = (f.activeCount <= Config::LOST_LINE_MAX_COUNT) || (f.analogSum < Config::LOST_LINE_MIN_SUM);
  f.allBlack = (f.activeCount >= Config::ALL_BLACK_MIN_COUNT);
  f.centerSeen = (f.bin[6] || f.bin[7]);

  // Strong edge hit for 90-degree detection
  const bool leftOuter = f.edgeBin[0] || f.edgeBin[1] || f.edgeBin[2];
  const bool rightOuter = f.edgeBin[11] || f.edgeBin[12] || f.edgeBin[13];
  const bool centerWeak = !(f.edgeBin[5] || f.edgeBin[6] || f.edgeBin[7] || f.edgeBin[8]);
  f.leftEdgeHit = leftOuter && centerWeak;
  f.rightEdgeHit = rightOuter && centerWeak;
}

float computeLinePosition(const SensorFrame &f) {
  float weightedSum = 0.0f;
  float total = 0.0f;
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    const float a = f.norm[i] * 0.001f;
    weightedSum += a * Config::SENSOR_WEIGHTS[i];
    total += a;
  }
  if (total < 0.001f) {
    return (ctx.lastKnownLineSide == TurnDir::LEFT) ? -7.0f :
           (ctx.lastKnownLineSide == TurnDir::RIGHT) ? 7.0f : 0.0f;
  }
  return weightedSum / total;
}

float computeThicknessBalance(const SensorFrame &f) {
  // Idea you proposed: compare right mass vs left mass.
  // Near zero means centered even if line thickness changes.
  return f.rightMass - f.leftMass;
}

// ========================= STATE MACHINE =========================
void updateStateMachine() {
  switch (ctx.state) {
    case RunState::FOLLOW_LINE:         followLine(); break;
    case RunState::PREPARE_TURN:        prepareTurn(); break;
    case RunState::EXECUTE_TURN:        executeTurn(); break;
    case RunState::HANDLE_INTERSECTION: handleIntersection(); break;
    case RunState::RECOVER_LINE:        recoverLine(); break;
    case RunState::STOPPED:             stopRobot(); break;
  }
}

void followLine() {
  const SensorFrame &f = ctx.sensors;

  if (f.position < -0.3f) ctx.lastKnownLineSide = TurnDir::LEFT;
  else if (f.position > 0.3f) ctx.lastKnownLineSide = TurnDir::RIGHT;

  if (f.allBlack) {
    if (ctx.allBlackSinceMs == 0) ctx.allBlackSinceMs = millis();
    if (millis() - ctx.allBlackSinceMs >= Config::BLACK_PATCH_STOP_MS) {
      enterState(RunState::STOPPED);
      return;
    }
    enterState(RunState::HANDLE_INTERSECTION);
    return;
  } else {
    ctx.allBlackSinceMs = 0;
  }

  if (f.allWhite) {
    if (ctx.lostLineSinceMs == 0) ctx.lostLineSinceMs = millis();
    if (millis() - ctx.lostLineSinceMs >= Config::LOST_LINE_STOP_MS) {
      enterState(RunState::RECOVER_LINE);
      return;
    }
  } else {
    ctx.lostLineSinceMs = 0;
  }

  TurnDir sharp = detectSharpTurnDirection(f);
  if (sharp != TurnDir::NONE) {
    ctx.pendingTurn = sharp;
    enterState(RunState::PREPARE_TURN);
    return;
  }

  // PID on analog position + small thickness-balance assist.
  ctx.pid.error = f.position + 0.35f * f.thicknessBalance;
  ctx.pid.integral += ctx.pid.error;
  if (ctx.pid.integral > Config::I_LIMIT) ctx.pid.integral = Config::I_LIMIT;
  if (ctx.pid.integral < -Config::I_LIMIT) ctx.pid.integral = -Config::I_LIMIT;
  ctx.pid.derivative = ctx.pid.error - ctx.pid.prevError;
  ctx.pid.output = Config::KP * ctx.pid.error
                 + Config::KI * ctx.pid.integral
                 + Config::KD * ctx.pid.derivative;
  ctx.pid.prevError = ctx.pid.error;

  int left = (int)lroundf(Config::BASE_SPEED - ctx.pid.output);
  int right = (int)lroundf(Config::BASE_SPEED + ctx.pid.output);
  left = constrain(left, Config::MIN_FOLLOW_SPEED, Config::MAX_FOLLOW_SPEED);
  right = constrain(right, Config::MIN_FOLLOW_SPEED, Config::MAX_FOLLOW_SPEED);
  motorDrive(left, right);
}

void prepareTurn() {
  // Small delay lets body center before hard brake, exactly like your idea.
  if (millis() - ctx.stateEnteredMs < Config::TURN_BRAKE_DELAY_MS) {
    followLine();
    return;
  }
  motorBrake(Config::BRAKE_PULSE_MS);
  enterState(RunState::EXECUTE_TURN);
}

void executeTurn() {
  const SensorFrame &f = ctx.sensors;
  const uint32_t elapsed = millis() - ctx.stateEnteredMs;

  if (elapsed > Config::TURN_TIMEOUT_MS) {
    enterState(RunState::RECOVER_LINE);
    return;
  }

  if (ctx.pendingTurn == TurnDir::LEFT) {
    motorDrive(-Config::TURN_SPEED_INNER, Config::TURN_SPEED_OUTER);
  } else if (ctx.pendingTurn == TurnDir::RIGHT) {
    motorDrive(Config::TURN_SPEED_OUTER, -Config::TURN_SPEED_INNER);
  } else {
    enterState(RunState::FOLLOW_LINE);
    return;
  }

  // Resume PID once middle sensors reacquire the line.
  if (f.centerSeen) {
    ctx.pendingTurn = TurnDir::NONE;
    ctx.pid.prevError = 0.0f;
    ctx.pid.integral = 0.0f;
    enterState(RunState::FOLLOW_LINE);
  }
}

void handleIntersection() {
  const SensorFrame &f = ctx.sensors;

  // Your idea: all black => go straight briefly, if it stays black long enough => stop patch.
  if (f.allBlack) {
    motorDrive(Config::BASE_SPEED, Config::BASE_SPEED);
    if (millis() - ctx.stateEnteredMs >= Config::INTERSECTION_STRAIGHT_MS) {
      enterState(RunState::FOLLOW_LINE);
    }
    return;
  }

  if (detectTJunctionOrCross(f)) {
    motorDrive(Config::BASE_SPEED, Config::BASE_SPEED);
    return;
  }

  enterState(RunState::FOLLOW_LINE);
}

void recoverLine() {
  TurnDir dir = inferLostLineDirection(ctx.sensors);
  if (dir == TurnDir::LEFT) {
    motorDrive(-Config::SEARCH_SPEED, Config::SEARCH_SPEED);
  } else if (dir == TurnDir::RIGHT) {
    motorDrive(Config::SEARCH_SPEED, -Config::SEARCH_SPEED);
  } else {
    motorDrive(0, 0);
  }

  if (ctx.sensors.centerSeen || ctx.sensors.activeCount >= 2) {
    ctx.pid.prevError = 0.0f;
    ctx.pid.integral = 0.0f;
    enterState(RunState::FOLLOW_LINE);
  }
}

void stopRobot() {
  motorDrive(0, 0);
}

void enterState(RunState next) {
  ctx.state = next;
  ctx.stateEnteredMs = millis();
}

// ========================= DECISION HELPERS =========================
TurnDir detectSharpTurnDirection(const SensorFrame &f) {
  if (f.leftEdgeHit && !f.rightEdgeHit) return TurnDir::LEFT;
  if (f.rightEdgeHit && !f.leftEdgeHit) return TurnDir::RIGHT;
  return TurnDir::NONE;
}

TurnDir inferLostLineDirection(const SensorFrame &f) {
  if (f.leftMass > f.rightMass + 0.05f) return TurnDir::LEFT;
  if (f.rightMass > f.leftMass + 0.05f) return TurnDir::RIGHT;
  return ctx.lastKnownLineSide;
}

bool detectTJunctionOrCross(const SensorFrame &f) {
  const bool left = f.bin[0] || f.bin[1] || f.bin[2] || f.bin[3];
  const bool center = f.bin[5] || f.bin[6] || f.bin[7] || f.bin[8];
  const bool right = f.bin[10] || f.bin[11] || f.bin[12] || f.bin[13];
  return (left && center && right) || (f.activeCount >= Config::T_CROSS_MIN_COUNT);
}

// ========================= MOTOR LAYER =========================
int clampPwm(int v) {
  return constrain(v, -Config::PWM_MAX, Config::PWM_MAX);
}

void motorDrive(int leftCmd, int rightCmd) {
  setMotorChannel(true, clampPwm(leftCmd));
  setMotorChannel(false, clampPwm(rightCmd));
}

void motorBrake(uint32_t brakeMs) {
  // Replace this with the exact TB9051 electrical brake mode you validated.
  // For now: short aggressive reverse-torque pulse would be too risky,
  // so do a command-zero brake placeholder.
  motorDrive(0, 0);
  delay(brakeMs);
}

void setMotorChannel(bool left, int cmd) {
  using namespace Config::MotorPins;
  const uint8_t pwm = left ? LEFT_PWM : RIGHT_PWM;
  const uint8_t in1 = left ? LEFT_IN1 : RIGHT_IN1;
  const uint8_t in2 = left ? LEFT_IN2 : RIGHT_IN2;

  if (cmd > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, cmd);
  } else if (cmd < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -cmd);
  } else {
    // Coast/stop placeholder; adapt for true TB9051 brake mode.
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}

// ========================= DEBUG =========================
const char* stateName(RunState s) {
  switch (s) {
    case RunState::FOLLOW_LINE: return "FOLLOW";
    case RunState::PREPARE_TURN: return "PRE_TURN";
    case RunState::EXECUTE_TURN: return "TURN";
    case RunState::HANDLE_INTERSECTION: return "XING";
    case RunState::RECOVER_LINE: return "RECOVER";
    case RunState::STOPPED: return "STOP";
    default: return "?";
  }
}

void printDebugFrame(const SensorFrame &f) {
  Serial.print(stateName(ctx.state));
  Serial.print(" pos=");
  Serial.print(f.position, 3);
  Serial.print(" thick=");
  Serial.print(f.thicknessBalance, 3);
  Serial.print(" count=");
  Serial.print(f.activeCount);
  Serial.print(" mask=");
  Serial.print(f.binaryMask, BIN);
  Serial.print(" norm=");
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    Serial.print(f.norm[i]);
    if (i != Config::SENSOR_COUNT - 1) Serial.print(',');
  }
  Serial.println();
}
