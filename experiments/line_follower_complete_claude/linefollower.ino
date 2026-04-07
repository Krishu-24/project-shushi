// =============================================================================
//  COMPETITION LINE FOLLOWER  —  Teensy 4.1
//  Architecture: Layered Behavior
//    Layer 0 — Sensing      : Raw → Normalized → Binary → Derived metrics
//    Layer 1 — Estimation   : Weighted position, confidence, balance, curvature
//    Layer 2 — Control Base : Analog PID always running
//    Layer 3 — Override     : Binary/pattern logic fires hard overrides
//    Layer 4 — Governor     : State machine with hysteresis + frame-confirmation
//                             decides which layer dominates and sets target speed
//
//  Why this architecture?
//    A plain FSM swaps all logic at once and thrashes on noisy transitions.
//    A pure PID ignores track topology. Layering lets the PID run continuously
//    while discrete pattern events inject short-lived overrides that fade back
//    to normal follow once conditions clear — much more stable at speed.
// =============================================================================

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
//  HARDWARE PINS
// ─────────────────────────────────────────────────────────────────────────────

// 14 IR sensors, left→right
static const uint8_t SENSOR_PINS[14] = {21,20,19,18,17,16,15,14,13,41,40,39,38,37};
static const uint8_t NUM_SENSORS = 14;

// Motor A = Left,  Motor B = Right
// Each motor: one PWM pin + one direction pin (generic signed-command style)
// Swap/invert to match your actual TB9051FTG or other driver wiring.
static const uint8_t MOTOR_L_PWM = 2;
static const uint8_t MOTOR_L_DIR = 3;
static const uint8_t MOTOR_R_PWM = 4;
static const uint8_t MOTOR_R_DIR = 5;

// Optional: driver enable pins (tie high if not used)
static const uint8_t MOTOR_L_EN  = 6;
static const uint8_t MOTOR_R_EN  = 7;

// ─────────────────────────────────────────────────────────────────────────────
//  TUNING CONSTANTS  —  grouped so you can find everything in one place
// ─────────────────────────────────────────────────────────────────────────────

// --- ADC ---
static const uint8_t  ADC_BITS        = 12;
static const uint8_t  ADC_AVERAGING   = 4;
static const uint16_t ADC_MAX         = 4095;

// --- Sensor normalization (0–1000, black=high) ---
// Replace these arrays with values from your calibration capture routine.
// Index order matches SENSOR_PINS left→right.
static const int32_t CAL_WHITE[14] = { 300, 310, 305, 295, 300, 308, 302,
                                        298, 305, 300, 295, 310, 300, 305 };
static const int32_t CAL_BLACK[14] = {3600,3580,3590,3610,3600,3570,3600,
                                       3590,3610,3600,3590,3580,3600,3610 };

// --- Binary thresholds (applied to normalized 0–1000 values) ---
static const int32_t THRESH_BINARY    = 500;   // on/off line detection
static const int32_t THRESH_EDGE      = 700;   // "strongly on" (for pattern logic)
static const int32_t THRESH_CONFIDENT = 300;   // minimum norm to count as contributing

// --- Symmetric sensor weights, left→right, center between sensors 6 & 7 ---
static const float SENSOR_WEIGHTS[14] = {
    -6.5f, -5.5f, -4.5f, -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f,  4.5f,  5.5f,  6.5f
};

// --- PID gains (start conservative; tune Kp first, then Kd) ---
static const float KP_FOLLOW   = 28.0f;
static const float KI_FOLLOW   = 0.0f;   // keep 0 until robot is otherwise stable
static const float KD_FOLLOW   = 180.0f;

// Gain schedule: when |error| is large we reduce Kp slightly to avoid windup
// and increase Kd to damp oscillation — simple two-zone schedule
static const float ERROR_HIGH_THRESH  = 3.5f;   // in weight units
static const float KP_HIGH            = 20.0f;
static const float KD_HIGH            = 220.0f;

// Anti-windup: clamp integral contribution
static const float KI_CLAMP          = 50.0f;

// --- Speed scheduling ---
static const float BASE_SPEED        = 0.72f;  // 0–1, fraction of full PWM
static const float SPEED_MIN         = 0.30f;  // floor when cornering hard
static const float SPEED_CORNER      = 0.42f;  // during turn-approach braking
static const float SPEED_RECOVERY    = 0.35f;  // during line-loss search
// Speed reduction factor based on |error| (higher error → slower)
static const float SPEED_ERR_SCALE   = 0.04f;  // subtract this × |error| from base

// --- Sharp turn detection ---
// A sharp corner is indicated when outer sensors fire strongly while
// center sensors are weak or absent.  We use a "corner score" rather
// than a hard sensor count to avoid triggering on thick-line variations.
static const uint8_t  OUTER_SENSORS_EACH = 3;    // how many edge sensors count as "outer"
static const int32_t  CORNER_OUTER_THRESH = 700; // norm threshold for outer sensors
static const int32_t  CORNER_CENTER_MAX  = 300;  // norm ceiling for center sensors
static const uint8_t  CORNER_OUTER_MIN   = 2;    // minimum outer sensors needed
static const uint8_t  CORNER_CENTER_MAX_CNT = 2; // maximum center sensors allowed
static const uint8_t  CORNER_CONFIRM_FRAMES = 3; // consecutive frames before acting
static const uint32_t CORNER_SETTLE_MS  = 25;    // brief forward settle before pivot
static const uint32_t CORNER_TIMEOUT_MS = 800;   // max turn time before giving up
static const int32_t  CORNER_REACQ_THRESH = 450; // center norm to call line reacquired

// --- All-black / intersection logic ---
static const uint8_t  ALL_BLACK_MIN_COUNT = 11;  // sensors needed to call "all black"
static const uint32_t ALLBLACK_CROSS_MS  = 120;  // treat as crossing; go straight
static const uint32_t ALLBLACK_STOP_MS   = 2000; // treat as stop patch

// --- Line loss / recovery ---
static const uint32_t LOSS_CONFIRM_MS    = 40;   // debounce before declaring loss
static const uint32_t RECOVERY_TIMEOUT_MS= 1500; // give up search after this long
static const float    RECOVERY_SPIN_SPEED= 0.28f;// slow spin to find line

// --- Braking ---
// Placeholder: currently coast-to-stop.  Replace body with reverse-torque
// pulse or staged decel ramp when you have characterised the drivetrain.
static const uint32_t BRAKE_DURATION_MS  = 60;

// --- PWM ---
static const uint32_t PWM_FREQ_HZ        = 20000; // 20 kHz — above audible range
static const uint16_t PWM_MAX            = 255;   // analogWrite resolution

// --- Debug serial ---
static const uint32_t DEBUG_INTERVAL_MS  = 40;    // print every N ms

// ─────────────────────────────────────────────────────────────────────────────
//  DATA STRUCTURES
// ─────────────────────────────────────────────────────────────────────────────

// All sensor data for one processing frame
struct SensorFrame {
    uint16_t raw[14];          // 12-bit ADC readings
    int32_t  norm[14];         // normalized 0–1000 (black=high)
    bool     binary[14];       // true = on line (norm >= THRESH_BINARY)
    bool     edge[14];         // true = strongly on line (norm >= THRESH_EDGE)

    uint8_t  activeCount;      // number of binary-true sensors
    uint8_t  edgeCount;        // number of edge-true sensors
    uint16_t binaryMask;       // compact 14-bit mask of binary[] (LSB = sensor 0)

    int32_t  analogSum;        // sum of norm values (proxy for line width/confidence)
    float    confidence;       // 0.0–1.0: how much we trust the position estimate
    float    position;         // weighted analog position in weight units
    float    leftMass;         // analog mass of left half (sensors 0–6)
    float    rightMass;        // analog mass of right half (sensors 7–13)
    float    balance;          // (rightMass - leftMass) / (rightMass + leftMass + 1)
                                // +1 = right-heavy, −1 = left-heavy, ≈0 = centered

    // Pattern flags (set by analyzePatterns())
    bool     allBlack;
    bool     allWhite;
    bool     sharpLeft;        // outer-left sensors hot, center cold
    bool     sharpRight;
    bool     tJunctionLeft;
    bool     tJunctionRight;
    bool     crossroad;
    bool     lineLost;
};

// PID state
struct PIDState {
    float prevError;
    float integral;
    float derivative;
    float output;
    uint32_t lastTimeUs;
};

// Robot governor state machine
enum class BotState : uint8_t {
    FOLLOW,          // normal line following — PID drives steering
    CORNER_ENTER,    // brief forward settle before pivot
    TURNING,         // hard pivot until line reacquired
    INTERSECTION,    // crossing or thick patch — straight ahead
    STOP_PATCH,      // terminal black area — halt
    LINE_LOSS,       // line lost — searching
    STOPPED,         // deliberate halt (stop patch or E-stop)
};

// Complete robot state, passed around by reference
struct RobotState {
    SensorFrame  sf;
    PIDState     pid;
    BotState     mode;
    BotState     prevMode;

    float        targetSpeed;       // 0–1 normalised
    float        steeringOutput;    // –1 to +1 (negative=left, positive=right)

    int8_t       lastTurnDir;       // +1=right, -1=left, 0=unknown
    float        lastGoodPosition;  // position estimate when line was last seen
    uint32_t     modeEntryMs;       // millis() when current mode was entered

    // Corner detection: consecutive-frame confirmation counter
    uint8_t      cornerConfirmLeft;
    uint8_t      cornerConfirmRight;

    // All-black / intersection timer
    uint32_t     allBlackEntryMs;

    // Line-loss timer
    uint32_t     lineLossEntryMs;
    bool         lineLossActive;

    // Curvature estimator: derivative of position over recent frames
    // Simple ring buffer of recent positions for smoothed dPos/dt
    static const uint8_t CRV_BUF = 6;
    float        posHistory[CRV_BUF];
    uint8_t      posHistIdx;
    float        curvature;         // approximate d(position)/dt in units/s

    // Debug
    uint32_t     lastDebugMs;
    uint32_t     loopCount;
};

// ─────────────────────────────────────────────────────────────────────────────
//  GLOBAL ROBOT STATE  (single instance)
// ─────────────────────────────────────────────────────────────────────────────

static RobotState R;

// ─────────────────────────────────────────────────────────────────────────────
//  MOTOR DRIVER  —  generic signed-command interface
//  commandL / commandR: –255 … +255  (positive = forward)
//  Adapt the body to TB9051FTG pin logic when wiring is confirmed.
// ─────────────────────────────────────────────────────────────────────────────

void motorSetup() {
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_DIR, OUTPUT);
    pinMode(MOTOR_L_EN,  OUTPUT);
    pinMode(MOTOR_R_EN,  OUTPUT);

    analogWriteFrequency(MOTOR_L_PWM, PWM_FREQ_HZ);
    analogWriteFrequency(MOTOR_R_PWM, PWM_FREQ_HZ);

    digitalWrite(MOTOR_L_EN, HIGH);
    digitalWrite(MOTOR_R_EN, HIGH);
}

inline void motorWrite(uint8_t pwmPin, uint8_t dirPin, int16_t cmd) {
    // cmd: –255…+255.  Positive = forward.
    if (cmd >= 0) {
        digitalWrite(dirPin, LOW);
        analogWrite(pwmPin, (uint8_t)min(cmd, (int16_t)PWM_MAX));
    } else {
        digitalWrite(dirPin, HIGH);
        analogWrite(pwmPin, (uint8_t)min(-cmd, (int16_t)PWM_MAX));
    }
}

// High-level drive: speedFraction 0–1, steering –1…+1
// Positive steering turns right (right motor slows / reverses)
void driveMotors(float speedFrac, float steering) {
    speedFrac = constrain(speedFrac, -1.0f, 1.0f);
    steering  = constrain(steering,  -1.0f, 1.0f);

    float leftCmd  = speedFrac - steering;
    float rightCmd = speedFrac + steering;

    // Normalise if either exceeds ±1 while preserving ratio
    float maxMag = max(fabsf(leftCmd), fabsf(rightCmd));
    if (maxMag > 1.0f) {
        leftCmd  /= maxMag;
        rightCmd /= maxMag;
    }

    motorWrite(MOTOR_L_PWM, MOTOR_L_DIR, (int16_t)(leftCmd  * PWM_MAX));
    motorWrite(MOTOR_R_PWM, MOTOR_R_DIR, (int16_t)(rightCmd * PWM_MAX));
}

// Hard brake — placeholder; replace with reverse-torque pulse or ramp
// when drivetrain is characterised.  Currently coasts both motors to zero.
void hardBrake() {
    motorWrite(MOTOR_L_PWM, MOTOR_L_DIR, 0);
    motorWrite(MOTOR_R_PWM, MOTOR_R_DIR, 0);
    // Future: brief reverse pulse for active braking, then 0
    // Example staged ramp:
    //   driveMotors(-0.4f, 0.0f); delay(15);
    //   driveMotors( 0.0f, 0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
//  LAYER 0 — SENSING
//  Reads ADC, normalises per-sensor, computes binary masks
// ─────────────────────────────────────────────────────────────────────────────

void readSensors(SensorFrame &sf) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sf.raw[i] = (uint16_t)analogRead(SENSOR_PINS[i]);

        // Per-sensor normalisation: map [white..black] → [0..1000]
        // Clamp to keep rounding/noise from going out of range
        int32_t span = CAL_BLACK[i] - CAL_WHITE[i];
        if (span < 1) span = 1;  // guard div-by-zero on bad calibration
        sf.norm[i] = constrain(
            (int32_t)((sf.raw[i] - CAL_WHITE[i]) * 1000L / span),
            0, 1000);

        sf.binary[i] = (sf.norm[i] >= THRESH_BINARY);
        sf.edge[i]   = (sf.norm[i] >= THRESH_EDGE);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  LAYER 1 — ESTIMATION
//  Computes all derived metrics from one SensorFrame
// ─────────────────────────────────────────────────────────────────────────────

void computeMetrics(SensorFrame &sf) {
    sf.activeCount = 0;
    sf.edgeCount   = 0;
    sf.binaryMask  = 0;
    sf.analogSum   = 0;
    sf.leftMass    = 0.0f;
    sf.rightMass   = 0.0f;

    float weightedSum = 0.0f;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sf.binary[i]) {
            sf.activeCount++;
            sf.binaryMask |= (1u << i);
        }
        if (sf.edge[i]) sf.edgeCount++;

        sf.analogSum += sf.norm[i];

        // Only sensors above THRESH_CONFIDENT contribute to position estimate
        // (avoids noise from barely-lit sensors pulling the average)
        if (sf.norm[i] >= THRESH_CONFIDENT) {
            float w = (float)sf.norm[i];
            weightedSum += SENSOR_WEIGHTS[i] * w;

            if (i < 7) sf.leftMass  += w;
            else       sf.rightMass += w;
        }
    }

    float totalMass = sf.leftMass + sf.rightMass;

    if (totalMass > 0.0f) {
        sf.position = weightedSum / totalMass;
    } else {
        sf.position = 0.0f;  // fallback; caller should check confidence
    }

    // Balance: normalised –1…+1.  Independent of line width — useful as
    // a secondary centering check when totalMass is uncertain.
    sf.balance = (sf.rightMass - sf.leftMass) / (totalMass + 1.0f);

    // Confidence: based on how many sensors are active AND how strong the signal is.
    // High analogSum with moderate activeCount = well-lit single line = high confidence.
    // Too few active = lost.  Too many = crossing or wide patch = lower confidence.
    float normSum = (float)sf.analogSum / (1000.0f * NUM_SENSORS);  // 0–1
    float activeFrac = (float)sf.activeCount / (float)NUM_SENSORS;

    if (sf.activeCount == 0) {
        sf.confidence = 0.0f;
    } else if (sf.activeCount >= ALL_BLACK_MIN_COUNT) {
        sf.confidence = 0.3f;  // penalise all-black; don't trust position
    } else {
        // Peak confidence around 3–5 sensors active, degrades toward extremes
        float peakFrac = 4.0f / NUM_SENSORS;
        float sharpness = 1.0f - fabsf(activeFrac - peakFrac) * 4.0f;
        sf.confidence = constrain(sharpness * normSum * 3.0f, 0.0f, 1.0f);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  PATTERN ANALYSIS
//  All binary / structural pattern detection lives here.
//  Uses binaryMask + norm values to classify the current track condition.
// ─────────────────────────────────────────────────────────────────────────────

void analyzePatterns(SensorFrame &sf) {
    sf.allBlack      = (sf.activeCount >= ALL_BLACK_MIN_COUNT);
    sf.allWhite      = (sf.activeCount == 0);
    sf.lineLost      = (sf.activeCount == 0);
    sf.sharpLeft     = false;
    sf.sharpRight    = false;
    sf.tJunctionLeft = false;
    sf.tJunctionRight= false;
    sf.crossroad     = false;

    // --- Sharp turn detection ---
    // Outer sensors: [0..OUTER_SENSORS_EACH-1] (left) and
    //                [14-OUTER_SENSORS_EACH..13] (right)
    // A sharp corner scores high on ONE outer side while center is cold.
    // Using norm values (not just binary) gives graded confidence.

    uint8_t outerLeftHot  = 0;
    uint8_t outerRightHot = 0;
    uint8_t centerHot     = 0;
    const uint8_t CENTER_START = OUTER_SENSORS_EACH;
    const uint8_t CENTER_END   = NUM_SENSORS - OUTER_SENSORS_EACH;  // exclusive

    for (uint8_t i = 0; i < OUTER_SENSORS_EACH; i++) {
        if (sf.norm[i] >= CORNER_OUTER_THRESH) outerLeftHot++;
    }
    for (uint8_t i = NUM_SENSORS - OUTER_SENSORS_EACH; i < NUM_SENSORS; i++) {
        if (sf.norm[i] >= CORNER_OUTER_THRESH) outerRightHot++;
    }
    for (uint8_t i = CENTER_START; i < CENTER_END; i++) {
        if (sf.norm[i] >= CORNER_CENTER_MAX) centerHot++;
    }

    bool centerCold = (centerHot <= CORNER_CENTER_MAX_CNT);

    if (outerLeftHot  >= CORNER_OUTER_MIN && centerCold && outerRightHot == 0)
        sf.sharpLeft = true;
    if (outerRightHot >= CORNER_OUTER_MIN && centerCold && outerLeftHot  == 0)
        sf.sharpRight = true;

    // --- T-junction / crossroad ---
    // If outer sensors on BOTH sides fire strongly = T or cross
    // Distinguish T from cross using center sensor count
    if (!sf.allBlack && outerLeftHot >= CORNER_OUTER_MIN && outerRightHot >= CORNER_OUTER_MIN) {
        if (centerHot >= 4) {
            sf.crossroad     = true;
        } else if (outerLeftHot > outerRightHot) {
            sf.tJunctionLeft = true;
        } else {
            sf.tJunctionRight= true;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  CURVATURE ESTIMATOR
//  Approximate dPosition/dt over a short history window.
//  Used by the speed governor to slow down proactively before confirmed corners.
// ─────────────────────────────────────────────────────────────────────────────

void updateCurvature(RobotState &r, float dt) {
    // Push position into ring buffer
    r.posHistory[r.posHistIdx] = r.sf.position;
    r.posHistIdx = (r.posHistIdx + 1) % RobotState::CRV_BUF;

    // Simple linear regression slope over the ring buffer as curvature proxy
    // (could be replaced by a proper velocity estimate; kept fast for loop budget)
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    uint8_t n = RobotState::CRV_BUF;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (r.posHistIdx + i) % n;
        float x = (float)i;
        float y = r.posHistory[idx];
        sumX  += x;
        sumY  += y;
        sumXY += x * y;
        sumX2 += x * x;
    }
    float denom = n * sumX2 - sumX * sumX;
    if (fabsf(denom) > 0.01f)
        r.curvature = (n * sumXY - sumX * sumY) / denom / (dt + 1e-6f);
    else
        r.curvature = 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  LAYER 2 — CONTROL BASE  (PID)
//  Runs continuously in FOLLOW mode.
//  Gain scheduling and integral management happen here.
// ─────────────────────────────────────────────────────────────────────────────

float runPID(PIDState &pid, float error, float dt) {
    // --- Gain schedule ---
    float kp = KP_FOLLOW, kd = KD_FOLLOW, ki = KI_FOLLOW;
    if (fabsf(error) > ERROR_HIGH_THRESH) {
        kp = KP_HIGH;
        kd = KD_HIGH;
    }

    // --- Derivative on measurement (not error) to avoid derivative kick ---
    // Use smoothed finite-difference from stored prevError
    float derivative = 0.0f;
    if (dt > 0.0001f) {
        derivative = (error - pid.prevError) / dt;
    }
    pid.derivative = derivative;

    // --- Integral with anti-windup clamp ---
    pid.integral += error * dt;
    pid.integral  = constrain(pid.integral, -KI_CLAMP, KI_CLAMP);

    // --- Output ---
    float output = kp * error + kd * derivative + ki * pid.integral;
    pid.prevError = error;
    pid.output    = output;

    return output;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SPEED GOVERNOR
//  Sets targetSpeed based on current mode, error magnitude, curvature, and
//  sensor confidence.  Decoupled from steering so both can be tuned separately.
// ─────────────────────────────────────────────────────────────────────────────

float computeTargetSpeed(const RobotState &r) {
    const SensorFrame &sf = r.sf;

    switch (r.mode) {
        case BotState::STOPPED:
        case BotState::STOP_PATCH:
            return 0.0f;

        case BotState::CORNER_ENTER:
            return SPEED_CORNER;

        case BotState::TURNING:
            // Slow pivot speed; direction handled separately
            return SPEED_CORNER * 0.6f;

        case BotState::LINE_LOSS:
            return SPEED_RECOVERY;

        case BotState::INTERSECTION:
            return BASE_SPEED * 0.85f;  // cross straight at near-base speed

        case BotState::FOLLOW:
        default: {
            float speed = BASE_SPEED;

            // Reduce for large positional error
            speed -= SPEED_ERR_SCALE * fabsf(r.sf.position);

            // Reduce for high curvature (anticipate upcoming bend)
            speed -= 0.012f * fabsf(r.curvature);

            // Reduce for low confidence (uncertain where line is)
            speed -= 0.15f * (1.0f - sf.confidence);

            return constrain(speed, SPEED_MIN, BASE_SPEED);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  LAYER 3 — OVERRIDE LOGIC  (state transitions)
//  Decides which BotState to be in based on confirmed pattern events.
//  Uses consecutive-frame confirmation to avoid single-frame glitches.
// ─────────────────────────────────────────────────────────────────────────────

void updateGovernor(RobotState &r) {
    uint32_t now = millis();
    const SensorFrame &sf = r.sf;

    auto enterMode = [&](BotState s) {
        if (r.mode != s) {
            r.prevMode     = r.mode;
            r.mode         = s;
            r.modeEntryMs  = now;
        }
    };

    // ── STOPPED is terminal ──────────────────────────────────────────────────
    if (r.mode == BotState::STOPPED) return;

    // ── STOP_PATCH: wait indefinitely ───────────────────────────────────────
    if (r.mode == BotState::STOP_PATCH) {
        hardBrake();
        enterMode(BotState::STOPPED);
        return;
    }

    // ── ALL-BLACK logic ──────────────────────────────────────────────────────
    if (sf.allBlack) {
        if (r.mode != BotState::INTERSECTION) {
            r.allBlackEntryMs = now;
            enterMode(BotState::INTERSECTION);
        } else {
            uint32_t elapsed = now - r.allBlackEntryMs;
            if (elapsed >= ALLBLACK_STOP_MS) {
                enterMode(BotState::STOP_PATCH);
            }
            // else stay in INTERSECTION and drive straight (handled in execute)
        }
        return;
    } else {
        // Leaving all-black: return to FOLLOW if we were in INTERSECTION
        if (r.mode == BotState::INTERSECTION) {
            r.pid.integral = 0.0f;  // flush integral before re-entering follow
            r.pid.prevError = r.sf.position;
            enterMode(BotState::FOLLOW);
        }
    }

    // ── SHARP CORNER state machine ───────────────────────────────────────────
    // TURNING: check for line reacquisition
    if (r.mode == BotState::TURNING) {
        // Reacquired if center sensors are seeing the line again
        uint8_t centerActive = 0;
        for (uint8_t i = 4; i < 10; i++) {  // sensors 4..9 = center band
            if (sf.norm[i] >= CORNER_REACQ_THRESH) centerActive++;
        }
        bool reacquired = (centerActive >= 2);

        if (reacquired) {
            // Reset PID and resume following
            r.pid.integral  = 0.0f;
            r.pid.prevError = sf.position;
            enterMode(BotState::FOLLOW);
        } else if (now - r.modeEntryMs > CORNER_TIMEOUT_MS) {
            // Timed out: fall through to line-loss
            r.lineLossEntryMs = now;
            r.lineLossActive  = true;
            enterMode(BotState::LINE_LOSS);
        }
        return;
    }

    // CORNER_ENTER: short settle then pivot
    if (r.mode == BotState::CORNER_ENTER) {
        if (now - r.modeEntryMs >= CORNER_SETTLE_MS) {
            hardBrake();
            delayMicroseconds(8000);  // brief pause
            enterMode(BotState::TURNING);
        }
        return;
    }

    // Detect corner in FOLLOW mode with frame-confirmation
    if (r.mode == BotState::FOLLOW) {
        if (sf.sharpLeft) {
            r.cornerConfirmLeft++;
            r.cornerConfirmRight = 0;
        } else if (sf.sharpRight) {
            r.cornerConfirmRight++;
            r.cornerConfirmLeft = 0;
        } else {
            r.cornerConfirmLeft  = 0;
            r.cornerConfirmRight = 0;
        }

        if (r.cornerConfirmLeft >= CORNER_CONFIRM_FRAMES) {
            r.lastTurnDir       = -1;
            r.cornerConfirmLeft  = 0;
            enterMode(BotState::CORNER_ENTER);
            return;
        }
        if (r.cornerConfirmRight >= CORNER_CONFIRM_FRAMES) {
            r.lastTurnDir        = +1;
            r.cornerConfirmRight = 0;
            enterMode(BotState::CORNER_ENTER);
            return;
        }
    }

    // ── LINE LOSS ────────────────────────────────────────────────────────────
    if (sf.lineLost) {
        if (!r.lineLossActive) {
            r.lineLossEntryMs = now;
            r.lineLossActive  = true;
        }
        if (now - r.lineLossEntryMs >= LOSS_CONFIRM_MS) {
            enterMode(BotState::LINE_LOSS);
        }
    } else {
        r.lineLossActive = false;
        r.lastGoodPosition = sf.position;
        if (r.mode == BotState::LINE_LOSS) {
            r.pid.integral  = 0.0f;
            r.pid.prevError = sf.position;
            enterMode(BotState::FOLLOW);
        }
    }

    // ── LINE_LOSS: recovery search timeout ──────────────────────────────────
    if (r.mode == BotState::LINE_LOSS) {
        if (now - r.modeEntryMs > RECOVERY_TIMEOUT_MS) {
            // Give up and stop
            enterMode(BotState::STOPPED);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  LAYER 4 — EXECUTE
//  Turns current mode + control output into actual motor commands
// ─────────────────────────────────────────────────────────────────────────────

void executeMotion(RobotState &r, float dt) {
    switch (r.mode) {

        case BotState::FOLLOW: {
            // Primary: analog-weighted position for smooth continuous error
            float error = r.sf.position;

            // Secondary correction: blend in balance metric lightly.
            // balance tells us whether mass is left or right of center,
            // useful when line is thick and position estimate wiggles.
            // Use a small gain so it doesn't dominate.
            float balanceCorrection = r.sf.balance * 0.8f;
            error += balanceCorrection;

            float pidOut = runPID(r.pid, error, dt);

            // Normalise PID output to –1…+1 steering
            // Clamp to allow some motor differential without reversal on normal curves
            r.steeringOutput = constrain(pidOut / (KP_FOLLOW * 7.0f + 1.0f), -1.0f, 1.0f);

            driveMotors(r.targetSpeed, r.steeringOutput);
            break;
        }

        case BotState::CORNER_ENTER:
            // Slow approach: straight ahead at reduced speed
            driveMotors(SPEED_CORNER, 0.0f);
            break;

        case BotState::TURNING:
            // Hard pivot: one motor forward, other backward
            // lastTurnDir +1 = turn right: left motor fwd, right motor back
            {
                float pivotSpeed = SPEED_CORNER * 0.55f;
                float pivotSteering = (float)r.lastTurnDir * pivotSpeed * 1.8f;
                driveMotors(0.0f, constrain(pivotSteering, -1.0f, 1.0f));
            }
            break;

        case BotState::INTERSECTION:
            // Drive straight through crossing at moderate speed
            driveMotors(r.targetSpeed, 0.0f);
            break;

        case BotState::LINE_LOSS: {
            // Search: spin slowly toward last known turn direction.
            // If no prior direction, oscillate (implemented as spin toward
            // whichever side the last good position was on).
            float searchDir = (r.lastGoodPosition >= 0.0f) ? 1.0f : -1.0f;
            if (r.lastTurnDir != 0) searchDir = (float)r.lastTurnDir;
            driveMotors(RECOVERY_SPIN_SPEED * 0.3f,
                        searchDir * RECOVERY_SPIN_SPEED);
            break;
        }

        case BotState::STOP_PATCH:
        case BotState::STOPPED:
        default:
            hardBrake();
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  DEBUG SERIAL OUTPUT
// ─────────────────────────────────────────────────────────────────────────────

static const char* modeNames[] = {
    "FOLLOW","CORNER_ENTER","TURNING","INTERSECTION",
    "STOP_PATCH","LINE_LOSS","STOPPED"
};

void debugPrint(const RobotState &r) {
    const SensorFrame &sf = r.sf;

    Serial.print(F("MODE:"));
    Serial.print(modeNames[(uint8_t)r.mode]);
    Serial.print(F(" POS:"));
    Serial.print(sf.position, 3);
    Serial.print(F(" BAL:"));
    Serial.print(sf.balance, 3);
    Serial.print(F(" CONF:"));
    Serial.print(sf.confidence, 2);
    Serial.print(F(" ACT:"));
    Serial.print(sf.activeCount);
    Serial.print(F(" CRV:"));
    Serial.print(r.curvature, 2);
    Serial.print(F(" SPD:"));
    Serial.print(r.targetSpeed, 2);
    Serial.print(F(" STR:"));
    Serial.print(r.steeringOutput, 3);
    Serial.print(F(" PID_D:"));
    Serial.print(r.pid.derivative, 1);
    Serial.print(F(" FLAGS:"));
    if (sf.sharpLeft)      Serial.print(F("SL "));
    if (sf.sharpRight)     Serial.print(F("SR "));
    if (sf.allBlack)       Serial.print(F("AB "));
    if (sf.allWhite)       Serial.print(F("AW "));
    if (sf.tJunctionLeft)  Serial.print(F("TL "));
    if (sf.tJunctionRight) Serial.print(F("TR "));
    if (sf.crossroad)      Serial.print(F("CR "));
    if (sf.lineLost)       Serial.print(F("LL "));
    Serial.print(F(" MASK:0x"));
    Serial.print(sf.binaryMask, HEX);
    Serial.print(F(" NORM:"));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sf.norm[i]);
        if (i < NUM_SENSORS-1) Serial.print(',');
    }
    Serial.print(F(" LOOP:"));
    Serial.println(r.loopCount);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(500000);  // 500 kbaud — fast enough not to slow the loop

    // ADC config for Teensy 4.1
    analogReadResolution(ADC_BITS);
    analogReadAveraging(ADC_AVERAGING);

    motorSetup();

    // Zero robot state
    memset(&R, 0, sizeof(RobotState));
    R.mode         = BotState::FOLLOW;
    R.prevMode     = BotState::FOLLOW;
    R.lastTurnDir  = 0;
    R.targetSpeed  = 0.0f;
    R.pid.lastTimeUs = micros();

    // Pre-fill position history so curvature starts sane
    for (uint8_t i = 0; i < RobotState::CRV_BUF; i++)
        R.posHistory[i] = 0.0f;

    Serial.println(F("=== Competition Line Follower READY ==="));
    delay(500);  // brief pause before start
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN LOOP
//  Sequence: sense → estimate → pattern → govern → execute → (debug)
//  Target loop time on Teensy 4.1: well under 1 ms
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
    uint32_t nowUs = micros();
    float dt = (float)(nowUs - R.pid.lastTimeUs) * 1e-6f;
    dt = constrain(dt, 0.0001f, 0.02f);  // guard against timer wrap or pause
    R.pid.lastTimeUs = nowUs;

    // ── Layer 0 & 1: Sense + Estimate ───────────────────────────────────────
    readSensors(R.sf);
    computeMetrics(R.sf);
    analyzePatterns(R.sf);
    updateCurvature(R, dt);

    // ── Layer 4: Governor decides state ─────────────────────────────────────
    updateGovernor(R);

    // ── Speed target (decoupled from steering) ───────────────────────────────
    R.targetSpeed = computeTargetSpeed(R);

    // ── Layer 3 + 2: Execute (PID inside for FOLLOW, overrides for others) ──
    executeMotion(R, dt);

    // ── Debug output (throttled) ─────────────────────────────────────────────
    R.loopCount++;
    uint32_t nowMs = millis();
    if (nowMs - R.lastDebugMs >= DEBUG_INTERVAL_MS) {
        R.lastDebugMs = nowMs;
        debugPrint(R);
    }
}
