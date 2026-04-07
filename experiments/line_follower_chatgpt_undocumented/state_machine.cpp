#include "state_machine.h"
#include "config.h"
#include "motors.h"
#include "patterns.h"
#include "pid_control.h"
#include "hard_brake.h"
#include <Arduino.h>

static void enterState(ControllerContext &ctx, RunState next) {
  ctx.state = next;
  ctx.stateEnteredMs = millis();
}

static void followLine(ControllerContext &ctx) {
  const SensorFrame &f = ctx.sensors;

  if (f.position < -0.3f) ctx.lastKnownLineSide = TurnDir::LEFT;
  else if (f.position > 0.3f) ctx.lastKnownLineSide = TurnDir::RIGHT;

  if (f.allBlack) {
    if (ctx.allBlackSinceMs == 0) ctx.allBlackSinceMs = millis();
    if (millis() - ctx.allBlackSinceMs >= Config::BLACK_PATCH_STOP_MS) {
      enterState(ctx, RunState::STOPPED);
      return;
    }
    enterState(ctx, RunState::HANDLE_INTERSECTION);
    return;
  } else {
    ctx.allBlackSinceMs = 0;
  }

  if (f.allWhite) {
    if (ctx.lostLineSinceMs == 0) ctx.lostLineSinceMs = millis();
    if (millis() - ctx.lostLineSinceMs >= Config::LOST_LINE_STOP_MS) {
      enterState(ctx, RunState::RECOVER_LINE);
      return;
    }
  } else {
    ctx.lostLineSinceMs = 0;
  }

  TurnDir sharp = detectSharpTurnDirection(f);
  if (sharp != TurnDir::NONE) {
    ctx.pendingTurn = sharp;
    enterState(ctx, RunState::PREPARE_TURN);
    return;
  }

  float pidOut = computePID(ctx);

  int left = (int)lroundf(Config::BASE_SPEED - pidOut);
  int right = (int)lroundf(Config::BASE_SPEED + pidOut);

  left = constrain(left, Config::MIN_FOLLOW_SPEED, Config::MAX_FOLLOW_SPEED);
  right = constrain(right, Config::MIN_FOLLOW_SPEED, Config::MAX_FOLLOW_SPEED);

  motorDrive(left, right);
}

static void prepareTurn(ControllerContext &ctx) {
  if (millis() - ctx.stateEnteredMs < Config::TURN_BRAKE_DELAY_MS) {
    followLine(ctx);
    return;
  }

  applyHardBrake(Config::BRAKE_PULSE_MS);
  enterState(ctx, RunState::EXECUTE_TURN);
}

static void executeTurn(ControllerContext &ctx) {
  const SensorFrame &f = ctx.sensors;
  uint32_t elapsed = millis() - ctx.stateEnteredMs;

  if (elapsed > Config::TURN_TIMEOUT_MS) {
    enterState(ctx, RunState::RECOVER_LINE);
    return;
  }

  if (ctx.pendingTurn == TurnDir::LEFT) {
    motorDrive(-Config::TURN_SPEED_INNER, Config::TURN_SPEED_OUTER);
  } else if (ctx.pendingTurn == TurnDir::RIGHT) {
    motorDrive(Config::TURN_SPEED_OUTER, -Config::TURN_SPEED_INNER);
  } else {
    enterState(ctx, RunState::FOLLOW_LINE);
    return;
  }

  if (f.centerSeen) {
    ctx.pendingTurn = TurnDir::NONE;
    ctx.pid.prevError = 0.0f;
    ctx.pid.integral = 0.0f;
    enterState(ctx, RunState::FOLLOW_LINE);
  }
}

static void handleIntersection(ControllerContext &ctx) {
  const SensorFrame &f = ctx.sensors;

  if (f.allBlack) {
    motorDrive(Config::BASE_SPEED, Config::BASE_SPEED);
    if (millis() - ctx.stateEnteredMs >= Config::INTERSECTION_STRAIGHT_MS) {
      enterState(ctx, RunState::FOLLOW_LINE);
    }
    return;
  }

  if (detectTJunctionOrCross(f)) {
    motorDrive(Config::BASE_SPEED, Config::BASE_SPEED);
    return;
  }

  enterState(ctx, RunState::FOLLOW_LINE);
}

static void recoverLine(ControllerContext &ctx) {
  TurnDir dir = inferLostLineDirection(ctx.sensors, ctx.lastKnownLineSide);

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
    enterState(ctx, RunState::FOLLOW_LINE);
  }
}

static void stopRobot() {
  motorDrive(0, 0);
}

void initController(ControllerContext &ctx) {
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    ctx.cal.white[i] = 3000;
    ctx.cal.black[i] = 800;
  }
  ctx.cal.valid = true;

  ctx.state = RunState::FOLLOW_LINE;
  ctx.pendingTurn = TurnDir::NONE;
  ctx.lastKnownLineSide = TurnDir::NONE;
  ctx.stateEnteredMs = millis();
  ctx.allBlackSinceMs = 0;
  ctx.lostLineSinceMs = 0;
}

void updateStateMachine(ControllerContext &ctx) {
  switch (ctx.state) {
    case RunState::FOLLOW_LINE:
      followLine(ctx);
      break;
    case RunState::PREPARE_TURN:
      prepareTurn(ctx);
      break;
    case RunState::EXECUTE_TURN:
      executeTurn(ctx);
      break;
    case RunState::HANDLE_INTERSECTION:
      handleIntersection(ctx);
      break;
    case RunState::RECOVER_LINE:
      recoverLine(ctx);
      break;
    case RunState::STOPPED:
      stopRobot();
      break;
  }
}