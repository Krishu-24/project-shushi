#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include "config.h"

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
  int norm[Config::SENSOR_COUNT];
  uint8_t bin[Config::SENSOR_COUNT];
  uint8_t edgeBin[Config::SENSOR_COUNT];
  uint8_t activeCount = 0;
  uint16_t binaryMask = 0;
  float analogSum = 0.0f;
  float position = 0.0f;
  float leftMass = 0.0f;
  float rightMass = 0.0f;
  float thicknessBalance = 0.0f;
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

#endif