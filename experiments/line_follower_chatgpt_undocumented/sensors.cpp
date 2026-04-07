#include "sensors.h"
#include "config.h"
#include "normalization.h"
#include <Arduino.h>
#include <string.h>

void setupSensors() {
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    pinMode(Config::SENSOR_PINS[i], INPUT);
  }
}

float computeLinePosition(const SensorFrame &f, TurnDir lastKnownLineSide) {
  float weightedSum = 0.0f;
  float total = 0.0f;

  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    float a = f.norm[i] * 0.001f;
    weightedSum += a * Config::SENSOR_WEIGHTS[i];
    total += a;
  }

  if (total < 0.001f) {
    if (lastKnownLineSide == TurnDir::LEFT) return -7.0f;
    if (lastKnownLineSide == TurnDir::RIGHT) return 7.0f;
    return 0.0f;
  }

  return weightedSum / total;
}

float computeThicknessBalance(const SensorFrame &f) {
  return f.rightMass - f.leftMass;
}

void updateSensors(ControllerContext &ctx) {
  SensorFrame &f = ctx.sensors;
  memset(&f, 0, sizeof(f));

  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    f.raw[i] = analogRead(Config::SENSOR_PINS[i]);
    f.norm[i] = normalizeSensor(f.raw[i], ctx.cal.white[i], ctx.cal.black[i]);

    f.bin[i] = (f.norm[i] >= Config::ANALOG_ON_THRESHOLD) ? 1 : 0;
    f.edgeBin[i] = (f.norm[i] >= Config::EDGE_ON_THRESHOLD) ? 1 : 0;

    f.activeCount += f.bin[i];
    f.binaryMask = (uint16_t)((f.binaryMask << 1) | f.bin[i]);

    float a = f.norm[i] * 0.001f;
    f.analogSum += a;

    if (Config::SENSOR_WEIGHTS[i] < 0) f.leftMass += a;
    if (Config::SENSOR_WEIGHTS[i] > 0) f.rightMass += a;
  }

  f.position = computeLinePosition(f, ctx.lastKnownLineSide);
  f.thicknessBalance = computeThicknessBalance(f);

  f.allWhite = (f.activeCount <= Config::LOST_LINE_MAX_COUNT) || (f.analogSum < Config::LOST_LINE_MIN_SUM);
  f.allBlack = (f.activeCount >= Config::ALL_BLACK_MIN_COUNT);
  f.centerSeen = (f.bin[6] || f.bin[7]);

  bool leftOuter = f.edgeBin[0] || f.edgeBin[1] || f.edgeBin[2];
  bool rightOuter = f.edgeBin[11] || f.edgeBin[12] || f.edgeBin[13];
  bool centerWeak = !(f.edgeBin[5] || f.edgeBin[6] || f.edgeBin[7] || f.edgeBin[8]);

  f.leftEdgeHit = leftOuter && centerWeak;
  f.rightEdgeHit = rightOuter && centerWeak;
}