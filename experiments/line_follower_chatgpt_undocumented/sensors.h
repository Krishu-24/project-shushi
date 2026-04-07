#ifndef SENSORS_H
#define SENSORS_H

#include "types.h"

void setupSensors();
void updateSensors(ControllerContext &ctx);
float computeLinePosition(const SensorFrame &f, TurnDir lastKnownLineSide);
float computeThicknessBalance(const SensorFrame &f);

#endif