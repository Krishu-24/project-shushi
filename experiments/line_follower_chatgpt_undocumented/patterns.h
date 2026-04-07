#ifndef PATTERNS_H
#define PATTERNS_H

#include "types.h"

TurnDir detectSharpTurnDirection(const SensorFrame &f);
TurnDir inferLostLineDirection(const SensorFrame &f, TurnDir lastKnownLineSide);
bool detectTJunctionOrCross(const SensorFrame &f);

#endif