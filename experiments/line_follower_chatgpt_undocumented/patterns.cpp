#include "patterns.h"

TurnDir detectSharpTurnDirection(const SensorFrame &f) {
  if (f.leftEdgeHit && !f.rightEdgeHit) return TurnDir::LEFT;
  if (f.rightEdgeHit && !f.leftEdgeHit) return TurnDir::RIGHT;
  return TurnDir::NONE;
}

TurnDir inferLostLineDirection(const SensorFrame &f, TurnDir lastKnownLineSide) {
  if (f.leftMass > f.rightMass + 0.05f) return TurnDir::LEFT;
  if (f.rightMass > f.leftMass + 0.05f) return TurnDir::RIGHT;
  return lastKnownLineSide;
}

bool detectTJunctionOrCross(const SensorFrame &f) {
  bool left = f.bin[0] || f.bin[1] || f.bin[2] || f.bin[3];
  bool center = f.bin[5] || f.bin[6] || f.bin[7] || f.bin[8];
  bool right = f.bin[10] || f.bin[11] || f.bin[12] || f.bin[13];

  return (left && center && right) || (f.activeCount >= 10);
}