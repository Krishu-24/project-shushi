#include "normalization.h"
#include <Arduino.h>

int normalizeSensor(int rawValue, int whiteValue, int blackValue) {
  int denom = whiteValue - blackValue;
  if (abs(denom) < 20) return 0;

  float n = (float)(whiteValue - rawValue) / (float)denom;
  int out = (int)lroundf(n * 1000.0f);

  if (out < 0) out = 0;
  if (out > 1000) out = 1000;
  return out;
}