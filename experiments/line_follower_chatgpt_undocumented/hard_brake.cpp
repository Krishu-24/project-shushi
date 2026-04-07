#include "hard_brake.h"
#include "motors.h"
#include <Arduino.h>

void applyHardBrake(uint32_t brakeMs) {
  motorDrive(0, 0);
  delay(brakeMs);
}