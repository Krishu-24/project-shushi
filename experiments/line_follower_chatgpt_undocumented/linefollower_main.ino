#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "motors.h"
#include "sensors.h"
#include "state_machine.h"

ControllerContext ctx;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogReadAveraging(4);

  setupMotors();
  setupSensors();
  initController(ctx);
}

void loop() {
  static uint32_t lastLoopUs = 0;
  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastLoopUs) < Config::LOOP_US) return;
  lastLoopUs = nowUs;

  updateSensors(ctx);
  updateStateMachine(ctx);
}