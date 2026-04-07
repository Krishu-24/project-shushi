#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

void setupMotors();
void motorDrive(int leftCmd, int rightCmd);
void setMotorChannel(bool left, int cmd);
int clampPwm(int v);

#endif