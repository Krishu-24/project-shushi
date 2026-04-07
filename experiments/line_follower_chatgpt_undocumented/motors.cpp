#include "motors.h"
#include "config.h"
#include <Arduino.h>

void setupMotors() {
  using namespace Config::MotorPins;

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(STBY_OR_EN, OUTPUT);

  digitalWrite(STBY_OR_EN, HIGH);

  analogWriteResolution(12);
  analogWriteFrequency(LEFT_PWM, 20000);
  analogWriteFrequency(RIGHT_PWM, 20000);
}

int clampPwm(int v) {
  return constrain(v, -Config::PWM_MAX, Config::PWM_MAX);
}

void motorDrive(int leftCmd, int rightCmd) {
  setMotorChannel(true, clampPwm(leftCmd));
  setMotorChannel(false, clampPwm(rightCmd));
}

void setMotorChannel(bool left, int cmd) {
  using namespace Config::MotorPins;

  uint8_t pwm = left ? LEFT_PWM : RIGHT_PWM;
  uint8_t in1 = left ? LEFT_IN1 : RIGHT_IN1;
  uint8_t in2 = left ? LEFT_IN2 : RIGHT_IN2;

  if (cmd > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, cmd);
  } else if (cmd < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -cmd);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}