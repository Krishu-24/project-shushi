#include "pid_control.h"
#include "config.h"
#include <Arduino.h>

float computePID(ControllerContext &ctx) {
  const SensorFrame &f = ctx.sensors;

  ctx.pid.error = f.position + 0.35f * f.thicknessBalance;

  ctx.pid.integral += ctx.pid.error;
  if (ctx.pid.integral > Config::I_LIMIT) ctx.pid.integral = Config::I_LIMIT;
  if (ctx.pid.integral < -Config::I_LIMIT) ctx.pid.integral = -Config::I_LIMIT;

  ctx.pid.derivative = ctx.pid.error - ctx.pid.prevError;

  ctx.pid.output =
      Config::KP * ctx.pid.error +
      Config::KI * ctx.pid.integral +
      Config::KD * ctx.pid.derivative;

  ctx.pid.prevError = ctx.pid.error;
  return ctx.pid.output;
}