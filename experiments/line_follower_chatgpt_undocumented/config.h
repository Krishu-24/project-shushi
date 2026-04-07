#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace Config {
  constexpr uint8_t SENSOR_COUNT = 14;

  constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {
    21, 20, 19, 18, 17, 16, 15, 14, 13, 41, 40, 39, 38, 37
  };

  constexpr float SENSOR_WEIGHTS[SENSOR_COUNT] = {
    -6.5f, -5.5f, -4.5f, -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f,  4.5f,  5.5f,  6.5f
  };

  constexpr int ADC_MAX = 4095;
  constexpr int ANALOG_ON_THRESHOLD = 600;
  constexpr int EDGE_ON_THRESHOLD = 750;
  constexpr float LOST_LINE_MIN_SUM = 0.10f;

  constexpr float KP = 22.0f;
  constexpr float KI = 0.0f;
  constexpr float KD = 110.0f;
  constexpr float I_LIMIT = 120.0f;

  constexpr int PWM_MAX = 4095;
  constexpr int BASE_SPEED = 2200;
  constexpr int MIN_FOLLOW_SPEED = 0;
  constexpr int MAX_FOLLOW_SPEED = 3600;
  constexpr int TURN_SPEED_INNER = 1200;
  constexpr int TURN_SPEED_OUTER = 2200;
  constexpr int SEARCH_SPEED = 1200;

  constexpr uint32_t LOOP_US = 1000;
  constexpr uint32_t TURN_BRAKE_DELAY_MS = 12;
  constexpr uint32_t BRAKE_PULSE_MS = 18;
  constexpr uint32_t INTERSECTION_STRAIGHT_MS = 50;
  constexpr uint32_t LOST_LINE_STOP_MS = 250;
  constexpr uint32_t BLACK_PATCH_STOP_MS = 2000;
  constexpr uint32_t TURN_TIMEOUT_MS = 450;

  constexpr uint8_t ALL_BLACK_MIN_COUNT = 13;
  constexpr uint8_t T_CROSS_MIN_COUNT = 10;
  constexpr uint8_t LOST_LINE_MAX_COUNT = 0;

  namespace MotorPins {
    constexpr uint8_t LEFT_PWM  = 2;
    constexpr uint8_t LEFT_IN1  = 3;
    constexpr uint8_t LEFT_IN2  = 4;
    constexpr uint8_t RIGHT_PWM = 5;
    constexpr uint8_t RIGHT_IN1 = 6;
    constexpr uint8_t RIGHT_IN2 = 7;
    constexpr uint8_t STBY_OR_EN = 8;
  }
}

#endif