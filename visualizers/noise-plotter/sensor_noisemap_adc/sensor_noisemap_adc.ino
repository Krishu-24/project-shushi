/*
  Teensy 4.1 — 14 Sensor Raw/Smoothed Streamer
  Compatible with your PyQt5 app
*/

#include <Arduino.h>

// --------- USER SETTINGS ----------
static const uint8_t NUM_SENSORS = 14;
static const uint8_t sensorPins[NUM_SENSORS] = {
  21, 20, 19, 18, 17, 16, 15,
  14, 13, 41, 40, 39, 38, 37
};

static const uint32_t BAUD = 115200;
static const uint8_t ADC_BITS = 12;
static const uint8_t ADC_AVG = 4;
static const uint32_t STREAM_HZ = 200;
static const uint16_t STARTUP_DELAY_MS = 1500;

// 0 = raw output
// 1 = simple EMA smoothed output
static const bool USE_EMA = false;

// EMA alpha: lower = smoother, slower
static const float EMA_ALPHA = 0.25f;
// ----------------------------------

static const uint32_t LOOP_PERIOD_US = 1000000UL / STREAM_HZ;

uint16_t rawVals[NUM_SENSORS];
float emaVals[NUM_SENSORS];
bool emaInitialized = false;

elapsedMicros loopTimer;

void readRawSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    rawVals[i] = analogRead(sensorPins[i]);
  }
}

void updateEMA() {
  if (!emaInitialized) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      emaVals[i] = rawVals[i];
    }
    emaInitialized = true;
    return;
  }

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    emaVals[i] = EMA_ALPHA * rawVals[i] + (1.0f - EMA_ALPHA) * emaVals[i];
  }
}

void sendFrame() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t out = USE_EMA ? (uint16_t)(emaVals[i] + 0.5f) : rawVals[i];
    Serial.print(out);
    if (i < NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();
}

void setup() {
  Serial.begin(BAUD);

  analogReadResolution(ADC_BITS);
  analogReadAveraging(ADC_AVG);

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  delay(STARTUP_DELAY_MS);
  loopTimer = 0;
}

void loop() {
  if (loopTimer >= LOOP_PERIOD_US) {
    loopTimer -= LOOP_PERIOD_US;

    readRawSensors();
    if (USE_EMA) updateEMA();
    sendFrame();
  }
}