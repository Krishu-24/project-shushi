# Noise Plotter / Sensor Noisemap

This folder contains the sensor characterization tool used to measure raw ADC noise and separation between white and black surfaces for the 14-sensor line follower array.

It has two parts:

- `sensor_noisemap_app.py` — PyQt5 desktop app for recording, viewing, and exporting sensor statistics
- `sensor_noisemap_adc.ino` — Teensy 4.1 firmware that streams 14 sensor readings over serial

The goal of this tool is to help check:
- sensor-to-sensor consistency
- raw noise levels
- white vs black separation
- whether thresholding is safe while stationary and moving

---

## Folder contents

### `sensor_noisemap_app.py`
Desktop application for:
- connecting to the Teensy over serial
- viewing live recent mean and standard deviation
- recording labeled runs
- exporting a PDF report, JSON summary, and CSV raw data

### `sensor_noisemap_adc.ino`
Teensy firmware that:
- reads 14 reflectance sensors
- optionally applies light EMA smoothing
- streams one comma-separated frame per line at a fixed rate

---

## Exact settings that must match

For this setup, these values should match between the Python app and the Teensy code:

- `PORT = "COM6"`
- `BAUD = 115200`
- `NUM_SENSORS = 14`
- `TIMER_MS = 5`

Teensy-side important values:
- `BAUD = 115200`
- `NUM_SENSORS = 14`
- `STREAM_HZ = 200` → matches 5 ms period
- `ADC_BITS = 12`
- `ADC_AVG = 4`

Since `TIMER_MS = 5`, the app updates every 5 ms, which aligns well with:
- `STREAM_HZ = 200`
- `LOOP_PERIOD_US = 1000000 / 200 = 5000 us`

---

## Output format requirement

The Python app expects **only** raw numeric frames in this exact format:

```text
123,118,121,140,220,390,415,402,388,210,155,132,120,119