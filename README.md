# Embedded IMU Attitude Estimation

### Author
Daniel Ludlow — UCLA Aerospace Engineering

---

## Overview
This project implements a real-time IMU attitude estimation algorithm on an Arduino Uno using the MPU-9250 sensor. It reads 9-DOF data (accelerometer, gyroscope, magnetometer), converts raw values to SI units, and fuses them with a complementary filter to estimate the board’s orientation in quaternions.

This serves as an embedded testbed for Guidance, Navigation, and Control (GNC) and Hardware-in-the-Loop (HIL) applications.

---

## Features
- I²C communication with MPU-9250 (0x68)
- Conversion of raw sensor data to SI units  
  (m/s², rad/s, µT)
- Bias calibration on startup
- Quaternion-based attitude estimation
- Complementary filter for drift correction
- Runs at 200 Hz loop rate

---

## Hardware Setup
| Component | Connection | Notes |
|------------|-------------|-------|
| VCC | 3.3 V | Power |
| GND | GND | Ground |
| SDA | A4 | I²C Data |
| SCL | A5 | I²C Clock |

> The MPU-9250 uses 3.3 V logic.  
> If your breakout board is not level-shifted, use a logic converter.

---

## Software Setup
1. Open the project in Arduino IDE.  
2. Connect the Arduino Uno via USB.  
3. Upload the `embedded_IMU.ino` file.  
4. Open the Serial Monitor at 115200 baud to view sensor data and quaternion outputs.

---

## How It Works
- The accelerometer and gyroscope data are fused with a complementary filter:  
  gyro for fast response, accel/mag for long-term stability.  
- The result is a stable real-time quaternion estimate of orientation.  
- The system runs at 200 Hz (5 ms delay per loop).

---

## Future Improvements
- Add magnetometer yaw fusion  
- Implement Madgwick/Mahony filters  
- Use attitude data to drive LED “thruster” outputs for control simulation

---
