# Hardware Wiring Guide

This document outlines the pinout and wiring connections between the ESP32 microcontroller and the various environmental sensors, navigation modules, and motor drivers used in the system.

## 🌡️ Environmental Sensors

| Component | Sensor Pin | ESP32 Pin / Connection | Notes |
| :--- | :--- | :--- | :--- |
| **DHT11/22** | VCC | 3.3V | |
| | GND | GND | |
| | DATA | **GPIO 4** | Add a 10kΩ pull-up resistor between DATA and 3.3V if your sensor isn't on a breakout board. |
| **MQ-2 Gas** | VCC | 5V (VIN/VUSB) | Needs 5V for the heating element. |
| | GND | GND | |
| | AOUT | **GPIO 34 (ADC)** | **⚠️ Recommended:** Use a voltage divider to step 5V down to 3.3V. |
| **Optical Dust (Sharp)**| V-LED | 5V | Add a 150Ω resistor in series and a 220µF capacitor to GND as per the datasheet. |
| | LED-GND | GND | |
| | LED-IN | **GPIO 5** | This pin pulses the IR LED to take a reading. |
| | S-VCC | 5V | |
| | S-GND | GND | |
| | S-OUT | **GPIO 35 (ADC)** | **⚠️ Recommended:** Use a voltage divider to step 5V down to 3.3V. |

---

## 🧭 Navigation, IMU, Power & Motors

| Component | Pin / Function | ESP32 Connection | Notes |
| :--- | :--- | :--- | :--- |
| **EVE L89 (GNSS)** | TX | **GPIO 16 (RX2)** | We use Hardware Serial 2 for reliable GPS parsing. |
| | RX | **GPIO 17 (TX2)** | |
| | VCC | 3.3V | |
| | GND | GND | |
| **MPU6050 (IMU)** | SDA | **GPIO 21** | Standard I2C Data. |
| | SCL | **GPIO 22** | Standard I2C Clock. |
| **Magnetometer** | SDA | **GPIO 21** | Connect in parallel with the MPU6050 (I2C Bus). |
| | SCL | **GPIO 22** | Connect in parallel with the MPU6050 (I2C Bus). |
| **INA3221 (Power)** | SDA | **GPIO 21** | Connect in parallel with the MPU6050 (I2C Bus). |
| | SCL | **GPIO 22** | Connect in parallel with the MPU6050 (I2C Bus). |
| **Motor Driver** | IN1 (Left) | **GPIO 25** | Standard dual H-bridge (like L298N or TB6612). |
| | IN2 (Left) | **GPIO 26** | |
| | IN3 (Right)| **GPIO 27** | |
| | IN4 (Right)| **GPIO 14** | |

---

## ⚠️ Important Hardware Notes

### 1. I2C Bus Configuration
The MPU6050, Magnetometer, and INA3221 all share the same I2C bus:
* **SDA:** GPIO 21
* **SCL:** GPIO 22
> **Note:** Ensure each device has a unique I2C address to avoid bus conflicts.

### 2. Logic Level Protection (5V to 3.3V)
The ESP32 uses strict 3.3V logic. **Do not connect 5V output pins directly to ESP32 GPIOs.** As noted in the tables, use voltage dividers for 5V analog outputs (like the MQ-2 Gas Sensor and Sharp Optical Dust Sensor) to protect the ESP32's ADC pins (GPIO 34 and GPIO 35).

### 3. Power Distribution
Sensors requiring 5V (MQ-2, Sharp Optical Dust) must be powered from the `VIN`/`VUSB` pin, assuming the ESP32 is powered via USB or a stable 5V regulator. Be cautious of the total current draw, especially when the MQ-2 heater is active.
