# ESP32-S3 LoRa Transmitter

## Overview
This project is an ESP32-S3-based LoRa transmitter for the xiao esp32s3 & wio-sx1262 that collects battery voltage and temperature data using an ADC and an NTC sensor, then transmits the data using an SX1262 LoRa module.

## Features
- Reads battery voltage via ADC
- Measures temperature using an NTC sensor
- Sends data over LoRa using the SX1262 module
- Enabled deep sleep to send data every 15 min.

## Hardware Requirements
- ESP32-S3 Development Board
- SX1262 LoRa Module
- NTC Thermistor (10kΩ)
- Battery (for ADC measurement)
- Supporting circuitry (resistors, connectors)

## Pin Configuration
| Signal | ESP32-S3 Pin |
|--------|-------------|
| MOSI | 9 |  # SPI MOSI Pin
| MISO | 8 |  # SPI MISO Pin
| SCK | 7 |  # SPI Clock Pin
| NSS | 41 |  # LoRa Chip Select Pin
| RST | 42 |  # LoRa Reset Pin
| BUSY | 40 |  # LoRa Busy Pin
| ANT SW | 38 |  # Antenna Switch Pin
| Battery ADC | 1 |  # ADC Pin for Battery Voltage Measurement
| Temp ADC | 2 |  # ADC Pin for Temperature Measurement

## Software Requirements
- ESP-IDF  # Required development framework
- FreeRTOS  # Real-time operating system for ESP32
- SX1262 LoRa Library  # LoRa module driver library

## Installation
1. Clone the repository.
2. [Set up ESP-IDF environment.](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#setting-up-development-environment)
3. Build and flash the firmware:
   ```sh
   idf.py build  # Compiles the firmware
   idf.py flash monitor  # Flashes the firmware and starts monitoring
   ```

## Usage
- The system initializes the ADC and LoRa module.
- It periodically reads the temperature and battery voltage.
- The collected data is transmitted over LoRa.
- Sleeps for 15 mins
- Logs are displayed via the serial monitor.

## License
This project is licensed under the MIT License.

