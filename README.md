# ESP Multi Sensor

This project is an ESP8266-based multi-sensor platform that integrates a PIR motion sensor and a BME280 environmental sensor. The device connects to WiFi and publishes sensor data to an MQTT broker, making it suitable for home automation, monitoring, or IoT applications.

## Features

- **WiFi Connectivity:** Connects to your local WiFi network using credentials from `WlanConfig.h`.
- **MQTT Integration:** Publishes sensor data (motion, temperature, humidity, pressure) to configurable MQTT topics.
- **PIR Sensor:** Detects motion and publishes state changes.
- **BME280 Sensor:** Measures temperature, humidity, and pressure.
- **Threshold-based Publishing:** Sensor data is only published if the value changes beyond a configurable threshold, reducing unnecessary MQTT traffic.
- **OTA Updates:** Supports over-the-air firmware updates via ArduinoOTA.
- **Logging:** Uses EspMultiLogger for structured logging and debugging.

## Usage

1. **Configure WiFi:** Set your WiFi credentials in `WlanConfig.h`.
2. **Configure MQTT:** Set your MQTT server address, port, and (optionally) authentication in `main.cpp`.
3. **Build & Upload:** Use PlatformIO to build and upload the firmware to your ESP8266 device.
4. **Monitor:** Subscribe to the configured MQTT topics to receive sensor updates.

## Todo

### MQTT integration

- [x] General usage
- [ ] Usage of authentication
- [ ] Usage of encryption

### PIR integration

- [x] make it happen

### BME280 integration

- [x] make it happen

### General stuff

- [ ] version automation
- [ ] check if firmware from github could be flashed directly so that releases make sense
- [ ] testing :D
