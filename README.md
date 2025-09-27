# ESP Multi Sensor

This project is an ESP8266-based multi-sensor platform that integrates a PIR motion sensor and a BME280 environmental sensor. The device connects to WiFi and publishes sensor data to an MQTT broker, making it suitable for home automation, monitoring, or IoT applications.

## Features

- **WiFi Connectivity:** Uses [WiFiManager](https://github.com/tzapu/WiFiManager) to automatically create a hotspot if WiFi credentials are missing or invalid. Credentials are entered via a captive portal and stored on the ESP—no need to hardcode them.
- **MQTT Integration:** Publishes sensor data (motion, temperature, humidity, pressure) to configurable MQTT topics. MQTT authentication is supported if configured.
- **PIR Sensor:** Detects motion and publishes state changes to MQTT.
- **BME280 Sensor:** Measures temperature, humidity, and pressure.
- **Threshold-based Publishing:** Sensor data is only published if the value changes beyond a configurable threshold, reducing unnecessary MQTT traffic.
- **OTA Updates:** Supports over-the-air firmware updates via ArduinoOTA.
- **Logging:** Uses EspMultiLogger for structured logging and debugging.

## Usage

1. **WiFi Setup:**  
   On first boot or if WiFi credentials are missing, the ESP creates a hotspot (default: `ESP_AccessPoint`).  
   Connect to this WiFi with your phone or PC, open [http://192.168.4.1](http://192.168.4.1), and enter your WiFi credentials.  
   The ESP will store these and connect automatically on future boots.

2. **Configure MQTT:**  
   Set your MQTT server address, port, and (optionally) authentication in `main.cpp`.

3. **Build & Upload:**  
   Use PlatformIO to build and upload the firmware to your ESP8266 device.

4. **Monitor:**  
   Subscribe to the configured MQTT topics to receive sensor updates.

## Improvements

- WiFi credentials are now managed via a captive portal using WiFiManager, making setup easier and more secure.
- Sensor data is only published to MQTT if the value changes beyond a defined threshold.
- PIR sensor state changes are detected and published immediately.
- OTA and structured logging are integrated for easier maintenance and debugging.

## Todo

### MQTT integration

- [x] General usage
- [x] Usage of authentication
- [ ] Usage of encryption

### PIR integration

- [x] make it happen

### BME280 integration

- [x] make it happen

### General stuff

- [ ] version automation
- [ ] check if firmware from github could be flashed directly so that releases make sense
- [ ] testing :D
