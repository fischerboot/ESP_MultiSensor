/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
#include <EspMultiLogger.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <time.h>
#include <EEPROM.h>

/*
Configuration
*/
const char* versionStr = "20251222v1.11";

#define LED 2

// char mqtt_server[40] = "10.0.0.13";
char mqtt_server[40] = "192.168.2.127";
char mqtt_port[6] = "1883";

#define MQTT_USER "yourUser"
#define MQTT_PASS "yourPass"

char MQTT_TOPIC_PIR[60];
char MQTT_TOPIC_BME_PRESSUR[60];
char MQTT_TOPIC_BME_temp[60];
char MQTT_TOPIC_BMP_PRESSUR[60];
char MQTT_TOPIC_BMP_temp[60];
char MQTT_TOPIC_BME_hum[60];
char MQTT_TOPIC_TEMP[60];
char MQTT_TOPIC_PRESSURE[60];

#define PIR_PIN 12 // D6/GPIO2 PIR sensor pin
Adafruit_BME280 bme; // I2C (changed from bme to bmp)
Adafruit_BMP280 bmp; // I2C (changed from bme to bmp)
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Sensor detection
enum SensorType { SENSOR_UNKNOWN = 0, SENSOR_BME280, SENSOR_BMP280 };
SensorType detectedSensor = SENSOR_UNKNOWN;
#define SENSOR_IS_BME() (detectedSensor == SENSOR_BME280)
#define SENSOR_IS_BMP() (detectedSensor == SENSOR_BMP280)

unsigned long lastSensorCheck = 0;
const long Sensor_read_Interval = 500; // ms

// Thresholds for publishing (adjust as needed)
const float TEMP_THRESHOLD = 0.5;     // °C
const float PRESSURE_THRESHOLD = 1.0; // hPa
const float HUM_THRESHOLD = 2.0;      // %
const float myAltitude = 138.4; // Example: 350 meters above sea level

// Last published values
float lastTemp = NAN;
float lastHum = NAN;
float lastPressure = NAN;

EspMultiLogger* InfoLogger;
EspMultiLogger* DebugLogger;
bool on = true;
int pirState =0;
#define DEFAULT_DEVICE_PREFIX "ESP_Default"
char device_prefix[16] = DEFAULT_DEVICE_PREFIX ; // Default prefix

// Persistent config stored in EEPROM
#define CONFIG_MAGIC "CFG1"
struct DeviceConfig {
  char magic[4];
  char device_prefix[16];
  char mqtt_server[40];
  char mqtt_port[6];
};

void loadConfig() {
  EEPROM.begin(512);
  DeviceConfig cfg;
  EEPROM.get(0, cfg);
  if (memcmp(cfg.magic, CONFIG_MAGIC, 4) == 0) {
    // copy values into globals, ensure NUL termination
    memcpy(device_prefix, cfg.device_prefix, sizeof(device_prefix));
    device_prefix[sizeof(device_prefix)-1] = '\0';
    memcpy(mqtt_server, cfg.mqtt_server, sizeof(mqtt_server));
    mqtt_server[sizeof(mqtt_server)-1] = '\0';
    memcpy(mqtt_port, cfg.mqtt_port, sizeof(mqtt_port));
    mqtt_port[sizeof(mqtt_port)-1] = '\0';
    Serial.println("Loaded config from EEPROM");
  } else {
    Serial.println("No valid config in EEPROM, using defaults");
  }
}

void saveConfig() {
  DeviceConfig cfg;
  memcpy(cfg.magic, CONFIG_MAGIC, 4);
  memset(cfg.device_prefix, 0, sizeof(cfg.device_prefix));
  memset(cfg.mqtt_server, 0, sizeof(cfg.mqtt_server));
  memset(cfg.mqtt_port, 0, sizeof(cfg.mqtt_port));
  strncpy(cfg.device_prefix, device_prefix, sizeof(cfg.device_prefix)-1);
  strncpy(cfg.mqtt_server, mqtt_server, sizeof(cfg.mqtt_server)-1);
  strncpy(cfg.mqtt_port, mqtt_port, sizeof(cfg.mqtt_port)-1);
  EEPROM.put(0, cfg);
  EEPROM.commit();
  Serial.println("Saved config to EEPROM");
}

// NTP configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;        // GMT+1 for Germany (adjust for your timezone)
const int daylightOffset_sec = 3600;    // Daylight saving time offset

void setupTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  InfoLogger->println("Waiting for NTP time sync...");
  
  time_t now = time(nullptr);
  int attempts = 0;
  while (now < 8 * 3600 * 2 && attempts < 20) { // Wait until we get a valid time
    delay(1000);
    InfoLogger->print(".");
    now = time(nullptr);
    attempts++;
  }
  
  if (now > 8 * 3600 * 2) {
    InfoLogger->println("\nTime synchronized!");
    InfoLogger->printf("Current time: %s", ctime(&now));
  } else {
    InfoLogger->println("\nFailed to sync time, using millis() fallback");
  }
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    InfoLogger->println("Attempting MQTT connection...");
    char mqttClientName[32];
    snprintf(mqttClientName, sizeof(mqttClientName), "%s_MultiSensor", device_prefix);
    if (mqttClient.connect(mqttClientName)) {
      InfoLogger->println("MQTT connected");
    } else {
      InfoLogger->printf("MQTT failed, rc=%d\n", mqttClient.state());
      delay(5000);
    }
  }
}

void myTelnetWelcome(WiFiClient& client) {
    client.println("=== ESP Multi Sensor Configuration ===\r\n");
    
    // Show current time and uptime
    time_t now = time(nullptr);
    if (now > 8 * 3600 * 2) {
        client.print("Current time: "); client.print(ctime(&now));
    } else {
        client.print("NTP time not synced\r\n");
    }
    
    unsigned long ms = millis();
    unsigned long seconds = ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24;
    client.print("\rUptime: ");
    client.print(days); client.print("d ");
    client.print(hours % 24); client.print("h ");
    client.print(minutes % 60); client.print("m ");
    client.print(seconds % 60); client.print("s\r\n");
    
    client.print("Firmware Version: "); client.print(versionStr); client.print("\r\n");
    client.print("Device Prefix: "); client.print(device_prefix); client.print("\r\n");
    client.print("MQTT Server: "); client.print(mqtt_server); client.print("\r\n");
    client.print("MQTT Port: "); client.print(mqtt_port); client.print("\r\n");
    client.print("MQTT Client Name: "); 
    char mqttClientName[32];
    snprintf(mqttClientName, sizeof(mqttClientName), "%s_MultiSensor", device_prefix);
    client.print(mqttClientName); client.print("\r\n");
    client.print("MQTT Topics:\r\n  PIR: "); client.print(MQTT_TOPIC_PIR); client.print("\r\n");
    client.print("  Temp: "); client.print(MQTT_TOPIC_TEMP); client.print("\r\n");
    client.print("  Pressure: "); client.print(MQTT_TOPIC_PRESSURE); client.print("\r\n");
    if (detectedSensor == SENSOR_BME280) {
      client.print("  Hum: "); client.print(MQTT_TOPIC_BME_hum); client.print("\r\n");
    } else {
      client.print("  Hum: N/A\r\n");
    }
    client.print("PIR Pin: "); client.print(PIR_PIN); client.print("\r\n");
    client.print("Altitude (m): "); client.print(myAltitude); client.print("\r\n");
    client.print("Sensor thresholds:\r\n  Temp: "); client.print(TEMP_THRESHOLD); client.print(" °C\r\n  Pressure: "); client.print(PRESSURE_THRESHOLD); client.print(" hPa\r\n");
    client.print("======================================\r\n");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n Starting Version:");
  Serial.println(versionStr);
  pinMode(PIR_PIN, INPUT);

  // Load persisted config (if any) before creating WiFiManager parameters
  loadConfig();

  // W-Lan Activating
  WiFi.mode(WIFI_STA);
  WiFiManager wifiManager;

  // --- Add custom parameters for MQTT and device prefix ---
  WiFiManagerParameter custom_device_prefix("prefix", "Device Prefix", device_prefix, 16);
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port, 6);

  wifiManager.addParameter(&custom_device_prefix);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  // --- Read updated config from WiFiManager ---
  // Use prefix as part of AP name
  char apName[32];
  snprintf(apName, sizeof(apName), "%s_AccessPoint", device_prefix);

  // wifiManager.setConfigPortalTimeout(60); // timeout in seconds (e.g., 60 seconds = 1 minute)
  // wifiManager.startConfigPortal(apName);

  if (!wifiManager.autoConnect(apName)) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }

  Serial.println("WiFi connected: " + WiFi.localIP().toString());

  // --- Read updated config from WiFiManager ---
  strcpy(device_prefix, custom_device_prefix.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(device_prefix, custom_device_prefix.getValue());
  if (device_prefix[0] == '\0') {
    strcpy(device_prefix, DEFAULT_DEVICE_PREFIX); // fallback to default if empty
  }

  // Persist updated configuration
  saveConfig();

  // Use prefix in logger initialisation
  InfoLogger = new EspMultiLogger(Info);
  DebugLogger = new EspMultiLogger(Debug);
  EspMultiLogger::setLogLevel(Debug);
  EspMultiLogger::initLogger();

  // Combine version string and device prefix for logger
  //int lenOfVersionWithPrefix = sizeof(versionStr) + sizeof(device_prefix) + 4; // 4 for " @ "
  char versionWithPrefix[48];
  snprintf(versionWithPrefix, sizeof(versionWithPrefix), "%s%s", versionStr, device_prefix);
  EspMultiLogger::setUserVersionString(versionWithPrefix);

  // OTA setup
  char otaHostname[32];
  snprintf(otaHostname, sizeof(otaHostname), "%s_Generic", device_prefix);
  ArduinoOTA.setHostname(otaHostname);
  // Setup NTP time synchronization
  setupTime();
  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    InfoLogger->println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    InfoLogger->println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    InfoLogger->printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    InfoLogger->printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      InfoLogger->println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      InfoLogger->println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      InfoLogger->println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      InfoLogger->println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      InfoLogger->println("End Failed");
    }
  });
  ArduinoOTA.begin();
  // BME/BMP280 init (try both common I2C addresses)
  bool bmeFound = bme.begin(0x76) || bme.begin(0x77);
  if (bmeFound) {
    detectedSensor = SENSOR_BME280;
    InfoLogger->println("Detected BME280 (humidity available)");
  } else {
    bool bmpFound = bmp.begin(0x76) || bmp.begin(0x77);
    if (bmpFound) {
      detectedSensor = SENSOR_BMP280;
      InfoLogger->println("Detected BMP280 (no humidity)");
    } else {
      detectedSensor = SENSOR_UNKNOWN;
      InfoLogger->println("Could not find BME/BMP280 sensor!");
    }
  }

  // --- MQTT setup with dynamic server/port and client name ---
  char mqttClientName[32];
  snprintf(mqttClientName, sizeof(mqttClientName), "%s_MultiSensor", device_prefix);
  mqttClient.setServer(mqtt_server, atoi(mqtt_port));
  // Store client name for use in mqttReconnect
  mqttClient.setBufferSize(256); // Optional: increase if needed

  // Set MQTT topics based on device prefix (generic topics for temp/pressure)
  snprintf(MQTT_TOPIC_PIR, sizeof(MQTT_TOPIC_PIR), "esp/sensor/pir/%s", device_prefix);
  snprintf(MQTT_TOPIC_TEMP, sizeof(MQTT_TOPIC_TEMP), "esp/sensor/temperature/%s", device_prefix);
  snprintf(MQTT_TOPIC_PRESSURE, sizeof(MQTT_TOPIC_PRESSURE), "esp/sensor/pressure/%s", device_prefix);
  // keep BME humidity topic (only published when BME present)
  snprintf(MQTT_TOPIC_BME_hum, sizeof(MQTT_TOPIC_BME_hum), "esp/sensor/humidity/%s", device_prefix);

  EspMultiLogger::setTelnetWelcomeCallback(myTelnetWelcome);
}

void loop() {
  // Begin Generic loop
  ArduinoOTA.handle();
  EspMultiLogger::loopLogger();

  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();
  unsigned long now = millis();
  if (now - lastSensorCheck > Sensor_read_Interval) {
    lastSensorCheck = now;
    int newPIRState = digitalRead(PIR_PIN);
    if(newPIRState != pirState) {
      InfoLogger->printf("PIR state changed from %d to %d\n", pirState, newPIRState);
      pirState = newPIRState;
      mqttClient.publish(MQTT_TOPIC_PIR, pirState ? "1" : "0");
    }

    // Read current sensor values from detected sensor
    float temp = NAN;
    float pressure = NAN;
    float hum = NAN;
    float seaLevelPressure = NAN;

    if (SENSOR_IS_BME()) {
      temp = bme.readTemperature();
      pressure = bme.readPressure() / 100.0F;
      hum = bme.readHumidity();
      seaLevelPressure = bme.seaLevelForAltitude(myAltitude, pressure);
    } else if (SENSOR_IS_BMP()) {
      temp = bmp.readTemperature();
      pressure = bmp.readPressure() / 100.0F;
      seaLevelPressure = bmp.seaLevelForAltitude(myAltitude, pressure);
    }

    // use generic topics for temp/pressure regardless of sensor
    const char* topicTemp = MQTT_TOPIC_TEMP;
    const char* topicPressure = MQTT_TOPIC_PRESSURE;

    // Only publish if value changed beyond threshold
    if (!isnan(temp) && (isnan(lastTemp) || fabs(temp - lastTemp) > TEMP_THRESHOLD)) {
      char payload[16];
      snprintf(payload, sizeof(payload), "%.2f", temp);
      mqttClient.publish(topicTemp, payload);
      lastTemp = temp;
      
      InfoLogger->printf("Temp published: %.2f\n", temp);
    }
    // Humidity
    if (SENSOR_IS_BME()) {
      if (!isnan(hum) && (isnan(lastHum) || fabs(hum - lastHum) > HUM_THRESHOLD)) {
        char payload[16];
        snprintf(payload, sizeof(payload), "%.2f", hum);
        mqttClient.publish(MQTT_TOPIC_BME_hum, payload);
        lastHum = hum;
        InfoLogger->printf("Hum published: %.2f\n", hum);
      }
    }
    // Pressure
    if (!isnan(seaLevelPressure) && (isnan(lastPressure) || fabs(seaLevelPressure - lastPressure) > PRESSURE_THRESHOLD)) {
      char payload[16];
      snprintf(payload, sizeof(payload), "%.2f", seaLevelPressure);
      mqttClient.publish(topicPressure, payload);
      lastPressure = seaLevelPressure;
      InfoLogger->printf("Pressure published: %.2f\n", seaLevelPressure);
    }
  }

}

