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
#include <Adafruit_Sensor.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

/*
Configuration
*/
const char* versionStr = "20250927v1.4";

#define LED 2

char mqtt_server[40] = "192.168.2.127";
char mqtt_port[6] = "1883";

#define MQTT_USER "yourUser"
#define MQTT_PASS "yourPass"
#define MQTT_TOPIC_PIR "esp/sensor/pir"
#define MQTT_TOPIC_BME_PRESSUR "esp/sensor/bme280/pressure"
#define MQTT_TOPIC_BME_hum "esp/sensor/bme280/hum"
#define MQTT_TOPIC_BME_temp "esp/sensor/bme280/temp"


#define PIR_PIN 12 // D6/GPIO2 PIR sensor pin
Adafruit_BME280 bme; // I2C
WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastSensorCheck = 0;
const long Sensor_read_Interval = 500; // ms

// Thresholds for publishing (adjust as needed)
const float TEMP_THRESHOLD = 0.5;     // °C
const float HUM_THRESHOLD = 2.0;      // %
const float PRESSURE_THRESHOLD = 1.0; // hPa

// Last published values
float lastTemp = NAN;
float lastHum = NAN;
float lastPressure = NAN;

EspMultiLogger* InfoLogger;
EspMultiLogger* DebugLogger;
bool on = true;
int pirState =0;

char device_prefix[16] = "ESPMS"; // Default prefix

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n Starting Version:");
  Serial.println(versionStr);
  pinMode(PIR_PIN, INPUT);

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
    strcpy(device_prefix, "ESPMS"); // fallback to default if empty
  }

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

  // BME280 init
  if (!bme.begin(0x76)) {
    InfoLogger->println("Could not find BME280 sensor!");
  } else {
    // Replace with your actual altitude in meters
    float myAltitude = 138.4; // Example: 350 meters above sea level
    float measuredPressure = bme.readPressure() / 100.0F; // in hPa
    float seaLevelPressure = bme.seaLevelForAltitude(myAltitude, measuredPressure);
    InfoLogger->printf("Sea level pressure set to: %.2f hPa\n", seaLevelPressure);
  }

  // --- MQTT setup with dynamic server/port and client name ---
  char mqttClientName[32];
  snprintf(mqttClientName, sizeof(mqttClientName), "%s_MultiSensor", device_prefix);
  mqttClient.setServer(mqtt_server, atoi(mqtt_port));
  // Store client name for use in mqttReconnect
  mqttClient.setBufferSize(256); // Optional: increase if needed
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

    // Read current sensor values
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;

    // Only publish if value changed beyond threshold
    if (isnan(lastTemp) || fabs(temp - lastTemp) > TEMP_THRESHOLD) {
      char payload[16];
      snprintf(payload, sizeof(payload), "%.2f", temp);
      mqttClient.publish(MQTT_TOPIC_BME_temp, payload);
      lastTemp = temp;
      InfoLogger->printf("Temp published: %.2f\n", temp);
    }
    if (isnan(lastHum) || fabs(hum - lastHum) > HUM_THRESHOLD) {
      char payload[16];
      snprintf(payload, sizeof(payload), "%.2f", hum);
      mqttClient.publish(MQTT_TOPIC_BME_hum, payload);
      lastHum = hum;
      InfoLogger->printf("Hum published: %.2f\n", hum);
    }
    if (isnan(lastPressure) || fabs(pressure - lastPressure) > PRESSURE_THRESHOLD) {
      char payload[16];
      snprintf(payload, sizeof(payload), "%.2f", pressure);
      mqttClient.publish(MQTT_TOPIC_BME_PRESSUR, payload);
      lastPressure = pressure;
      InfoLogger->printf("Pressure published: %.2f\n", pressure);
    }
  }

}

