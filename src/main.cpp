/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
#include <EspMultiLogger.h>
#include <ArduinoOTA.h>
#include "WlanConfig.h"
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

/*
Configuration
*/
const char* versionStr = "20250920v1.1";

#define LED 2

#ifndef WlanConfig_h
#define STASSID "------------"
#define STAPSK  "------------"
#endif
const char* ssid = STASSID;
const char* password = STAPSK;

#define MQTT_SERVER "192.168.2.127"
#define MQTT_PORT 1883
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

unsigned long lastPublish = 0;
unsigned long lastPIRCheck = 0;
const long publishInterval = 10000; // ms
const long PIR_read_Interval = 500; // ms

EspMultiLogger* InfoLogger;
EspMultiLogger* DebugLogger;
bool on = true;
int pirState =0;

void mqttReconnect() {
  while (!mqttClient.connected()) {
    InfoLogger->println("Attempting MQTT connection...");
    if (mqttClient.connect("ESP_MultiSensor")) {
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
  //pinMode(LED, OUTPUT);
  pinMode(PIR_PIN, INPUT);

  // W-Lan Activating
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.print("local ip: ");
  Serial.println(WiFi.localIP());

  InfoLogger = new EspMultiLogger(Info);
  DebugLogger = new EspMultiLogger(Debug);
  EspMultiLogger::setLogLevel(Debug);
  EspMultiLogger::initLogger();
  EspMultiLogger::setUserVersionString(versionStr);

  // OTA Begin
    // OTA (only after connection is established)
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("fischerboot_Generic");

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
  //OTA End

  // BME280 init
  if (!bme.begin(0x76)) {
    InfoLogger->println("Could not find BME280 sensor!");
  }

  // MQTT setup
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
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
  if (now - lastPIRCheck > PIR_read_Interval) {
    lastPIRCheck = now;
    int newPIRState = digitalRead(PIR_PIN);
    if(newPIRState != pirState) {
      InfoLogger->printf("PIR state changed from %d to %d\n", pirState, newPIRState);
      pirState = newPIRState;
      mqttClient.publish(MQTT_TOPIC_PIR, pirState ? "1" : "0");
      // if(pirState==0){
      //   digitalWrite(LED,LOW);
      //   InfoLogger->println("LED is on");
      //   InfoLogger->println(versionStr);
      // }else{
      //   on = true;
      //   digitalWrite(LED, HIGH);
      //   InfoLogger->println("LED is off");
      // }
    }
  }
  
  if (now - lastPublish > publishInterval) {
    lastPublish = now;
    // BME280 sensor
    char payload[128];
    snprintf(payload, sizeof(payload), "%.2f", bme.readPressure()/100.0F);
    char payloadhum[128];
    snprintf(payloadhum, sizeof(payload), "%.2f", bme.readHumidity());
    char payloadtemp[128];
    snprintf(payloadtemp, sizeof(payload), "%.2f", bme.readTemperature());
    mqttClient.publish(MQTT_TOPIC_BME_PRESSUR, payload);
    mqttClient.publish(MQTT_TOPIC_BME_hum, payloadhum);
    mqttClient.publish(MQTT_TOPIC_BME_temp, payloadtemp);
    InfoLogger->printf("Sensor data published to MQTT at %lu \n", now);
    InfoLogger->println(bme.readTemperature());
    InfoLogger->println(bme.readHumidity());
    InfoLogger->println(bme.readPressure()/100.0F);
  }

}

