/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
#include <EspMultiLogger.h>
#include <ArduinoOTA.h>
#include "WlanConfig.h"
/*
Configuration
*/
const char* versionStr = "20220105v0.4";

#define LED 2

#ifndef WlanConfig_h
#define STASSID "------------"
#define STAPSK  "------------"
#endif
const char* ssid = STASSID;
const char* password = STAPSK;

EspMultiLogger* InfoLogger;
EspMultiLogger* DebugLogger;
bool on = true;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n Starting Version:");
  Serial.println(versionStr);
  pinMode(LED, OUTPUT);
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
}

void loop() {
  // Begin Generic loop
  ArduinoOTA.handle();
  EspMultiLogger::loopLogger();
  // End Generic loop
  // put your main code here, to run repeatedly:
  if(millis()%1000==0){
    if(on == true){
      on = false;
      digitalWrite(LED, HIGH);
      InfoLogger->println("LED is on");
    }else{
      on = true;
      digitalWrite(LED, LOW);
      InfoLogger->println("LED is off");
    }
  }
}