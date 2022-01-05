/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
#include <EspMultiLogger.h>
#include "WlanConfig.h"
/*
Configuration
*/
const char* versionStr = "20220105v0.3";

#define LED 2

#ifndef WlanConfig_h
#define STASSID "------------"
#define STAPSK  "------------"
#endif
const char* ssid = STASSID;
const char* password = STAPSK;

EspMultiLogger* Logger;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n Starting Version:");
  Serial.println(versionStr);
  pinMode(LED, OUTPUT);

  Logger = new EspMultiLogger(Debug);
  Logger->setLogLevel(Debug);
  EspMultiLogger::initLogger();
}

void loop() {
  bool on = true;
  EspMultiLogger::loopLogger();
  // put your main code here, to run repeatedly:
  if(millis()%1000==0){
    if(on == true){
      on = false;
      digitalWrite(LED, HIGH);
      Logger->println("LED is on");
    }else{
      on = true;
      digitalWrite(LED, LOW);
      Logger->println("LED is off");
    }
  }
}