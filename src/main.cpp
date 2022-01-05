/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
#include <ESP8266WiFi.h>

#include <EspMultiLogger.h>
#include "WlanConfig.h"
/*
Configuration
*/
const char* versionStr = "20220104v0.2";
#define LoggingWithTimeout

#ifdef LoggingWithTimeout
#define logTimeout (1800) // 60 min * 60sec = 1 std
#endif 

#define LED 2

#ifndef WlanConfig_h
#define STASSID "------------"
#define STAPSK  "------------"
#endif
const char* ssid = STASSID;
const char* password = STAPSK;

#define MAX_TELNET_CLIENTS 2

WiFiServer TelnetServer(23);
WiFiClient TelnetClient[MAX_TELNET_CLIENTS];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\n Starting Version:");
  Serial.println(versionStr);
  pinMode(LED, OUTPUT);

  Serial.println("Starting Telnet server");
  TelnetServer.begin();
  TelnetServer.setNoDelay(true);

  EspMultiLogger Logger = new EspMultiLogger(Debug);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, HIGH);
  Serial.println("LED is on");
  delay(1000);
  digitalWrite(LED, LOW);
  Serial.println("LED is off");
  delay(1000);
}