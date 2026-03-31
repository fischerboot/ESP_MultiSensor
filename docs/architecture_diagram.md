# ESP MultiSensor Health Monitoring - Architecture Diagram

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         ESP8266 MultiSensor Device                       │
│                                                                           │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                         Main Application                          │  │
│  │                           (main.cpp)                              │  │
│  │                                                                    │  │
│  │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐     │  │
│  │  │   Sensors    │    │    WiFi      │    │     MQTT     │     │  │
│  │  │  BME280/     │    │   Manager    │    │   Client     │     │  │
│  │  │   BMP280     │    │              │    │  PubSubClient│     │  │
│  │  │     PIR      │    │              │    │              │     │  │
│  │  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘     │  │
│  │         │                    │                    │              │  │
│  │         └────────────────────┼────────────────────┘              │  │
│  │                              ▼                                   │  │
│  │                    ┌──────────────────┐                         │  │
│  │                    │  HealthMonitor   │                         │  │
│  │                    │     Class        │                         │  │
│  │                    │                  │                         │  │
│  │                    │  - WiFi Metrics  │                         │  │
│  │                    │  - MQTT Metrics  │                         │  │
│  │                    │  - System State  │                         │  │
│  │                    │  - Sensor Status │                         │  │
│  │                    │  - NTP Status    │                         │  │
│  │                    └────────┬─────────┘                         │  │
│  │                             │                                    │  │
│  │                             ▼                                    │  │
│  │                    ┌──────────────────┐                         │  │
│  │                    │  JSON Generator  │                         │  │
│  │                    │  Status Calc     │                         │  │
│  │                    └────────┬─────────┘                         │  │
│  └─────────────────────────────┼──────────────────────────────────┘  │
│                                 │                                      │
└─────────────────────────────────┼──────────────────────────────────────┘
                                  │
                                  │ MQTT Topics
                                  │ (every 60s + events)
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                           MQTT Broker                                    │
│                        (e.g., Mosquitto)                                │
│                                                                           │
│  Topics:                                                                 │
│  ├─ {prefix}/health/status     (JSON, retained)                        │
│  ├─ {prefix}/health/rssi       (number, retained)                      │
│  ├─ {prefix}/health/heap       (number, retained)                      │
│  ├─ {prefix}/health/uptime     (number)                                │
│  ├─ {prefix}/health/wifi       (string, retained)                      │
│  ├─ {prefix}/health/mqtt       (string, retained)                      │
│  ├─ {prefix}/health/sensor     (string, retained)                      │
│  └─ {prefix}/health/mqtt_fails (number)                                │
│                                                                           │
└───────────────────────┬─────────────────────┬───────────────────────────┘
                        │                     │
                        │                     │
                        ▼                     ▼
        ┌───────────────────────┐   ┌───────────────────────┐
        │   Home Assistant      │   │      OpenHAB          │
        │                       │   │                       │
        │  ┌─────────────────┐ │   │  ┌─────────────────┐ │
        │  │ Auto-Discovery  │ │   │  │  MQTT Binding   │ │
        │  │   (optional)    │ │   │  └────────┬────────┘ │
        │  └────────┬────────┘ │   │           │          │
        │           ▼           │   │           ▼          │
        │  ┌─────────────────┐ │   │  ┌─────────────────┐ │
        │  │   Entities      │ │   │  │   Things        │ │
        │  │   - Sensors     │ │   │  │   - Channels    │ │
        │  │   - Diagnostics │ │   │  └────────┬────────┘ │
        │  └────────┬────────┘ │   │           ▼          │
        │           ▼           │   │  ┌─────────────────┐ │
        │  ┌─────────────────┐ │   │  │     Items       │ │
        │  │   Dashboard     │ │   │  │   - Health      │ │
        │  │   - Lovelace    │ │   │  │   - Sensors     │ │
        │  └─────────────────┘ │   │  └────────┬────────┘ │
        └───────────────────────┘   │           ▼          │
                                    │  ┌─────────────────┐ │
                                    │  │   Sitemaps      │ │
                                    │  │   - Dashboard   │ │
                                    │  └────────┬────────┘ │
                                    │           │          │
                                    │           ▼          │
                                    │  ┌─────────────────┐ │
                                    │  │     Rules       │ │
                                    │  │   - Alerts      │ │
                                    │  │   - Automation  │ │
                                    │  └─────────────────┘ │
                                    └───────────────────────┘
```

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       Health Monitoring Cycle                            │
└─────────────────────────────────────────────────────────────────────────┘

    ┌──────────────────────────────────────────────────────────────────┐
    │  Every 60 seconds OR on Event (WiFi disconnect, etc.)           │
    └──────────────────────┬───────────────────────────────────────────┘
                           │
                           ▼
           ┌───────────────────────────────┐
           │   healthMonitor.update()      │
           │   Collect all metrics:        │
           └───────────┬───────────────────┘
                       │
        ┏──────────────┴──────────────┓
        ▼                             ▼
┌───────────────┐             ┌───────────────┐
│ WiFi Metrics  │             │ System Metrics│
│ - RSSI        │             │ - Free Heap   │
│ - IP Address  │             │ - Uptime      │
│ - Connected   │             │ - Version     │
└───────┬───────┘             └───────┬───────┘
        │                             │
        ▼                             ▼
┌───────────────┐             ┌───────────────┐
│ MQTT Metrics  │             │ Sensor Metrics│
│ - Connected   │             │ - Type        │
│ - Failures    │             │ - Status      │
│ - Reconnects  │             │ - Last Read   │
└───────┬───────┘             └───────┬───────┘
        │                             │
        └──────────────┬──────────────┘
                       │
                       ▼
           ┌───────────────────────┐
           │ Calculate Health      │
           │ Status:               │
           │ - healthy             │
           │ - degraded            │
           │ - critical            │
           └───────┬───────────────┘
                   │
                   ▼
           ┌───────────────────────┐
           │ Generate JSON Payload │
           └───────┬───────────────┘
                   │
                   ▼
           ┌───────────────────────┐
           │  Publish to MQTT      │
           │  - status (JSON)      │
           │  - individual metrics │
           └───────┬───────────────┘
                   │
                   ▼
           ┌───────────────────────┐
           │  MQTT Broker          │
           │  Distributes to       │
           │  subscribers          │
           └───────┬───────────────┘
                   │
        ┏──────────┴──────────┓
        ▼                     ▼
┌───────────────┐     ┌───────────────┐
│ Home          │     │ OpenHAB       │
│ Assistant     │     │ - Items update│
│ - Auto update │     │ - Rules check │
│ - Dashboard   │     │ - Alerts fire │
└───────────────┘     └───────────────┘
```

---

## Component Interaction Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      ESP8266 Main Loop Cycle                             │
└─────────────────────────────────────────────────────────────────────────┘

loop() {
    │
    ├─► handleMqttConnection()
    │   └─► if (connected) {
    │       │   mqttClient.loop()
    │       │   healthMonitor.setMQTTConnected(true)
    │       │
    │       │   if (!discoveryPublished) {
    │       │       publishDiscoveryMessages()  ◄── Includes health sensors
    │       │       publishHealthMetrics()      ◄── Initial health status
    │       │   }
    │       │
    │       } else {
    │           healthMonitor.setMQTTConnected(false)
    │           healthMonitor.recordMQTTFailure()
    │           attemptReconnect()
    │       }
    │
    ├─► ArduinoOTA.handle()
    │
    ├─► if (healthMonitor.shouldUpdate()) {     ◄── Every 60 seconds
    │       healthMonitor.update()              ◄── Collect metrics
    │       publishHealthMetrics()              ◄── Publish to MQTT
    │   }
    │
    ├─► if (time for sensor read) {
    │       readBME280orBMP280()
    │       healthMonitor.recordSensorRead()    ◄── Track successful read
    │       publishSensorData()
    │   }
    │
    └─► checkPIR()
        if (motion detected) {
            publishMotion()
        }
}
```

---

## Health Status Decision Tree

```
                    ┌─────────────────────┐
                    │  Collect Metrics    │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │  Check Critical     │
                    │  Conditions:        │
                    │  - WiFi down?       │
                    │  - MQTT down?       │
                    │  - Heap < 5KB?      │
                    │  - Sensor failed?   │
                    └──────────┬──────────┘
                               │
                    ┌──────────┴──────────┐
                    │                     │
                 YES │                    │ NO
                    │                     │
                    ▼                     ▼
         ┌─────────────────┐   ┌─────────────────────┐
         │   CRITICAL      │   │  Check Degraded     │
         │   Status        │   │  Conditions:        │
         │                 │   │  - RSSI < -80 dBm?  │
         │   🔴 Alert!     │   │  - Heap < 10KB?     │
         └─────────────────┘   │  - MQTT fails > 5?  │
                               │  - No sensor read?  │
                               │  - NTP not synced?  │
                               └──────────┬──────────┘
                                          │
                               ┌──────────┴──────────┐
                               │                     │
                            YES │                    │ NO
                               │                     │
                               ▼                     ▼
                    ┌─────────────────┐   ┌─────────────────┐
                    │   DEGRADED      │   │    HEALTHY      │
                    │   Status        │   │    Status       │
                    │                 │   │                 │
                    │   ⚠️  Warning   │   │    ✅  Good     │
                    └─────────────────┘   └─────────────────┘
```

---

## MQTT Message Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Message Publishing                               │
└─────────────────────────────────────────────────────────────────────────┘

ESP8266 Device
    │
    │  Periodic Update (60s)
    ├─► Publish: {prefix}/health/status
    │   Payload: {...JSON with all health data...}
    │   Retained: YES
    │   QoS: 0
    │
    ├─► Publish: {prefix}/health/rssi
    │   Payload: "-65"
    │   Retained: YES
    │
    ├─► Publish: {prefix}/health/heap
    │   Payload: "25600"
    │   Retained: YES
    │
    ├─► Publish: {prefix}/health/uptime
    │   Payload: "86400"
    │   Retained: NO  ◄── Changes frequently
    │
    ├─► Publish: {prefix}/health/wifi
    │   Payload: "online"
    │   Retained: YES
    │
    ├─► Publish: {prefix}/health/mqtt
    │   Payload: "online"
    │   Retained: YES
    │
    ├─► Publish: {prefix}/health/sensor
    │   Payload: "BME280"
    │   Retained: YES
    │
    └─► Publish: {prefix}/health/mqtt_fails
        Payload: "0"
        Retained: NO

        │
        ▼
MQTT Broker
        │
        ├──────────────┬──────────────┬──────────────┐
        │              │              │              │
        ▼              ▼              ▼              ▼
  Home Assistant   OpenHAB      MQTT Explorer    Custom
    (optional)     (Things)     (Debug tool)     Clients
```

---

## OpenHAB Configuration Hierarchy

```
OpenHAB Configuration
│
├─► Bindings
│   └─► MQTT Binding (installed)
│
├─► Things
│   └─► mqtt:broker:mybroker
│       └─► mqtt:topic:esp_multisensor
│           ├─► Channel: temperature
│           ├─► Channel: pressure
│           ├─► Channel: humidity
│           ├─► Channel: pir
│           ├─► Channel: health_status
│           ├─► Channel: health_rssi
│           ├─► Channel: health_heap
│           ├─► Channel: health_uptime
│           ├─► Channel: health_wifi
│           ├─► Channel: health_mqtt
│           ├─► Channel: health_sensor
│           └─► Channel: health_mqtt_fails
│
├─► Items
│   ├─► Group: gSensors
│   │   ├─► ESP_Temperature
│   │   ├─► ESP_Pressure
│   │   ├─► ESP_Humidity
│   │   └─► ESP_Motion
│   │
│   └─► Group: gHealth
│       ├─► ESP_Health
│       ├─► ESP_RSSI
│       ├─► ESP_Heap
│       ├─► ESP_Uptime
│       ├─► ESP_WiFi
│       ├─► ESP_MQTT
│       ├─► ESP_Sensor
│       └─► ESP_MQTT_Fails
│
├─► Sitemaps
│   └─► esp_health.sitemap
│       ├─► Frame: Sensor Readings
│       │   └─► (sensor items with color coding)
│       │
│       └─► Frame: Device Health
│           ├─► Frame: Connection Status
│           │   └─► (WiFi, RSSI, MQTT items)
│           │
│           └─► Frame: System Status
│               └─► (Uptime, Heap, Sensor items)
│
├─► Rules
│   └─► esp_health_alerts.rules
│       ├─► Rule: Health Status Changed
│       ├─► Rule: WiFi Disconnected
│       ├─► Rule: MQTT Disconnected
│       ├─► Rule: Low Memory Warning
│       └─► Rule: High MQTT Failure Rate
│
├─► Persistence
│   └─► influxdb.persist (or rrd4j, etc.)
│       └─► Groups: gSensors, gHealth
│
└─► Transformations
    ├─► health.map (status → display text)
    └─► onoff.map (online/offline → display text)
```

---

## Memory Layout

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    ESP8266 Memory Usage                                  │
└─────────────────────────────────────────────────────────────────────────┘

Typical Free Heap: ~40KB

┌────────────────────────────────────────────────────────────────┐
│                      Flash Memory (Program)                     │
│                                                                  │
│  Existing Code:              ~200 KB                           │
│  HealthMonitor class:        ~8 KB                             │
│  Libraries (WiFi, MQTT):     ~50 KB                            │
│  Arduino Core:               ~256 KB                           │
│  Total Used:                 ~514 KB / 1 MB                    │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│                         RAM (Heap)                              │
│                                                                  │
│  WiFi Stack:                 ~15 KB                            │
│  MQTT Client:                ~5 KB                             │
│  Application:                ~10 KB                            │
│  HealthMonitor Instance:     ~0.5 KB                           │
│  JSON Buffer (temporary):    ~0.3 KB                           │
│  ────────────────────────────────────────────────────────────  │
│  Total Used:                 ~31 KB                            │
│  Free Heap:                  ~40 KB  ✅ Safe margin           │
└────────────────────────────────────────────────────────────────┘
```

---

## Timing Diagram

```
Time ────────────────────────────────────────────────────────────────►

     0s        10s       20s       30s       40s       50s       60s
     │         │         │         │         │         │         │
     │         │         │         │         │         │         │
WiFi │─────────●─────────────────────────────────────────────────● Connected
     │         │                                                 │
MQTT │─────────●───────●───────────────────────────●─────────────● Messages
     │         │       │                           │             │
Sens │─────────●───────●───────────────────────────●─────────────● Every 500ms
     │         │       │                           │             │
Hlth │─────────●────────────────────────────────────────────────● 60s interval
     │         │                                                 │
     │  ┌──────┴──────┐                                  ┌───────┴────────┐
     │  │Initial      │                                  │ Periodic       │
     │  │Health Check │                                  │ Health Check   │
     │  └─────────────┘                                  └────────────────┘

Legend:
───  Continuous operation
●    Event/Action
```

---

## Error Handling Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      Error Scenarios & Recovery                          │
└─────────────────────────────────────────────────────────────────────────┘

Scenario 1: WiFi Disconnection
    │
    ├─► WiFi.status() != WL_CONNECTED
    │   └─► healthMonitor.collectWiFiHealth()
    │       ├─► Sets wifi.connected = false
    │       ├─► Sets RSSI = -100
    │       └─► calculateStatus() → CRITICAL
    │
    ├─► publishHealthMetrics()
    │   └─► Attempts to publish (will fail if MQTT down)
    │
    ├─► WiFiManager attempts reconnection
    │
    └─► On reconnect:
        └─► Next health update shows healthy state

Scenario 2: MQTT Broker Down
    │
    ├─► mqttClient.connected() == false
    │   └─► healthMonitor.setMQTTConnected(false)
    │       └─► calculateStatus() → CRITICAL
    │
    ├─► handleMqttConnection()
    │   ├─► Attempts reconnection every 5 seconds
    │   └─► healthMonitor.recordMQTTFailure()
    │
    └─► On reconnect:
        ├─► healthMonitor.recordMQTTReconnect()
        ├─► Resets failure count
        ├─► Republishes discovery
        └─► Publishes current health status

Scenario 3: Sensor Failure
    │
    ├─► bme.begin() or bmp.begin() fails
    │   └─► healthMonitor.setSensorOperational(false)
    │       └─► calculateStatus() → CRITICAL
    │
    ├─► No sensor reads recorded
    │   └─► lastReadAge increases
    │
    └─► publishHealthMetrics()
        └─► Shows sensor as "not operational"

Scenario 4: Low Memory
    │
    ├─► ESP.getFreeHeap() < 5000
    │   └─► calculateStatus() → CRITICAL
    │
    ├─► publishHealthMetrics()
    │   └─► Alerts via MQTT
    │
    └─► OpenHAB Rule triggers
        └─► Sends notification

Scenario 5: NTP Sync Failure
    │
    ├─► time(nullptr) < threshold
    │   └─► ntp.synced = false
    │       └─► calculateStatus() → DEGRADED
    │
    └─► Continues with fallback time calculation
```

---

## Legend

```
Symbols Used:
┌─┐ ││└─┘  Box/Container
─       Horizontal line
│       Vertical line
├─      Branch/Connection
└─      End branch
▼       Flow direction down
▶       Flow direction right
●       Event/Action point
✅      Success/OK
⚠️      Warning
❌      Error/Critical
🔴      Alert
```

---

**Version:** 1.0  
**Last Updated:** March 31, 2026  
**Diagram Type:** ASCII Art Architecture  
**Purpose:** Visual Guide for Health Monitoring System
