# ESP MultiSensor Health Monitoring - Implementation Guide

## Quick Start Implementation

This guide provides step-by-step instructions for implementing the health monitoring system in your ESP MultiSensor project.

---

## Implementation Overview

The implementation consists of:
1. **HealthMonitor class** - Core health monitoring logic
2. **Main.cpp modifications** - Integration with existing code
3. **MQTT topics** - Health data publishing
4. **Home Assistant discovery** - Auto-configuration

---

## Step 1: Create HealthMonitor Header File

**File:** `include/HealthMonitor.h`

```cpp
#ifndef HEALTHMONITOR_H
#define HEALTHMONITOR_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <time.h>

// Configuration
#define HEALTH_CHECK_INTERVAL 60000      // 60 seconds
#define HEALTH_RSSI_GOOD -70             // dBm
#define HEALTH_RSSI_POOR -80             // dBm
#define HEALTH_HEAP_LOW 10000            // bytes
#define HEALTH_HEAP_CRITICAL 5000        // bytes
#define HEALTH_SENSOR_READ_TIMEOUT 300   // seconds

class HealthMonitor {
public:
    enum HealthStatus {
        HEALTHY,
        DEGRADED,
        CRITICAL,
        UNKNOWN
    };

    struct WiFiHealth {
        bool connected;
        int rssi;
        String ip;
    };

    struct MQTTHealth {
        bool connected;
        uint32_t failures;
        uint32_t reconnects;
    };

    struct SystemHealth {
        uint32_t uptime;
        uint32_t freeHeap;
        String version;
    };

    struct SensorHealth {
        String type;
        bool operational;
        uint32_t lastReadAge;  // seconds since last read
    };

    struct NTPHealth {
        bool synced;
        time_t timestamp;
    };

    struct HealthData {
        HealthStatus status;
        WiFiHealth wifi;
        MQTTHealth mqtt;
        SystemHealth system;
        SensorHealth sensor;
        NTPHealth ntp;
    };

    HealthMonitor();
    void begin(const char* fwVersion);
    bool shouldUpdate();  // Check if it's time to update
    void update();        // Collect all metrics
    
    // Status
    HealthStatus getStatus();
    String getStatusString();
    HealthData getData();
    String getStatusJSON();
    
    // Event recording
    void recordMQTTFailure();
    void recordMQTTReconnect();
    void recordSensorRead();
    void setSensorType(const char* type);
    void setSensorOperational(bool operational);
    void setMQTTConnected(bool connected);
    
    // Configuration
    void setCheckInterval(uint32_t intervalMs);
    uint32_t getCheckInterval();
    
private:
    HealthStatus calculateStatus();
    void collectWiFiHealth();
    void collectMQTTHealth();
    void collectSystemHealth();
    void collectSensorHealth();
    void collectNTPHealth();
    
    // State
    HealthData _data;
    unsigned long _lastUpdate;
    unsigned long _lastSensorRead;
    uint32_t _checkInterval;
    String _firmwareVersion;
    String _sensorType;
    bool _sensorOperational;
    bool _mqttConnected;
};

#endif // HEALTHMONITOR_H
```

---

## Step 2: Create HealthMonitor Implementation

**File:** `src/HealthMonitor.cpp`

```cpp
#include "HealthMonitor.h"

HealthMonitor::HealthMonitor() 
    : _lastUpdate(0)
    , _lastSensorRead(0)
    , _checkInterval(HEALTH_CHECK_INTERVAL)
    , _firmwareVersion("unknown")
    , _sensorType("UNKNOWN")
    , _sensorOperational(false)
    , _mqttConnected(false)
{
    _data.status = UNKNOWN;
    _data.mqtt.failures = 0;
    _data.mqtt.reconnects = 0;
}

void HealthMonitor::begin(const char* fwVersion) {
    _firmwareVersion = String(fwVersion);
    _lastUpdate = 0;
    _lastSensorRead = millis();
    
    // Initial update
    update();
}

bool HealthMonitor::shouldUpdate() {
    return (millis() - _lastUpdate >= _checkInterval);
}

void HealthMonitor::update() {
    collectWiFiHealth();
    collectMQTTHealth();
    collectSystemHealth();
    collectSensorHealth();
    collectNTPHealth();
    
    _data.status = calculateStatus();
    _lastUpdate = millis();
}

// WiFi Health Collection
void HealthMonitor::collectWiFiHealth() {
    _data.wifi.connected = WiFi.status() == WL_CONNECTED;
    
    if (_data.wifi.connected) {
        _data.wifi.rssi = WiFi.RSSI();
        _data.wifi.ip = WiFi.localIP().toString();
    } else {
        _data.wifi.rssi = -100;
        _data.wifi.ip = "0.0.0.0";
    }
}

// MQTT Health Collection
void HealthMonitor::collectMQTTHealth() {
    _data.mqtt.connected = _mqttConnected;
}

// System Health Collection
void HealthMonitor::collectSystemHealth() {
    _data.system.uptime = millis() / 1000;  // seconds
    _data.system.freeHeap = ESP.getFreeHeap();
    _data.system.version = _firmwareVersion;
}

// Sensor Health Collection
void HealthMonitor::collectSensorHealth() {
    _data.sensor.type = _sensorType;
    _data.sensor.operational = _sensorOperational;
    _data.sensor.lastReadAge = (millis() - _lastSensorRead) / 1000;  // seconds
}

// NTP Health Collection
void HealthMonitor::collectNTPHealth() {
    time_t now = time(nullptr);
    _data.ntp.synced = (now > 8 * 3600 * 2);  // Valid time check
    _data.ntp.timestamp = now;
}

// Calculate Overall Health Status
HealthMonitor::HealthStatus HealthMonitor::calculateStatus() {
    bool hasCritical = false;
    bool hasDegraded = false;
    
    // Critical conditions
    if (!_data.wifi.connected) hasCritical = true;
    if (!_data.mqtt.connected) hasCritical = true;
    if (_data.system.freeHeap < HEALTH_HEAP_CRITICAL) hasCritical = true;
    if (!_data.sensor.operational && _sensorType != "UNKNOWN") hasCritical = true;
    
    // Degraded conditions
    if (_data.wifi.rssi < HEALTH_RSSI_POOR) hasDegraded = true;
    if (_data.system.freeHeap < HEALTH_HEAP_LOW) hasDegraded = true;
    if (_data.mqtt.failures > 5) hasDegraded = true;
    if (_data.sensor.lastReadAge > HEALTH_SENSOR_READ_TIMEOUT) hasDegraded = true;
    if (!_data.ntp.synced) hasDegraded = true;
    
    if (hasCritical) return CRITICAL;
    if (hasDegraded) return DEGRADED;
    return HEALTHY;
}

// Status Getters
HealthMonitor::HealthStatus HealthMonitor::getStatus() {
    return _data.status;
}

String HealthMonitor::getStatusString() {
    switch (_data.status) {
        case HEALTHY: return "healthy";
        case DEGRADED: return "degraded";
        case CRITICAL: return "critical";
        default: return "unknown";
    }
}

HealthMonitor::HealthData HealthMonitor::getData() {
    return _data;
}

// Generate JSON Status
String HealthMonitor::getStatusJSON() {
    String json = "{";
    
    // Overall status
    json += "\"status\":\"" + getStatusString() + "\",";
    
    // WiFi health
    json += "\"wifi\":{";
    json += "\"connected\":" + String(_data.wifi.connected ? "true" : "false") + ",";
    json += "\"rssi\":" + String(_data.wifi.rssi) + ",";
    json += "\"ip\":\"" + _data.wifi.ip + "\"";
    json += "},";
    
    // MQTT health
    json += "\"mqtt\":{";
    json += "\"connected\":" + String(_data.mqtt.connected ? "true" : "false") + ",";
    json += "\"failures\":" + String(_data.mqtt.failures) + ",";
    json += "\"reconnects\":" + String(_data.mqtt.reconnects);
    json += "},";
    
    // System health
    json += "\"system\":{";
    json += "\"uptime\":" + String(_data.system.uptime) + ",";
    json += "\"heap\":" + String(_data.system.freeHeap) + ",";
    json += "\"version\":\"" + _data.system.version + "\"";
    json += "},";
    
    // Sensor health
    json += "\"sensor\":{";
    json += "\"type\":\"" + _data.sensor.type + "\",";
    json += "\"operational\":" + String(_data.sensor.operational ? "true" : "false") + ",";
    json += "\"last_read\":" + String(_data.sensor.lastReadAge);
    json += "},";
    
    // NTP health
    json += "\"ntp\":{";
    json += "\"synced\":" + String(_data.ntp.synced ? "true" : "false") + ",";
    json += "\"timestamp\":" + String(_data.ntp.timestamp);
    json += "}";
    
    json += "}";
    return json;
}

// Event Recording
void HealthMonitor::recordMQTTFailure() {
    _data.mqtt.failures++;
}

void HealthMonitor::recordMQTTReconnect() {
    _data.mqtt.reconnects++;
    _data.mqtt.failures = 0;  // Reset failure count on successful reconnect
}

void HealthMonitor::recordSensorRead() {
    _lastSensorRead = millis();
    _sensorOperational = true;
}

void HealthMonitor::setSensorType(const char* type) {
    _sensorType = String(type);
}

void HealthMonitor::setSensorOperational(bool operational) {
    _sensorOperational = operational;
}

void HealthMonitor::setMQTTConnected(bool connected) {
    _mqttConnected = connected;
}

// Configuration
void HealthMonitor::setCheckInterval(uint32_t intervalMs) {
    _checkInterval = intervalMs;
}

uint32_t HealthMonitor::getCheckInterval() {
    return _checkInterval;
}
```

---

## Step 3: Integrate into main.cpp

### 3.1 Add Include and Global Variable

Add near the top of `main.cpp` after other includes:

```cpp
#include <HealthMonitor.h>

// After other global variables
HealthMonitor healthMonitor;

// Add health MQTT topics
char MQTT_TOPIC_HEALTH_STATUS[60];
char MQTT_TOPIC_HEALTH_RSSI[60];
char MQTT_TOPIC_HEALTH_HEAP[60];
char MQTT_TOPIC_HEALTH_UPTIME[60];
char MQTT_TOPIC_HEALTH_WIFI[60];
char MQTT_TOPIC_HEALTH_MQTT[60];
char MQTT_TOPIC_HEALTH_SENSOR[60];
char MQTT_TOPIC_HEALTH_MQTT_FAILS[60];
```

### 3.2 Initialize Health Topics

Add to the setup where other MQTT topics are initialized:

```cpp
// After existing topic initialization (around line with MQTT_TOPIC_TEMP)
snprintf(MQTT_TOPIC_HEALTH_STATUS, sizeof(MQTT_TOPIC_HEALTH_STATUS), 
         "%s/health/status", device_prefix);
snprintf(MQTT_TOPIC_HEALTH_RSSI, sizeof(MQTT_TOPIC_HEALTH_RSSI), 
         "%s/health/rssi", device_prefix);
snprintf(MQTT_TOPIC_HEALTH_HEAP, sizeof(MQTT_TOPIC_HEALTH_HEAP), 
         "%s/health/heap", device_prefix);
snprintf(MQTT_TOPIC_HEALTH_UPTIME, sizeof(MQTT_TOPIC_HEALTH_UPTIME), 
         "%s/health/uptime", device_prefix);
snprintf(MQTT_TOPIC_HEALTH_WIFI, sizeof(MQTT_TOPIC_HEALTH_WIFI), 
         "%s/health/wifi", device_prefix);
snprintf(MQTT_TOPIC_HEALTH_MQTT, sizeof(MQTT_TOPIC_HEALTH_MQTT), 
         "%s/health/mqtt", device_prefix);
snprintf(MQTT_TOPIC_HEALTH_SENSOR, sizeof(MQTT_TOPIC_HEALTH_SENSOR), 
         "%s/health/sensor", device_prefix);
snprintf(MQTT_TOPIC_HEALTH_MQTT_FAILS, sizeof(MQTT_TOPIC_HEALTH_MQTT_FAILS), 
         "%s/health/mqtt_fails", device_prefix);
```

### 3.3 Initialize Health Monitor in setup()

Add near the end of `setup()` function, after MQTT is initialized:

```cpp
// Initialize Health Monitor
healthMonitor.begin(versionStr);

// Set initial sensor type after sensor detection
if (detectedSensor == SENSOR_BME280) {
    healthMonitor.setSensorType("BME280");
    healthMonitor.setSensorOperational(true);
} else if (detectedSensor == SENSOR_BMP280) {
    healthMonitor.setSensorType("BMP280");
    healthMonitor.setSensorOperational(true);
} else {
    healthMonitor.setSensorType("NONE");
    healthMonitor.setSensorOperational(false);
}

InfoLogger->println("Health monitoring initialized");
```

### 3.4 Add Health Publishing Function

Add this new function before `loop()`:

```cpp
void publishHealthMetrics() {
    if (!mqttClient.connected()) {
        healthMonitor.setMQTTConnected(false);
        return;
    }
    
    healthMonitor.setMQTTConnected(true);
    
    // Update health data
    healthMonitor.update();
    
    // Get health data
    HealthMonitor::HealthData health = healthMonitor.getData();
    
    // Publish JSON status (retained)
    String jsonStatus = healthMonitor.getStatusJSON();
    if (!mqttClient.publish(MQTT_TOPIC_HEALTH_STATUS, jsonStatus.c_str(), true)) {
        healthMonitor.recordMQTTFailure();
    }
    
    // Publish individual metrics
    char buffer[16];
    
    // RSSI
    snprintf(buffer, sizeof(buffer), "%d", health.wifi.rssi);
    if (!mqttClient.publish(MQTT_TOPIC_HEALTH_RSSI, buffer, true)) {
        healthMonitor.recordMQTTFailure();
    }
    
    // Heap
    snprintf(buffer, sizeof(buffer), "%u", health.system.freeHeap);
    if (!mqttClient.publish(MQTT_TOPIC_HEALTH_HEAP, buffer, true)) {
        healthMonitor.recordMQTTFailure();
    }
    
    // Uptime
    snprintf(buffer, sizeof(buffer), "%u", health.system.uptime);
    if (!mqttClient.publish(MQTT_TOPIC_HEALTH_UPTIME, buffer, false)) {
        healthMonitor.recordMQTTFailure();
    }
    
    // WiFi Status
    const char* wifiStatus = health.wifi.connected ? "online" : "offline";
    if (!mqttClient.publish(MQTT_TOPIC_HEALTH_WIFI, wifiStatus, true)) {
        healthMonitor.recordMQTTFailure();
    }
    
    // MQTT Status
    const char* mqttStatus = health.mqtt.connected ? "online" : "offline";
    if (!mqttClient.publish(MQTT_TOPIC_HEALTH_MQTT, mqttStatus, true)) {
        healthMonitor.recordMQTTFailure();
    }
    
    // Sensor Type
    if (!mqttClient.publish(MQTT_TOPIC_HEALTH_SENSOR, health.sensor.type.c_str(), true)) {
        healthMonitor.recordMQTTFailure();
    }
    
    // MQTT Failures
    snprintf(buffer, sizeof(buffer), "%u", health.mqtt.failures);
    if (!mqttClient.publish(MQTT_TOPIC_HEALTH_MQTT_FAILS, buffer, false)) {
        healthMonitor.recordMQTTFailure();
    }
    
    InfoLogger->printf("Health published: %s\n", healthMonitor.getStatusString().c_str());
}
```

### 3.5 Update handleMqttConnection()

Modify the MQTT connection handler to track connection state:

```cpp
void handleMqttConnection() {
  if (mqttClient.connected()){
    mqttClient.loop();
    // Publish discovery messages on first connection
    if (!discoveryPublished && discoveryPlatform != DISCOVERY_NONE) {
      publishDiscoveryMessages();
      discoveryPublished = true;
      // Publish initial health status
      publishHealthMetrics();
    }
  } else{
    healthMonitor.setMQTTConnected(false);  // ADD THIS LINE
    discoveryPublished = false;
    unsigned long now = millis();
    if (now - lastMqttAttempt < MQTT_RETRY_INTERVAL) return;
    lastMqttAttempt = now;

    InfoLogger->println("Attempting MQTT connection...");
    char mqttClientName[32];
    snprintf(mqttClientName, sizeof(mqttClientName), "%s_MultiSensor", device_prefix);
    bool connected = false;
    if (mqtt_user[0] == '\0') {
      connected = mqttClient.connect(mqttClientName);
    } else {
      connected = mqttClient.connect(mqttClientName, mqtt_user, mqtt_pass);
    }
    if (connected) {
      InfoLogger->println("MQTT connected");
      healthMonitor.recordMQTTReconnect();  // ADD THIS LINE
      healthMonitor.setMQTTConnected(true);  // ADD THIS LINE
    } else {
      InfoLogger->printf("MQTT failed, rc=%d\n", mqttClient.state());
      healthMonitor.recordMQTTFailure();  // ADD THIS LINE
    }
  }
}
```

### 3.6 Update Sensor Reading Code

In the sensor reading section of `loop()`, add:

```cpp
// After successful sensor readings
healthMonitor.recordSensorRead();
```

### 3.7 Add Health Check to loop()

In the `loop()` function, add health monitoring:

```cpp
void loop() {
  // Existing code...
  handleMqttConnection();
  ArduinoOTA.handle();
  
  // Add health monitoring check
  if (healthMonitor.shouldUpdate()) {
    publishHealthMetrics();
  }
  
  // Rest of existing loop code...
}
```

### 3.8 Extend Home Assistant Discovery

Add health sensors to the `publishHomeAssistantDiscovery()` function:

```cpp
void publishHomeAssistantDiscovery() {
  char topic[128];
  char payload[512];  // Increase size for longer payloads

  InfoLogger->println("Publishing Home Assistant discovery messages...");

  // ... existing sensor discovery code ...

  // Health Status Sensor
  snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_health/config", device_prefix);
  snprintf(payload, sizeof(payload), 
    "{\"name\":\"Health %s\",\"state_topic\":\"%s\",\"value_template\":\"{{ value_json.status }}\","
    "\"json_attributes_topic\":\"%s\",\"unique_id\":\"%s_health\","
    "\"entity_category\":\"diagnostic\",\"icon\":\"mdi:heart-pulse\","
    "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\"}}",
    device_prefix, MQTT_TOPIC_HEALTH_STATUS, MQTT_TOPIC_HEALTH_STATUS, device_prefix, device_prefix, device_prefix);
  mqttClient.publish(topic, payload);
  InfoLogger->println("  Health status discovery published");

  // WiFi RSSI Sensor
  snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_rssi/config", device_prefix);
  snprintf(payload, sizeof(payload), 
    "{\"name\":\"WiFi Signal %s\",\"device_class\":\"signal_strength\","
    "\"state_topic\":\"%s\",\"unit_of_measurement\":\"dBm\",\"unique_id\":\"%s_rssi\","
    "\"entity_category\":\"diagnostic\",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\"}}",
    device_prefix, MQTT_TOPIC_HEALTH_RSSI, device_prefix, device_prefix, device_prefix);
  mqttClient.publish(topic, payload);
  InfoLogger->println("  RSSI discovery published");

  // Free Heap Sensor
  snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_heap/config", device_prefix);
  snprintf(payload, sizeof(payload), 
    "{\"name\":\"Free Memory %s\",\"state_topic\":\"%s\",\"unit_of_measurement\":\"B\","
    "\"unique_id\":\"%s_heap\",\"entity_category\":\"diagnostic\",\"icon\":\"mdi:memory\","
    "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\"}}",
    device_prefix, MQTT_TOPIC_HEALTH_HEAP, device_prefix, device_prefix, device_prefix);
  mqttClient.publish(topic, payload);
  InfoLogger->println("  Heap discovery published");

  // Uptime Sensor
  snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_uptime/config", device_prefix);
  snprintf(payload, sizeof(payload), 
    "{\"name\":\"Uptime %s\",\"device_class\":\"duration\","
    "\"state_topic\":\"%s\",\"unit_of_measurement\":\"s\",\"unique_id\":\"%s_uptime\","
    "\"entity_category\":\"diagnostic\",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\"}}",
    device_prefix, MQTT_TOPIC_HEALTH_UPTIME, device_prefix, device_prefix, device_prefix);
  mqttClient.publish(topic, payload);
  InfoLogger->println("  Uptime discovery published");
}
```

---

## Step 4: Update Version Number

Update the version string in main.cpp:

```cpp
const char* versionStr = "20260331v1.15";  // Increment version
```

---

## Step 5: Compile and Upload

```bash
# In PlatformIO terminal
pio run --target upload

# Or use OTA
pio run --target upload --upload-port 192.168.2.75
```

---

## Step 6: Verify MQTT Topics

Use MQTT Explorer or mosquitto_sub to verify health topics:

```bash
mosquitto_sub -h 192.168.2.127 -t "ESP_Default/health/#" -v
```

Expected output:
```
ESP_Default/health/status {"status":"healthy",...}
ESP_Default/health/rssi -65
ESP_Default/health/heap 25600
ESP_Default/health/uptime 300
ESP_Default/health/wifi online
ESP_Default/health/mqtt online
ESP_Default/health/sensor BME280
ESP_Default/health/mqtt_fails 0
```

---

## Troubleshooting

### Issue: Health metrics not publishing

**Check:**
1. MQTT connection status
2. Health check interval (default 60s)
3. Serial monitor for errors
4. MQTT broker logs

### Issue: High memory usage

**Solution:**
- Reduce JSON payload size
- Increase health check interval
- Disable unused metrics

### Issue: MQTT flooding

**Solution:**
- Ensure retained messages are used
- Check publish-on-change logic
- Verify interval timing

---

## Testing Checklist

- [ ] Health status publishes every 60s
- [ ] WiFi RSSI updates correctly
- [ ] Heap memory reported accurately
- [ ] Uptime increments properly
- [ ] MQTT failures increment on disconnect
- [ ] Home Assistant discovers health sensors
- [ ] Retained messages persist across restarts
- [ ] Status changes to "degraded" when WiFi weak
- [ ] Status changes to "critical" when disconnected
- [ ] Memory usage stable over 24 hours

---

## Next Steps

1. Configure OpenHAB (see health_monitoring_proposal.md Section 5)
2. Create health dashboard in OpenHAB sitemap
3. Set up alerting rules
4. Monitor for 7 days to ensure stability

---

**Implementation Version:** 1.0  
**Last Updated:** March 31, 2026  
**Status:** Ready for Implementation
