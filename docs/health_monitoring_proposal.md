# ESP MultiSensor Health Monitoring System
## Proposal & Implementation Plan

**Version:** 1.0  
**Date:** March 31, 2026  
**Target Platform:** ESP8266 with MQTT & OpenHAB Integration

---

## 1. Executive Summary

This proposal introduces a comprehensive health monitoring system for the ESP MultiSensor device that enables real-time monitoring of device health, connectivity, and operational status via MQTT. The system will integrate seamlessly with OpenHAB for centralized health dashboard creation.

---

## 2. Health Metrics Overview

### 2.1 Core Health Metrics

The following metrics will be monitored and published:

| Metric | Description | Unit/Format | MQTT Topic |
|--------|-------------|-------------|------------|
| **WiFi RSSI** | Signal strength indicator | dBm | `{prefix}/health/rssi` |
| **WiFi Status** | Connection state | online/offline | `{prefix}/health/wifi` |
| **MQTT Status** | Broker connection state | online/offline | `{prefix}/health/mqtt` |
| **Free Heap** | Available RAM | Bytes | `{prefix}/health/heap` |
| **Uptime** | Device uptime | Seconds | `{prefix}/health/uptime` |
| **Firmware Version** | Current firmware | String | `{prefix}/health/version` |
| **Sensor Status** | Sensor availability | BME280/BMP280/NONE | `{prefix}/health/sensor` |
| **NTP Sync** | Time synchronization state | synced/unsynced | `{prefix}/health/ntp` |
| **Last Sensor Read** | Time since last sensor reading | Seconds | `{prefix}/health/last_read` |
| **MQTT Failures** | Failed publish count | Counter | `{prefix}/health/mqtt_fails` |
| **Overall Health** | Composite health status | healthy/degraded/critical | `{prefix}/health/status` |

### 2.2 Health Status Levels

- **Healthy**: All systems operational
  - WiFi connected (RSSI > -80 dBm)
  - MQTT connected
  - Sensor operational
  - Heap > 10KB
  - NTP synchronized

- **Degraded**: Partially operational
  - WiFi weak (RSSI < -80 dBm)
  - MQTT experiencing failures
  - Heap low (< 10KB)
  - Sensor intermittent issues

- **Critical**: Major issues detected
  - WiFi disconnected
  - MQTT disconnected
  - Sensor failed
  - Heap critically low (< 5KB)
  - System unresponsive

---

## 3. MQTT Integration

### 3.1 Topic Structure

```
{device_prefix}/health/
├── status              # Overall health (JSON payload)
├── rssi                # WiFi signal strength
├── wifi                # WiFi connection status
├── mqtt                # MQTT connection status
├── heap                # Free heap memory
├── uptime              # Uptime in seconds
├── version             # Firmware version
├── sensor              # Sensor type/status
├── ntp                 # NTP sync status
├── last_read           # Seconds since last sensor read
└── mqtt_fails          # Failed publish count
```

### 3.2 Health Status JSON Payload

The main status topic will publish a comprehensive JSON payload:

```json
{
  "status": "healthy",
  "wifi": {
    "connected": true,
    "rssi": -65,
    "ip": "192.168.2.75"
  },
  "mqtt": {
    "connected": true,
    "failures": 0
  },
  "system": {
    "uptime": 86400,
    "heap": 25600,
    "version": "20260331v1.15"
  },
  "sensor": {
    "type": "BME280",
    "status": "operational",
    "last_read": 5
  },
  "ntp": {
    "synced": true,
    "timestamp": 1743465600
  }
}
```

### 3.3 Publishing Strategy

- **Periodic Health Updates**: Every 60 seconds (configurable)
- **Event-Driven Updates**: Immediate publish on status changes
  - WiFi connection/disconnection
  - MQTT connection/disconnection
  - Sensor failures
  - Critical heap threshold crossed
- **Lightweight Updates**: Individual metrics published for efficiency
- **Retained Messages**: Health status retained for immediate visibility

---

## 4. Home Assistant Auto-Discovery

Health sensors will be auto-discovered by Home Assistant:

```json
// WiFi RSSI Sensor
{
  "name": "WiFi RSSI {prefix}",
  "device_class": "signal_strength",
  "state_topic": "{prefix}/health/rssi",
  "unit_of_measurement": "dBm",
  "entity_category": "diagnostic"
}

// Uptime Sensor
{
  "name": "Uptime {prefix}",
  "device_class": "duration",
  "state_topic": "{prefix}/health/uptime",
  "unit_of_measurement": "s",
  "entity_category": "diagnostic"
}

// Memory Sensor
{
  "name": "Free Heap {prefix}",
  "state_topic": "{prefix}/health/heap",
  "unit_of_measurement": "B",
  "entity_category": "diagnostic"
}

// Overall Health Sensor
{
  "name": "Health Status {prefix}",
  "state_topic": "{prefix}/health/status",
  "value_template": "{{ value_json.status }}",
  "json_attributes_topic": "{prefix}/health/status",
  "entity_category": "diagnostic"
}
```

---

## 5. OpenHAB Integration

### 5.1 MQTT Things Configuration

```conf
Thing mqtt:topic:esp_multisensor "ESP MultiSensor" (mqtt:broker:mybroker) {
    Channels:
        // Existing sensor channels...
        
        // Health channels
        Type number : health_rssi "WiFi Signal" [ 
            stateTopic="ESP_Default/health/rssi",
            transformationPattern="JS:rssi.js"
        ]
        Type number : health_uptime "Uptime" [ 
            stateTopic="ESP_Default/health/uptime"
        ]
        Type number : health_heap "Free Memory" [ 
            stateTopic="ESP_Default/health/heap"
        ]
        Type string : health_status "Health Status" [ 
            stateTopic="ESP_Default/health/status",
            transformationPattern="JSONPATH:$.status"
        ]
        Type string : health_wifi "WiFi Status" [ 
            stateTopic="ESP_Default/health/wifi"
        ]
        Type string : health_mqtt "MQTT Status" [ 
            stateTopic="ESP_Default/health/mqtt"
        ]
        Type string : health_sensor "Sensor Status" [ 
            stateTopic="ESP_Default/health/sensor"
        ]
        Type number : health_mqtt_fails "MQTT Failures" [ 
            stateTopic="ESP_Default/health/mqtt_fails"
        ]
}
```

### 5.2 Items Configuration

```conf
// Health Items
Number ESP_RSSI "WiFi Signal [%d dBm]" 
    <network> (gHealth) {channel="mqtt:topic:esp_multisensor:health_rssi"}

Number ESP_Uptime "Uptime [%d s]" 
    <time> (gHealth) {channel="mqtt:topic:esp_multisensor:health_uptime"}

Number ESP_Heap "Free Memory [%d B]" 
    <memory> (gHealth) {channel="mqtt:topic:esp_multisensor:health_heap"}

String ESP_Health "Health Status [%s]" 
    <status> (gHealth) {channel="mqtt:topic:esp_multisensor:health_status"}

String ESP_WiFi "WiFi [%s]" 
    <network> (gHealth) {channel="mqtt:topic:esp_multisensor:health_wifi"}

String ESP_MQTT "MQTT [%s]" 
    <network> (gHealth) {channel="mqtt:topic:esp_multisensor:health_mqtt"}

String ESP_Sensor "Sensor [%s]" 
    <sensor> (gHealth) {channel="mqtt:topic:esp_multisensor:health_sensor"}

Number ESP_MQTT_Fails "MQTT Failures [%d]" 
    <error> (gHealth) {channel="mqtt:topic:esp_multisensor:health_mqtt_fails"}
```

### 5.3 Sitemap for Health Dashboard

```conf
sitemap health label="Sensor Health Dashboard" {
    Frame label="ESP MultiSensor Health" {
        Text item=ESP_Health valuecolor=[
            ESP_Health=="healthy"="green",
            ESP_Health=="degraded"="orange",
            ESP_Health=="critical"="red"
        ]
        Text item=ESP_RSSI valuecolor=[
            ESP_RSSI>-70="green",
            ESP_RSSI>-80="orange",
            ESP_RSSI<=-80="red"
        ]
        Text item=ESP_Uptime
        Text item=ESP_Heap valuecolor=[
            ESP_Heap>=10000="green",
            ESP_Heap>=5000="orange",
            ESP_Heap<5000="red"
        ]
        Text item=ESP_WiFi valuecolor=[
            ESP_WiFi=="online"="green",
            ESP_WiFi=="offline"="red"
        ]
        Text item=ESP_MQTT valuecolor=[
            ESP_MQTT=="online"="green",
            ESP_MQTT=="offline"="red"
        ]
        Text item=ESP_Sensor
        Text item=ESP_MQTT_Fails
    }
}
```

### 5.4 Rules for Alerting

```java
rule "ESP MultiSensor Health Alert"
when
    Item ESP_Health changed
then
    if (ESP_Health.state == "critical") {
        sendNotification("admin@example.com", "ESP MultiSensor Critical Health!")
        logWarn("ESP_Health", "MultiSensor entered critical state")
    } else if (ESP_Health.state == "degraded") {
        logWarn("ESP_Health", "MultiSensor health degraded")
    }
end

rule "ESP WiFi Disconnected"
when
    Item ESP_WiFi changed to "offline"
then
    sendNotification("admin@example.com", "ESP MultiSensor WiFi Disconnected!")
end

rule "ESP Low Memory"
when
    Item ESP_Heap changed
then
    if ((ESP_Heap.state as Number) < 5000) {
        logWarn("ESP_Health", "Critical: Low memory - " + ESP_Heap.state + " bytes")
    }
end
```

---

## 6. Implementation Plan

### Phase 1: Core Health Monitoring (Week 1)

**Tasks:**
1. Create `HealthMonitor` class structure
2. Implement metric collection functions:
   - WiFi metrics (RSSI, status, IP)
   - System metrics (heap, uptime)
   - MQTT metrics (connection, failures)
3. Add health status calculation logic
4. Implement periodic health check (60s interval)

**Files to Create/Modify:**
- Create: `include/HealthMonitor.h`
- Create: `src/HealthMonitor.cpp`
- Modify: `src/main.cpp`

**Estimated Effort:** 4-6 hours

---

### Phase 2: MQTT Health Publishing (Week 1-2)

**Tasks:**
1. Define health MQTT topics
2. Implement health message publishing functions
3. Add JSON payload generation for status overview
4. Implement event-driven health updates
5. Add retained message support

**Files to Modify:**
- `src/main.cpp`
- `src/HealthMonitor.cpp`

**Estimated Effort:** 3-4 hours

---

### Phase 3: Home Assistant Discovery (Week 2)

**Tasks:**
1. Extend discovery function to include health sensors
2. Add diagnostic entity category
3. Test discovery messages
4. Verify Home Assistant integration

**Files to Modify:**
- `src/main.cpp` (publishHomeAssistantDiscovery function)

**Estimated Effort:** 2-3 hours

---

### Phase 4: Configuration & Tuning (Week 2)

**Tasks:**
1. Add configurable health check interval
2. Add configurable thresholds (RSSI, heap, etc.)
3. Add enable/disable health monitoring option
4. Store health config in EEPROM
5. Add telnet commands for health inspection

**Files to Modify:**
- `src/main.cpp`
- `src/HealthMonitor.cpp`

**Estimated Effort:** 2-3 hours

---

### Phase 5: Testing & Documentation (Week 3)

**Tasks:**
1. Test all health metrics
2. Test event-driven updates
3. Verify MQTT publishing
4. Test OpenHAB integration
5. Create user documentation
6. Add code comments

**Deliverables:**
- Tested health monitoring system
- User guide for OpenHAB setup
- Code documentation

**Estimated Effort:** 3-4 hours

---

## 7. Technical Implementation Details

### 7.1 HealthMonitor Class Structure

```cpp
class HealthMonitor {
public:
    enum HealthStatus {
        HEALTHY,
        DEGRADED,
        CRITICAL
    };

    struct WiFiHealth {
        bool connected;
        int rssi;
        String ip;
    };

    struct MQTTHealth {
        bool connected;
        uint32_t failures;
    };

    struct SystemHealth {
        uint32_t uptime;
        uint32_t freeHeap;
        String version;
    };

    struct SensorHealth {
        String type;
        bool operational;
        uint32_t lastReadSec;
    };

    struct NTPHealth {
        bool synced;
        time_t timestamp;
    };

    HealthMonitor();
    void begin();
    void update();
    HealthStatus getStatus();
    String getStatusJSON();
    
    // Individual metric getters
    WiFiHealth getWiFiHealth();
    MQTTHealth getMQTTHealth();
    SystemHealth getSystemHealth();
    SensorHealth getSensorHealth();
    NTPHealth getNTPHealth();
    
    void recordMQTTFailure();
    void recordSensorRead();
    
private:
    HealthStatus calculateStatus();
    unsigned long lastUpdate;
    unsigned long lastSensorRead;
    uint32_t mqttFailures;
};
```

### 7.2 Integration Points in main.cpp

```cpp
// Global instance
HealthMonitor healthMonitor;

// In setup()
healthMonitor.begin();

// In loop()
healthMonitor.update();

// In handleMqttConnection() on success
// Reset failure count on successful connection

// In sensor reading code
healthMonitor.recordSensorRead();

// On MQTT publish failure
healthMonitor.recordMQTTFailure();
```

### 7.3 Memory Considerations

- Health monitor overhead: ~500 bytes RAM
- JSON payload: ~300 bytes temporary
- Total impact: < 1KB additional RAM usage
- Safe for ESP8266 with typical 40-50KB free heap

### 7.4 Performance Impact

- Health check cycle: < 50ms
- MQTT publish overhead: ~200ms per update
- Network impact: ~1KB/minute additional traffic
- Minimal impact on sensor reading cycles

---

## 8. Configuration Options

### 8.1 Compile-Time Options

```cpp
// Enable/disable health monitoring
#define ENABLE_HEALTH_MONITORING 1

// Health check interval (milliseconds)
#define HEALTH_CHECK_INTERVAL 60000  // 60 seconds

// Thresholds
#define HEALTH_RSSI_GOOD -70
#define HEALTH_RSSI_POOR -80
#define HEALTH_HEAP_LOW 10000
#define HEALTH_HEAP_CRITICAL 5000

// Enable individual health metrics
#define HEALTH_PUBLISH_RSSI 1
#define HEALTH_PUBLISH_HEAP 1
#define HEALTH_PUBLISH_UPTIME 1
#define HEALTH_PUBLISH_JSON 1
```

### 8.2 Runtime Configuration

Add to WiFiManager parameters:
- Health check interval
- Enable/disable health monitoring
- Health thresholds

Store in EEPROM with existing config.

---

## 9. Future Enhancements

### 9.1 Short-term (3-6 months)
- Add boot reason tracking (reset cause)
- Add crash counter and last crash time
- Add WiFi reconnection count
- Add sensor reading error rate

### 9.2 Long-term (6-12 months)
- Historical health metrics (trend data)
- Predictive health alerts (ML-based)
- Remote diagnostics commands via MQTT
- Health report export (CSV/JSON)
- Integration with Grafana for visualization

---

## 10. Success Criteria

The health monitoring system will be considered successful when:

1. ✅ All defined metrics are accurately collected and published
2. ✅ Health status correctly reflects device state
3. ✅ MQTT publishing is reliable with < 1% message loss
4. ✅ OpenHAB dashboard displays all health metrics
5. ✅ Event-driven alerts trigger within 5 seconds
6. ✅ Memory overhead stays under 1KB
7. ✅ No impact on sensor reading intervals
8. ✅ System stable for 7+ days continuous operation

---

## 11. Risk Assessment

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Memory exhaustion | High | Low | Monitor heap, add safety limits |
| MQTT flooding | Medium | Low | Rate limiting, publish only on change |
| WiFi instability | Medium | Medium | Retry logic, exponential backoff |
| Sensor interference | Low | Low | Run health checks between sensor reads |
| Performance degradation | Medium | Low | Asynchronous updates, timing analysis |

---

## 12. Resources Required

### Hardware
- ✅ Existing ESP8266 MultiSensor (no additional hardware)

### Software/Tools
- ✅ PlatformIO (already in use)
- ✅ MQTT Broker (already configured)
- ✅ OpenHAB (to be configured)
- Optional: MQTT Explorer for debugging

### Documentation
- This proposal document
- Implementation guide (to be created)
- User setup guide (to be created)

---

## 13. Conclusion

This health monitoring system provides comprehensive visibility into the ESP MultiSensor's operational status, enabling proactive maintenance and reliability improvements. The implementation is designed to be lightweight, non-invasive, and fully compatible with existing functionality while adding significant value for system monitoring and diagnostics.

The phased implementation approach minimizes risk and allows for iterative testing and refinement. Integration with OpenHAB will provide a centralized health dashboard for all IoT devices in your home automation system.

**Recommendation:** Proceed with implementation starting with Phase 1.

---

## Appendix A: Testing Checklist

- [ ] WiFi RSSI reading accuracy
- [ ] Heap memory reporting accuracy
- [ ] Uptime calculation correctness
- [ ] MQTT connection status accuracy
- [ ] Sensor status reporting
- [ ] NTP sync status detection
- [ ] Health status calculation logic
- [ ] JSON payload formatting
- [ ] MQTT topic publishing
- [ ] Home Assistant discovery
- [ ] OpenHAB integration
- [ ] Event-driven updates
- [ ] Retained message persistence
- [ ] Memory leak testing (24h run)
- [ ] Performance impact analysis
- [ ] WiFi reconnection handling
- [ ] MQTT reconnection handling
- [ ] Sensor failure simulation
- [ ] Low memory simulation
- [ ] Power cycle recovery

---

**Document Version:** 1.0  
**Last Updated:** March 31, 2026  
**Author:** ESP MultiSensor Health Monitoring Project  
**Status:** Proposal - Pending Approval
