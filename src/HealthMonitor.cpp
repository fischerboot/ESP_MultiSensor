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
    // Use char buffer to avoid String fragmentation
    // Reserve space for worst-case JSON size (~400 bytes)
    char json[450];
    time_t now = time(nullptr);
    
    // Build JSON in one shot using snprintf
    snprintf(json, sizeof(json),
        "{\"status\":\"%s\","
        "\"timestamp\":%ld,"
        "\"last_update_ms\":%lu,"
        "\"wifi\":{\"connected\":%s,\"rssi\":%d,\"ip\":\"%s\"},"
        "\"mqtt\":{\"connected\":%s,\"failures\":%u,\"reconnects\":%u},"
        "\"system\":{\"uptime\":%u,\"heap\":%u,\"version\":\"%s\"},"
        "\"sensor\":{\"type\":\"%s\",\"operational\":%s,\"last_read\":%u},"
        "\"ntp\":{\"synced\":%s,\"timestamp\":%ld}}",
        getStatusString().c_str(),
        (long)now,
        millis(),
        _data.wifi.connected ? "true" : "false",
        _data.wifi.rssi,
        _data.wifi.ip.c_str(),
        _data.mqtt.connected ? "true" : "false",
        _data.mqtt.failures,
        _data.mqtt.reconnects,
        _data.system.uptime,
        _data.system.freeHeap,
        _data.system.version.c_str(),
        _data.sensor.type.c_str(),
        _data.sensor.operational ? "true" : "false",
        _data.sensor.lastReadAge,
        _data.ntp.synced ? "true" : "false",
        (long)_data.ntp.timestamp
    );
    
    return String(json);
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
