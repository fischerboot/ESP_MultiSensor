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
