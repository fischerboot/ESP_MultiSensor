# Health Monitoring Watchdog - Quick Reference Guide

## Overview

The health monitoring system now includes automatic device availability tracking using MQTT Last Will and Testament (LWT) and timestamps. This allows you to detect:

1. **Immediate disconnects** - Device goes offline (power loss, crash, network failure)
2. **Stale data** - Health updates stop being published
3. **Connection state** - Real-time availability status

---

## How It Works

### 1. MQTT Last Will and Testament (LWT)

When the ESP connects to MQTT, it sets up a "Last Will":
- **Will Topic:** `{device_prefix}/availability`
- **Will Message:** `"offline"`
- **Retained:** Yes (persists across broker restarts)

**What happens:**
- ✅ **On connect:** Device publishes `"online"` to availability topic
- ❌ **On unexpected disconnect:** Broker automatically publishes `"offline"` (the "will")
- 🔄 **On graceful disconnect:** Same as unexpected (broker can't distinguish)

### 2. Timestamp in Health Data

Every health status JSON includes:
```json
{
  "status": "healthy",
  "timestamp": 1743465600,      // Unix epoch timestamp
  "last_update_ms": 3600000,    // Milliseconds since ESP boot
  "wifi": {...},
  "mqtt": {...}
}
```

### 3. Home Assistant Integration

All entities include `availability_topic`. Home Assistant will:
- Show all entities as **"unavailable"** when `availability` = `"offline"`
- Show entities as **"available"** when `availability` = `"online"`
- Automatically display unavailable entities in grey

---

## OpenHAB Configuration

### Thing Configuration

Add the availability channel:

```java
Thing mqtt:topic:esp_multisensor "ESP MultiSensor" (mqtt:broker:mybroker) {
    Channels:
        // Existing channels...
        
        // Availability monitoring
        Type string : availability "Device Availability" [ 
            stateTopic="ESP_Default/availability"
        ]
        
        // Health status with timestamp
        Type string : health_status_raw "Health Status (Raw)" [ 
            stateTopic="ESP_Default/health/status"
        ]
}
```

### Item Configuration

```java
// Availability Item
String ESP_Availability "Availability [%s]" 
    <network> (gHealth)
    {channel="mqtt:topic:esp_multisensor:availability"}

// Health Status with timestamp extraction
String ESP_Health_Raw "Health Status Raw" 
    {channel="mqtt:topic:esp_multisensor:health_status_raw"}

Number ESP_Health_Timestamp "Last Health Update [%1$tY-%1$tm-%1$td %1$tH:%1$tM:%1$tS]" 
    <time> (gHealth)

String ESP_Health_Age "Health Age [%s]" 
    <time> (gHealth)
```

### Rules for Monitoring

#### Rule 1: Detect Device Offline

```java
rule "ESP Device Went Offline"
when
    Item ESP_Availability changed to "offline"
then
    logWarn("ESP_Health", "ESP MultiSensor went OFFLINE!")
    sendNotification("admin@example.com", "⚠️ ESP MultiSensor Offline!")
    
    // Optional: Send to other notification services
    // sendTelegram("bot_name", "ESP MultiSensor is OFFLINE")
end

rule "ESP Device Came Online"
when
    Item ESP_Availability changed to "online"
then
    logInfo("ESP_Health", "ESP MultiSensor came back ONLINE")
    sendNotification("admin@example.com", "✅ ESP MultiSensor Online")
end
```

#### Rule 2: Extract and Monitor Timestamp

```java
rule "Update Health Timestamp"
when
    Item ESP_Health_Raw changed
then
    // Parse JSON to extract timestamp
    val json = ESP_Health_Raw.state.toString
    
    // Simple regex extraction (or use JSONPath transformation)
    val timestampMatch = java.util.regex.Pattern.compile("\"timestamp\":(\\d+)").matcher(json)
    
    if (timestampMatch.find()) {
        val timestamp = Long.parseLong(timestampMatch.group(1))
        ESP_Health_Timestamp.postUpdate(new DateTimeType(new Date(timestamp * 1000)))
    }
end
```

#### Rule 3: Detect Stale Health Data

```java
rule "Detect Stale Health Data"
when
    Time cron "0 */5 * * * ?" // Every 5 minutes
then
    // Check if device is online but data is stale
    if (ESP_Availability.state == "online") {
        val lastUpdate = ESP_Health_Timestamp.state as DateTimeType
        val now = new DateTimeType()
        val ageMinutes = (now.zonedDateTime.toEpochSecond - lastUpdate.zonedDateTime.toEpochSecond) / 60
        
        // Update age display
        if (ageMinutes < 2) {
            ESP_Health_Age.postUpdate("Fresh")
        } else if (ageMinutes < 5) {
            ESP_Health_Age.postUpdate(ageMinutes + " min ago")
        } else {
            ESP_Health_Age.postUpdate(ageMinutes + " min ago [STALE]")
        }
        
        // Alert on stale data (no updates for 5+ minutes)
        if (ageMinutes > 5) {
            logWarn("ESP_Health", "Health data is STALE: " + ageMinutes + " minutes old")
            sendNotification("admin@example.com", 
                "⚠️ ESP health data stale (" + ageMinutes + " min)")
        }
    }
end
```

#### Rule 4: Combined Health Monitor

```java
rule "ESP Health Monitor - Combined"
when
    Time cron "0 */2 * * * ?" // Every 2 minutes
then
    val availability = ESP_Availability.state.toString
    val healthStatus = ESP_Health.state.toString
    
    var alertLevel = ""
    var message = ""
    
    // Check availability first
    if (availability == "offline") {
        alertLevel = "CRITICAL"
        message = "Device OFFLINE"
    } else {
        // Check health status
        if (healthStatus == "critical") {
            alertLevel = "CRITICAL"
            message = "Device online but health CRITICAL"
        } else if (healthStatus == "degraded") {
            alertLevel = "WARNING"
            message = "Device health DEGRADED"
        }
        
        // Check data staleness
        val lastUpdate = ESP_Health_Timestamp.state as DateTimeType
        val now = new DateTimeType()
        val ageMinutes = (now.zonedDateTime.toEpochSecond - lastUpdate.zonedDateTime.toEpochSecond) / 60
        
        if (ageMinutes > 5) {
            alertLevel = "WARNING"
            message = message + " (data stale: " + ageMinutes + " min)"
        }
    }
    
    // Send alert if needed
    if (alertLevel == "CRITICAL") {
        logError("ESP_Health", message)
        sendNotification("admin@example.com", "🔴 ESP: " + message)
    } else if (alertLevel == "WARNING") {
        logWarn("ESP_Health", message)
    }
end
```

---

## Sitemap with Availability

```java
sitemap esp_health label="ESP MultiSensor Dashboard" {
    Frame label="Device Status" {
        // Availability indicator
        Text item=ESP_Availability valuecolor=[
            ESP_Availability=="online"="green",
            ESP_Availability=="offline"="red"
        ] {
            Frame label="Connection Details" {
                Text item=ESP_RSSI
                Text item=ESP_WiFi
                Text item=ESP_MQTT
            }
        }
        
        // Health status with age
        Text item=ESP_Health valuecolor=[
            ESP_Health=="healthy"="green",
            ESP_Health=="degraded"="orange",
            ESP_Health=="critical"="red"
        ]
        
        Text item=ESP_Health_Age
        Text item=ESP_Health_Timestamp
    }
    
    // ... rest of sitemap
}
```

---

## Testing the Watchdog

### Test 1: Power Loss Simulation

1. Disconnect power from ESP
2. Wait 5-30 seconds
3. **Expected:** `availability` topic changes to `"offline"` immediately
4. **Expected:** OpenHAB rule triggers "Device Went Offline" alert

### Test 2: Network Disconnect

1. Disable WiFi on ESP's router/AP
2. **Expected:** Within 30 seconds, `availability` → `"offline"`
3. Restore network
4. **Expected:** ESP reconnects, `availability` → `"online"`

### Test 3: Stale Data Detection

1. Comment out `publishHealthMetrics()` in main.cpp
2. Upload code
3. Wait 5+ minutes
4. **Expected:** "Stale Health Data" rule triggers alert
5. Restore code and upload

### Test 4: MQTT Broker Restart

1. Restart MQTT broker
2. **Expected:** LWT retained, shows `"offline"` briefly
3. **Expected:** ESP reconnects and publishes `"online"`

---

## Monitoring Dashboard Ideas

### Option 1: Status Badge

```java
String ESP_Status_Badge "ESP Status" 
    <status>

rule "Update Status Badge"
when
    Item ESP_Availability changed or
    Item ESP_Health changed
then
    val avail = ESP_Availability.state.toString
    val health = ESP_Health.state.toString
    
    var badge = ""
    
    if (avail == "offline") {
        badge = "🔴 OFFLINE"
    } else if (health == "critical") {
        badge = "🟠 CRITICAL"
    } else if (health == "degraded") {
        badge = "🟡 DEGRADED"
    } else {
        badge = "🟢 HEALTHY"
    }
    
    ESP_Status_Badge.postUpdate(badge)
end
```

### Option 2: Uptime Tracking

Track continuous uptime (resets on disconnect):

```java
Number ESP_Continuous_Uptime "Continuous Uptime [%d s]"
DateTime ESP_Last_Online "Last Online [%1$tY-%1$tm-%1$td %1$tH:%1$tM]"

rule "Track Continuous Uptime"
when
    Item ESP_Availability changed to "online"
then
    ESP_Last_Online.postUpdate(new DateTimeType())
end

rule "Calculate Continuous Uptime"
when
    Time cron "0 * * * * ?" // Every minute
then
    if (ESP_Availability.state == "online") {
        val lastOnline = (ESP_Last_Online.state as DateTimeType).zonedDateTime
        val now = ZonedDateTime.now()
        val uptimeSeconds = java.time.Duration.between(lastOnline, now).getSeconds()
        ESP_Continuous_Uptime.postUpdate(uptimeSeconds)
    } else {
        ESP_Continuous_Uptime.postUpdate(0)
    }
end
```

---

## Home Assistant Example

Home Assistant automatically uses the availability topic. You can also add automation:

```yaml
automation:
  - alias: "ESP MultiSensor Offline Alert"
    trigger:
      - platform: state
        entity_id: binary_sensor.esp_default_availability
        to: "offline"
    action:
      - service: notify.mobile_app
        data:
          title: "ESP Sensor Offline"
          message: "ESP MultiSensor has gone offline!"
          data:
            priority: high
            
  - alias: "ESP Health Critical"
    trigger:
      - platform: state
        entity_id: sensor.esp_default_health
        to: "critical"
    action:
      - service: notify.mobile_app
        data:
          title: "ESP Health Critical"
          message: "ESP MultiSensor health is critical!"
```

---

## Troubleshooting

### Availability stuck on "offline"

**Cause:** MQTT broker didn't receive birth message

**Solution:**
1. Check ESP serial monitor for "Published availability: online"
2. Verify MQTT broker is receiving messages: `mosquitto_sub -h broker -t "+/availability" -v`
3. Clear retained message: `mosquitto_pub -h broker -t "ESP_Default/availability" -r -n`
4. Restart ESP

### Stale data alerts when device is fine

**Cause:** Clock skew or timestamp parsing error

**Solution:**
1. Verify NTP is working on ESP
2. Check OpenHAB system time is correct
3. Verify timestamp extraction in rule is working
4. Increase stale threshold from 5 to 10 minutes

### LWT not working

**Cause:** LWT not set correctly in MQTT connect

**Solution:**
1. Verify connect call includes LWT parameters
2. Check MQTT broker logs
3. Test manually: `mosquitto_pub -h broker -t "test/lwt" -m "offline" -r --will-topic "test/lwt" --will-payload "offline"`

---

## Summary

✅ **What you get:**
- Immediate offline detection (via MQTT LWT)
- Stale data detection (via timestamps)
- Availability in Home Assistant
- Flexible OpenHAB monitoring rules

🎯 **Best practices:**
- Monitor both `availability` topic AND timestamp age
- Set reasonable thresholds (5-10 minutes for stale data)
- Use graduated alerts (warning → critical)
- Test regularly (power cycle, network disconnect)

---

**Last Updated:** March 31, 2026  
**Version:** 1.0  
**Compatible with:** ESP8266, MQTT, OpenHAB 3.x/4.x, Home Assistant
