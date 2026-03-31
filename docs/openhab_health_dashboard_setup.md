# ESP MultiSensor Health Dashboard - OpenHAB Quick Setup Guide

## Overview

This guide helps you quickly set up a health monitoring dashboard in OpenHAB for your ESP MultiSensor.

---

## Prerequisites

- ✅ ESP MultiSensor with health monitoring implemented
- ✅ OpenHAB 3.x or 4.x installed
- ✅ MQTT Binding installed in OpenHAB
- ✅ MQTT Broker connection configured

---

## Step 1: Install MQTT Binding

If not already installed:

1. Open OpenHAB web interface
2. Navigate to **Settings → Bindings**
3. Search for "MQTT"
4. Click **Install** on "MQTT Binding"

---

## Step 2: Configure MQTT Broker Thing

If not already configured:

1. Go to **Settings → Things**
2. Click **+** to add a new Thing
3. Select **MQTT Binding**
4. Choose **MQTT Broker**
5. Configure:
   - **Broker Hostname/IP:** 192.168.2.127
   - **Port:** 1883
   - **Username:** (if required)
   - **Password:** (if required)
6. Click **Save**

---

## Step 3: Create MQTT Thing for ESP MultiSensor

### Option A: Using UI (Recommended for beginners)

1. Go to **Settings → Things**
2. Click **+** → **MQTT Binding** → **Generic MQTT Thing**
3. Name it: `ESP MultiSensor`
4. Select your MQTT broker
5. Click **Save**

### Option B: Using Configuration Files

Create file: `things/esp_multisensor.things`

```java
Thing mqtt:topic:esp_multisensor "ESP MultiSensor" (mqtt:broker:mybroker) {
    Channels:
        // Sensor channels (existing)
        Type number : temperature "Temperature" [ 
            stateTopic="ESP_Default/sensors/temp"
        ]
        Type number : pressure "Pressure" [ 
            stateTopic="ESP_Default/sensors/pressure"
        ]
        Type number : humidity "Humidity" [ 
            stateTopic="ESP_Default/sensors/bme/hum"
        ]
        Type string : pir "Motion" [ 
            stateTopic="ESP_Default/sensors/pir"
        ]
        
        // Health channels
        Type string : health_status "Health Status" [ 
            stateTopic="ESP_Default/health/status",
            transformationPattern="JSONPATH:$.status"
        ]
        Type number : health_rssi "WiFi Signal" [ 
            stateTopic="ESP_Default/health/rssi"
        ]
        Type number : health_heap "Free Memory" [ 
            stateTopic="ESP_Default/health/heap"
        ]
        Type number : health_uptime "Uptime" [ 
            stateTopic="ESP_Default/health/uptime"
        ]
        Type string : health_wifi "WiFi Connection" [ 
            stateTopic="ESP_Default/health/wifi"
        ]
        Type string : health_mqtt "MQTT Connection" [ 
            stateTopic="ESP_Default/health/mqtt"
        ]
        Type string : health_sensor "Sensor Type" [ 
            stateTopic="ESP_Default/health/sensor"
        ]
        Type number : health_mqtt_fails "MQTT Failures" [ 
            stateTopic="ESP_Default/health/mqtt_fails"
        ]
}
```

**Note:** Replace `mybroker` with your MQTT broker thing ID, and `ESP_Default` with your device prefix.

---

## Step 4: Create Items

Create file: `items/esp_multisensor.items`

```java
// ============================================
// ESP MultiSensor - Sensor Items
// ============================================
Number ESP_Temperature "Temperature [%.1f °C]" 
    <temperature> (gSensors, gChart)
    {channel="mqtt:topic:esp_multisensor:temperature"}

Number ESP_Pressure "Pressure [%.1f hPa]" 
    <pressure> (gSensors, gChart)
    {channel="mqtt:topic:esp_multisensor:pressure"}

Number ESP_Humidity "Humidity [%.1f %%]" 
    <humidity> (gSensors, gChart)
    {channel="mqtt:topic:esp_multisensor:humidity"}

String ESP_Motion "Motion [%s]" 
    <motion> (gSensors)
    {channel="mqtt:topic:esp_multisensor:pir"}

// ============================================
// ESP MultiSensor - Health Items
// ============================================
String ESP_Health "Health Status [MAP(health.map):%s]" 
    <status> (gHealth)
    {channel="mqtt:topic:esp_multisensor:health_status"}

Number ESP_RSSI "WiFi Signal [%d dBm]" 
    <network> (gHealth)
    {channel="mqtt:topic:esp_multisensor:health_rssi"}

Number ESP_Heap "Free Memory [%d B]" 
    <memory> (gHealth)
    {channel="mqtt:topic:esp_multisensor:health_heap"}

Number ESP_Uptime "Uptime [%d s]" 
    <time> (gHealth)
    {channel="mqtt:topic:esp_multisensor:health_uptime"}

String ESP_WiFi "WiFi [MAP(onoff.map):%s]" 
    <network> (gHealth)
    {channel="mqtt:topic:esp_multisensor:health_wifi"}

String ESP_MQTT "MQTT [MAP(onoff.map):%s]" 
    <network> (gHealth)
    {channel="mqtt:topic:esp_multisensor:health_mqtt"}

String ESP_Sensor "Sensor [%s]" 
    <sensor> (gHealth)
    {channel="mqtt:topic:esp_multisensor:health_sensor"}

Number ESP_MQTT_Fails "MQTT Failures [%d]" 
    <error> (gHealth)
    {channel="mqtt:topic:esp_multisensor:health_mqtt_fails"}
```

---

## Step 5: Create Groups

Add to `items/groups.items` (or create if doesn't exist):

```java
Group gSensors "Sensors"
Group gHealth "Device Health"
Group gChart "Charts"
```

---

## Step 6: Create Transformation Maps

### health.map

Create file: `transform/health.map`

```
healthy=✓ Healthy
degraded=⚠ Degraded
critical=✗ Critical
unknown=? Unknown
-=Unknown
```

### onoff.map

Create file: `transform/onoff.map`

```
online=Online
offline=Offline
true=Online
false=Offline
-=Unknown
```

---

## Step 7: Create Sitemap

Create file: `sitemaps/esp_health.sitemap`

```java
sitemap esp_health label="ESP MultiSensor Dashboard" {
    Frame label="Sensor Readings" {
        Text item=ESP_Temperature valuecolor=[<0="blue", <18="green", <25="orange", >=25="red"]
        Text item=ESP_Pressure
        Text item=ESP_Humidity valuecolor=[<30="red", <60="green", >=60="blue"]
        Text item=ESP_Motion
    }
    
    Frame label="Device Health" icon="status" {
        Text item=ESP_Health valuecolor=[
            ESP_Health=="healthy"="green",
            ESP_Health=="degraded"="orange",
            ESP_Health=="critical"="red"
        ] {
            Frame label="Connection Status" {
                Text item=ESP_WiFi valuecolor=[
                    ESP_WiFi=="online"="green",
                    ESP_WiFi=="offline"="red"
                ]
                Text item=ESP_RSSI valuecolor=[
                    ESP_RSSI>-70="green",
                    ESP_RSSI>-80="orange",
                    ESP_RSSI<=-80="red"
                ]
                Text item=ESP_MQTT valuecolor=[
                    ESP_MQTT=="online"="green",
                    ESP_MQTT=="offline"="red"
                ]
                Text item=ESP_MQTT_Fails visibility=[ESP_MQTT_Fails>0]
            }
            
            Frame label="System Status" {
                Text item=ESP_Uptime
                Text item=ESP_Heap valuecolor=[
                    ESP_Heap>=10000="green",
                    ESP_Heap>=5000="orange",
                    ESP_Heap<5000="red"
                ]
                Text item=ESP_Sensor
            }
        }
    }
    
    Frame label="Charts" {
        Chart item=gChart period=h refresh=30000
    }
}
```

---

## Step 8: Create Alert Rules

Create file: `rules/esp_health_alerts.rules`

```java
rule "ESP Health Status Changed"
when
    Item ESP_Health changed
then
    val status = ESP_Health.state.toString
    
    if (status == "critical") {
        logWarn("ESP_Health", "MultiSensor entered CRITICAL state!")
        // Uncomment to send notification
        // sendNotification("your@email.com", "ESP MultiSensor Critical!")
    } else if (status == "degraded") {
        logWarn("ESP_Health", "MultiSensor health DEGRADED")
    } else {
        logInfo("ESP_Health", "MultiSensor health: " + status)
    }
end

rule "ESP WiFi Disconnected"
when
    Item ESP_WiFi changed from "online" to "offline"
then
    logWarn("ESP_Health", "MultiSensor WiFi DISCONNECTED!")
    // Uncomment to send notification
    // sendNotification("your@email.com", "ESP MultiSensor WiFi Disconnected!")
end

rule "ESP MQTT Disconnected"
when
    Item ESP_MQTT changed from "online" to "offline"
then
    logWarn("ESP_Health", "MultiSensor MQTT DISCONNECTED!")
end

rule "ESP Low Memory Warning"
when
    Item ESP_Heap changed
then
    val heap = (ESP_Heap.state as Number).intValue
    
    if (heap < 5000) {
        logError("ESP_Health", "CRITICAL: Very low memory - " + heap + " bytes!")
    } else if (heap < 10000) {
        logWarn("ESP_Health", "WARNING: Low memory - " + heap + " bytes")
    }
end

rule "ESP High MQTT Failure Rate"
when
    Item ESP_MQTT_Fails changed
then
    val failures = (ESP_MQTT_Fails.state as Number).intValue
    
    if (failures > 10) {
        logWarn("ESP_Health", "High MQTT failure rate: " + failures + " failures")
    }
end
```

---

## Step 9: Advanced: Create HABPanel Widget

For a visual dashboard widget:

1. Open HABPanel
2. Create new dashboard: "Device Health"
3. Add Label widget:
   - Item: `ESP_Health`
   - Background color based on value
4. Add Gauge widgets for:
   - `ESP_RSSI` (range: -100 to -30)
   - `ESP_Heap` (range: 0 to 40000)
5. Add Uptime widget:
   - Type: Duration
   - Item: `ESP_Uptime`

---

## Step 10: Persistence (Optional)

For historical data and charts:

Create/edit file: `persistence/influxdb.persist` (or use your persistence service)

```java
Strategies {
    everyMinute : "0 * * * * ?"
    everyHour   : "0 0 * * * ?"
    everyDay    : "0 0 0 * * ?"
    default = everyChange
}

Items {
    // Persist sensor data
    gSensors* : strategy = everyChange, everyMinute
    
    // Persist health metrics
    ESP_RSSI : strategy = everyChange, everyMinute
    ESP_Heap : strategy = everyChange, everyMinute
    ESP_Uptime : strategy = everyHour
    ESP_Health : strategy = everyChange
    ESP_MQTT_Fails : strategy = everyChange
}
```

---

## Testing

### 1. Verify MQTT Messages

Use MQTT Explorer or terminal:

```bash
mosquitto_sub -h 192.168.2.127 -t "ESP_Default/health/#" -v
```

### 2. Check OpenHAB Log

```bash
tail -f /var/log/openhab/openhab.log
```

Look for health item updates.

### 3. Test Sitemap

1. Open OpenHAB: http://your-openhab-ip:8080
2. Navigate to your sitemap
3. Verify all health items display correctly
4. Check color coding works

### 4. Test Alerts

1. Disconnect ESP from WiFi
2. Check if alert rule triggers
3. Reconnect and verify recovery

---

## Troubleshooting

### Items show "NULL" or "-"

**Cause:** MQTT messages not received

**Solution:**
1. Check MQTT broker connection
2. Verify ESP is publishing
3. Check topic names match exactly
4. Ensure transformations are correct

### Charts not showing data

**Cause:** Persistence not configured

**Solution:**
1. Install persistence service (InfluxDB, RRD4j, etc.)
2. Configure persistence strategy
3. Wait for data to accumulate

### Colors not working

**Cause:** Value comparison issues

**Solution:**
- For strings: Use `==` (e.g., `ESP_Health=="healthy"`)
- For numbers: Use comparison operators (e.g., `ESP_RSSI>-70`)
- Check item data type matches conditions

### Transformation errors

**Cause:** Missing transformation files or incorrect syntax

**Solution:**
1. Verify files exist in `transform/` directory
2. Check file syntax
3. Restart OpenHAB after creating files

---

## File Structure Summary

```
openhab/
├── things/
│   └── esp_multisensor.things
├── items/
│   ├── esp_multisensor.items
│   └── groups.items
├── sitemaps/
│   └── esp_health.sitemap
├── rules/
│   └── esp_health_alerts.rules
├── persistence/
│   └── influxdb.persist
└── transform/
    ├── health.map
    └── onoff.map
```

---

## Customization Tips

### Adjust Update Intervals

In ESP code, change:
```cpp
#define HEALTH_CHECK_INTERVAL 60000  // 60 seconds
```

### Customize Thresholds

In OpenHAB rules, adjust warning levels:
```java
if (heap < 8000) {  // Changed from 5000
    logWarn(...)
}
```

### Add Email Notifications

Requires Mail Binding:

```java
sendMail("your@email.com", 
         "ESP Alert", 
         "Health status: " + ESP_Health.state)
```

### Add Telegram Notifications

Requires Telegram Binding:

```java
sendTelegram("bot_name", "⚠ ESP Health Alert!")
```

---

## Advanced Monitoring

### Create Derived Items

Calculate uptime in human-readable format:

```java
String ESP_Uptime_Formatted "Uptime" 
    <time> (gHealth)

rule "Format ESP Uptime"
when
    Item ESP_Uptime changed
then
    val seconds = (ESP_Uptime.state as Number).intValue
    val days = seconds / 86400
    val hours = (seconds % 86400) / 3600
    val minutes = (seconds % 3600) / 60
    
    val formatted = String.format("%dd %dh %dm", days, hours, minutes)
    ESP_Uptime_Formatted.postUpdate(formatted)
end
```

### WiFi Signal Quality Percentage

```java
Number ESP_WiFi_Quality "WiFi Quality [%d %%]" 
    <network> (gHealth)

rule "Calculate WiFi Quality"
when
    Item ESP_RSSI changed
then
    val rssi = (ESP_RSSI.state as Number).intValue
    // Convert dBm to percentage (rough estimate)
    var quality = 0
    if (rssi >= -50) quality = 100
    else if (rssi <= -100) quality = 0
    else quality = 2 * (rssi + 100)
    
    ESP_WiFi_Quality.postUpdate(quality)
end
```

---

## Next Steps

1. ✅ Verify all items are receiving data
2. ✅ Customize sitemap layout
3. ✅ Configure alerts/notifications
4. ✅ Set up persistence for historical data
5. ✅ Create HABPanel dashboard (optional)
6. ✅ Monitor for 24 hours to ensure stability

---

## Support Resources

- **OpenHAB Documentation:** https://www.openhab.org/docs/
- **MQTT Binding:** https://www.openhab.org/addons/bindings/mqtt/
- **Community Forum:** https://community.openhab.org/

---

**Quick Setup Version:** 1.0  
**Last Updated:** March 31, 2026  
**Compatible with:** OpenHAB 3.x, 4.x  
**Status:** Ready to Use
