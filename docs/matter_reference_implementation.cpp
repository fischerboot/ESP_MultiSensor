/*
 * ESP32 Matter Multi-Sensor Reference Implementation
 * 
 * This shows how the sensor would work with Matter protocol instead of MQTT
 * Requires: ESP32-C3 or ESP32 (NOT compatible with ESP8266)
 * 
 * Key differences from MQTT version:
 * - No MQTT broker needed
 * - Direct communication with Matter controllers
 * - Built-in commissioning (QR code/setup code)
 * - Uses Matter clusters instead of MQTT topics
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

// Matter SDK includes (ESP-IDF based)
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <app_priv.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

// Sensor configuration
#define PIR_PIN 12
const float TEMP_THRESHOLD = 0.5;
const float PRESSURE_THRESHOLD = 1.0;
const float HUM_THRESHOLD = 2.0;
const float myAltitude = 138.4;

// Sensor hardware
Adafruit_BME280 bme;
Adafruit_BMP280 bmp;
enum SensorType { SENSOR_UNKNOWN = 0, SENSOR_BME280, SENSOR_BMP280 };
SensorType detectedSensor = SENSOR_UNKNOWN;

// Matter node and endpoint handles
uint16_t temperature_endpoint_id = 0;
uint16_t humidity_endpoint_id = 0;
uint16_t pressure_endpoint_id = 0;
uint16_t occupancy_endpoint_id = 0;

// Last sensor values
float lastTemp = NAN;
float lastHum = NAN;
float lastPressure = NAN;
int lastPirState = 0;

// Matter attribute update function
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type,
                                         uint16_t endpoint_id,
                                         uint32_t cluster_id,
                                         uint32_t attribute_id,
                                         esp_matter_attr_val_t *val,
                                         void *priv_data)
{
    // Handle attribute changes from Matter controller
    if (type == PRE_UPDATE) {
        Serial.printf("Matter attribute update: endpoint=%d cluster=%lu attr=%lu\n",
                     endpoint_id, cluster_id, attribute_id);
    }
    return ESP_OK;
}

// Matter identification callback (LED blink when identifying)
static esp_err_t app_identification_cb(identification::callback_type_t type,
                                       uint16_t endpoint_id,
                                       uint8_t effect_id,
                                       uint8_t effect_variant,
                                       void *priv_data)
{
    Serial.println("Matter identification requested");
    // Blink LED or provide other identification
    return ESP_OK;
}

// Initialize Matter stack
static void matter_init()
{
    // Create Matter node
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    
    if (!node) {
        Serial.println("Matter node creation failed");
        return;
    }

    // === Temperature Sensor Endpoint ===
    temperature_sensor::config_t temp_config;
    endpoint_t *temp_endpoint = temperature_sensor::create(node, &temp_config, ENDPOINT_FLAG_NONE, NULL);
    temperature_endpoint_id = endpoint::get_id(temp_endpoint);
    
    // Configure temperature measurement cluster
    cluster_t *temp_cluster = cluster::get(temp_endpoint, TemperatureMeasurement::Id);
    // Temperature is in 0.01°C units in Matter
    attribute::create(temp_cluster, TemperatureMeasurement::Attributes::MeasuredValue::Id, 
                     ATTRIBUTE_FLAG_NONE, esp_matter_int16(2000)); // 20.00°C default
    attribute::create(temp_cluster, TemperatureMeasurement::Attributes::MinMeasuredValue::Id,
                     ATTRIBUTE_FLAG_NONE, esp_matter_int16(-4000)); // -40.00°C
    attribute::create(temp_cluster, TemperatureMeasurement::Attributes::MaxMeasuredValue::Id,
                     ATTRIBUTE_FLAG_NONE, esp_matter_int16(8500)); // 85.00°C

    // === Humidity Sensor Endpoint (if BME280) ===
    if (detectedSensor == SENSOR_BME280) {
        humidity_sensor::config_t hum_config;
        endpoint_t *hum_endpoint = humidity_sensor::create(node, &hum_config, ENDPOINT_FLAG_NONE, NULL);
        humidity_endpoint_id = endpoint::get_id(hum_endpoint);
        
        cluster_t *hum_cluster = cluster::get(hum_endpoint, RelativeHumidityMeasurement::Id);
        // Humidity is in 0.01% units
        attribute::create(hum_cluster, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
                         ATTRIBUTE_FLAG_NONE, esp_matter_uint16(5000)); // 50.00%
        attribute::create(hum_cluster, RelativeHumidityMeasurement::Attributes::MinMeasuredValue::Id,
                         ATTRIBUTE_FLAG_NONE, esp_matter_uint16(0));
        attribute::create(hum_cluster, RelativeHumidityMeasurement::Attributes::MaxMeasuredValue::Id,
                         ATTRIBUTE_FLAG_NONE, esp_matter_uint16(10000)); // 100.00%
    }

    // === Pressure Sensor Endpoint ===
    // Note: Matter 1.2 has PressureMeasurement cluster
    pressure_sensor::config_t press_config;
    endpoint_t *press_endpoint = pressure_sensor::create(node, &press_config, ENDPOINT_FLAG_NONE, NULL);
    pressure_endpoint_id = endpoint::get_id(press_endpoint);
    
    cluster_t *press_cluster = cluster::get(press_endpoint, PressureMeasurement::Id);
    // Pressure in kPa (Matter standard)
    attribute::create(press_cluster, PressureMeasurement::Attributes::MeasuredValue::Id,
                     ATTRIBUTE_FLAG_NONE, esp_matter_int16(1013)); // 101.3 kPa = 1013 hPa

    // === Occupancy Sensor Endpoint (PIR) ===
    occupancy_sensor::config_t occ_config;
    endpoint_t *occ_endpoint = occupancy_sensor::create(node, &occ_config, ENDPOINT_FLAG_NONE, NULL);
    occupancy_endpoint_id = endpoint::get_id(occ_endpoint);
    
    cluster_t *occ_cluster = cluster::get(occ_endpoint, OccupancySensing::Id);
    attribute::create(occ_cluster, OccupancySensing::Attributes::Occupancy::Id,
                     ATTRIBUTE_FLAG_NONE, esp_matter_bitmap8(0)); // Not occupied
    attribute::create(occ_cluster, OccupancySensing::Attributes::OccupancySensorType::Id,
                     ATTRIBUTE_FLAG_NONE, esp_matter_enum8(0)); // PIR sensor

    // Start Matter stack
    esp_matter::start(app_attribute_update_cb);
    
    Serial.println("Matter device ready for commissioning");
    Serial.println("Scan QR code in your Matter controller app to add this device");
}

// Update Matter attribute values
void updateMatterTemperature(float temp)
{
    if (temperature_endpoint_id == 0) return;
    
    // Convert to Matter format (0.01°C units)
    int16_t matter_temp = (int16_t)(temp * 100);
    
    esp_matter_attr_val_t val = esp_matter_int16(matter_temp);
    attribute::update(temperature_endpoint_id, TemperatureMeasurement::Id,
                     TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
    
    Serial.printf("Matter: Temperature updated to %.2f°C\n", temp);
}

void updateMatterHumidity(float hum)
{
    if (humidity_endpoint_id == 0) return;
    
    // Convert to Matter format (0.01% units)
    uint16_t matter_hum = (uint16_t)(hum * 100);
    
    esp_matter_attr_val_t val = esp_matter_uint16(matter_hum);
    attribute::update(humidity_endpoint_id, RelativeHumidityMeasurement::Id,
                     RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val);
    
    Serial.printf("Matter: Humidity updated to %.2f%%\n", hum);
}

void updateMatterPressure(float pressure)
{
    if (pressure_endpoint_id == 0) return;
    
    // Convert hPa to kPa for Matter (1 hPa = 0.1 kPa)
    int16_t matter_pressure = (int16_t)(pressure / 10);
    
    esp_matter_attr_val_t val = esp_matter_int16(matter_pressure);
    attribute::update(pressure_endpoint_id, PressureMeasurement::Id,
                     PressureMeasurement::Attributes::MeasuredValue::Id, &val);
    
    Serial.printf("Matter: Pressure updated to %.2f hPa\n", pressure);
}

void updateMatterOccupancy(bool occupied)
{
    if (occupancy_endpoint_id == 0) return;
    
    // Bit 0 = occupied
    uint8_t occupancy_bitmap = occupied ? 0x01 : 0x00;
    
    esp_matter_attr_val_t val = esp_matter_bitmap8(occupancy_bitmap);
    attribute::update(occupancy_endpoint_id, OccupancySensing::Id,
                     OccupancySensing::Attributes::Occupancy::Id, &val);
    
    Serial.printf("Matter: Occupancy updated to %s\n", occupied ? "OCCUPIED" : "UNOCCUPIED");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Matter Multi-Sensor ===");
    
    pinMode(PIR_PIN, INPUT);
    
    // Initialize sensors
    bool bmeFound = bme.begin(0x76) || bme.begin(0x77);
    if (bmeFound) {
        detectedSensor = SENSOR_BME280;
        Serial.println("Detected BME280");
    } else {
        bool bmpFound = bmp.begin(0x76) || bmp.begin(0x77);
        if (bmpFound) {
            detectedSensor = SENSOR_BMP280;
            Serial.println("Detected BMP280");
        }
    }
    
    // Initialize Matter
    // Note: WiFi credentials are provisioned during Matter commissioning
    matter_init();
    
    Serial.println("\n** COMMISSIONING INSTRUCTIONS **");
    Serial.println("1. Open your Matter controller app (Apple Home, Google Home, etc.)");
    Serial.println("2. Select 'Add Device' or 'Add Matter Device'");
    Serial.println("3. Scan the QR code shown in serial console");
    Serial.println("4. Follow on-screen instructions");
    Serial.println("********************************\n");
}

void loop()
{
    static unsigned long lastSensorCheck = 0;
    const long SENSOR_READ_INTERVAL = 500;
    
    unsigned long now = millis();
    if (now - lastSensorCheck > SENSOR_READ_INTERVAL) {
        lastSensorCheck = now;
        
        // PIR sensor
        int pirState = digitalRead(PIR_PIN);
        if (pirState != lastPirState) {
            updateMatterOccupancy(pirState == HIGH);
            lastPirState = pirState;
        }
        
        // Temperature
        float temp = NAN;
        if (detectedSensor == SENSOR_BME280) {
            temp = bme.readTemperature();
        } else if (detectedSensor == SENSOR_BMP280) {
            temp = bmp.readTemperature();
        }
        
        if (!isnan(temp) && (isnan(lastTemp) || fabs(temp - lastTemp) > TEMP_THRESHOLD)) {
            updateMatterTemperature(temp);
            lastTemp = temp;
        }
        
        // Humidity (BME280 only)
        if (detectedSensor == SENSOR_BME280) {
            float hum = bme.readHumidity();
            if (!isnan(hum) && (isnan(lastHum) || fabs(hum - lastHum) > HUM_THRESHOLD)) {
                updateMatterHumidity(hum);
                lastHum = hum;
            }
        }
        
        // Pressure
        float pressure = NAN;
        if (detectedSensor == SENSOR_BME280) {
            pressure = bme.readPressure() / 100.0F;
            pressure = bme.seaLevelForAltitude(myAltitude, pressure);
        } else if (detectedSensor == SENSOR_BMP280) {
            pressure = bmp.readPressure() / 100.0F;
            pressure = bmp.seaLevelForAltitude(myAltitude, pressure);
        }
        
        if (!isnan(pressure) && (isnan(lastPressure) || fabs(pressure - lastPressure) > PRESSURE_THRESHOLD)) {
            updateMatterPressure(pressure);
            lastPressure = pressure;
        }
    }
    
    // Matter stack handles networking, commissioning, and communication
    // No manual MQTT publish calls needed!
    delay(10);
}

/*
 * platformio.ini configuration for Matter:
 * 
 * [env:esp32c3]
 * platform = espressif32
 * board = esp32-c3-devkitm-1
 * framework = arduino, espidf
 * 
 * lib_deps = 
 *     espressif/esp-matter@^1.2.0
 *     adafruit/Adafruit BME280 Library
 *     adafruit/Adafruit BMP280 Library
 *     adafruit/Adafruit Unified Sensor
 * 
 * build_flags = 
 *     -DCHIP_HAVE_CONFIG_H
 *     -DCONFIG_ENABLE_CHIP_SHELL=1
 * 
 * board_build.partitions = partitions_matter.csv
 * 
 * KEY DIFFERENCES FROM MQTT VERSION:
 * ===================================
 * 
 * 1. NO WiFi Credentials in Code
 *    - Provisioned during Matter commissioning
 *    - Controller (phone) shares WiFi credentials
 * 
 * 2. NO MQTT Broker
 *    - Direct device-to-controller communication
 *    - Uses Matter protocol over WiFi/Thread
 * 
 * 3. NO Manual Topic Publishing
 *    - attribute::update() notifies all subscribed controllers
 *    - Matter handles subscriptions automatically
 * 
 * 4. Built-in Discovery
 *    - Matter commissioning replaces MQTT discovery
 *    - Device advertises via mDNS/BLE
 * 
 * 5. Standardized Device Types
 *    - Temperature Sensor (Matter Device Type 0x0302)
 *    - Humidity Sensor (0x0307)
 *    - Occupancy Sensor (0x0107)
 *    - All controllers understand these types
 * 
 * 6. Security Built-in
 *    - Matter has end-to-end encryption
 *    - Device attestation certificates
 *    - Secure commissioning
 * 
 * 7. Multi-Controller Support
 *    - Works with Apple Home, Google Home, Alexa, Home Assistant simultaneously
 *    - All see the same device
 * 
 * 8. Larger Resource Requirements
 *    - ~1.5MB flash needed (vs ~400KB for MQTT version)
 *    - ~200KB RAM (vs ~40KB for MQTT version)
 *    - Cannot run on ESP8266
 */
