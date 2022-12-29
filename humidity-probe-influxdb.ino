/***************************************************************************
  Transfers Sensor data from the BME280 to a time series database and caches 
  results, if the network is not available yet.

  Sensor library:
     https://bitbucket.org/christandlg/bmx280mi/

  Written by Albert Weichselbraun based on examples taken from various ESP32
  tutorials.
 ***************************************************************************/

#include <stdbool.h>
#include "time.h"
#include <HTTPClient.h>

#include "WiFi.h"
#include <Wire.h>
#include <BMx280I2C.h>

#include <string>
#include <sstream>

#include <esp_sleep.h>
#include <esp_log.h>    // support for timestamps


/* custom.h defines the following constants:
   - WIFI_SSID
   - WIFI_PASSWORD
   - INFLUXDB_REST_SERVICE_URL
*/
#include "custom.h"


#define WIFI_RECONNECT_TRY_MAX 10
#define WIFI_RECONNECT_DELAY_MS 500
#define MAX_READINGS 150                    // preliminary estimation of the maximum number of readings we can cache

// Define data record
RTC_DATA_ATTR int numRestart = 0;
RTC_DATA_ATTR int numMeasurement = 0;

struct Measurement {
  float temperature;
  float humidity;
  float pressure;
  byte address;
  time_t time;
};

RTC_DATA_ATTR Measurement measurements[MAX_READINGS];

HTTPClient http;                // HTTPClient user for transfering data to InfluxDB
BMx280I2C bmx280x76(0x76);      // Sensor at 0x76
BMx280I2C bmx280x77(0x77);      // Sensor at 0x77

RTC_DATA_ATTR bool hasSensor0x76;
RTC_DATA_ATTR bool hasSensor0x77;
RTC_DATA_ATTR bool initialized = false;

/**
 Tries to setup the sensor on the given I2C address.

   Returns:
     true if successful, otherwise false
*/
bool setupSensor(byte address) {
  Wire.beginTransmission(address);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  BMx280I2C *currentSensor;
  if (address == 0x76) {
    currentSensor = &bmx280x76;
  } else {
    currentSensor = &bmx280x77;
  }
  Serial.println("Detected BMx280 sensor at 0x" + String(address, HEX) + ".");
  return true;
}

/**
   Connects to Wifi and returns true if a connection has
   been successfully established.

   Returns:
     true if the setting up the Wifi succeeded, false otherwise.
*/
bool setupWifi() {
  int count = 0;
  WiFi.disconnect(true);
  delay(WIFI_RECONNECT_DELAY_MS);
  WiFi.mode(WIFI_AP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED && count < WIFI_RECONNECT_TRY_MAX) {
    Serial.print(".");
    Serial.flush();
    delay(WIFI_RECONNECT_DELAY_MS);
    count++;
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.flush();
  return WiFi.status() == WL_CONNECTED;
}

/**
 * Update time from timeserver.
 */
void updateTime() {
  Serial.println("Obtaining time from time server...");
  configTime(0, 0, TIMESERVER);
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time from timeserver. Waiting for retry.");
    delay(WIFI_RECONNECT_DELAY_MS);
  } 
}

/**
 * Perform the initial program setup.
 * - disable Bluetooth
 * - setup sensors
 * - setup WiFi and time
 */
void initialSetup() {
  // disable Bluetooth to save power
  btStop();
  
  // setup sensors
  Serial.println(F("Detecting sensor..."));
  Wire.begin();
  hasSensor0x76 = setupSensor(0x76);
  hasSensor0x77 = setupSensor(0x77);    
  if (hasSensor0x76 == false && hasSensor0x77 == false) {
    Serial.println("ERROR: No BMx280 sensor found.");
    while (1);
  }

  // setup wifi and time
  Serial.println("Connecting to Wifi...");
  while (!setupWifi());
  updateTime();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

/**
   Setup the humidity probe component.
*/
void setup() {
  numRestart++;
  Serial.begin(115200);

  if (!initialized) {
    initialSetup();
    initialized = true;
  } 
  
  //
  // force transfer of sensor data, if the touch fields have been 
  // touched.
  //
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TOUCHPAD) {
    Serial.println("Forcing write of sensor data....");
    transferSensorData();
  }
 
  //
  // obtain a measurement and enter deep sleep afterwards
  // 
  long now = esp_log_timestamp();
  // initialize the sensor
  if (hasSensor0x76 == true) {
    writeSensorMeasure(&bmx280x76, 0x76);
    transferSensorDataIfNecessary();
  }

  if (hasSensor0x77 == true) {
    writeSensorMeasure(&bmx280x77, 0x77);
    transferSensorDataIfNecessary();
  }

  Serial.println("Sleeping....");
  Serial.flush();
  esp_sleep_enable_touchpad_wakeup();
  esp_sleep_enable_timer_wakeup(60 * 1000 * 1000 - (esp_log_timestamp() - now) * 1000);
  esp_deep_sleep_start();
}

/**
   Adds the measurement for the given sensor and i2c address to the
   measurements array.

   Args:
 *  *currentSensor: pointer to the sensor to use for the measure
    address: i2c address of the sensor (0x76 or 0x77)

*/
void writeSensorMeasure(BMx280I2C *currentSensor, byte address) {

  Wire.begin();
  currentSensor->begin();
  //reset sensor to default parameters.
  currentSensor->resetToDefaults();

  //by default sensing is disabled and must be enabled by setting a non-zero
  //oversampling setting.
  //set an oversampling setting for pressure and temperature measurements.
  currentSensor->writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  currentSensor->writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

  //if sensor is a BME280, set an oversampling setting for humidity measurements.
  if (currentSensor->isBME280()) {
    currentSensor->writeOversamplingHumidity(BMx280MI::OSRS_H_x16);
  }

  if (!currentSensor->measure()) {
    Serial.println("Could not start measurement, is a measurement already running?");
    return;
  }
  do {
    delay(100);
  } while (!currentSensor->hasValue());

  // obtain measurments from the sensor
  delay(50);
  measurements[numMeasurement % MAX_READINGS].temperature = currentSensor->getTemperature();
  measurements[numMeasurement % MAX_READINGS].pressure = currentSensor->getPressure();
  if (currentSensor->isBME280()) {
    measurements[numMeasurement % MAX_READINGS].humidity = currentSensor->getHumidity();
  } else {
    measurements[numMeasurement % MAX_READINGS].humidity = -1;
  }
  measurements[numMeasurement % MAX_READINGS].address = address;
  time(&measurements[numMeasurement % MAX_READINGS].time);
  Serial.print("Recording measurement on ");
  Serial.print(ctime(&measurements[numMeasurement % MAX_READINGS].time));
  Serial.print("with temperature: ");
  Serial.print(measurements[numMeasurement % MAX_READINGS].temperature);
  Serial.print(", pressure: ");
  Serial.print(measurements[numMeasurement % MAX_READINGS].pressure);
  if (&measurements[numMeasurement % MAX_READINGS].humidity >= 0) {
    Serial.print(", humidity: ");
    Serial.print(measurements[numMeasurement % MAX_READINGS].humidity);
  }
  Serial.println();
  numMeasurement++;
}

/**
   Loop is never executed due to the use of deep sleep mode
*/
void loop() {}

/**
   Transfers the data to the server, if that's necessary, i.e., one of the following conditions
   is met:
   - we haven't yet collected more than DATA_TRANSFER_BATCH_SIZE data points and
   - neither the temperature nor the humidity threshold is exceeded
*/
void transferSensorDataIfNecessary() {
  if (numMeasurement < DATA_TRANSFER_BATCH_SIZE 
      && abs(measurements[0].temperature - measurements[(numMeasurement-1) % MAX_READINGS].temperature) <  TRANSFER_TEMP_DELTA_THRESHOLD
      && abs(measurements[0].humidity - measurements[(numMeasurement-1) % MAX_READINGS].humidity) < TRANSFER_HUMIDITY_DELTA_THRESHOLD) {
    return;  
  }
  transferSensorData();
}

/**
 * Transfer the sensor data.
 */
void transferSensorData() {
  // Connect to Wifi
  if (!setupWifi()) {
    Serial.println("Cannot setup Wifi. Skipping transfer.");
    return;
  }

  // update time from timeserver, if required
  if (numRestart >= TIMESERVER_UPDATE_MINUTES) {
    updateTime();
    numRestart = 0;
  }

  // Transfer data
  Serial.println("Transferring data to InfluxDB");
  transferBatch();
  Serial.println("\nCompleted :)");
  // disable Wifi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);  
}

/**
   Transfer all stored measurements to the time series database.
   This is necessary since the httpClient does not allow large posts.

   Returns:
    `true` if the transfer succeeds `else` otherwise.
*/
boolean transferBatch() {
  http.begin(INFLUXDB_REST_SERVICE_URL);
  http.addHeader("Content-type", "text/plain");
  std::ostringstream oss;
  
  for (int i=0; i < min(numMeasurement, MAX_READINGS); i++) {
    if (measurements[i].humidity >= 0) {
      oss << "bme280,sensor=bme280x0" << String(measurements[i].address, HEX) << ",host=" << WiFi.macAddress().c_str() << " temperature=" << measurements[i].temperature << ",humidity=" 
          << measurements[i].humidity << ",pressure=" << measurements[i].pressure  << " " << measurements[i].time << "000000000\n"; // we need to add 9 zeros to the time, since InfluxDB expects ns.
    } else {
      oss << "bme280,sensor=bmp280x0" << String(measurements[i].address, HEX) <<  ",host=" << WiFi.macAddress().c_str() << " temperature=" << measurements[i].temperature
          << ",pressure=" << measurements[i].pressure << " " << measurements[i].time << "000000000\n"; // we need to add 9 zeros to the time, since InfluxDB expects ns.
    }
  }
  Serial.println(oss.str().c_str());
  int httpResponseCode = http.POST(oss.str().c_str());
  http.end();

  if (httpResponseCode != 204) {
    Serial.println("Error on sending POST");
    return false;
  }
  // transfer completed; reset the number of measurements
  numMeasurement = 0;
  return true;
}
