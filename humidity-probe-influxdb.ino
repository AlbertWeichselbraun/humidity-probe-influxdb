/***************************************************************************
  Transfers Sensor data from the BME280 to InfluxDB and caches results,
  if the network is not available yet.

  Sensor library:
     https://bitbucket.org/christandlg/bmx280mi/

  Written by Albert Weichselbraun based on examples taken from various ESP32
  tutorials.
 ***************************************************************************/

/**
 * Todo:
 *  1. replace queue with struct
 *  2. deep sleep support
 *  3. connect to wifi only at request
 *  
 */

#include <stdbool.h>
#include <WiFi.h>
#include "time.h"
#include <HTTPClient.h>

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

#define TIMESERVER "pool.ntp.org"
#define SLEEP_TIME 60*1000                  // time between measurements in ms
#define DATA_TRANSFER_BATCH_SIZE 10         // transfer after this number of items have been collected
#define TRANSFER_TEMP_DELTA_THRESHOLD 1     // transfer, if the given temperature threshold between the first and current reading is exceeded
#define TRANSFER_HUMIDITY_DELTA_THRESHOLD 5 // transfer, if the given humidity threshold between the first and current readings is exceeded
#define MAX_READINGS 150                    // preliminary estimation of the maximum number of readings we can cache

// Define data record
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

bool firstStart = false;
bool hasSensor0x76;
bool hasSensor0x77;

/**
 ar  Tries to setup the sensor on the given I2C address.

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
  if (currentSensor->isBME280()) {
    Serial.println("Identified sensor as BME280.");
  } else {
    Serial.println("Identified sensor as BMP280.");
  }

  return true;
}

/**
   Setup the humidity probe component.
*/
void setup() {
  Serial.begin(115200);
  Serial.println(F("Detecting sensor..."));

  // disable bluetooth to save power
  btStop();

  // setup sensors
  Wire.begin();
  hasSensor0x76 = setupSensor(0x76);
  hasSensor0x77 = setupSensor(0x77);
  if (hasSensor0x76 == false && hasSensor0x77 == false) {
    Serial.println("ERROR: No BMx280 sensor found.");
    while (1);
  }

  Serial.println("Connecting to Wifi...");
  while (!connectWifi());
  Serial.println("Obtaining time from time server...");
  configTime(0, 0, TIMESERVER);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("ERROR: Failed to obtain time!");
    while (1);
  } else {
    Serial.println("Done...");
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  WiFi.mode(WIFI_OFF);
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
  measurements[numMeasurement % MAX_READINGS].temperature = currentSensor->getTemperature();
  measurements[numMeasurement % MAX_READINGS].pressure = currentSensor->getPressure();
  if (currentSensor->isBME280()) {
    measurements[numMeasurement % MAX_READINGS].humidity = currentSensor->getHumidity();
  } else {
    measurements[numMeasurement % MAX_READINGS].humidity = -1;
  }
  measurements[numMeasurement % MAX_READINGS].address = address;
  time(&measurements[numMeasurement % MAX_READINGS].time);
  Serial.print("Recording measurement with temperature: ");
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
   Main program
*/
void loop() {
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
  esp_sleep_enable_timer_wakeup(60 * 1000 * 1000 - (esp_log_timestamp() - now) * 1000);
  esp_light_sleep_start();
  Serial.println("Resuming...");
}

/**
   Transfers the data to the server.
*/
void transferSensorDataIfNecessary() {
  // return if no data transfer is necessary, i.e., 
  // - we haven't yet collected more than DATA_TRANSFER_BATCH_SIZE data points and
  // - neither the temperature nor the humidity threshold is exceeded
  if (numMeasurement < DATA_TRANSFER_BATCH_SIZE 
      && abs(measurements[0].temperature - measurements[numMeasurement % MAX_READINGS].temperature) <  TRANSFER_TEMP_DELTA_THRESHOLD
      && abs(measurements[0].humidity - measurements[numMeasurement % MAX_READINGS].humidity) < TRANSFER_HUMIDITY_DELTA_THRESHOLD) {
    return;  
  }

  // Connect to Wifi
  if (!connectWifi()) {
    return;
  }

  // Transfer data
  Serial.println("Transfering data to InfluxDB");
  transferBatch();
  Serial.println("\nCompleted :)");
  // disable Wifi
  WiFi.mode(WIFI_OFF);
}

/**
   Transfer a batch of size DATA_TRANSFER_BATCH_SIZE to InfluxDB.
   This is necessary since the httpClient does not allow large posts.

   Returns:
    `true` if the transfer succeeds `else` otherwise.
*/
boolean transferBatch() {
  http.begin(INFLUXDB_REST_SERVICE_URL);
  http.addHeader("Content-type", "text/plain");
  std::ostringstream oss;
  
  for (int i=1; i < min(numMeasurement, MAX_READINGS); i++) {
    if (measurements[i].humidity >= 0) {
      oss << "bme280,sensor=bme280x0" << String(measurements[i].address, HEX) << ",host=" << WiFi.macAddress().c_str() << " temperature=" << measurements[i].temperature << ",humidity=" 
          << measurements[i].humidity << ",pressure=" << measurements[i].pressure  << " " << measurements[i].time << "000000000\n"; // we need to add 9 zeros to the time, since InfluxDB expects ns.
    } else {
      oss << "bme280,sensor=bmp280x0" << String(measurements[i].address, HEX) <<  ",host=" << WiFi.macAddress().c_str() << " temperature=" << measurements[i].temperature
          << ",pressure=" << measurements[i].pressure << " " << measurements[i].time << "000000000\n"; // we need to add 9 zeros to the time, since InfluxDB expects ns.
    }
  }
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

/**
   Connects to Wifi and returns true if a connection has
   been successfully established.

   Returns:
     `true` if the connection succeeds `false` otherwise.
*/
bool connectWifi() {
  int count = 0;
  WiFi.mode(WIFI_AP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED && count < WIFI_RECONNECT_TRY_MAX) {
    delay(WIFI_RECONNECT_DELAY_MS);
    count++;
  }
  return WiFi.status() == WL_CONNECTED;
}
