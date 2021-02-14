/***************************************************************************
  Transfers Sensor data from the BME280 to InfluxDB and caches results,
  if the network is not available yet.

  Sensor library:
     https://bitbucket.org/christandlg/bmx280mi/

  Written by Albert Weichselbraun based on examples taken from various ESP32
  tutorials.
 ***************************************************************************/

#include <stdbool.h> 
#include <QList.h>
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
 * - WIFI_SSID
 * - WIFI_PASSWORD
 * - INFLUXDB_REST_SERVICE_URL
 */
#include "custom.h"     

#define WIFI_RECONNECT_TRY_MAX 10
#define WIFI_RECONNECT_DELAY_MS 500

// Define data record
struct Measurement {
  float temperature;
  float humidity;
  float pressure;
  byte address;
  time_t time;
};

QList<Measurement> queue;       // Queue
HTTPClient http;                // HTTPClient user for transfering data to InfluxDB
BMx280I2C bmx280x76(0x76);      // Sensor at 0x76
BMx280I2C bmx280x77(0x77);      // Sensor at 0x77

bool hasSensor0x76;
bool hasSensor0x77;

/**
 * Tries to setup the sensor on the given I2C address.
 * 
 * Returns:
 *   true if successful, otherwise false
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
 * Setup the humidity probe component.
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
    if(!getLocalTime(&timeinfo)){
      Serial.println("ERROR: Failed to obtain time!");
      while(1);
    } else {
      Serial.println("Done...");
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    WiFi.mode(WIFI_OFF);
}

/**
 * Updates the measure for the given sensor and i2c address
 * 
 * Args:
 *  *m: pointer to the Measurement to update
 *  *currentSensor: pointer to the sensor to use for the measure
 *  address: i2c address of the sensor (0x76 or 0x77)
 *  
 */
void writeSensorMeasure(struct Measurement *m, BMx280I2C *currentSensor, byte address) {

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
    m->temperature = currentSensor->getTemperature();
    m->pressure = currentSensor->getPressure();
    if (currentSensor->isBME280()) {
      m->humidity = currentSensor->getHumidity();
    } else {
      m->humidity = -1;
    }
    m->address = address;
    time(&m->time);
    Serial.print("Recording measurement with temperature: ");
    Serial.print(m->temperature);
    Serial.print(", pressure: ");
    Serial.print(m->pressure);
    if (m->humidity >=0) {
      Serial.print(", humidity: ");
      Serial.print(m->humidity);
    }
    Serial.println();
}

/**
 * Main program
 */
void loop() { 
    long now = esp_log_timestamp();
    // initialize the sensor
    if (hasSensor0x76 == true) {
      struct Measurement m;
      writeSensorMeasure(&m, &bmx280x76, 0x76);
      queue.push_back(m);
    }

    if (hasSensor0x77 == true) {
      struct Measurement m;
      writeSensorMeasure(&m, &bmx280x77, 0x77);
      queue.push_back(m);
    }

    if (queue.size() % DATA_TRANSFER_BATCH_SIZE == 0) {
      transferData();
    }
    Serial.println("Sleeping....");
    esp_sleep_enable_timer_wakeup(60*1000*1000 - (esp_log_timestamp()-now)*1000);
    esp_light_sleep_start();
    Serial.println("Resuming...");
}

/** 
 * Transfers the data to the server.
 */
void transferData() {
  // Connect to Wifi
  if (!connectWifi()) {
    return;
  }

  // Transfer data
  Serial.println("Transfering data to InfluxDB");
  while (queue.size() > 0 && transferBatch()) {
    Serial.print(".");
  }
  Serial.println("\nCompleted :)");
  // disable Wifi
  WiFi.mode(WIFI_OFF);
}

/**
 * Transfer a batch of size DATA_TRANSFER_BATCH_SIZE to InfluxDB.
 * This is necessary since the httpClient does not allow large posts.
 * 
 * Returns:
 *  `true` if the transfer succeeds `else` otherwise.
 */
boolean transferBatch() {
    int endIndex = queue.size()-1;
    if (endIndex < 0) {
      return false;
    }
    int startIndex = endIndex - DATA_TRANSFER_BATCH_SIZE;
    if (startIndex < 0) {
      startIndex = 0;
    }
    
    http.begin(INFLUXDB_REST_SERVICE_URL);
    http.addHeader("Content-type", "text/plain");
    std::ostringstream oss;
    for (int i=endIndex; i>=startIndex; i--) {
      struct Measurement m = queue[i];
      if (m.humidity >= 0) {
      oss << "bme280,sensor=bme280x0" << String(m.address, HEX) << ",host=" << WiFi.macAddress().c_str() << " temperature=" << m.temperature << ",humidity=" << m.humidity
          << ",pressure="<< m.pressure  << " " << m.time << "000000000\n"; // we need to add 9 zeros to the time, since InfluxDB expects ns.
      } else {
      oss << "bme280,sensor=bmp280x0" << String(m.address, HEX) <<  ",host=" << WiFi.macAddress().c_str() << " temperature=" << m.temperature 
          << ",pressure="<< m.pressure << " " << m.time << "000000000\n"; // we need to add 9 zeros to the time, since InfluxDB expects ns.
      }
    }
    int httpResponseCode = http.POST(oss.str().c_str());
    http.end(); 

    if (httpResponseCode != 204) {
      Serial.println("Error on sending POST");
      return false;
    } 
    
    // clear the transferred items from the queue
    for (int i=endIndex; i>=startIndex; i--) {
      queue.clear(i);
    }
    return true;
}

/**
 * Connects to Wifi and returns true if a connection has
 * been successfully established.
 * 
 * Returns:
 *   `true` if the connection succeeds `false` otherwise.
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
