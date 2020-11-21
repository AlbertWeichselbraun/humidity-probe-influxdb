/***************************************************************************
  Transfers Sensor data from the BME280 to InfluxDB and caches results,
  if the network is not available yet.

  Written by Albert Weichselbraun based on examples taken from various ESP32
  tutorials.
 ***************************************************************************/

#include <QList.h>
#include <WiFi.h>
#include "time.h"
#include <HTTPClient.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

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

#define TIMESERVER "pool.ntp.org"
#define SLEEP_TIME 60*1000          // time between measurements in ms
#define DATA_TRANSFER_BATCH_SIZE 10 // transfer after this number of items have been collected

// Define data record
struct Measurement {
  float temperature;
  float humidity;
  float pressure;
  time_t time;
};

QList<Measurement> queue;       // Queue
HTTPClient http;                // HTTPClient user for transfering data to InfluxDB
Adafruit_BME280 bme;            // I2C


/**
 * Setup the humidity probe component.
 */
void setup() {
    Serial.begin(115200);
    Serial.println(F("BME280 test"));

    // disable bluetooth to save power
    btStop();

    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    bool status = bme.begin(0x76);  
    if (status) {
        Serial.println("Detected BME280 sensor at 0x76.");
    } else {
        status = bme.begin(0x77);
        if (status) {
          Serial.println("Detected BME280 sensor at 0x77.");
        } else {
          Serial.println("ERROR: No BME280 sensor found. Please check the wiring!");
          while(1);
        }
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
 * Main program
 */
void loop() { 
    long now = esp_log_timestamp();
    // initialize the sensor
    bme.init();
    // obtain measurments from the sensor
    struct Measurement m;
    m.temperature = bme.readTemperature();
    m.humidity = bme.readHumidity();
    m.pressure = bme.readPressure() / 100.F;
    time(&m.time);

    queue.push_back(m);
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
      oss << "bme280,sensor=bme280,host=" << WiFi.macAddress().c_str() << " temperature=" << m.temperature << ",humidity=" << m.humidity
          << ",pressure="<< m.pressure << " " << m.time << "000000000\n"; // we need to add 9 zeros to the time, since InfluxDB expects ns.
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
