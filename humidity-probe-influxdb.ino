/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout 
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
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

#define TIMESERVER "europe.pool.ntp.org"
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
HTTPClient http;
Adafruit_BME280 bme;            // I2C


void setup() {
    Serial.begin(115200);
    Serial.println(F("BME280 test"));

    // disable bluetooth to save power
    btStop();

    bool status;
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin(0x76);  
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
    WiFi.mode(WIFI_OFF);
}


void loop() { 
    long now = esp_log_timestamp();
    // obtain measurments from the sensor
    struct Measurement m;
    m.temperature = bme.readTemperature();
    m.humidity = bme.readHumidity();
    m.pressure = bme.readPressure() / 100.F;
    time(&m.time);
    Serial.println(m.time);

    queue.push_front(m);
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
  Serial.print("Transfering ");
  Serial.print(queue.size());
  Serial.println(" records...");
  http.begin(INFLUXDB_REST_SERVICE_URL);
  http.addHeader("Content-type", "text/plain");
  std::ostringstream oss;
  for (int i=queue.size()-1; i >=0; i--)  {
    struct Measurement m = queue[i];
    oss << "bme280,sensor=bme280,host=" << WiFi.macAddress() << " temperature=" << m.temperature << ",humidity=" << m.humidity
      << ",pressure="<< m.pressure << " " << m.time << "000000000\n"; // we need to add 9 zeros to the time, since InfluxDB expects ns.
  }
  Serial.println(oss.str().c_str());
  int httpResponseCode = http.POST(oss.str().c_str());
  if (httpResponseCode <= 0) {
    Serial.println("Error on sending POST");
  } else {
    // sending did succeed, so let's delete the queue
    queue.clear();
  }
  http.end();
  // disable Wifi
  WiFi.mode(WIFI_OFF);
}

/**
 * Connects to Wifi and returns true if a connection has
 * been successfully estabilshed.
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
