/***************************************************************************
 *  Transfers Sensor data from the BMEx280 or AHT20 to a time series database 
 *  and  caches measurements, if the network is not available yet.
 *  
 *  Sensor libraries:     
 *  - BMx280: https://bitbucket.org/christandlg/bmx280mi/
 *  - AHT20: https://github.com/dvarrel/AHT20
 *    
 *  (C)opyrights 2020-2024 by Albert Weichselbraun.
 *  
 ***************************************************************************/

#include <stdbool.h>
#include "time.h"
#include <HTTPClient.h>

#include <WiFi.h>
#include <Wire.h>
#include <BMx280I2C.h>
#include <AHT20.h>

#include <string>
#include <sstream>

#include <esp_sleep.h>
#include <esp_log.h>    // support for timestamps
#include "esp_bt.h"
#include "esp_wifi.h"


/***************************************************************************  
 * custom.h defines the following constants:
 *   - WIFI_SSID
 *   - WIFI_PASSWORD
 *   - INFLUXDB_REST_SERVICE_URL
 *     
 *  and additional optional tuning parameters.
 ***************************************************************************/
#include "custom.h"

#define WIFI_RECONNECT_DELAY_MS 500
#define MAX_READINGS 300                    // preliminary estimation of the maximum number of readings we can cache; RTC memory amounts to 8192 bytes
#define HTTP_TRANSFER_BATCH_SIZE 10         // http transfer batch size

// Define data record
RTC_DATA_ATTR int numRestart = 0;
RTC_DATA_ATTR int numMeasurement = 0;
RTC_DATA_ATTR int skip = 0;                 // metrics to skip

__attribute__((packed))
struct Measurement {
  float temperature;
  float humidity;
  float pressure;
  byte address;  // 0x76, 0x77 or 0x20 (AHT20)
  time_t time;
};
__attribute__((packed))

RTC_DATA_ATTR Measurement measurements[MAX_READINGS];

HTTPClient http;                // HTTPClient user for transfering data to InfluxDB
BMx280I2C bmx280x76(0x76);      // Sensor at 0x76
BMx280I2C bmx280x77(0x77);      // Sensor at 0x77
AHT20 aht20;                    // AHT20 Sensor

RTC_DATA_ATTR bool hasSensor0x76;
RTC_DATA_ATTR bool hasSensor0x77;
RTC_DATA_ATTR bool hasSensorAHT;
RTC_DATA_ATTR bool firstRun = true;


/***************************************************************************
 * loop is never called (deep sleep)
 ***************************************************************************/
void loop() {}


/***************************************************************************
 *  Setup sensor on the given I2C address.
 *  
 *  Returns:     
 *    true if successful, otherwise false
 ***************************************************************************/
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

/***************************************************************************
 *  Connects to Wifi and returns true if a connection has 
 *  been successfully established.
 *   
 *  Returns:     
 *    true if the setting up the Wifi succeeded, false otherwise.
 ***************************************************************************/
bool setupWifi() {
  Serial.print("Connecting to Wifi.");
  if (WiFi.getMode() != WIFI_AP) {
    WiFi.mode(WIFI_AP);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 12) {
    count++;
    if (count % 4 == 0) {
      WiFi.disconnect(true);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    Serial.print(".");
    Serial.flush();
    delay(WIFI_RECONNECT_DELAY_MS);
  }
  Serial.println("\nWifi: IP Address: ");
  Serial.println(WiFi.localIP());
  return WiFi.status() == WL_CONNECTED;
}

/*************************************************************************** #
 *  
 *  Update time from timeserver.
 *  
 ***************************************************************************/
void updateTime() {
  Serial.println("Obtaining time from time server...");
  setenv("TZ", "UTC0", 1);
  configTime(0, 0, TIMESERVER);
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time from timeserver. Waiting for retry.");
    delay(WIFI_RECONNECT_DELAY_MS);
  } 
}

/***************************************************************************
 * Perform the initial program setup.
 * - disable Bluetooth
 * - setup sensors
 * - setup WiFi and time
 ***************************************************************************/
void initialSetup() {
  // disable Bluetooth to save power
  Serial.print("Measurments size...");
  Serial.println(sizeof(measurements));
  
  // setup sensors
  Serial.println(F("Detecting sensor..."));
  Wire.begin();
  hasSensor0x76 = setupSensor(0x76);
  hasSensor0x77 = setupSensor(0x77);
  if (aht20.begin() == true) {
    hasSensorAHT = true;
    Serial.println("Detected AHT 20 sensor.");
  } else {
    hasSensorAHT = false;
  }

  if (hasSensor0x76 == false && hasSensor0x77 == false && hasSensorAHT == false) {
    Serial.println("ERROR: No BMx280 or AHT20 sensor found.");
    while (1);
  }

  // setup wifi and time
  while (!setupWifi());
  updateTime();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

/***************************************************************************   
 *    
 * Main program called after boot and deep sleep.
 * 
 ***************************************************************************/
void setup() {
  numRestart++;
  esp_bt_controller_disable();
  Serial.begin(115200);

  if (firstRun) {
    initialSetup();
  } 
 
  //
  // obtain a measurement and enter deep sleep afterwards
  // 
  long then = esp_log_timestamp();
  // get the sensor data
  if (hasSensor0x76 == true) {
    addBmeSensorMeasure(&bmx280x76, 0x76);
  }
  if (hasSensor0x77 == true) {
    addBmeSensorMeasure(&bmx280x77, 0x77);
  }
  if (hasSensorAHT == true) {
    addAhtSensorMeasure();
  }
  // transfer data, if necessary.
  transferSensorDataIfNecessary();

  if (firstRun) {
    transferSensorData();
    firstRun = false;
  }

  // Disable Wi-Fi
  esp_wifi_stop();

  // go to sleep
  Serial.println("Sleeping....");
  Serial.flush();
  // sleep for SLEEP time adjusted for the time already spend since the last wakeup
  esp_sleep_enable_timer_wakeup(SLEEP_TIME * 1000 - (esp_log_timestamp() - then) * 1000);
  esp_deep_sleep_start();
}

/***************************************************************************
 *  Determine whether to skip the current reading.
 ***************************************************************************/
bool skipCurrentReading() {
  // skip sensor reading based on the available puffer size
  if (numMeasurement > MAX_READINGS * 0.8 && skip < 15) {
      skip += 1 ;
      return true;
  }
  if (numMeasurement > MAX_READINGS * 0.6 && skip < 7) {
      skip += 1 ;
      return true;
  }
  if (numMeasurement > MAX_READINGS * 0.4 && skip < 3) {
    skip += 1;
    return true;
  }
  if (numMeasurement > MAX_READINGS * 0.2 && skip < 1) {
    skip += 1;
    return true;
  }
  skip = 0;
  return false;
}


/***************************************************************************
 *  Adds the measurement for the given sensor and i2c address to the
 *  measurements array.
 * 
 *  Args:
 *  - currentSensor: pointer to the sensor to use for the measure
 *  - address: i2c address of the sensor (0x76 or 0x77)
 ***************************************************************************/
void addBmeSensorMeasure(BMx280I2C *currentSensor, byte address) {
  if (skipCurrentReading()) {
    return;
  }
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
  measurements[numMeasurement % MAX_READINGS].time = time(NULL);
  // time(&measurements[numMeasurement % MAX_READINGS].time);
  Serial.print("Recording measurement #");
  Serial.print(numMeasurement + 1);
  Serial.print(" at ");
  Serial.print(ctime(&measurements[numMeasurement % MAX_READINGS].time));
  Serial.print("with temperature: ");
  Serial.print(measurements[numMeasurement % MAX_READINGS].temperature);
  Serial.print(", pressure: ");
  Serial.print(measurements[numMeasurement % MAX_READINGS].pressure);
  if (&measurements[numMeasurement % MAX_READINGS].humidity >= 0) {
    
    Serial.print(measurements[numMeasurement % MAX_READINGS].humidity);
  }
  Serial.println();

}

/***************************************************************************
 *  Adds the measurement for the AHT sensor to the measurements array.
 ***************************************************************************/
void addAhtSensorMeasure() {
  Wire.begin();
  aht20.begin();
    do {
      delay(100);
  } while (aht20.available() == false);

  measurements[numMeasurement % MAX_READINGS].time = time(NULL);
  measurements[numMeasurement % MAX_READINGS].temperature = aht20.getTemperature();
  measurements[numMeasurement % MAX_READINGS].humidity = aht20.getHumidity();
  measurements[numMeasurement % MAX_READINGS].address = 0x20;
  measurements[numMeasurement % MAX_READINGS].pressure = -1;

  Serial.print("Recording measurement #");
  Serial.print(numMeasurement + 1);
  Serial.print(" at ");
  Serial.print(ctime(&measurements[numMeasurement % MAX_READINGS].time));
  Serial.print("with temperature: ");
  Serial.print(measurements[numMeasurement % MAX_READINGS].temperature);
  Serial.print(" and humidity: ");
  Serial.println(measurements[numMeasurement % MAX_READINGS].humidity);
  numMeasurement++;
}

/***************************************************************************   
 *    Transfers the data to the server, if that's necessary, i.e., one of 
 *    the following conditions is met:   
 *    - we haven't yet collected more than DATA_TRANSFER_BATCH_SIZE data 
 *      points, or
 *    - neither the temperature nor the humidity threshold is exceeded.
 ***************************************************************************/
void transferSensorDataIfNecessary() {
  if (numMeasurement < DATA_TRANSFER_BATCH_SIZE 
      && abs(measurements[0].temperature - measurements[(numMeasurement-1) % MAX_READINGS].temperature) <  TRANSFER_TEMP_DELTA_THRESHOLD
      && abs(measurements[0].humidity - measurements[(numMeasurement-1) % MAX_READINGS].humidity) < TRANSFER_HUMIDITY_DELTA_THRESHOLD) {
    return;  
  }
  transferSensorData();
}

/***************************************************************************
 *  Transfer the sensor data and update the time from timeserver, 
 *  if required.
 ***************************************************************************/
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
  transferBatch();
  // disable Wifi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);  
}

/***************************************************************************
 *  Transfer all stored measurements to the time series database.
 *  This is necessary since the httpClient does not allow large posts. 
 * 
 *  Use the InfluxDB line protocol
 *  see: https://docs.influxdata.com/influxdb/v2/reference/syntax/line-protocol/
 *  
 ***************************************************************************/
void transferBatch() {
  std::ostringstream oss;  
  Serial.print("Transfering a batch of "); 
  Serial.print(numMeasurement);
  Serial.println(" measurements.");
  for (int i=0; i < min(numMeasurement, MAX_READINGS); i++) {    
    oss << "sensor,host=" << WiFi.macAddress().c_str() << ",temperature=" << measurements[i].temperature;
    // add sensor specific tags and metrics
    if (measurements[i].address == 0x20) {
      oss << ",sensor=aht20,humidity=" << measurements[i].humidity;
    } else {
      oss << ",pressure=" << measurements[i].pressure;  
      if (measurements[i].humidity > 0) {
        oss << ",sensor=bme280x0" << String(measurements[i].address, HEX) << ",humidity=" << measurements[i].humidity;
      } else {
        oss << ",sensor=bmp280x0" << String(measurements[i].address, HEX);
      }
    }
    // add timestamp
    // we need to add nine zeros to the time, since InfluxDB expects ns.
    oss << " " << measurements[i].time << "000000000\n"; 
    if (((i+1) % HTTP_TRANSFER_BATCH_SIZE) == 0) {
      transferString(oss.str().c_str());
      oss.str("");
      oss.clear();
    }
  }
  transferString(oss.str().c_str());
  numMeasurement = 0;
}

/***************************************************************************
 *  Transfers the given string to the time series database.
 *  
 *  Returns:    
 *    true if the transfer succeeds else otherwise.
 ***************************************************************************/

boolean transferString(const char* c_str) {
  if (strlen(c_str) == 0) {
    return true;
  }
  Serial.println(c_str);
  http.begin(INFLUXDB_REST_SERVICE_URL);
  http.addHeader("Content-type", "text/plain");
  int httpResponseCode = http.POST(c_str);
  http.end();
  return httpResponseCode == 204;
}
