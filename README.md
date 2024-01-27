# humidity-probe-influxdb
An ESP32 program that collects sensor data from up to two BMx280 or AHT20 sensors and saves it to a time series database such as InfluxDB or Victoria Metrics.

## Supported sensors

- AHT20 (temperature and humidity)
- BME280 (temperature, pressure and humidity)
- BMP280 (temperature and pressure)

## Features

* Performs measurements every 60 seconds (customizable)
* Intelligent sync: send data to the time series database after 
  - a given number (default: 20) of measurements has been taken.
  - a significant change in temperature (default: +/- 1°C) or humidity (default: +/- 2%) has been detected.
  - decreases sampling rate, if the cache runs full.
* Automatically detects the sensor type (BME280 or BMP280) and the sensor's I2C bus ID.
* Minimize power consumption by
    - disabling Bluetooth
    - putting the ESP32 into deep sleep mode between measurements 
    - caching a customizable number of results prior to transferring them to the time series database.
* Synchronize the time with a time server after a specified amount of minutes.
* Reliability: the program caches measurements, if either the InfluxDB database or WiFi are not available.
* Supported time series databases: 
  - InfluxDB
  - Victoria Metrics

## Required Libraries

* [AHT20](https://github.com/dvarrel/AHT20)
* [BMx280i](https://bitbucket.org/christandlg/bmx280mi) sensor library from Gregor Christandl

## Tutorial

The [following tutorial](https://semanticlab.net/linux/iot/esp32/bme280/sensor/influxdb/Record-Temperature-Humidity-Pressure-Monitoring-with-an-ESP32-a-BME280-and-InfluxDB/) summarizes the steps required for setting everyting up.
