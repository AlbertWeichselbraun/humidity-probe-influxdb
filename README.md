# humidity-probe-influxdb
An ESP32 program for obtaining sensor data from an BME280 and saving it to an InfluxDB.

## Features

* Performs measurements every 60 seconds (customizable) and transfers them to an InfluxDB database.
* Automatically detects the sensor type (BME280 or BMP280) and the sensor's I2C bus ID.
* Minimize power consumption by
    - disabling Bluetooth
    - putting the ESP32 into light sleep mode between measurements 
    - a customizable number of results is cached prior to transferring them to InfluxDB.
* Reliability: the program caches measurements, if either the InfluxDB database or WiFi are not available.

## Required Libraries

* [BMx280i](https://bitbucket.org/christandlg/bmx280mi) sensor library from Gregor Christandl
* QList (for caching sensor data)

## Tutorial

The [following tutorial](https://semanticlab.net/linux/iot/esp32/bme280/sensor/influxdb/Record-Temperature-Humidity-Pressure-Monitoring-with-an-ESP32-a-BME280-and-InfluxDB/) summarizes the steps required for setting everyting up.
