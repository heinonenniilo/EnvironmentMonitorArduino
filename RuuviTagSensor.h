#ifndef RUUVIS_H
#define RUUVIS_H

#include <NimBLEAdvertisedDevice.h>
#include <Arduino.h>
#include "Sensor.h"

class RuuviTagSensor : public Sensor {
  private:
    std::string allowedMac;
    float temperatureTotal = 0.0;
    float humidityTotal = 0.0;
    float pressureTotal = 0.0;
    float lastTemperature = Sensor::ERROR_FAILED_READING;
    float lastHumidity = Sensor::ERROR_FAILED_READING;
    float lastPressure = Sensor::ERROR_FAILED_READING;
    int measureCount = 0;    
  public:
    RuuviTagSensor(const std::string& macToAllow, int sensorId);
    void begin() override;
    void handleMeasurement(const NimBLEAdvertisedDevice* advertisedDevice);
    float readTemperature(bool average) override;
    float readHumidity(bool average) override;
    float readPressure(bool average) override;
    void resetAverages() override;
};

#endif