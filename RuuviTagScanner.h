#ifndef RUUVI_H
#define RUUVI_H


#include <BLEAdvertisedDevice.h>
#include <Arduino.h>
#include "Sensor.h"

class RuuviTagScanner : public BLEAdvertisedDeviceCallbacks, public Sensor {
  private:
    String allowedMac;
    float temperatureTotal = 0.0;
    float humidityTotal = 0.0;
    float lastTemperature = Sensor::ERROR_FAILED_READING;
    float lastHumidity = Sensor::ERROR_FAILED_READING;
    int measureCount = 0;    
  public:
    RuuviTagScanner(const String& macToAllow, int sensorId) : allowedMac(macToAllow), Sensor(sensorId) {}
    void onResult(BLEAdvertisedDevice advertisedDevice) override;
    void begin() override;
    float readTemperature(bool average) override;
    float readHumidity(bool average) override;
    void resetAverages() override;
};

#endif