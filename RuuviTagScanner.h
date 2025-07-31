#ifndef RUUVI_H
#define RUUVI_H

#include <NimBLEAdvertisedDevice.h>
#include <Arduino.h>
#include "RuuviTagSensor.h"
#include <vector>

class RuuviTagScanner : public NimBLEScanCallbacks {
  private:
    std::vector<RuuviTagSensor*> sensors;
    int measureCount = 0;    
  public:
    RuuviTagScanner(const std::vector<RuuviTagSensor*>& registeredSensors);
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override;
};

#endif