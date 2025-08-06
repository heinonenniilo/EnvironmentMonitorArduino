#include "RuuviTagScanner.h"
#include <NimBLEDevice.h>
#include <string>

RuuviTagScanner::RuuviTagScanner(const std::vector<RuuviTagSensor*>& registeredSensors) 
{
    sensors = registeredSensors;
}

void RuuviTagScanner::onResult(const NimBLEAdvertisedDevice* advertisedDevice)
{
  if (!advertisedDevice->haveManufacturerData())
  {
    return;
  }
  for (auto sensor : sensors) {
    if (sensor != nullptr) {
      sensor->handleMeasurement(advertisedDevice);
    }
  }
}


