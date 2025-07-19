#ifndef RUUVI_H
#define RUUVI_H


#include <BLEAdvertisedDevice.h>
#include <Arduino.h>

class RuuviTagScanner : public BLEAdvertisedDeviceCallbacks {
public:
  void onResult(BLEAdvertisedDevice advertisedDevice) override;
};

#endif