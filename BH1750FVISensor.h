#ifndef H1750FVI_SENSOR_H
#define H1750FVI_SENSOR_H

#include "Sensor.h"
#include <Arduino.h>
#include <BH1750.h>
#include <Wire.h>

class BH1750FVISensor : public Sensor {
private:
    float lightTotal = 0;
    int measureCount = 0;
    BH1750 lightMeter;
    bool hasReadLight = 0;

public:
    BH1750FVISensor(int sensorId);
    void begin() override;
    float readLight(bool average) override;
    void resetAverages() override;
};

#endif

