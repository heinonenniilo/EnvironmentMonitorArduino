#ifndef TMP36_SENSOR_H
#define TMP36_SENSOR_H

#include "Sensor.h"
#include <Arduino.h>

class TMP36Sensor : public Sensor {
private:
    int pin;  // Pin connected to the TMP36 sensor
    unsigned long temperatureTotal = 0;
    int measureCount = 0;

public:
    TMP36Sensor(int sensorId, int pin);
    void begin() override;
    float readTemperature(bool average) override;
    void resetAverages() override;
};

#endif