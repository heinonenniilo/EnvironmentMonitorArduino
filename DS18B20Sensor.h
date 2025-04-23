#ifndef DS18B20_SENSOR_H
#define DS18B20_SENSOR_H

#include "Sensor.h"
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class DS18B20Sensor : public Sensor {
private:
    int pin; 
    float temperatureTotal = 0;
    int measureCount = 0;
    OneWire oneWire;           // OneWire instance
    DallasTemperature sensors; // DallasTemperature instance for managing DS18B20
    bool hasReadTemperature = 0;

public:
    DS18B20Sensor(int sensorId, int pin);
    void begin() override;
    float readTemperature(bool average) override;
    void resetAverages() override;
};

#endif