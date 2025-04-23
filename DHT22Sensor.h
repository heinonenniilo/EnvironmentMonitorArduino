#ifndef DHT22_SENSOR_H
#define DHT22_SENSOR_H

#include "Sensor.h"
#include <DHT.h>
#include <Arduino.h>

class DHT22Sensor : public Sensor {
private:
    int pin;
    DHT dht;    
    float temperatureTotal = 0.0;
    float humidityTotal = 0.0;
    int temperatureMeasureCount = 0;
    int humidityMeasureCount = 0;
    bool hasReadTemperature = 0;
    bool hasReadHumidity = 0;

public:
    DHT22Sensor(int sensorId, int pin);
    void begin() override;
    float readTemperature(bool average) override;
    float readHumidity(bool average) override;
    void resetAverages() override;
};

#endif