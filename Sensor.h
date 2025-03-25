#ifndef SENSOR_H
#define SENSOR_H
#include "SerialLogger.h"
#define DEBUG_SENSOR 0

class Sensor {
protected:
    int sensorId;  // Unique identifier for the sensor
    bool debug;
public:

    static constexpr float ERROR_UNSUPPORTED = -200.0f;  // Error for unsupported measurements
    static constexpr float ERROR_FAILED_READING = -999.0f;  // Error for failed readings

    Sensor(int id) : sensorId(id), debug(DEBUG_SENSOR) {}

    void debugPrint(const String& message) const 
    {
        if (debug) 
        {
          Logger.Info("[Sensor " + String(sensorId) + "] " + message);
        }
    }    
    virtual void begin() = 0;
    virtual float readTemperature(bool average) {return ERROR_UNSUPPORTED;};
    virtual float readHumidity(bool average) {return ERROR_UNSUPPORTED;};
    virtual float readLight(bool average) {return ERROR_UNSUPPORTED;};
    virtual int readMotion(bool aggregate) {return -1;}; 
    virtual void resetAverages();   
    virtual ~Sensor() = default;  // Virtual destructor for cleanup

    int getSensorId() const { return sensorId; }
};

#endif