#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include "Sensor.h"
#include <Arduino.h>
#include <initializer_list>

class MotionSensor : public Sensor {
private:
  const int* inPins;
  const int* outPins;
  size_t numInPins;
  size_t numOutPins;
  bool motionDetected = 0;

public:
    MotionSensor(int sensorId, std::initializer_list<int> inPinsList, std::initializer_list<int> outPinsList);

    void begin() override;
    bool readMotion(bool aggregate) override;
    void resetAverages() override;
};

#endif


// virtual bool readMotion(bool aggregate) {return ERROR_UNSUPPORTED;};

/*
        : inPins(inPinsList.begin()), 
          outPins(outPinsList.begin()), 
          numInPins(inPinsList.size()),
          numOutPins(outPinsList.size()) {}
*/