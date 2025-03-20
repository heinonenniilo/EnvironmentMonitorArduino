#include "MotionSensor.h"
#include "SerialLogger.h"

MotionSensor::MotionSensor(int sensorId, std::initializer_list<int> inPinsList, std::initializer_list<int> outPinsList) : Sensor(sensorId), 
  inPins(inPinsList.begin()), 
  outPins(outPinsList.begin()),
  numInPins(inPinsList.size()),
  numOutPins(outPinsList.size())  
{}

void MotionSensor::begin() 
{
  debugPrint("Starting...");
  for (size_t i = 0; i < numInPins; i++) {
    Serial.print(inPins[i]);
    Serial.print(" ");
  }  
}

bool MotionSensor::readMotion(bool aggregate)
{
  if (aggregate) 
  {
    return motionDetected;
  }
  return 0;
}

void MotionSensor::resetAverages() 
{
  motionDetected = 0;
}

