#include "MotionSensor.h"
#include "SerialLogger.h"

MotionSensor::MotionSensor(int sensorId, std::initializer_list<uint8_t> inPinsList, std::initializer_list<uint8_t> outPinsList) : Sensor(sensorId), 
  inPins(inPinsList), 
  outPins(outPinsList)
{}

void MotionSensor::begin() 
{
  debugPrint("Starting...");
  for (size_t i = 0; i < inPins.size(); i++) 
  {
    debugPrint("Index: " + String(i));
    debugPrint("Setting pin: " + String(inPins[i]) + " as input for motion control");
    pinMode(inPins[i], INPUT);
  }
  for (size_t i = 0; i < outPins.size(); i++) 
  {
    debugPrint("Index: " + String(i));
    debugPrint("Setting pin: " + String(outPins[i]) + " as output for motion control");
    pinMode(outPins[i], OUTPUT);
  }
  debugPrint("Motion control inited");      
}

int MotionSensor::readMotion(bool aggregate)
{
  if (aggregate) 
  {
    return motionDetected;
  }
  bool motionNow = readCurrentMotion();
  debugPrint("Motion detected: " + String(motionNow));
  if (motionNow) 
  {
    motionDetected = 1;
  }
  return motionNow;
}

void MotionSensor::checkOutputs()
{
  bool motionNow = readMotion(0);
  if (motionControlStatus != MotionControl) 
  {
    debugPrint("Motion control status is: " + String(motionControlStatus));
    return;
  }
  unsigned long millisNow = millis();
  unsigned long diff = millisNow - lastMotionOnMillis;
  debugPrint("Diff: " + String(diff));
  if (lastMotionOnMillis != 0 && ((millisNow - lastMotionOnMillis) < motionControlDelaysMs)) 
  {
    debugPrint("Skipping output control. lastMotionOnMillis: " + String(lastMotionOnMillis) + ". Millis: " + String(millisNow) + ", DIFF: " + String(diff) );
    return;
  }

  if (motionNow != lastMotionStatus) 
  {
    debugPrint("Setting last motion status to: " + String(motionNow));
    lastMotionStatus = motionNow;
    if (motionNow) 
    {
      debugPrint("Motion detected at: "  + String(millisNow));
      lastMotionOnMillis = millisNow;
    } else 
    {
      lastMotionOnMillis = 0;
    }
  }
}

// Set status
void MotionSensor::setMotionControlStatus(MotionControlStatus status) 
{
  motionControlStatus = status;
  lastMotionOnMillis = 0;
  lastMotionStatus = 0;

  if (status != MotionControl) 
  {
    setOutputs(status);
  }
}
// Set delay in ms
void MotionSensor::setMotionControlDelay(uint8_t delayInMs)
{
  motionControlDelaysMs = delayInMs;
}

void MotionSensor::resetAverages() 
{
  motionDetected = 0;
}


// Private functions
bool MotionSensor::readCurrentMotion()
{
  bool motionNow = 0;
  for (uint8_t i = 0; i < inPins.size(); i++) 
  {
    int reading = digitalRead(inPins[i]);
    if (reading == HIGH)
    {
      motionNow = 1;
      debugPrint("Motion detected in pin: " + String(inPins[i]));
      continue;
    }
  }
  return motionNow;  
}

void MotionSensor::setOutputs(bool mode) 
{
  debugPrint("Setting outputs as " + String(mode));
  for (uint8_t i = 0; i < outPins.size(); i++) 
  {
    if (mode)
    {
      digitalWrite(outPins[i], HIGH);
    } else 
    {
      digitalWrite(outPins[i], LOW);
    }
  }
}

