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
    return motionDetectedTelemetry;
  }
  bool motionNow = readCurrentMotion();
  debugPrint("Motion now: " + String(motionNow));
  debugPrint("Motion detected during loop: " + String(motionDetectedDisplay));
  bool toReturn = motionDetectedDisplay || motionNow;
  motionDetectedDisplay = 0;
  return toReturn;
}

// Main control function. To be called from loop in a high interval.
void MotionSensor::checkOutputs()
{
  bool motionNow = readCurrentMotion();
  if (motionNow) 
  {
    motionDetectedDisplay = motionNow;
    motionDetectedTelemetry = motionNow;
  }
  if (motionControlStatus != MotionControl) 
  {
    debugPrint("Motion control status is: " + String(motionControlStatus));
    return;
  }
  unsigned long millisNow = millis();
  unsigned long diff = millisNow - lastMotionOnMillis;
  if (lastMotionOnMillis != 0 && ((diff) < motionControlDelaysMs) && diff > 0) 
  {
    debugPrint("Skipping output control. lastMotionOnMillis: " + String(lastMotionOnMillis) + ". Millis: " + String(millisNow) + ", DIFF: " + String(diff) + ", Limit: " + String(motionControlDelaysMs));
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
      setOutputs(1);
    } else 
    {
      lastMotionOnMillis = 0;
      setOutputs(0);
    }
  }
}

unsigned long MotionSensor::getOutputDelayLeft()
{
  if (lastMotionOnMillis != 0) 
  {
    unsigned long millisNow = millis();
    unsigned long diff = millisNow - lastMotionOnMillis;
    return motionControlDelaysMs - diff;
  } else 
  {
    return 0;
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
void MotionSensor::setMotionControlDelay(unsigned long delayInMs)
{
  debugPrint("Setting motion control delay");
  unsigned long valueToSet = delayInMs;
  if (delayInMs < 10000) 
  {
    valueToSet = 10000;
  } else if (delayInMs > 600000) 
  {
    valueToSet = 600000;
  } 

  debugPrint("Setting motion control delay to: " + String(valueToSet));
  motionControlDelaysMs = delayInMs;
}

void MotionSensor::resetAverages() 
{
  motionDetectedTelemetry = 0;
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
  debugPrint("Setting outputs to: " + String(mode));
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

