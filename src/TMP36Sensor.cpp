#include "TMP36Sensor.h"
#include "SerialLogger.h"

TMP36Sensor::TMP36Sensor(int sensorId, int pin) : Sensor(sensorId), pin(pin) {}

void TMP36Sensor::begin() 
{
}

float TMP36Sensor::readTemperature(bool average) 
{
    float milliVoltsAverage = 0;
    if (average) 
    {
      if (measureCount == 0) 
      {
        return 0;
      }
      milliVoltsAverage = temperatureTotal / (float)measureCount;
      float averageTemp = ((milliVoltsAverage / 1000.0) - 0.5) * 100;      
      debugPrint("Total volts: " + String(temperatureTotal));
      debugPrint("Measure count: " + String(measureCount));
      return averageTemp;
    }  
    uint32_t milliVolts = analogReadMilliVolts(pin);
    temperatureTotal+= milliVolts;
    measureCount++;
    float tempC = ( (milliVolts / 1000.0) - 0.5) * 100;
    milliVoltsAverage = temperatureTotal / (float)measureCount;
    float avgTemp = ( (milliVoltsAverage / 1000.0) - 0.5) * 100;
    debugPrint("Total:" + String(temperatureTotal));
    debugPrint("Measure count: " + String(measureCount));
    debugPrint("Avg mv: " + String(milliVoltsAverage) );
    debugPrint("Avg temp: " + String(avgTemp) );    
    debugPrint("Current temp :" + String(tempC));
    return tempC;
}

void TMP36Sensor::resetAverages() 
{
    measureCount = 0;
    temperatureTotal = 0;
}