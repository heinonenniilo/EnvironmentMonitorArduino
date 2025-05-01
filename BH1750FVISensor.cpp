#include "BH1750FVISensor.h"
#include "SerialLogger.h"

BH1750FVISensor::BH1750FVISensor(int sensorId) : Sensor(sensorId), lightMeter() {}

void BH1750FVISensor::begin() 
{
  Wire.begin();
  delay(1000);
  debugPrint("Begin");
  lightMeter.begin();
  lightMeter.configure(BH1750::CONTINUOUS_HIGH_RES_MODE);
}

float BH1750FVISensor::readLight(bool average) 
{
    if (average) 
    {
      if (!measureCount) 
      {
        return Sensor::ERROR_FAILED_READING;
      }
      return lightTotal / measureCount;
    }   

    float lux = lightMeter.readLightLevel();
    if (lux < 0) 
    {
      debugPrint("Failed to read ambient light reading");
      return Sensor::ERROR_FAILED_READING;
    }
    lightTotal += lux;
    measureCount++;
    float lightAverage = lightTotal / measureCount;
    debugPrint("Light: " + String(lux) + " lx");
    debugPrint("Light avg:" + String(lightAverage) + " lx");
    debugPrint("Measure count: " + String(measureCount));
    return lux;
}

void BH1750FVISensor::resetAverages() 
{
    measureCount = 0;
    lightTotal = 0;
}