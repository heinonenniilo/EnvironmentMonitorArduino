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
    float milliVoltsAverage = 0;
    if (average) 
    {
      if (measureCount == 0) 
      {
        return 0;
      }
      return lightTotal / measureCount;
    }   

    float lux = lightMeter.readLightLevel();
    lightTotal += lux;
    measureCount++;
    float lightAverage = lightTotal / measureCount;
    debugPrint("Light: " + String(lux) + " lux");
    debugPrint("Light avg:" + String(lightAverage) + " lux");
    debugPrint("Measure count: " + String(measureCount));
    return lux;
}

void BH1750FVISensor::resetAverages() 
{
    measureCount = 0;
    lightTotal = 0;
}