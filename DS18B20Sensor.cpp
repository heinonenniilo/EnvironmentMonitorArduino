#include "DS18B20Sensor.h"
#include "SerialLogger.h"

DS18B20Sensor::DS18B20Sensor(int sensorId, int pin) : Sensor(sensorId), pin(pin), oneWire(pin), sensors(&oneWire) {}

void DS18B20Sensor::begin() 
{
  sensors.begin();
}

float DS18B20Sensor::readTemperature(bool average) 
{
  if (average) 
  {
    if (measureCount == 0 || !hasReadTemperature) 
    {
      return Sensor::ERROR_FAILED_READING;
    }
    return temperatureTotal / measureCount;
  }
  sensors.requestTemperatures(); 
  float temperature = sensors.getTempCByIndex(0); 
  if (temperature == DEVICE_DISCONNECTED_C) 
  {
    debugPrint("Failed to read temperature. Device disconnected.");
    return Sensor::ERROR_FAILED_READING;
  }
  hasReadTemperature = 1;
  temperatureTotal+=temperature;
  measureCount++;
  debugPrint("Temperature read: " + String(temperature) + " °C");
  debugPrint("Average: " + String(temperatureTotal/measureCount) + " °C");
  debugPrint("Total: " + String(temperatureTotal));
  debugPrint("Measure count: " + String(measureCount));
  return temperature;  
}

void DS18B20Sensor::resetAverages() 
{
    measureCount = 0;
    temperatureTotal = 0;
    hasReadTemperature = 0;
}