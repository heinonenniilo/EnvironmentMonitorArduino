#include "RuuviTagSensor.h"
#include <NimBLEDevice.h>
#include <algorithm>
#include <cctype>
#include <string>

RuuviTagSensor::RuuviTagSensor(const std::string& macToAllow, int sensorId) : allowedMac(macToAllow), Sensor(sensorId) {}

void RuuviTagSensor::begin() 
{
  Logger.Info("Inited Ruuvi sensor");
}

void RuuviTagSensor::handleMeasurement(const NimBLEAdvertisedDevice* advertisedDevice)
{

  std::string manufacturerData = advertisedDevice->getManufacturerData();
  if (manufacturerData.length() >= 24) {
    const uint8_t* data = (const uint8_t*)manufacturerData.c_str();
    Serial.println("MAN DATA FOUND");
    Serial.println(manufacturerData.c_str());
    if (data[0] == 0x99 && data[1] == 0x04 && data[2] == 0x05) {
      std::string mac = advertisedDevice->getAddress().toString();

      std::string macLower = mac;
      std::string allowedLower = allowedMac;
      std::transform(macLower.begin(), macLower.end(), macLower.begin(), [](unsigned char c){ return std::tolower(c); });
      std::transform(allowedLower.begin(), allowedLower.end(), allowedLower.begin(), [](unsigned char c){ return std::tolower(c); });      
      if (macLower != allowedLower) 
      {
        Serial.println("MAC: ");
        Serial.println(mac.c_str());
        Serial.println("Not allowed");
        return;
      }      
      float temp = ((int16_t)(data[3] << 8 | data[4])) * 0.005f;
      float hum  = ((uint16_t)(data[5] << 8 | data[6])) * 0.0025f;
      float pressure = ((uint16_t)(data[7] << 8 | data[8]) + 50000) / 100.0f;
      lastTemperature = temp;
      lastHumidity = hum;
      lastPressure = pressure;

      humidityTotal += hum;
      temperatureTotal += temp;
      pressureTotal += pressure;
      measureCount++; 
      Serial.println("RuuviTag:");
      Serial.println(mac.c_str());
      Serial.printf("Temp: %.2f °C\n", temp);
      Serial.printf("Humidity: %.2f %%\n", hum);
      Serial.printf("Pressure: %.2f hPa\n", pressure);
      Serial.println("--------------------------");
    }
  }
}

float RuuviTagSensor::readTemperature(bool average) 
{
  if (average)
  {
    if (!measureCount) 
    {
      return Sensor::ERROR_FAILED_READING;
    }
    return temperatureTotal / (float)measureCount;
  }
  return lastTemperature;
}

float RuuviTagSensor::readHumidity(bool average) 
{
  if (average)
  {
    if (!measureCount) 
    {
      return Sensor::ERROR_FAILED_READING;
    }
    return humidityTotal / (float)measureCount;
  }
  return lastHumidity;
}

float RuuviTagSensor::readPressure(bool average) 
{
  if (average)
  {
    if (!measureCount) 
    {
      return Sensor::ERROR_FAILED_READING;
    }
    return pressureTotal / (float)measureCount;
  }
  return lastPressure;
}

void RuuviTagSensor::resetAverages() 
{
    measureCount = 0;
    temperatureTotal = 0;
    humidityTotal = 0;
    pressureTotal = 0;
}


