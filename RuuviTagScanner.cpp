#include "RuuviTagScanner.h"
#include <BLEDevice.h>

static float parseTemperature(const uint8_t* data) {
  int16_t raw = (data[3] << 8) | data[4];
  return raw * 0.005f;
}

static float parseHumidity(const uint8_t* data) {
  uint16_t raw = (data[5] << 8) | data[6];
  return raw * 0.0025f;
}

static float parsePressure(const uint8_t* data) {
  uint16_t raw = (data[7] << 8) | data[8];
  return (raw + 50000) / 100.0f;  // Convert to hPa
}



void RuuviTagScanner::begin() 
{
  //
  Logger.Info("Ruuvi scanner initing");
}

void RuuviTagScanner::onResult(BLEAdvertisedDevice advertisedDevice) 
{
  String manufacturerData = advertisedDevice.getManufacturerData();
  Serial.println(manufacturerData);
  if (manufacturerData.length() >= 24) {
    const uint8_t* data = (const uint8_t*)manufacturerData.c_str();

    if (data[0] == 0x99 && data[1] == 0x04 && data[2] == 0x05) {
      String mac = advertisedDevice.getAddress().toString();
      if (mac != allowedMac) 
      {
        Serial.println("MAC: ");
        Serial.println(mac);
        Serial.println("Not allowed");
        return;
      }      
      float temp = ((int16_t)(data[3] << 8 | data[4])) * 0.005f;
      float hum  = ((uint16_t)(data[5] << 8 | data[6])) * 0.0025f;
      float pressure = ((uint16_t)(data[7] << 8 | data[8]) + 50000) / 100.0f;
      lastTemperature = temp;
      lastHumidity = hum;

      humidityTotal += hum;
      temperatureTotal += temp;
      measureCount++; 

      Serial.println("RuuviTag:");
      Serial.println("MAC: " + mac);
      Serial.printf("Temp: %.2f °C\n", temp);
      Serial.printf("Humidity: %.2f %%\n", hum);
      Serial.printf("Pressure: %.2f hPa\n", pressure);
      Serial.println("--------------------------");
    }
  }
}

float RuuviTagScanner::readTemperature(bool average) 
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

float RuuviTagScanner::readHumidity(bool average) 
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

void RuuviTagScanner::resetAverages() 
{
    measureCount = 0;
    temperatureTotal = 0;
    humidityTotal = 0;
}


