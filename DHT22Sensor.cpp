#include "DHT22Sensor.h"
#include "SerialLogger.h"

// Constructor implementation
DHT22Sensor::DHT22Sensor(int sensorId, int pin) : Sensor(sensorId) ,pin(pin), dht(pin, DHT22) {}

void DHT22Sensor::begin() 
{
    dht.begin();
}

float DHT22Sensor::readTemperature(bool average) 
{
    float averageTemperature = 0;
    if (average) 
    {
        if (temperatureMeasureCount == 0 || !hasReadTemperature) 
        {
          return Sensor::ERROR_FAILED_READING;
        }
        averageTemperature = temperatureTotal / (float)temperatureMeasureCount;
        debugPrint("TempTotal: " + String(temperatureTotal));
        debugPrint("MeasureCount : " + String(temperatureMeasureCount));
        debugPrint("Temp AVG : " + String(averageTemperature));
        return averageTemperature;
    }  
    float temperature = dht.readTemperature();
    if (isnan(temperature)) {
        Serial.println("Failed to read temperature from DHT22 on pin " + String(pin));
        return Sensor::ERROR_FAILED_READING;
    }
    hasReadTemperature = 1;
    temperatureTotal += temperature;
    temperatureMeasureCount++;    
    averageTemperature = temperatureTotal / (float)temperatureMeasureCount;
    debugPrint("TempTotal: " + String(temperatureTotal));
    debugPrint("MeasureCount : " + String(temperatureMeasureCount));
    debugPrint("Current temp:  " + String(temperature));
    debugPrint("Temp AVG : " + String(averageTemperature));
    return temperature;
}

float DHT22Sensor::readHumidity(bool average) 
{
    float averageHumidity = 0;
    if (average) 
    {
      if (humidityMeasureCount == 0 || !hasReadHumidity) 
      { 
        return Sensor::ERROR_FAILED_READING;
      }
      averageHumidity = humidityTotal / (float)humidityMeasureCount;
      debugPrint("Average humidity: " + String(averageHumidity) );
      return averageHumidity;
    }
    float humidity = dht.readHumidity();
    if (isnan(humidity)) {
        debugPrint("Failed to read humidity from DHT22 on pin ");
        return Sensor::ERROR_FAILED_READING;
    }
    hasReadHumidity = 1;
    humidityMeasureCount++;
    humidityTotal+=humidity;
    averageHumidity = humidityTotal / (float)humidityMeasureCount;
    debugPrint("Humidity: " + String(averageHumidity));
    debugPrint("Humidity avg: " + String(humidity));
    debugPrint("Humidity total: " + String(humidityTotal));
    debugPrint("Measure count: " + String(humidityMeasureCount) );
    return humidity;
}

void DHT22Sensor::resetAverages() 
{
    temperatureTotal = 0;
    humidityTotal = 0;
    humidityMeasureCount = 0;
    temperatureMeasureCount = 0;
    hasReadHumidity = 0;
    hasReadTemperature = 0;
}
