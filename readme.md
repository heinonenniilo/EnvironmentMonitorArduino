# Arduino code for EnvironmentMonitor

Arduino solution for reading and aggregating measurement data. The main purpose of the solution is to aggregate the measurement data and send it to Azure IoT hub. In addition, the solution can be used to display the measurement data in an Oled dispay.


Measurement data used in https://github.com/heinonenniilo/EnvironmentMonitor is provided by this solution.

Required libraries:
- ArduinoJSON
- Azure SDK for C
- DHT sensor library
  - Adafruid Unified Sensor
- DallasTemperature
  - OneWire
- Adafruit_SH110X
- Adafruit_SSD1306
- BH1750

Actual IOT hub configurations to be placed in **configs**-folder. Iot_configs.h in root is an example. 

_pins_CP2102.h_ / _pins_ESPDUINO.h_ are configs used for different kinds of ESP32s. More dynamic solution for defining the sensors to use could be considered.

Supported sensors:
- DHT22
- DS18B20
- TMP36 (analog)

Based on:
- https://github.com/Azure/azure-sdk-for-c-arduino
