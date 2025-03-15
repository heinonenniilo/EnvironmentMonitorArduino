# Arduino code for EnvironmentMonitor

Arduino solution for reading and aggregating measurement data. The code has been run on ESP32 devices.

The main purpose of the solution is to aggregate measurement data and send it to Azure IoT hub. The solution can also be used to display measurement data in an Oled dispay. Measurement data used in https://github.com/heinonenniilo/EnvironmentMonitor is provided by this solution.

In addition, functionality for reading a motion sensor and controlling outputs based on the input value has been implemented. In practice, this functionality has been used to control a relay based on motion sensor values.

Azure IoT hub can also be used to communicate with device for sending basic commands, such as:

- Reboot
- Output "ON delay" after input goes to zero
- Whether outputs are always on/off or controlled based on motion control value.

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
- BH1750

Based on:
- https://github.com/Azure/azure-sdk-for-c-arduino
