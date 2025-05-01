# Environment Monitor - Arduino

Arduino solution for reading and aggregating measurement data. The solution has been developed with Arduino IDE. The code has been run on ESP32 devices.

The main purpose of the solution is to aggregate measurement data and send it to Azure IoT hub. The solution can also be used to display measurement data in an Oled dispay. Measurement data used in [Environment Monitor](https://github.com/heinonenniilo/EnvironmentMonitor) is provided by this solution.

Additionally, functionality has been implemented for reading motion sensor input and controlling outputs based on those input values. In practice, this has been used to control a relay based on motion sensor values.

Azure IoT hub can also be used to communicate with device for sending basic commands, such as:

- Reboot
- Output ON delay, in milli seconds, after input goes to zero
- Whether outputs are always on/off or controlled based on motion sensor value

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

Required boards:
- ESP32 board support should be added to Arduino IDE. Minor changes likely needed if run with Arduino hardware.

Actual IOT hub configurations to be placed in **configs**-folder. Iot_configs.h in root is an example. 

_pins_CP2102.h_ / _pins_ESPDUINO.h_ are configs used for different kinds of ESP32s. More dynamic solution for defining the sensors to use could be considered.

Supported sensors:
- DHT22
- DS18B20
- TMP36 (analog)
- BH1750
- HC-SR501
  - Or any digital motion sensor.

Based on:
- https://github.com/Azure/azure-sdk-for-c-arduino
