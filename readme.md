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
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- ArduinoMqttClient (for MQTT broker mode)

Required boards:
- ESP32 board support should be added to Arduino IDE. Minor changes likely needed if run with Arduino hardware.

Actual IOT hub configurations to be placed in **configs**-folder. Iot_configs.h in root is an example. 

### MQTT broker communication

Define `USE_MQQT_BROKER` in `configs/iot_configs.h` to use ArduinoMqttClient instead of
Azure IoT Hub or the HTTP API. This flag takes precedence if either other flag is still defined.
Copy the `MQTT_BROKER_*` settings from the root example into your device configuration.
Set the broker hostname, username, password, and trusted root CA PEM certificate.
The certificate uses the same variable-based pattern as HTTP:

```cpp
#define USE_MQTT_ROOT_CA
#ifdef USE_MQTT_ROOT_CA
static const char mqtt_root_ca[] PROGMEM = R"PEM(-----BEGIN CERTIFICATE-----
PASTE_THE_COMPLETE_ROOT_CA_CERTIFICATE_HERE
-----END CERTIFICATE-----
)PEM";
#endif
```

TLS and certificate verification are enabled by default; the default port is 8883.
TLS requires `USE_MQTT_ROOT_CA` and a valid `mqtt_root_ca`; there is no insecure fallback.
For a non-TLS broker, set `MQTT_BROKER_USE_TLS` to `0` (default port 1883).
`MQTT_BROKER_PORT` can override either default. Credentials are sent in plaintext without TLS.

The topic templates are `environment/measurements/{IOT_CONFIG_DEVICE_ID}` and
`environment/commands/{IOT_CONFIG_DEVICE_ID}`, using `IOT_CONFIG_DEVICE_ID` as the device suffix
and MQTT client ID. For example, `EnvironmentMeasure_12` subscribes to
`environment/commands/EnvironmentMeasure_12`. Customize the static topic prefixes as needed.
Measurements use the existing telemetry JSON and are published with QoS 1, without retention.
The client reconnects and resubscribes automatically.

Commands are plain-text payloads, using the same handlers as IoT Hub:
`REBOOT`, `MOTIONCONTROLSTATUS:0` (always off), `MOTIONCONTROLSTATUS:1` (always on),
`MOTIONCONTROLSTATUS:2` (motion controlled), and `MOTIONCONTROLDELAY:45000`
(positive delay in milliseconds). Do not retain reboot commands.

_pins_CP2102.h_ / _pins_ESPDUINO.h_ are configs used for different kinds of ESP32s. More dynamic solution for defining the sensors to use could be considered.

Supported sensors:
- DHT22
- DS18B20
- TMP36 (analog)
- BH1750
- HC-SR501
  - Or any digital motion sensor.
- RuuviTag

Based on:
- https://github.com/Azure/azure-sdk-for-c-arduino
