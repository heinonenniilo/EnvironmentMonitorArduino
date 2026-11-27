# Environment Monitor - Arduino

Arduino firmware for reading and aggregating measurement data on ESP32 devices.

The main purpose of the solution is to aggregate measurement data and send it to Azure IoT hub. The solution can also be used to display measurement data in an Oled dispay. Measurement data used in [Environment Monitor](https://github.com/heinonenniilo/EnvironmentMonitor) is provided by this solution.

Additionally, functionality has been implemented for reading motion sensor input and controlling outputs based on those input values. In practice, this has been used to control a relay based on motion sensor values.

Azure IoT hub can also be used to communicate with device for sending basic commands, such as:

- Reboot
- Output ON delay, in milli seconds, after input goes to zero
- Whether outputs are always on/off or controlled based on motion sensor value

### Build and flash with PlatformIO

1. Open this repository folder in VS Code with the PlatformIO extension enabled. PlatformIO detects `platformio.ini` and installs the ESP32 toolchain and libraries on the first build.
2. Check `configs/iot_configs.h` for the Wi-Fi and telemetry settings for this device, and `configs/pins.h` for its GPIO assignments. For a fresh checkout, copy the root `iot_configs.h` example to `configs/iot_configs.h`, and copy either `configs/pins_CP2102.h` or `configs/pins_ESPDUINO.h` to `configs/pins.h` to match your wiring. Git ignores both local configuration files.
3. Connect the ESP32 with a USB data cable and close any serial monitor using its COM port. In **PlatformIO → Project Tasks → esp32dev → General**, click **Build**, then **Upload** to flash the board.
4. Open **Monitor** in the same menu to see the startup logs at 115200 baud.

The equivalent commands in a PlatformIO terminal are:

```powershell
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor -e esp32dev
```

If PlatformIO picks the wrong COM port, run `pio device list` and pass the board's port to Upload with `--upload-port COM5` or to Monitor with `--port COM5`, replacing `COM5` as needed. If Upload stalls at `Connecting...`, hold **BOOT**, tap **EN/RESET** if present, and release **BOOT** once writing begins. [Espressif bootloader guide](https://docs.espressif.com/projects/esptool/en/latest/esp32/esptool/entering-bootloader.html)

The `esp32dev` environment targets a classic ESP32 with 4 MB of flash. The `no_ota.csv` partition layout matches the existing MQTT build and gives the firmware more space. Git ignores the generated `.pio/` directory.

### Arduino IDE

Install ESP32 board support and these libraries when building with Arduino IDE:

- ArduinoJSON
- Azure SDK for C
- DHT sensor library
  - Adafruit Unified Sensor
- DallasTemperature
  - OneWire
- Adafruit_SH110X
- Adafruit_SSD1306
- BH1750
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- ArduinoMqttClient (for MQTT broker mode)

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
