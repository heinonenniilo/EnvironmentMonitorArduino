// ESPDUINO PINS
#define REDLEDPIN 13
#define GREENLEDPIN 12
#define YELLOWLEDPIN 5
// SENSORS
#define SENSORPOWERPIN 0 // 25
#define TMP36_PIN 36
#define TMP36_SENSORID 1

// #define DHT22_PIN 27
// #define DHT22_SENSORID 1

#define DS18B20_PIN 26
#define DS18B20_SENSORID 2

#define DS18B20_2_PIN 23
#define DS18B20_2_SENSORID 3

// #define BH1750FVI_SENSORID 4

// MotionSensor
#define MOTIONSENSOR_IN_PINS {19} // #define MOTIONSENSOR_IN_PINS {25}
#define MOTIONSENSOR_OUT_PINS {17,16,27}
#define MOTIONSENSOR_SENSORID 4
#define MOTIONSENSOR_MULTI_TRIGGER_MODE 1
#define MOTIONSENSOR_DISPLAY_ON_DELAY 1

// #define RUUVI_MAC "xx:xx:xx:xx:xx:xx"
// #define RUUVI_SENSORID 5

// #define RUUVI_MAC_2 "xx:xx:xx:xx:xx:xx"
// #define RUUVI_SENSORID_2 5

// Screen definitions
#define USE_DISPLAY 1 // Uncomment in order not to use display
#define SH1106 1 // Uncomment to use SSD1306
// #define SSD1306