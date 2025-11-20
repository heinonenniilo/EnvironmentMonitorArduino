// CP2102 PINS EXAMPLE
#define REDLEDPIN 27
#define GREENLEDPIN 26
#define YELLOWLEDPIN 33
#define SENSORPOWERPIN 16
// SENSORS
#define DHT22_PIN 32
#define DHT22_SENSORID 1
#define DS18B20_PIN 4
#define DS18B20_SENSORID 2

// #define DS18B20_2_PIN 19
// #define DS18B20_2_SENSORID 3

// #define BH1750FVI_SENSORID 3 // 4

#define MOTIONSENSOR_OUT_PINS {17} // {17,23} 
#define MOTIONSENSOR_IN_PINS {18} // {18, 2 }
#define MOTIONSENSOR_SENSORID 3 //    
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