#ifndef CONFIG_H
#define CONFIG_H

#define FIRMWARE_NAME "ESP32_AD7190"
#define FIRMWARE_VER "0.0.1"
#define WIFI_AP_NAME FIRMWARE_NAME

// #define MAIN_DEBUG_CALLS          //  Prints string at begining of class function execution
// #define MAIN_DEBUG_VERBOSE        //  Print some more information
#define MAIN_DEBUG_CONVERSION

// #define WIFI_DEBUG_VERBOSE        //  Print some more information

#define MAX_QUEUE_LENGTH               30
#define MAX_MESSAGE_BYTES              64

#define MEASURE_PIN 32

#endif
