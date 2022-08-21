#ifndef WIFI_TASKS_H
#define WIFI_TASKS_H

#include "config.h"
#include <Arduino.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

extern WiFiClient wifiClient;

void configureWifi();
void printWifiDebugInfo();
bool connectServer();
bool serverIsConnected();
inline bool streamAd7190DataMessageToServer(char* messageBlock) __attribute__((always_inline));
void getServerMessage();


//https://github.com/espressif/arduino-esp32/issues/4529
//https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-throughput

bool streamAd7190DataMessageToServer(char* messageBlock) {

    //printf("streamAd7190DataMessageToServer %d!\n", xPortGetCoreID() );
     
    int ret = wifiClient.print(messageBlock);

 #ifdef WIFI_DEBUG_VERBOSE
    Serial.print("Send len=");
    Serial.print(ret);
    Serial.print(" - ");
    Serial.print(messageBlock);
 #endif

    return true;
}

#endif
