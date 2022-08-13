#ifndef WIFI_TASKS_H
#define WIFI_TASKS_H

#include "config.h"
#include <Arduino.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic



void configureWifi();
void printWifiDebugInfo();
bool connectServer();
bool serverIsConnected();
bool sendInfoServer();
bool streamAd7190DataMessageToServer(char* messageBlock);

#endif
