
// Code from: ESP32Console project
// https://github.com/jbtronics/ESP32Console/

#ifndef ESP_INFO_H
#define ESP_INFO_H

#include <Arduino.h>
#include <core_version.h>
#include "SPIFFS.h"
#include "config.h"

void printBuildInfo();

static int meminfo();
static String mac2String(uint64_t mac);
static const char *getFlashModeStr();
static const char *getResetReasonStr();
void printFileSystem();

#endif
