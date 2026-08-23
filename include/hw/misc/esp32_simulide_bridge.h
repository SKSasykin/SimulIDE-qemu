/*
 * ESP32 SimulIDE bridge
 *
 * Copyright (c) 2026
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#ifndef HW_MISC_ESP32_SIMULIDE_BRIDGE_H
#define HW_MISC_ESP32_SIMULIDE_BRIDGE_H

#include "hw/sysbus.h"

#define TYPE_ESP32_SIMULIDE_BRIDGE "esp32-simulide-bridge"
#define TYPE_ESP32S3_SIMULIDE_BRIDGE "esp32s3-simulide-bridge"
#define TYPE_ESP32C3_SIMULIDE_BRIDGE "esp32c3-simulide-bridge"
#define TYPE_ESP8266_SIMULIDE_BRIDGE "esp8266-simulide-bridge"
#define ESP32_SIMULIDE_BRIDGE_MAX_RANGES 16

void esp32_simulide_bridge_create(MemoryRegion *sys_mem, DeviceState *intmatrix);
void esp32s3_simulide_bridge_create(MemoryRegion *sys_mem, DeviceState *intmatrix);
void esp32c3_simulide_bridge_create(MemoryRegion *sys_mem, DeviceState *intmatrix);
void esp8266_simulide_bridge_create(MemoryRegion *sys_mem, uint32_t user_entry);

#endif
