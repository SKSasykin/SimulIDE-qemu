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
#define ESP32_SIMULIDE_BRIDGE_MAX_RANGES 16

void esp32_simulide_bridge_create(MemoryRegion *sys_mem);

#endif
