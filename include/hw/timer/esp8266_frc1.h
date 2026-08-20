#pragma once

#include "hw/sysbus.h"
#include "qemu/timer.h"

#define TYPE_ESP8266_FRC1 "timer.esp8266.frc1"
OBJECT_DECLARE_SIMPLE_TYPE(Esp8266Frc1State, ESP8266_FRC1)

struct Esp8266Frc1State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer timer;

    uint32_t apb_freq;
    uint32_t load;
    uint32_t count_base;
    uint32_t prescaler;
    uint64_t ns_base;
    bool enabled;
    bool autoreload;
    bool level_interrupt;
    bool interrupt_pending;
};
