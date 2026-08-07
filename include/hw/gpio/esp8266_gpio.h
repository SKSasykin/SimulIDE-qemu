#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"

#define TYPE_ESP8266_GPIO "gpio.esp8266"
#define ESP8266_GPIO(obj) OBJECT_CHECK(Esp8266GpioState, (obj), TYPE_ESP8266_GPIO)

#define ESP8266_STRAP_MODE_SPI_BOOT 0x12 /* GPIO2=1, GPIO15=1 -> SPI boot */

typedef struct Esp8266GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t gpio_out;
    uint32_t gpio_enable;
    uint32_t gpio_in;
    uint32_t gpio_status;
    uint32_t strap_mode;
} Esp8266GpioState;
