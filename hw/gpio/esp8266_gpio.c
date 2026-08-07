/*
 * ESP8266 GPIO emulation
 *
 * 17 pins (GPIO0-16). Register layout differs from the ESP32:
 * base 0x60000300, OUT/SET/CLEAR then ENABLE/SET/CLEAR.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp8266_gpio.h"

#define ESP8266_GPIO_PINS 17
#define ESP8266_GPIO_MASK 0x1FFFF

/* Register offsets relative to 0x60000300 */
#define GPIO_OUT          0x00
#define GPIO_OUT_SET      0x04
#define GPIO_OUT_CLEAR    0x08
#define GPIO_ENABLE       0x0C
#define GPIO_ENABLE_SET   0x10
#define GPIO_ENABLE_CLEAR 0x14
#define GPIO_IN           0x18
#define GPIO_STATUS       0x1C
#define GPIO_STRAP        0x20

static uint64_t esp8266_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp8266GpioState *gpioS = ESP8266_GPIO(opaque);
    uint64_t r = 0;

    switch (addr) {
    case GPIO_OUT:      r = gpioS->gpio_out;      break;
    case GPIO_ENABLE:   r = gpioS->gpio_enable;   break;
    case GPIO_IN:       r = gpioS->gpio_in;       break;
    case GPIO_STATUS:   r = gpioS->gpio_status;   break;
    case GPIO_STRAP:    r = gpioS->strap_mode;    break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: unknown read addr=0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    }
    return r;
}

static void esp8266_gpio_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned int size)
{
    Esp8266GpioState *gpioS = ESP8266_GPIO(opaque);
    uint32_t val = value & ESP8266_GPIO_MASK;

    switch (addr) {
    case GPIO_OUT:
        gpioS->gpio_out = val;
        break;
    case GPIO_OUT_SET:
        gpioS->gpio_out |= val;
        break;
    case GPIO_OUT_CLEAR:
        gpioS->gpio_out &= ~val;
        break;
    case GPIO_ENABLE:
        gpioS->gpio_enable = val;
        break;
    case GPIO_ENABLE_SET:
        gpioS->gpio_enable |= val;
        break;
    case GPIO_ENABLE_CLEAR:
        gpioS->gpio_enable &= ~val;
        break;
    case GPIO_STATUS:
        gpioS->gpio_status &= ~val;
        break;
    case GPIO_STRAP:
        gpioS->strap_mode = val & 0x1F;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: unknown write addr=0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    }
}

static const MemoryRegionOps esp8266_gpio_ops = {
    .read  = esp8266_gpio_read,
    .write = esp8266_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp8266_gpio_reset(DeviceState *dev)
{
    Esp8266GpioState *gpioS = ESP8266_GPIO(dev);

    gpioS->gpio_out    = 0;
    gpioS->gpio_enable = 0;
    gpioS->gpio_in     = 0;
    gpioS->gpio_status = 0;
    gpioS->strap_mode  = ESP8266_STRAP_MODE_SPI_BOOT;
}

static void esp8266_gpio_init(Object *obj)
{
    Esp8266GpioState *gpioS = ESP8266_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&gpioS->iomem, obj, &esp8266_gpio_ops, gpioS,
                          TYPE_ESP8266_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &gpioS->iomem);
    sysbus_init_irq(sbd, &gpioS->irq);
}

static Property esp8266_gpio_properties[] = {
    DEFINE_PROP_UINT32("strap_mode", Esp8266GpioState, strap_mode,
                       ESP8266_STRAP_MODE_SPI_BOOT),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp8266_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->legacy_reset = esp8266_gpio_reset;
    device_class_set_props(dc, esp8266_gpio_properties);
}

static const TypeInfo esp8266_gpio_info = {
    .name          = TYPE_ESP8266_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp8266GpioState),
    .instance_init = esp8266_gpio_init,
    .class_init    = esp8266_gpio_class_init,
};

static void esp8266_gpio_register_types(void)
{
    type_register_static(&esp8266_gpio_info);
}

type_init(esp8266_gpio_register_types)
