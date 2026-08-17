/*
 * ESP32 SimulIDE bridge
 *
 * Creates narrow MMIO regions over exactly the peripheral ranges that
 * SimulIDE's C++ modules manage, and forwards every access to the
 * SimulIDE shared-memory arena through the legacy register protocol
 * (simulide_bridge_read / simulide_bridge_write). Each access is tagged
 * with the full IOMEM offset (address - 0x3FF00000), which is the offset
 * space SimulIDE's m_ioMem vector is indexed with.
 *
 * The regions are mapped at priority +1 so they shadow the corresponding
 * real peripheral devices (which remain dormant: SimulIDE owns those
 * ranges). Everything outside these ranges -- in particular SPI0/SPI1
 * (0x3FF41000/0x3FF42000), which the boot ROM needs to read the SPI
 * flash firmware image, plus DPORT, timers and RTC -- is left to the
 * real QEMU models so that firmware boot keeps working.
 *
 * Copyright (c) 2026
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32s3_reg.h"
#include "hw/misc/esp32c3_reg.h"
#include "hw/misc/esp32_simulide_bridge.h"

#include "../../system/simuliface.h"

uint32_t esp32_simulide_bridge_get_pc(void);

#define ESP32_SIMULIDE_BRIDGE_BASE   DR_REG_DPORT_BASE /* 0x3ff00000 */
#define ESP32_SIMULIDE_BRIDGE_SIZE   0x80000           /* full IOMEM    */
#define ESP32_SIMULIDE_BRIDGE_RANGE  0x1000

typedef struct Esp32SimulideBridgeState Esp32SimulideBridgeState;

typedef struct Esp32SimulideBridgeRange {
    Esp32SimulideBridgeState *state;
    uint32_t base;      /* SimulIDE IOMEM offset (relative to the chip IOMEM base) */
    hwaddr   map_base;  /* physical sys_mem address where the range is installed */
} Esp32SimulideBridgeRange;

typedef struct Esp32SimulideBridgeState {
    SysBusDevice parent_obj;

    MemoryRegion *regions[ESP32_SIMULIDE_BRIDGE_MAX_RANGES];
    Esp32SimulideBridgeRange ranges[ESP32_SIMULIDE_BRIDGE_MAX_RANGES];
    unsigned      n_ranges;

    uint32_t      iomem_size;    /* SimulIDE IOMEM window size               */
    uint32_t      strap_offset;  /* full IOMEM offset of the GPIO_STRAP reg  */
    uint32_t      strap_mode;
} Esp32SimulideBridgeState;

#define ESP32_SIMULIDE_BRIDGE(obj) \
    OBJECT_CHECK(Esp32SimulideBridgeState, (obj), TYPE_ESP32_SIMULIDE_BRIDGE)

/* GPIO_STRAP value that makes the ESP32 v3 boot ROM pick
 * SPI_FAST_FLASH_BOOT (boot:0x17) instead of UART download.
 *
 * The ROM's main() computes the boot mode from GPIO_STRAP as follows:
 *   - (strap & 0x18) == 0  (GPIO12 and GPIO15 both low) -> DOWNLOAD_BOOT
 *   - (strap & 0x18) != 0  -> SPI_FAST_FLASH_BOOT when GPIO15 (bit4)
 *     is high, HSPI_FAST_FLASH_BOOT when only GPIO12 (bit3) is high.
 * So GPIO15 must be high. 0x17 mirrors a real devkitC v4 (GPIO0=1,
 * GPIO2=1, GPIO5=1, GPIO12=0, GPIO15=1). */
#define ESP32_SIMULIDE_BRIDGE_STRAP_SPI_BOOT 0x17
#define ESP32S3_SIMULIDE_BRIDGE_STRAP_SPI_BOOT 0x4
#define ESP32C3_SIMULIDE_BRIDGE_STRAP_SPI_BOOT 0x8

static Property esp32_simulide_bridge_properties[] = {
    DEFINE_PROP_UINT32( "strap-mode", Esp32SimulideBridgeState,
                        strap_mode, ESP32_SIMULIDE_BRIDGE_STRAP_SPI_BOOT ),
    DEFINE_PROP_END_OF_LIST(),
};

typedef struct Esp32SimulideBridgeMap {
    hwaddr   map_base;   /* physical address to map at */
    uint32_t simul_off;  /* SimulIDE IOMEM offset      */
    uint32_t size;       /* zero uses the default 4 KiB peripheral window */
} Esp32SimulideBridgeMap;

/* One entry per peripheral SimulIDE models; offsets are IOMEM offsets
 * (relative to 0x3FF00000). Keep in sync with src/microsim/cores/qemu/
 * esp32/esp32.cpp in the SimulIDE sources.
 *
 * The AHB/APB alias rows shadow the UART TX FIFOs at their hardware
 * aliases (0x60000000 + ...). IDF's uart_ll writes the TX FIFO through
 * UART_FIFO_AHB_REG() (= 0x60000000 + i*0x10000 + (i>1 ? 0xe000 : 0)),
 * so without these rows application console output would land in the
 * real esp32_uart device and be dropped (no chardev attached). They are
 * forwarded to SimulIDE with the matching dport IOMEM offset so the
 * UART module sees the exact same register writes as dport accesses. */
static const Esp32SimulideBridgeMap esp32_simulide_bridge_maps[] = {
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00040000, 0x00040000 }, /* UART1 (hw UART0) */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00044000, 0x00044000 }, /* GPIO              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00049000, 0x00049000 }, /* IOMUX             */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00050000, 0x00050000 }, /* UART2 (hw UART1)  */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00053000, 0x00053000 }, /* I2C1              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00059000, 0x00059000 }, /* LEDC (LED)        */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00064000, 0x00064000 }, /* HSPI              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00065000, 0x00065000 }, /* VSPI              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00048800, 0x00048800, 0x400 }, /* SENS (SAR ADC) */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00067000, 0x00067000 }, /* I2C2              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x0006E000, 0x0006E000 }, /* UART3 (hw UART2)  */
    { APB_REG_BASE + 0x00000000,                0x00040000 }, /* UART0 AHB FIFO    */
    { APB_REG_BASE + 0x00010000,                0x00050000 }, /* UART1 AHB FIFO    */
    { APB_REG_BASE + 0x0002E000,                0x0006E000 }, /* UART2 AHB FIFO    */
};

/* ESP32-S3: only the ranges SimulIDE models are shadowed.
 * SPI0/SPI1 (0x60002000/0x60003000) are left to the real models because
 * the boot ROM needs them to load the flash firmware. Offsets are IOMEM
 * offsets relative to 0x60000000. Keep in sync with Esp32s3 class. */
static const Esp32SimulideBridgeMap esp32s3_simulide_bridge_maps[] = {
    { DR_REG_UART_BASE + 0x00000000, 0x00000000 }, /* UART0            */
    { DR_REG_GPIO_BASE + 0x00000000, 0x00004000 }, /* GPIO             */
    { DR_REG_UART1_BASE + 0x00000000, 0x00010000 }, /* UART1           */
    { DR_REG_I2C_EXT_BASE + 0x00000000, 0x00013000, 0x200 }, /* I2C0     */
    { DR_REG_SPI2_BASE + 0x00000000, 0x00024000 }, /* GP-SPI2          */
    { DR_REG_SPI3_BASE + 0x00000000, 0x00025000 }, /* GP-SPI3          */
    { DR_REG_I2C1_EXT_BASE + 0x00000000, 0x00027000, 0x200 }, /* I2C1    */
    { DR_REG_UART2_BASE + 0x00000000, 0x0002E000 }, /* UART2           */
    { DR_REG_SENS_BASE + 0x00000000, 0x00008800, 0x200 }, /* SENS (SAR ADC) */
};

/* ESP32-C3: same approach as the S3. */
static const Esp32SimulideBridgeMap esp32c3_simulide_bridge_maps[] = {
    { DR_REG_UART_BASE + 0x00000000, 0x00000000 }, /* UART0            */
    { DR_REG_GPIO_BASE + 0x00000000, 0x00004000 }, /* GPIO             */
    { DR_REG_UART1_BASE + 0x00000000, 0x00010000 }, /* UART1           */
    { DR_REG_I2C_EXT_BASE + 0x00000000, 0x00013000, 0x200 }, /* I2C0     */
    { DR_REG_SPI2_BASE + 0x00000000, 0x00024000 }, /* GP-SPI2          */
    { DR_REG_APB_SARADC_BASE + 0x00000000, 0x00040000 }, /* APB_SARADC (SAR ADC) */
};

/* ESP8266: modeled peripheral offsets are relative to
 * 0x60000000 (SimulIDE's Esp8266 class IOMEM_BASE). Keep in sync with
 * src/microsim/cores/qemu/esp8266/esp8266.cpp in the SimulIDE sources. */
static const Esp32SimulideBridgeMap esp8266_simulide_bridge_maps[] = {
    { 0x60000000, 0x0000, 0x100 }, /* UART0   */
    { 0x60000100, 0x0100, 0x100 }, /* HSPI    */
    { 0x60000300, 0x0300, 0x100 }, /* GPIO    */
    { 0x60000D00, 0x0D00, 0x100 }, /* SAR ADC */
    { 0x60000F00, 0x0F00, 0x100 }, /* UART1   */
};

/* GPIO_STRAP offset inside the GPIO block (0x44000 + 0x38). */
#define ESP32_SIMULIDE_BRIDGE_GPIO_STRAP 0x00044038
/* S3/C3 GPIO block is at 0x60004000, so full IOMEM offset is 0x4038. */
#define ESP32S3_SIMULIDE_BRIDGE_GPIO_STRAP 0x00004038
#define ESP32C3_SIMULIDE_BRIDGE_GPIO_STRAP 0x00004038

/* ESP8266 GPIO_STRAP is at GPIO_BASE(0x0300) + 0x20, a full IOMEM offset. */
#define ESP8266_SIMULIDE_BRIDGE_GPIO_STRAP 0x00000320
#define ESP8266_SIMULIDE_BRIDGE_STRAP_SPI_BOOT 0x12

static uint64_t esp32_simulide_bridge_read(void *opaque, hwaddr offset,
                                           unsigned size)
{
    Esp32SimulideBridgeRange *range = (Esp32SimulideBridgeRange *) opaque;
    Esp32SimulideBridgeState *s = range->state;
    uint32_t full = range->base + (uint32_t) offset;

    if( full >= s->iomem_size ) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read out of range offset=0x%"
                      HWADDR_PRIx "\n", __func__, offset);
        return 0;
    }
    /* The boot ROM selects its boot mode from GPIO straps (GPIO0/GPIO2/
     * GPIO12/GPIO15, read through GPIO_STRAP at 0x3FF44038). SimulIDE's
     * GPIO module has no notion of straps, so answer here or the ROM
     * would always fall into UART download mode. Default is SPI boot
     * (GPIO0=1, GPIO2=1, GPIO5=1, GPIO15=1). */
    if( full == s->strap_offset ) {
        return (uint64_t) s->strap_mode;
    }
    {
        uint32_t v = (uint32_t) simulide_bridge_read( full );
        return (uint64_t) v;
    }
}

static void esp32_simulide_bridge_write(void *opaque, hwaddr offset,
                                        uint64_t value, unsigned size)
{
    Esp32SimulideBridgeRange *range = (Esp32SimulideBridgeRange *) opaque;
    Esp32SimulideBridgeState *s = range->state;
    uint32_t full = range->base + (uint32_t) offset;

    if( full >= s->iomem_size ) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write out of range offset=0x%"
                      HWADDR_PRIx "\n", __func__, offset);
        return;
    }
    simulide_bridge_write( full, (uint32_t) value );
}

static const MemoryRegionOps esp32_simulide_bridge_ops = {
    .read = esp32_simulide_bridge_read,
    .write = esp32_simulide_bridge_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void esp32_simulide_bridge_init_common(
        Object *obj, const char *type_name,
        const Esp32SimulideBridgeMap *maps,
        unsigned n, uint32_t iomem_size, uint32_t strap_offset,
        uint32_t strap_mode )
{
    Esp32SimulideBridgeState *s = (Esp32SimulideBridgeState *)obj;
    unsigned i;

    s->n_ranges = n;
    s->iomem_size = iomem_size;
    s->strap_offset = strap_offset;
    s->strap_mode = strap_mode;

    for( i = 0; i < n; i++ ) {
        MemoryRegion *iomem = g_new(MemoryRegion, 1);
        char name[48];

        s->ranges[i].state = s;
        s->ranges[i].base = maps[i].simul_off;
        s->ranges[i].map_base = maps[i].map_base;
        snprintf( name, sizeof(name), "%s-%u", type_name, i );
        memory_region_init_io( iomem, OBJECT(obj), &esp32_simulide_bridge_ops,
                               &s->ranges[i], name,
                               maps[i].size ? maps[i].size : ESP32_SIMULIDE_BRIDGE_RANGE );
        s->regions[i] = iomem;
    }
}

static void esp32_simulide_bridge_init(Object *obj)
{
    esp32_simulide_bridge_init_common(
        obj, TYPE_ESP32_SIMULIDE_BRIDGE,
        esp32_simulide_bridge_maps, ARRAY_SIZE( esp32_simulide_bridge_maps ),
        ESP32_SIMULIDE_BRIDGE_SIZE,
        ESP32_SIMULIDE_BRIDGE_GPIO_STRAP,
        ESP32_SIMULIDE_BRIDGE_STRAP_SPI_BOOT );
}

static void esp32s3_simulide_bridge_init(Object *obj)
{
    esp32_simulide_bridge_init_common(
        obj, TYPE_ESP32S3_SIMULIDE_BRIDGE,
        esp32s3_simulide_bridge_maps, ARRAY_SIZE( esp32s3_simulide_bridge_maps ),
        ESP32_SIMULIDE_BRIDGE_SIZE,
        ESP32S3_SIMULIDE_BRIDGE_GPIO_STRAP,
        ESP32S3_SIMULIDE_BRIDGE_STRAP_SPI_BOOT );
}

static void esp32c3_simulide_bridge_init(Object *obj)
{
    esp32_simulide_bridge_init_common(
        obj, TYPE_ESP32C3_SIMULIDE_BRIDGE,
        esp32c3_simulide_bridge_maps, ARRAY_SIZE( esp32c3_simulide_bridge_maps ),
        ESP32_SIMULIDE_BRIDGE_SIZE,
        ESP32C3_SIMULIDE_BRIDGE_GPIO_STRAP,
        ESP32C3_SIMULIDE_BRIDGE_STRAP_SPI_BOOT );
}

static void esp8266_simulide_bridge_init(Object *obj)
{
    esp32_simulide_bridge_init_common(
        obj, TYPE_ESP8266_SIMULIDE_BRIDGE,
        esp8266_simulide_bridge_maps, ARRAY_SIZE( esp8266_simulide_bridge_maps ),
        0x10000,
        ESP8266_SIMULIDE_BRIDGE_GPIO_STRAP,
        ESP8266_SIMULIDE_BRIDGE_STRAP_SPI_BOOT );
}

static void esp32_simulide_bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "ESP32 SimulIDE bridge";
    device_class_set_props(dc, esp32_simulide_bridge_properties);
}

static const TypeInfo esp32_simulide_bridge_info = {
    .name = TYPE_ESP32_SIMULIDE_BRIDGE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SimulideBridgeState),
    .instance_init = esp32_simulide_bridge_init,
    .class_init = esp32_simulide_bridge_class_init,
};

static const TypeInfo esp32s3_simulide_bridge_info = {
    .name = TYPE_ESP32S3_SIMULIDE_BRIDGE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SimulideBridgeState),
    .instance_init = esp32s3_simulide_bridge_init,
    .class_init = esp32_simulide_bridge_class_init,
};

static const TypeInfo esp32c3_simulide_bridge_info = {
    .name = TYPE_ESP32C3_SIMULIDE_BRIDGE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SimulideBridgeState),
    .instance_init = esp32c3_simulide_bridge_init,
    .class_init = esp32_simulide_bridge_class_init,
};

static const TypeInfo esp8266_simulide_bridge_info = {
    .name = TYPE_ESP8266_SIMULIDE_BRIDGE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SimulideBridgeState),
    .instance_init = esp8266_simulide_bridge_init,
    .class_init = esp32_simulide_bridge_class_init,
};

static void esp32_simulide_bridge_register_types(void)
{
    type_register_static(&esp32_simulide_bridge_info);
    type_register_static(&esp32s3_simulide_bridge_info);
    type_register_static(&esp32c3_simulide_bridge_info);
    type_register_static(&esp8266_simulide_bridge_info);
}

type_init(esp32_simulide_bridge_register_types);

static void esp32_simulide_bridge_create_common(MemoryRegion *sys_mem,
                                                const char *type_name)
{
    Esp32SimulideBridgeState *s =
        (Esp32SimulideBridgeState *)qdev_new( type_name );
    unsigned i;

    sysbus_realize_and_unref( SYS_BUS_DEVICE( DEVICE(s) ), &error_fatal );

    for( i = 0; i < s->n_ranges; i++ ) {
        memory_region_add_subregion_overlap( sys_mem,
                                             s->ranges[i].map_base,
                                             s->regions[i], 1 );
    }
}

void esp32_simulide_bridge_create(MemoryRegion *sys_mem)
{
    esp32_simulide_bridge_create_common( sys_mem,
                                         TYPE_ESP32_SIMULIDE_BRIDGE );
}

void esp32s3_simulide_bridge_create(MemoryRegion *sys_mem)
{
    esp32_simulide_bridge_create_common( sys_mem,
                                         TYPE_ESP32S3_SIMULIDE_BRIDGE );
}

void esp32c3_simulide_bridge_create(MemoryRegion *sys_mem)
{
    esp32_simulide_bridge_create_common( sys_mem,
                                         TYPE_ESP32C3_SIMULIDE_BRIDGE );
}

void esp8266_simulide_bridge_create(MemoryRegion *sys_mem)
{
    esp32_simulide_bridge_create_common( sys_mem,
                                         TYPE_ESP8266_SIMULIDE_BRIDGE );
}
