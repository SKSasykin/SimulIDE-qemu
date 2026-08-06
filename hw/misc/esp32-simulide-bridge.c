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
#include "hw/misc/esp32_simulide_bridge.h"

#include "../../system/simuliface.h"

uint32_t esp32_simulide_bridge_get_pc(void);

#define ESP32_SIMULIDE_BRIDGE_BASE   DR_REG_DPORT_BASE /* 0x3ff00000 */
#define ESP32_SIMULIDE_BRIDGE_SIZE   0x80000           /* full IOMEM    */
#define ESP32_SIMULIDE_BRIDGE_RANGE  0x1000

typedef struct Esp32SimulideBridgeState Esp32SimulideBridgeState;

typedef struct Esp32SimulideBridgeRange {
    Esp32SimulideBridgeState *state;
    uint32_t base;      /* SimulIDE IOMEM offset (relative to 0x3FF00000) */
    hwaddr   map_base;  /* physical sys_mem address where the range is installed */
} Esp32SimulideBridgeRange;

typedef struct Esp32SimulideBridgeState {
    SysBusDevice parent_obj;

    MemoryRegion *regions[ESP32_SIMULIDE_BRIDGE_MAX_RANGES];
    Esp32SimulideBridgeRange ranges[ESP32_SIMULIDE_BRIDGE_MAX_RANGES];
    unsigned      n_ranges;

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

static Property esp32_simulide_bridge_properties[] = {
    DEFINE_PROP_UINT32( "strap-mode", Esp32SimulideBridgeState,
                        strap_mode, ESP32_SIMULIDE_BRIDGE_STRAP_SPI_BOOT ),
    DEFINE_PROP_END_OF_LIST(),
};

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
static const struct {
    hwaddr   map_base;   /* physical address to map at */
    uint32_t simul_off;  /* SimulIDE IOMEM offset      */
} esp32_simulide_bridge_maps[] = {
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00040000, 0x00040000 }, /* UART1 (hw UART0) */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00044000, 0x00044000 }, /* GPIO              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00049000, 0x00049000 }, /* IOMUX             */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00050000, 0x00050000 }, /* UART2 (hw UART1)  */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00053000, 0x00053000 }, /* I2C1              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00059000, 0x00059000 }, /* LEDC (LED)        */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00064000, 0x00064000 }, /* HSPI              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00065000, 0x00065000 }, /* VSPI              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x00067000, 0x00067000 }, /* I2C2              */
    { ESP32_SIMULIDE_BRIDGE_BASE + 0x0006E000, 0x0006E000 }, /* UART3 (hw UART2)  */
    { APB_REG_BASE + 0x00000000,                0x00040000 }, /* UART0 AHB FIFO    */
    { APB_REG_BASE + 0x00010000,                0x00050000 }, /* UART1 AHB FIFO    */
    { APB_REG_BASE + 0x0002E000,                0x0006E000 }, /* UART2 AHB FIFO    */
};

/* GPIO_STRAP offset inside the GPIO block (0x44000 + 0x38). */
#define ESP32_SIMULIDE_BRIDGE_GPIO_STRAP 0x00044038

static uint64_t esp32_simulide_bridge_read(void *opaque, hwaddr offset,
                                           unsigned size)
{
    Esp32SimulideBridgeRange *range = (Esp32SimulideBridgeRange *) opaque;
    Esp32SimulideBridgeState *s = range->state;
    uint32_t full = range->base + (uint32_t) offset;

    if( full >= ESP32_SIMULIDE_BRIDGE_SIZE ) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read out of range offset=0x%"
                      HWADDR_PRIx "\n", __func__, offset);
        return 0;
    }
    /* The boot ROM selects its boot mode from GPIO straps (GPIO0/GPIO2/
     * GPIO12/GPIO15, read through GPIO_STRAP at 0x3FF44038). SimulIDE's
     * GPIO module has no notion of straps, so answer here or the ROM
     * would always fall into UART download mode. Default is SPI boot
     * (GPIO0=1, GPIO2=1, GPIO5=1, GPIO15=1). */
    if( full == ESP32_SIMULIDE_BRIDGE_GPIO_STRAP ) {
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
    uint32_t full = range->base + (uint32_t) offset;

    if( full >= ESP32_SIMULIDE_BRIDGE_SIZE ) {
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

static void esp32_simulide_bridge_init(Object *obj)
{
    Esp32SimulideBridgeState *s = ESP32_SIMULIDE_BRIDGE(obj);
    unsigned n = ARRAY_SIZE( esp32_simulide_bridge_maps );
    unsigned i;

    s->n_ranges = n;
    s->strap_mode = ESP32_SIMULIDE_BRIDGE_STRAP_SPI_BOOT; /* SPI boot */

    for( i = 0; i < n; i++ ) {
        MemoryRegion *iomem = g_new(MemoryRegion, 1);
        char name[48];

        s->ranges[i].state = s;
        s->ranges[i].base = esp32_simulide_bridge_maps[i].simul_off;
        s->ranges[i].map_base = esp32_simulide_bridge_maps[i].map_base;
        snprintf( name, sizeof(name), "%s-%u",
                  TYPE_ESP32_SIMULIDE_BRIDGE, i );
        memory_region_init_io( iomem, OBJECT(obj), &esp32_simulide_bridge_ops,
                               &s->ranges[i], name,
                               ESP32_SIMULIDE_BRIDGE_RANGE );
        s->regions[i] = iomem;
    }
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

static void esp32_simulide_bridge_register_types(void)
{
    type_register_static(&esp32_simulide_bridge_info);
}

type_init(esp32_simulide_bridge_register_types);

void esp32_simulide_bridge_create(MemoryRegion *sys_mem)
{
    Esp32SimulideBridgeState *s =
        ESP32_SIMULIDE_BRIDGE( qdev_new( TYPE_ESP32_SIMULIDE_BRIDGE ) );
    unsigned i;

    sysbus_realize_and_unref( SYS_BUS_DEVICE( DEVICE(s) ), &error_fatal );

    for( i = 0; i < s->n_ranges; i++ ) {
        memory_region_add_subregion_overlap( sys_mem,
                                             s->ranges[i].map_base,
                                             s->regions[i], 1 );
    }
}
