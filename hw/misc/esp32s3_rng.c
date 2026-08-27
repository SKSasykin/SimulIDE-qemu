/*
 * ESP32S3 Random Number Generator peripheral
 *
 * Copyright (c) 2019-2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32s3_rng.h"


/* The ESP32-S3 bootloader's early RNG gather / DRAM-scrambling loops read the
 * RNG data register from a tight CCOUNT-budgeted loop, issuing a huge number of
 * reads. Using qemu_guest_getrandom_nofail() (a host `getrandom` syscall) for
 * every read makes each read cost microseconds, so filling a large buffer takes
 * minutes and the board looks hung. A small software PRNG is fast (a few cycles)
 * and perfectly adequate for emulation; the bootloader only uses the value for
 * entropy / memory scrambling, not for cryptography. */
static uint32_t esp32s3_rng_state;

static uint32_t esp32s3_rng_next(void)
{
    uint32_t x = esp32s3_rng_state;
    if (x == 0) {
        x = 0x9e3779b9u;
    }
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    esp32s3_rng_state = x;
    return x;
}

static uint64_t esp32s3_rng_read(void *opaque, hwaddr addr, unsigned int size)
{
    return esp32s3_rng_next();
}

static const MemoryRegionOps esp32s3_rng_ops = {
    .read = esp32s3_rng_read,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32s3_rng_init(Object *obj)
{
    Esp32s3RngState *s = ESP32S3_RNG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Each read must produce a new value: bootloader_fill_random() XORs
     * consecutive reads and retries forever if both obfuscation words are
     * zero. A RAM-backed register returns a constant and therefore deadlocks
     * image loading. The xorshift callback keeps reads cheap without making a
     * host getrandom syscall for every access. */
    memory_region_init_io(&s->iomem, obj, &esp32s3_rng_ops, s,
                          TYPE_ESP32S3_RNG, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
}


static const TypeInfo esp32s3_rng_info = {
    .name = TYPE_ESP32S3_RNG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32s3RngState),
    .instance_init = esp32s3_rng_init,
};

static void esp32s3_rng_register_types(void)
{
    type_register_static(&esp32s3_rng_info);
}

type_init(esp32s3_rng_register_types)
