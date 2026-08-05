#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/qdev-properties.h"
#include "hw/dma/esp32_slc.h"

static bool slc_debug;

static void esp32_slc_update_debug(void)
{
    slc_debug = getenv("ESP32_SLC_DEBUG") != NULL;
}

static uint64_t esp32_slc_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32SlcState *s = ESP32_SLC(opaque);

    if (addr & 0x3) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: unaligned read addr 0x%llx size %u\n",
                      TYPE_ESP32_SLC, (unsigned long long)addr, size);
        return 0;
    }
    if (addr >= ESP32_SLC_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: out-of-range read addr 0x%llx\n",
                      TYPE_ESP32_SLC, (unsigned long long)addr);
        return 0;
    }

    uint32_t v = s->regs[addr / 4];

    if (addr == SLC_0TX_LINK_REG || addr == SLC_0RX_LINK_REG) {
        v |= SLC_TXLINK_PARK;
    }

    if (slc_debug) {
        printf("SLC read  addr=%02x = %08x\n", (unsigned)addr, v);
        fflush(stdout);
    }
    return v;
}

static void esp32_slc_dump_link(Esp32SlcState *s, const char *kind, hwaddr head)
{
    if (!slc_debug) {
        return;
    }
    printf("SLC %s link head=0x%llx\n", kind, (unsigned long long)head);
    fflush(stdout);

    hwaddr cur = head;
    for (int i = 0; cur != 0 && i < 16; i++) {
        uint32_t w[3];
        cpu_physical_memory_read(cur, w, sizeof(w));
        uint32_t ctrl = w[0];
        uint32_t buf = w[1];
        uint32_t next = w[2];
        uint32_t size = (ctrl >> 12) & 0xfff;

        printf("  desc[%d] cur=0x%llx ctrl=%08x size=%u buf=0x%08x next=0x%08x\n",
               i, (unsigned long long)cur, ctrl, size, buf, next);
        fflush(stdout);

        if (size && size <= 0x100) {
            uint8_t d[0x100];
            cpu_physical_memory_read(buf, d, size);
            printf("    data:");
            for (int j = 0; j < (int)size; j++) {
                printf(" %02x", d[j]);
            }
            printf("\n");
            fflush(stdout);
        }
        cur = next;
    }
}

static void esp32_slc_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32SlcState *s = ESP32_SLC(opaque);

    if (addr & 0x3) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: unaligned write addr 0x%llx size %u\n",
                      TYPE_ESP32_SLC, (unsigned long long)addr, size);
        return;
    }
    if (addr >= ESP32_SLC_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: out-of-range write addr 0x%llx\n",
                      TYPE_ESP32_SLC, (unsigned long long)addr);
        return;
    }

    uint32_t v = (uint32_t)value;

    if (slc_debug) {
        printf("SLC write addr=%02x = %08x\n", (unsigned)addr, v);
        fflush(stdout);
    }

    if (addr == SLC_0INT_CLR_REG) {
        s->regs[SLC_0INT_RAW_REG / 4] &= ~v;
        s->regs[SLC_0INT_ST_REG / 4] &= ~v;
        return;
    }

    s->regs[addr / 4] = v;

    if (addr == SLC_0TX_LINK_REG || addr == SLC_0RX_LINK_REG) {
        if (v & (SLC_TXLINK_STOP | SLC_TXLINK_START | SLC_TXLINK_RESTART)) {
            hwaddr head = s->regs[SLC_0_TXPKT_H_DSCR_REG / 4];
            if (addr == SLC_0RX_LINK_REG) {
                head = s->regs[SLC_0_RXPKT_H_DSCR_REG / 4];
            }
            esp32_slc_dump_link(s, addr == SLC_0RX_LINK_REG ? "RX" : "TX", head);
        }
    }
}

static const MemoryRegionOps esp32_slc_ops = {
    .read  = esp32_slc_read,
    .write = esp32_slc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_slc_reset(DeviceState *dev)
{
    Esp32SlcState *s = ESP32_SLC(dev);
    memset(s->regs, 0, sizeof(s->regs));
}

static void esp32_slc_realize(DeviceState *dev, Error **errp)
{
    esp32_slc_update_debug();
}

static void esp32_slc_init(Object *obj)
{
    Esp32SlcState *s = ESP32_SLC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_slc_ops, s, TYPE_ESP32_SLC,
                          ESP32_SLC_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32_slc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->legacy_reset = esp32_slc_reset;
    dc->realize = esp32_slc_realize;
}

static const TypeInfo esp32_slc_info = {
    .name = TYPE_ESP32_SLC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SlcState),
    .instance_init = esp32_slc_init,
    .class_init = esp32_slc_class_init,
};

static void esp32_slc_register_types(void)
{
    type_register_static(&esp32_slc_info);
}

type_init(esp32_slc_register_types)
