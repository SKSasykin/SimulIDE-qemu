/*
 * ESP8266 FRC1 timer
 *
 * FRC1 is a 23-bit countdown timer. Writing LOAD rearms the timer and is
 * how the ESP8266 SDK schedules each software-PWM edge.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/timer/esp8266_frc1.h"
#include "qemu/timer.h"

#define FRC1_LOAD       0x00
#define FRC1_COUNT      0x04
#define FRC1_CTRL       0x08
#define FRC1_INT_CLEAR  0x0c
#define FRC1_MASK       0x7fffff

static uint64_t esp8266_frc1_ticks_to_ns(Esp8266Frc1State *s,
                                         uint32_t ticks)
{
    return muldiv64(ticks, NANOSECONDS_PER_SECOND * s->prescaler,
                    s->apb_freq);
}

static uint32_t esp8266_frc1_count(Esp8266Frc1State *s, uint64_t now)
{
    if (!s->enabled) {
        return s->count_base;
    }

    uint64_t elapsed = now - s->ns_base;
    uint64_t ticks = muldiv64(elapsed, s->apb_freq,
                              NANOSECONDS_PER_SECOND * s->prescaler);
    return ticks >= s->count_base ? 0 : s->count_base - ticks;
}

static void esp8266_frc1_arm(Esp8266Frc1State *s, uint32_t count)
{
    s->count_base = count & FRC1_MASK;
    s->ns_base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_del(&s->timer);
    if (s->enabled && s->count_base) {
        timer_mod(&s->timer,
                  s->ns_base + esp8266_frc1_ticks_to_ns(s, s->count_base));
    }
}

static void esp8266_frc1_cb(void *opaque)
{
    Esp8266Frc1State *s = ESP8266_FRC1(opaque);

    s->count_base = 0;
    s->ns_base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->interrupt_pending = true;
    if (s->level_interrupt) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_pulse(s->irq);
    }

    if (s->autoreload) {
        esp8266_frc1_arm(s, s->load);
    }
}

static uint64_t esp8266_frc1_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    Esp8266Frc1State *s = ESP8266_FRC1(opaque);

    switch (addr) {
    case FRC1_LOAD:
        return s->load;
    case FRC1_COUNT:
        return esp8266_frc1_count(s,
                                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    case FRC1_CTRL: {
        uint32_t divider = s->prescaler == 1 ? 0 :
                           s->prescaler == 16 ? 1 : 2;
        return (s->level_interrupt ? 1 : 0) | (divider << 2) |
               (s->autoreload ? (1u << 6) : 0) |
               (s->enabled ? (1u << 7) : 0);
    }
    case FRC1_INT_CLEAR:
    default:
        return 0;
    }
}

static void esp8266_frc1_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    Esp8266Frc1State *s = ESP8266_FRC1(opaque);

    switch (addr) {
    case FRC1_LOAD:
        s->load = value & FRC1_MASK;
        esp8266_frc1_arm(s, s->load);
        break;
    case FRC1_CTRL: {
        uint32_t count = esp8266_frc1_count(
            s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        uint32_t divider = (value >> 2) & 3;
        s->prescaler = divider == 0 ? 1 : divider == 1 ? 16 : 256;
        s->level_interrupt = value & 1;
        s->autoreload = value & (1u << 6);
        s->enabled = value & (1u << 7);
        esp8266_frc1_arm(s, count ? count : s->load);
        break;
    }
    case FRC1_INT_CLEAR:
        s->interrupt_pending = false;
        qemu_irq_lower(s->irq);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps esp8266_frc1_ops = {
    .read = esp8266_frc1_read,
    .write = esp8266_frc1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp8266_frc1_reset(DeviceState *dev)
{
    Esp8266Frc1State *s = ESP8266_FRC1(dev);

    timer_del(&s->timer);
    qemu_irq_lower(s->irq);
    s->load = 0;
    s->count_base = 0;
    s->prescaler = 1;
    s->ns_base = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->enabled = false;
    s->autoreload = false;
    s->level_interrupt = false;
    s->interrupt_pending = false;
}

static void esp8266_frc1_init(Object *obj)
{
    Esp8266Frc1State *s = ESP8266_FRC1(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp8266_frc1_ops, s,
                          TYPE_ESP8266_FRC1, 0x10);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, esp8266_frc1_cb, s);
    s->apb_freq = 80000000;
}

static void esp8266_frc1_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->legacy_reset = esp8266_frc1_reset;
}

static const TypeInfo esp8266_frc1_info = {
    .name = TYPE_ESP8266_FRC1,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp8266Frc1State),
    .instance_init = esp8266_frc1_init,
    .class_init = esp8266_frc1_class_init,
};

static void esp8266_frc1_register_types(void)
{
    type_register_static(&esp8266_frc1_info);
}

type_init(esp8266_frc1_register_types)
