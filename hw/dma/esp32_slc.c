#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "exec/memory.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/dma/esp32_slc.h"
#include "system/simuliface.h"

static bool slc_debug;

static void esp32_slc_update_debug(void)
{
    slc_debug = getenv("ESP32_SLC_DEBUG") != NULL;
}

/* ESP32 SLC Wi-Fi DMA descriptor (ESP-IDF lldesc_t), 12 bytes in guest RAM:
 *   w0 : size[11:0] | length[23:12] | offset[28:24] | sosf[29] | eof[30] | owner[31]
 *   w1 : buffer pointer (guest physical)
 *   w2 : next descriptor pointer
 */
#define SLC_DESC_SOSF    (1u << 29)
#define SLC_DESC_EOF     (1u << 30)
#define SLC_DESC_OWNER   (1u << 31)
#define SLC_RX_QUEUE_MAX 64

typedef struct Esp32SlcRxPacket {
    size_t len;
    int64_t ready_ns;
    uint8_t data[];
} Esp32SlcRxPacket;

static void esp32_slc_desc_read(hwaddr addr, uint32_t *w0, uint32_t *buf,
                                uint32_t *next)
{
    uint32_t w[3];
    cpu_physical_memory_read(addr, w, sizeof(w));
    *w0   = w[0];
    *buf  = w[1];
    *next = w[2];
}

/* ---- TX: walk guest TX descriptor chain -> wifi_tx ring (host side) ---- */
static void esp32_slc_tx_start(Esp32SlcState *s)
{
    hwaddr head = s->regs[SLC_0_TXPKT_H_DSCR_REG / 4];
    if (!head) {
        return;
    }

    uint8_t frame[QEMU_WIFI_FRAME_MAX];
    int flen = 0;
    hwaddr cur = head;

    for (int i = 0; cur && i < 256 && flen < QEMU_WIFI_FRAME_MAX; i++) {
        uint32_t w0, buf, next;
        esp32_slc_desc_read(cur, &w0, &buf, &next);

        uint32_t size   = w0 & 0xFFF;
        uint32_t length = (w0 >> 12) & 0xFFF;
        uint32_t offset = (w0 >> 24) & 0x1F;
        uint32_t eof    = (w0 >> 30) & 1;
        uint32_t n      = length ? length : size;

        if (n > (uint32_t)(QEMU_WIFI_FRAME_MAX - flen)) {
            n = QEMU_WIFI_FRAME_MAX - flen;
        }
        if (buf) {
            cpu_physical_memory_read(buf + offset, frame + flen, n);
        }
        flen += n;

        /* mark descriptor done: owner = software (0) */
        w0 &= ~SLC_DESC_OWNER;
        cpu_physical_memory_write(cur, &w0, 4);

        if (eof) {
            break;
        }
        cur = next;
    }

    if (s->nic && qemu_get_queue(s->nic)->peer && flen > 0) {
        qemu_send_packet(qemu_get_queue(s->nic), frame, flen);
    } else if (m_arena && flen > 0) {
        uint32_t tail = m_arena->wifi_tx.tail;
        uint32_t nxt  = (tail + 1) % QEMU_WIFI_RING_FRAMES;
        if (nxt != m_arena->wifi_tx.head) {
            qemuWifiFrame_t *f = (qemuWifiFrame_t *)&m_arena->wifi_tx.frames[tail];
            f->len = flen;
            memcpy(f->data, frame, flen);
            m_arena->wifi_tx.tail = nxt;
            m_arena->wifi_tx.seq++;
        } else if (slc_debug) {
            printf("SLC TX ring full, dropping frame (%d bytes)\n", flen);
        }
    }
}

/* ---- RX: network backend / wifi_rx ring -> guest RX descriptors ---- */
static bool esp32_slc_rx_ready(Esp32SlcState *s)
{
    uint32_t w0, buf, next;

    if (!s->rx_dsc_cur) {
        return false;
    }
    esp32_slc_desc_read(s->rx_dsc_cur, &w0, &buf, &next);
    return (w0 & SLC_DESC_OWNER) != 0;
}

static bool esp32_slc_deliver_rx(Esp32SlcState *s, const uint8_t *data,
                                 size_t len)
{
    hwaddr cur = s->rx_dsc_cur;
    uint32_t w0, buf, next;

    if (!cur) {
        return false;
    }
    esp32_slc_desc_read(cur, &w0, &buf, &next);
    if (!(w0 & SLC_DESC_OWNER)) {
        return false;
    }

    uint32_t desc_size = w0 & 0xFFF;
    uint32_t offset = (w0 >> 24) & 0x1F;
    uint32_t n = MIN(len, desc_size);

    if (buf) {
        cpu_physical_memory_write(buf + offset, data, n);
    }
    w0 = (desc_size & 0xFFF)
       | ((n & 0xFFF) << 12)
       | ((offset & 0x1F) << 24)
       | (w0 & SLC_DESC_SOSF)
       | SLC_DESC_EOF;
    cpu_physical_memory_write(cur, &w0, 4);
    s->rx_dsc_cur = next;
    qemu_set_irq(s->wifi_irq, 1);

    if (slc_debug) {
        printf("SLC RX delivered %u bytes to dscr 0x%llx\n",
               n, (unsigned long long)cur);
    }
    return true;
}

static void esp32_slc_try_rx(Esp32SlcState *s)
{
    if (!m_arena || !s->rx_dsc_cur) {
        return;
    }

    while (m_arena->wifi_rx.head != m_arena->wifi_rx.tail) {
        uint32_t head = m_arena->wifi_rx.head;
        qemuWifiFrame_t *f = (qemuWifiFrame_t *)&m_arena->wifi_rx.frames[head];

        if (!esp32_slc_deliver_rx(s, f->data, f->len)) {
            break;
        }
        m_arena->wifi_rx.head = (head + 1) % QEMU_WIFI_RING_FRAMES;
    }
}

static bool esp32_slc_can_receive(NetClientState *nc)
{
    Esp32SlcState *s = qemu_get_nic_opaque(nc);
    return g_queue_get_length(&s->rx_queue) < SLC_RX_QUEUE_MAX;
}

static ssize_t esp32_slc_receive(NetClientState *nc, const uint8_t *buf,
                                 size_t size)
{
    Esp32SlcState *s = qemu_get_nic_opaque(nc);
    Esp32SlcRxPacket *packet = g_malloc(sizeof(*packet) + size);

    packet->len = size;
    /* A real DMA completion is asynchronous. Deferring backend responses also
     * gives ESP-NETIF time to publish link-up before the first DHCP offer. */
    packet->ready_ns = qemu_clock_get_ns(QEMU_CLOCK_HOST) + 100 * SCALE_MS;
    memcpy(packet->data, buf, size);
    g_queue_push_tail(&s->rx_queue, packet);
    return size;
}

static NetClientInfo net_esp32_slc_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = esp32_slc_can_receive,
    .receive = esp32_slc_receive,
};

static void esp32_slc_rx_tick(void *opaque)
{
    Esp32SlcState *s = ESP32_SLC(opaque);
    /* emulate a level pulse: clear before re-delivering */
    qemu_set_irq(s->wifi_irq, 0);
    while (esp32_slc_rx_ready(s) && !g_queue_is_empty(&s->rx_queue)) {
        Esp32SlcRxPacket *packet = g_queue_peek_head(&s->rx_queue);

        if (packet->ready_ns > qemu_clock_get_ns(QEMU_CLOCK_HOST)) {
            break;
        }
        g_queue_pop_head(&s->rx_queue);

        esp32_slc_deliver_rx(s, packet->data, packet->len);
        g_free(packet);
    }
    esp32_slc_try_rx(s);
    if (s->nic && esp32_slc_can_receive(qemu_get_queue(s->nic))) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
    timer_mod(s->rx_timer, qemu_clock_get_ns(QEMU_CLOCK_HOST) + SCALE_MS);
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

    if (addr == SLC_0TX_LINK_REG) {
        if (v & (SLC_TXLINK_START | SLC_TXLINK_RESTART)) {
            esp32_slc_tx_start(s);
        }
    } else if (addr == SLC_0RX_LINK_REG) {
        if (v & (SLC_TXLINK_START | SLC_TXLINK_RESTART)) {
            s->rx_dsc_cur = s->regs[SLC_0_RXPKT_H_DSCR_REG / 4];
            esp32_slc_try_rx(s);
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
    s->rx_dsc_cur = 0;
    g_queue_clear_full(&s->rx_queue, g_free);
}

static void esp32_slc_realize(DeviceState *dev, Error **errp)
{
    Esp32SlcState *s = ESP32_SLC(dev);
    esp32_slc_update_debug();
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_esp32_slc_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_HOST, esp32_slc_rx_tick, s);
    timer_mod(s->rx_timer, qemu_clock_get_ns(QEMU_CLOCK_HOST) + SCALE_MS);
}

static void esp32_slc_init(Object *obj)
{
    Esp32SlcState *s = ESP32_SLC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    g_queue_init(&s->rx_queue);
    memory_region_init_io(&s->iomem, obj, &esp32_slc_ops, s, TYPE_ESP32_SLC,
                          ESP32_SLC_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->wifi_irq);
}

static void esp32_slc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    static Property properties[] = {
        DEFINE_NIC_PROPERTIES(Esp32SlcState, conf),
        DEFINE_PROP_END_OF_LIST(),
    };

    dc->legacy_reset = esp32_slc_reset;
    dc->realize = esp32_slc_realize;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    device_class_set_props(dc, properties);
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
