/* SimulIDE ESP-family DMA/mailbox BLE HCI transport. */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "exec/address-spaces.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_ble_hci.h"

#include "../../system/simuliface.h"

#define BLE_HCI_MMIO_SIZE       0x1000
#define BLE_HCI_ID_VALUE        0x424c4548 /* "BLEH" */
#define BLE_HCI_VERSION_VALUE   0x00010000
#define BLE_HCI_MAX_FRAME       QEMU_WIFI_FRAME_MAX
#define BLE_HCI_MAX_DESCRIPTORS 1024
#define BLE_HCI_RX_POLL_NS      SCALE_MS

#define BLE_HCI_CONTROL_ENABLE  BIT(0)
#define BLE_HCI_CONTROL_RESET   BIT(1)
#define BLE_HCI_STATUS_ENABLED  BIT(0)
#define BLE_HCI_STATUS_TX_FULL  BIT(1)
#define BLE_HCI_STATUS_RX_WAIT  BIT(2)
#define BLE_HCI_STATUS_ERROR    BIT(3)
#define BLE_HCI_INT_TX_DONE     BIT(0)
#define BLE_HCI_INT_RX_DONE     BIT(1)
#define BLE_HCI_INT_ERROR       BIT(2)
#define BLE_HCI_INT_MASK        (BLE_HCI_INT_TX_DONE | BLE_HCI_INT_RX_DONE | \
                                 BLE_HCI_INT_ERROR)
#define BLE_HCI_DESC_OWN        BIT(31)

enum {
    BLE_HCI_ID = 0x00,
    BLE_HCI_VERSION = 0x04,
    BLE_HCI_CONTROL = 0x08,
    BLE_HCI_STATUS = 0x0c,
    BLE_HCI_INT_RAW = 0x10,
    BLE_HCI_INT_ENA = 0x14,
    BLE_HCI_INT_ST = 0x18,
    BLE_HCI_INT_CLR = 0x1c,
    BLE_HCI_TX_DESC = 0x20,
    BLE_HCI_RX_DESC = 0x24,
    BLE_HCI_TX_KICK = 0x28,
    BLE_HCI_RX_KICK = 0x2c,
    BLE_HCI_MAX_FRAME_REG = 0x30,
};

typedef struct Esp32BleHciDesc {
    uint32_t buffer;
    uint32_t length;
    uint32_t next;
    uint32_t flags;
} Esp32BleHciDesc;

typedef struct Esp32BleHciState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t simulide_iomem_offset;
    uint32_t control;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t tx_desc;
    uint32_t rx_desc;
    uint32_t tx_current;
    uint32_t rx_current;
    QEMUTimer *rx_timer;
    bool error;
} Esp32BleHciState;

#define ESP32_BLE_HCI(obj) \
    OBJECT_CHECK(Esp32BleHciState, (obj), TYPE_ESP32_BLE_HCI)

static void esp32_ble_hci_update_irq(Esp32BleHciState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

static void esp32_ble_hci_fault(Esp32BleHciState *s, const char *what,
                                uint32_t address)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s at 0x%08x\n",
                  TYPE_ESP32_BLE_HCI, what, address);
    s->error = true;
    s->int_raw |= BLE_HCI_INT_ERROR;
    esp32_ble_hci_update_irq(s);
}

static bool esp32_ble_hci_dma_read(uint32_t address, void *buffer, size_t size)
{
    return address_space_read(&address_space_memory, address,
                              MEMTXATTRS_UNSPECIFIED, buffer, size) == MEMTX_OK;
}

static bool esp32_ble_hci_dma_write(uint32_t address, const void *buffer,
                                    size_t size)
{
    return address_space_write(&address_space_memory, address,
                               MEMTXATTRS_UNSPECIFIED, buffer, size) == MEMTX_OK;
}

static bool esp32_ble_hci_read_desc(Esp32BleHciState *s, uint32_t address,
                                    Esp32BleHciDesc *desc)
{
    Esp32BleHciDesc wire;

    if (!address || (address & 0xf)) {
        esp32_ble_hci_fault(s, "unaligned descriptor", address);
        return false;
    }
    if (!esp32_ble_hci_dma_read(address, &wire, sizeof(wire))) {
        esp32_ble_hci_fault(s, "descriptor DMA read failed", address);
        return false;
    }
    desc->buffer = le32_to_cpu(wire.buffer);
    desc->length = le32_to_cpu(wire.length);
    desc->next = le32_to_cpu(wire.next);
    desc->flags = le32_to_cpu(wire.flags);
    if ((desc->next & 0xf) || (desc->flags & ~BLE_HCI_DESC_OWN)) {
        esp32_ble_hci_fault(s, "invalid descriptor", address);
        return false;
    }
    return true;
}

static bool esp32_ble_hci_complete_desc(Esp32BleHciState *s,
                                        uint32_t address,
                                        const Esp32BleHciDesc *desc,
                                        uint32_t length)
{
    Esp32BleHciDesc wire = {
        .buffer = cpu_to_le32(desc->buffer),
        .length = cpu_to_le32(length),
        .next = cpu_to_le32(desc->next),
        .flags = 0,
    };

    if (!esp32_ble_hci_dma_write(address, &wire, sizeof(wire))) {
        esp32_ble_hci_fault(s, "descriptor DMA write failed", address);
        return false;
    }
    return true;
}

static bool esp32_ble_hci_valid_h4(const uint8_t *packet, uint32_t length)
{
    uint32_t payload;

    if (!length) {
        return false;
    }
    switch (packet[0]) {
    case 0x01:
        return length >= 4 && length == 4 + packet[3];
    case 0x02:
        if (length < 5) {
            return false;
        }
        payload = packet[3] | ((uint32_t)packet[4] << 8);
        return length == 5 + payload;
    case 0x03:
        return length >= 4 && length == 4 + packet[3];
    case 0x04:
        return length >= 3 && length == 3 + packet[2];
    case 0x05:
        if (length < 5) {
            return false;
        }
        payload = (packet[3] | ((uint32_t)packet[4] << 8)) & 0x3fff;
        return length == 5 + payload;
    default:
        return false;
    }
}

static bool esp32_ble_hci_tx_full(void)
{
    qemuWifiRing_t *ring;
    uint32_t tail;

    if (!m_arena) {
        return true;
    }
    ring = (qemuWifiRing_t *)&m_arena->bt_tx;
    tail = ring->tail;
    smp_rmb();
    return (tail + 1) % QEMU_WIFI_RING_FRAMES == ring->head;
}

static bool esp32_ble_hci_rx_pending(void)
{
    qemuWifiRing_t *ring;

    if (!m_arena) {
        return false;
    }
    ring = (qemuWifiRing_t *)&m_arena->bt_rx;
    return ring->head != ring->tail;
}

static bool esp32_ble_hci_ring_valid(Esp32BleHciState *s,
                                     qemuWifiRing_t *ring)
{
    if (ring->head < QEMU_WIFI_RING_FRAMES &&
        ring->tail < QEMU_WIFI_RING_FRAMES) {
        return true;
    }
    esp32_ble_hci_fault(s, "invalid shared ring indices", 0);
    return false;
}

static void esp32_ble_hci_rx(Esp32BleHciState *s)
{
    qemuWifiRing_t *ring;
    unsigned traversed = 0;
    bool consumed = false;

    if (!(s->control & BLE_HCI_CONTROL_ENABLE) || !m_arena) {
        return;
    }
    ring = (qemuWifiRing_t *)&m_arena->bt_rx;
    if (!esp32_ble_hci_ring_valid(s, ring)) {
        return;
    }
    while (ring->head != ring->tail && s->rx_current) {
        Esp32BleHciDesc desc;
        qemuWifiFrame_t *frame;
        uint32_t index = ring->head;
        uint32_t length;

        smp_rmb();
        frame = &ring->frames[index];
        length = frame->len;
        if (++traversed > BLE_HCI_MAX_DESCRIPTORS) {
            esp32_ble_hci_fault(s, "RX descriptor traversal limit", s->rx_current);
            goto done;
        }
        if (!esp32_ble_hci_read_desc(s, s->rx_current, &desc)) {
            goto done;
        }
        if (!(desc.flags & BLE_HCI_DESC_OWN)) {
            goto done;
        }
        if (!length || length > BLE_HCI_MAX_FRAME || length > desc.length ||
            !esp32_ble_hci_valid_h4(frame->data, length)) {
            esp32_ble_hci_fault(s, "invalid or oversized RX packet", s->rx_current);
            goto done;
        }
        if (!esp32_ble_hci_dma_write(desc.buffer, frame->data, length)) {
            esp32_ble_hci_fault(s, "RX buffer DMA write failed", desc.buffer);
            goto done;
        }
        if (!esp32_ble_hci_complete_desc(s, s->rx_current, &desc, length)) {
            goto done;
        }
        s->rx_current = desc.next;
        smp_wmb();
        ring->head = (index + 1) % QEMU_WIFI_RING_FRAMES;
        consumed = true;
        s->int_raw |= BLE_HCI_INT_RX_DONE;
    }
done:
    if (consumed) {
        /* Freeing RX space may unblock a command waiting in the host. */
        simulide_bt_notify(s->simulide_iomem_offset);
    }
    esp32_ble_hci_update_irq(s);
}

static void esp32_ble_hci_tx(Esp32BleHciState *s)
{
    qemuWifiRing_t *ring;
    unsigned traversed = 0;

    if (!(s->control & BLE_HCI_CONTROL_ENABLE) || !m_arena) {
        return;
    }
    ring = (qemuWifiRing_t *)&m_arena->bt_tx;
    if (!esp32_ble_hci_ring_valid(s, ring)) {
        return;
    }
    while (s->tx_current && !esp32_ble_hci_tx_full()) {
        Esp32BleHciDesc desc;
        uint8_t packet[BLE_HCI_MAX_FRAME];
        uint32_t index = ring->tail;

        if (++traversed > BLE_HCI_MAX_DESCRIPTORS) {
            esp32_ble_hci_fault(s, "TX descriptor traversal limit", s->tx_current);
            return;
        }
        if (!esp32_ble_hci_read_desc(s, s->tx_current, &desc)) {
            return;
        }
        if (!(desc.flags & BLE_HCI_DESC_OWN)) {
            break;
        }
        if (!desc.length || desc.length > BLE_HCI_MAX_FRAME ||
            !esp32_ble_hci_dma_read(desc.buffer, packet, desc.length)) {
            esp32_ble_hci_fault(s, "invalid TX length or DMA read", desc.buffer);
            return;
        }
        if (!esp32_ble_hci_valid_h4(packet, desc.length)) {
            esp32_ble_hci_fault(s, "invalid H4 TX packet", desc.buffer);
            return;
        }
        if (!esp32_ble_hci_complete_desc(s, s->tx_current, &desc,
                                         desc.length)) {
            return;
        }
        s->tx_current = desc.next;
        s->int_raw |= BLE_HCI_INT_TX_DONE;

        ring->frames[index].len = desc.length;
        memcpy(ring->frames[index].data, packet, desc.length);
        ring->seq++;
        smp_wmb();
        ring->tail = (index + 1) % QEMU_WIFI_RING_FRAMES;

        simulide_bt_notify(s->simulide_iomem_offset);
        esp32_ble_hci_rx(s);
    }
    esp32_ble_hci_update_irq(s);
}

static void esp32_ble_hci_rx_tick(void *opaque)
{
    Esp32BleHciState *s = opaque;

    esp32_ble_hci_rx(s);
    esp32_ble_hci_tx(s);
    timer_mod(s->rx_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + BLE_HCI_RX_POLL_NS);
}

static uint32_t esp32_ble_hci_status(Esp32BleHciState *s)
{
    uint32_t status = 0;

    if (s->control & BLE_HCI_CONTROL_ENABLE) {
        status |= BLE_HCI_STATUS_ENABLED;
    }
    if (esp32_ble_hci_tx_full()) {
        status |= BLE_HCI_STATUS_TX_FULL;
    }
    if (esp32_ble_hci_rx_pending()) {
        status |= BLE_HCI_STATUS_RX_WAIT;
    }
    if (s->error) {
        status |= BLE_HCI_STATUS_ERROR;
    }
    return status;
}

static uint64_t esp32_ble_hci_read(void *opaque, hwaddr address, unsigned size)
{
    Esp32BleHciState *s = opaque;

    esp32_ble_hci_rx(s);
    switch (address) {
    case BLE_HCI_ID: return BLE_HCI_ID_VALUE;
    case BLE_HCI_VERSION: return BLE_HCI_VERSION_VALUE;
    case BLE_HCI_CONTROL: return s->control;
    case BLE_HCI_STATUS: return esp32_ble_hci_status(s);
    case BLE_HCI_INT_RAW: return s->int_raw;
    case BLE_HCI_INT_ENA: return s->int_ena;
    case BLE_HCI_INT_ST: return s->int_raw & s->int_ena;
    case BLE_HCI_TX_DESC: return s->tx_desc;
    case BLE_HCI_RX_DESC: return s->rx_desc;
    case BLE_HCI_MAX_FRAME_REG: return BLE_HCI_MAX_FRAME;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad register read 0x%" HWADDR_PRIx "\n",
                      TYPE_ESP32_BLE_HCI, address);
        return 0;
    }
}

static void esp32_ble_hci_reset_state(Esp32BleHciState *s)
{
    if (m_arena) {
        qemuWifiRing_t *tx_ring = (qemuWifiRing_t *)&m_arena->bt_tx;
        qemuWifiRing_t *rx_ring = (qemuWifiRing_t *)&m_arena->bt_rx;

        /* Only move the indices owned by QEMU while discarding stale traffic. */
        if (tx_ring->head < QEMU_WIFI_RING_FRAMES) {
            smp_rmb();
            tx_ring->tail = tx_ring->head;
        }
        if (rx_ring->tail < QEMU_WIFI_RING_FRAMES) {
            smp_rmb();
            rx_ring->head = rx_ring->tail;
        }
        smp_wmb();
    }
    s->control = 0;
    s->int_raw = 0;
    s->int_ena = 0;
    s->tx_desc = 0;
    s->rx_desc = 0;
    s->tx_current = 0;
    s->rx_current = 0;
    s->error = false;
    esp32_ble_hci_update_irq(s);
}

static void esp32_ble_hci_write(void *opaque, hwaddr address,
                                uint64_t value, unsigned size)
{
    Esp32BleHciState *s = opaque;
    uint32_t val = value;

    switch (address) {
    case BLE_HCI_CONTROL:
        if (val & BLE_HCI_CONTROL_RESET) {
            esp32_ble_hci_reset_state(s);
        }
        s->control = val & BLE_HCI_CONTROL_ENABLE;
        if (s->control) {
            esp32_ble_hci_rx(s);
        }
        break;
    case BLE_HCI_INT_ENA:
        s->int_ena = val & BLE_HCI_INT_MASK;
        esp32_ble_hci_update_irq(s);
        break;
    case BLE_HCI_INT_CLR:
        s->int_raw &= ~(val & BLE_HCI_INT_MASK);
        if (val & BLE_HCI_INT_ERROR) {
            s->error = false;
        }
        esp32_ble_hci_update_irq(s);
        break;
    case BLE_HCI_TX_DESC:
        if (val & 0xf) {
            esp32_ble_hci_fault(s, "unaligned TX descriptor base", val);
        } else {
            s->tx_desc = s->tx_current = val;
        }
        break;
    case BLE_HCI_RX_DESC:
        if (val & 0xf) {
            esp32_ble_hci_fault(s, "unaligned RX descriptor base", val);
        } else {
            s->rx_desc = s->rx_current = val;
        }
        break;
    case BLE_HCI_TX_KICK:
        esp32_ble_hci_tx(s);
        break;
    case BLE_HCI_RX_KICK:
        esp32_ble_hci_rx(s);
        esp32_ble_hci_tx(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad register write 0x%" HWADDR_PRIx "\n",
                      TYPE_ESP32_BLE_HCI, address);
        break;
    }
}

static const MemoryRegionOps esp32_ble_hci_ops = {
    .read = esp32_ble_hci_read,
    .write = esp32_ble_hci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
    .impl = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
};

static void esp32_ble_hci_reset(DeviceState *dev)
{
    esp32_ble_hci_reset_state(ESP32_BLE_HCI(dev));
}

static void esp32_ble_hci_init(Object *obj)
{
    Esp32BleHciState *s = ESP32_BLE_HCI(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_ble_hci_ops, s,
                          TYPE_ESP32_BLE_HCI, BLE_HCI_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void esp32_ble_hci_realize(DeviceState *dev, Error **errp)
{
    Esp32BleHciState *s = ESP32_BLE_HCI(dev);

    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32_ble_hci_rx_tick, s);
    timer_mod(s->rx_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + BLE_HCI_RX_POLL_NS);
}

static void esp32_ble_hci_unrealize(DeviceState *dev)
{
    Esp32BleHciState *s = ESP32_BLE_HCI(dev);

    timer_free(s->rx_timer);
    s->rx_timer = NULL;
}

static Property esp32_ble_hci_properties[] = {
    DEFINE_PROP_UINT32("simulide-iomem-offset", Esp32BleHciState,
                       simulide_iomem_offset, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_ble_hci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "SimulIDE ESP BLE HCI DMA transport";
    dc->legacy_reset = esp32_ble_hci_reset;
    dc->realize = esp32_ble_hci_realize;
    dc->unrealize = esp32_ble_hci_unrealize;
    device_class_set_props(dc, esp32_ble_hci_properties);
}

static const TypeInfo esp32_ble_hci_info = {
    .name = TYPE_ESP32_BLE_HCI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32BleHciState),
    .instance_init = esp32_ble_hci_init,
    .class_init = esp32_ble_hci_class_init,
};

static void esp32_ble_hci_register_types(void)
{
    type_register_static(&esp32_ble_hci_info);
}
type_init(esp32_ble_hci_register_types)

void esp32_ble_hci_create(MemoryRegion *sys_mem, hwaddr base,
                          qemu_irq irq, uint32_t simulide_iomem_offset)
{
    DeviceState *dev = qdev_new(TYPE_ESP32_BLE_HCI);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    qdev_prop_set_uint32(dev, "simulide-iomem-offset", simulide_iomem_offset);
    sysbus_realize_and_unref(sbd, &error_fatal);
    memory_region_add_subregion(sys_mem, base, sysbus_mmio_get_region(sbd, 0));
    sysbus_connect_irq(sbd, 0, irq);
}
