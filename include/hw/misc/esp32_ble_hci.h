/*
 * SimulIDE ESP BLE HCI transport.
 *
 * Descriptor ABI (16 bytes, 16-byte aligned, little-endian):
 *   +0x00 u32 buffer_address
 *   +0x04 u32 length
 *   +0x08 u32 next_descriptor (zero terminates the chain)
 *   +0x0c u32 flags (bit 31 OWN; all other bits must be zero)
 *
 * For TX, length is the complete H4 packet size. For RX, length is the buffer
 * capacity while OWN is set and is replaced by the received H4 packet size
 * before OWN is cleared. The guest sets OWN; the device clears it. A packet is
 * never split across descriptors or truncated.
 */
#ifndef HW_MISC_ESP32_BLE_HCI_H
#define HW_MISC_ESP32_BLE_HCI_H

#include "exec/memory.h"
#include "hw/irq.h"

#define TYPE_ESP32_BLE_HCI "esp32-ble-hci"

void esp32_ble_hci_create(MemoryRegion *sys_mem, hwaddr base,
                          qemu_irq irq, uint32_t simulide_iomem_offset);

#endif
