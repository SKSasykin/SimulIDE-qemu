#pragma once

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"

#define TYPE_ESP32_SLC "esp32.slc"
#define ESP32_SLC(obj) OBJECT_CHECK(Esp32SlcState, (obj), TYPE_ESP32_SLC)

#define ESP32_SLC_SIZE 0x1000
#define ESP32_SLC_REG_COUNT (ESP32_SLC_SIZE / 4)

#define SLC_CONF_REG                 0x00
#define SLC_0INT_RAW_REG             0x04
#define SLC_0INT_ST_REG              0x08
#define SLC_0INT_ENA_REG             0x0c
#define SLC_0INT_CLR_REG             0x10
#define SLC_0RX_LINK_REG             0x3c
#define SLC_0TX_LINK_REG             0x40
#define SLC_0TOKEN0_REG              0x50
#define SLC_0TOKEN1_REG              0x54
#define SLC_CONF1_REG                0x60
#define SLC_0_LEN_CONF_REG           0xe4
#define SLC_0_LENGTH_REG             0xe8
#define SLC_0_TXPKT_H_DSCR_REG       0xec
#define SLC_0_TXPKT_E_DSCR_REG       0xf0
#define SLC_0_RXPKT_H_DSCR_REG       0xf4
#define SLC_0_RXPKT_E_DSCR_REG       0xf8
#define SLC_0_TXPKTU_H_DSCR_REG      0xfc
#define SLC_0_DSCR_REC_CONF_REG      0x118

#define SLC_TXLINK_PARK   BIT(31)
#define SLC_TXLINK_RESTART BIT(30)
#define SLC_TXLINK_START  BIT(29)
#define SLC_TXLINK_STOP   BIT(28)
#define SLC_TXLINK_ADDR_MASK 0x000fffff

typedef struct Esp32SlcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t regs[ESP32_SLC_REG_COUNT];

    SSIBus *spi;
} Esp32SlcState;
