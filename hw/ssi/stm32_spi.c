/*-
 * Copyright (c) 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
 * QEMU model of the stm32f2xx SPI controller.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
//#include "hw/ssi/ssi.h"

#include "../../system/simuliface.h"

#define CR1_OFFSET     0x00
#define CR2_OFFSET     0x04
#define SR_OFFSET      0x08
#define DR_OFFSET      0x0C
#define CRCPR_OFFSET   0x10
#define RXCRCR_OFFSET  0x14
#define TXCRCR_OFFSET  0x18
#define I2SCFGR_OFFSET 0x1C
#define I2SPR_OFFSET   0x20

#define R_SR_RESET      0x0002
#define R_CRCPR_RESET   0x0007
#define R_I2SPR_RESET   0x0002

#define R_CR1_DFF      (1 << 11)
#define R_CR1_LSBFIRST (1 <<  7)
#define R_CR1_SPE      (1 <<  6)

#define R_SR_MASK       0x01FF
#define R_SR_OVR       (1 << 6)
#define R_SR_TXE       (1 << 1)
#define R_SR_RXNE      (1 << 0)

enum stm32_spi_action_t {
    STM32_SPI_CR1=1,
    STM32_SPI_CR2,
    STM32_SPI_READ,
    STM32_SPI_WRITE
};

typedef struct Stm32Spi {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq irq;

    uint8_t number;

    //SSIBus *spi;

    stm32_periph_t periph;

    int32_t rx;
    int rx_full; 
    uint16_t regs[9];
} Stm32Spi;

static void stm32_spi_write_CR1( Stm32Spi *s, uint16_t newCR1 )
{
    if( s->regs[CR1_OFFSET/4] == newCR1 ) return;
    s->regs[CR1_OFFSET/4] = newCR1;

    //if ((data & R_CR1_DFF) != s->regs[CR1_OFFSET/4] && (s->regs[CR1_OFFSET/4] & R_CR1_SPE) != 0)
    //    qemu_log_mask(LOG_GUEST_ERROR, "cannot change DFF with SPE set\n");

    // Bit 0  CPHA:     Clock phase
    // Bit 1  CPOL:     Clock polarity
    // Bit 2  MSTR:     Master selection: 1 = Master
    // Bit 3-5 BR:      Baud rate control: 2,4,8,16,32,64,128,256
    // Bit 6  SPE:      SPI enable
    // Bit 7  LSBFIRST: Frame format: 1 = LSB first
    // Bit 8  SSI:      Internal slave select
    // Bit 9  SSM:      Software slave management
    // Bit 10 RXONLY:   Receive only
    // Bit 11 DFF:      Data frame format: 1 = 16 bit
    // Bit 12 CRCNEXT:  CRC transfer next
    // Bit 13 CRCEN:    Hardware CRC calculation enable
    // Bit 14 BIDIOE:   Output enable in bidirectional mode
    // Bit 15 BIDIMODE: Bidirectional data mode enable

    if( !m_arena->running ) return;   // check if simulation is still running
    m_arena->simuAction = SIM_SPI;
    m_arena->data8  = STM32_SPI_CR1;
    m_arena->data16 = s->number;          // Spi number
    m_arena->data32 = newCR1;
    doAction();
}

static void stm32_spi_write_CR2( Stm32Spi *s, uint8_t newCR2 )
{
    if( s->regs[CR2_OFFSET/4] == newCR2 ) return;
    s->regs[CR2_OFFSET/4] = newCR2;

    // Bit 0 RXDMAEN: Rx buffer DMA enable
    // Bit 1 TXDMAEN: Tx buffer DMA enable
    // Bit 2 SSOE:    SS output enable
    // Bits 4:3       Reserved
    // Bit 5 ERRIE:   Error interrupt enable
    // Bit 6 RXNEIE:  RX buffer not empty interrupt enable
    // Bit 7 TXEIE:   Tx buffer empty interrupt enable

    if( !m_arena->running ) return;   // check if simulation is still running
    m_arena->simuAction = SIM_SPI;
    m_arena->data8  = STM32_SPI_CR2;
    m_arena->data16 = s->number;          // Spi number
    m_arena->data32 = newCR2;
    doAction();
}

static void stm32_spi_write_DR( Stm32Spi *s, uint8_t newDR )
{
    s->regs[SR_OFFSET/4] &= ~R_SR_TXE;
    if (s->regs[SR_OFFSET/4] & R_SR_RXNE) s->regs[SR_OFFSET/4] |= R_SR_OVR;

    //if (s->regs[CR1_OFFSET/4] & R_CR1_LSBFIRST) ; s->regs[DR_OFFSET/4] = bitswap(ssi_transfer(s->spi, bitswap(data)));
    //else                                        ; s->regs[DR_OFFSET/4] = ssi_transfer(s->spi, data);

    /// SEND DATA

    s->regs[SR_OFFSET/4] |= R_SR_RXNE | R_SR_TXE;

    if( !m_arena->running ) return;   // check if simulation is still running
    m_arena->simuAction = SIM_SPI;
    m_arena->data8  = STM32_SPI_WRITE;
    m_arena->data16 = s->number;          // Spi number
    m_arena->data32 = newDR;
    doAction();
}

static void stm32_spi_write_I2SCFGR( Stm32Spi *s, uint8_t newI2SCFGR )
{
    if( s->regs[I2SCFGR_OFFSET/4] == newI2SCFGR ) return;
    s->regs[I2SCFGR_OFFSET/4] = newI2SCFGR;
}

static void stm32_spi_write_I2SPR( Stm32Spi *s, uint8_t newI2SPR )
{
    if( s->regs[I2SPR_OFFSET/4] == newI2SPR ) return;
    s->regs[I2SPR_OFFSET/4] = newI2SPR;
}

static uint64_t stm32_spi_read( void *arg, hwaddr offset, unsigned size )
{
    Stm32Spi *s = arg;
    if( offset > 0x20 ) return 0;
    uint16_t r = s->regs[offset/4];

    switch (offset) {
    case DR_OFFSET:
        s->regs[SR_OFFSET/4] &= ~R_SR_RXNE;

        if( !m_arena->running ) break;   // check if simulation is still running
        m_arena->simuAction = SIM_SPI;
        m_arena->data8  = STM32_SPI_READ;
        m_arena->data16 = s->number;          // Spi number
        doAction();

        r = m_arena->data16;
        break;
    }

    return r;
}

//static uint8_t bitswap(uint8_t val)
//{
//    return ((val * 0x0802LU & 0x22110LU) | (val * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
//}

static void stm32_spi_write(void *arg, hwaddr addr, uint64_t data, unsigned size)
{
    struct Stm32Spi *s = (struct Stm32Spi *)arg;
    //int offset = addr & 0x3;

    /* SPI registers are all at most 16 bits wide */
    //data &= 0xFFFFF;
    //addr >>= 2;

    //switch (size) {
    //    case 1: data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8); break;
    //    case 2: data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8); break;
    //    case 4: break;
    //    default: abort();
    //}

    switch (addr) {
    case CR1_OFFSET:     stm32_spi_write_CR1( s, data );     break;
    case CR2_OFFSET:     stm32_spi_write_CR2( s, data );     break;
    case SR_OFFSET :                                         break;
    case DR_OFFSET:      stm32_spi_write_DR( s, data );      break;
    case CRCPR_OFFSET:   s->regs[CRCPR_OFFSET/4] = data;     break;
    case RXCRCR_OFFSET:                                      break;
    case TXCRCR_OFFSET:                                      break;
    case I2SCFGR_OFFSET: stm32_spi_write_I2SCFGR( s, data ); break;
    case I2SPR_OFFSET:   stm32_spi_write_I2SPR( s, data );   break;
    //default:
    //    if (addr < ARRAY_SIZE(s->regs)) s->regs[addr] = data;
    //    else                            STM32_BAD_REG(addr, size);
    }
}

static const MemoryRegionOps stm32_spi_ops = {
    .read = stm32_spi_read,
    .write = stm32_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_spi_reset(DeviceState *dev)
{
    struct Stm32Spi *s = STM32_SPI(dev);

    s->regs[SR_OFFSET/4] = R_SR_RESET;
    switch (s->periph) {
    case 0: break;
    case 1: break;
    default: break;
    }
}

static void stm32_spi_init(Object *obj)
{
    struct Stm32Spi *s = STM32_SPI(obj);

    memory_region_init_io(&s->iomem, obj, &stm32_spi_ops, s, "spi", 0x3ff);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    //s->spi = ssi_create_bus(DEVICE(obj), "ssi");

    return;
}

void stm32_spi_set_number( Stm32Spi *spi, int spi_num ) {
    spi->number = spi_num -1;
}

static Property stm32_spi_properties[] = {
    DEFINE_PROP_INT32("periph", struct Stm32Spi, periph, -1),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    //dc->reset = stm32_spi_reset;
    device_class_set_legacy_reset( dc,stm32_spi_reset);
    device_class_set_props(dc, stm32_spi_properties);
}

static const TypeInfo stm32_spi_info = {
    .name = TYPE_STM32_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct Stm32Spi),
    .instance_init = stm32_spi_init,
    .class_init = stm32_spi_class_init
};

static void
stm32_spi_register_types(void)
{
    type_register_static(&stm32_spi_info);
}

type_init(stm32_spi_register_types)
