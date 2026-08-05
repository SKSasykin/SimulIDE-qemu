/*
 * STM32 Microcontroller UART module
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Source code based on pl011.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/arm/stm32.h"

#include "hw/hw.h"
#include "hw/irq.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"

#include "../../system/simuliface.h"

/* DEFINITIONS*/

#define SR_OFFSET 0x00
#define SR_TXE_BIT 7
#define SR_TC_BIT 6
#define SR_RXNE_BIT 5
#define SR_ORE_BIT 3

#define DR_OFFSET 0x04

#define BRR_OFFSET 0x08

#define CR1_OFFSET 0x0c
#define CR1_UE_BIT    13
//#define CR1_M_BIT     12
//#define CR1_PCE_BIT   10
//#define CR1_PS_BIT     9
#define CR1_TXEIE_BIT  7
#define CR1_TCIE_BIT   6
#define CR1_RXNEIE_BIT 5
#define CR1_TE_BIT     3
#define CR1_RE_BIT     2

#define CR2_OFFSET 0x10
#define CR2_r_STOP_START 12
#define CR2_r_STOP_MASK 0x00003000

#define CR3_OFFSET 0x14
#define CR3_r_CTSE_BIT 9
#define CR3_r_RTSE_BIT 8

#define CR3_r_DMAR_BIT 0x40
#define CR3_r_DMAT_BIT 0x80

#define GTPR_OFFSET 0x18

enum stm32_usart_action {
    SIM_USART_READ=1,
    SIM_USART_WRITE,
    SIM_USART_BAUD
};

struct Stm32Uart {
  /* Inherited */
  SysBusDevice busdev;

  /* Properties */
  stm32_periph_t periph;

  /* Private */
  MemoryRegion iomem;

  Stm32Rcc *stm32_rcc;

  uint32_t bits_per_sec;
  int64_t ns_per_char;

  /* Register Values */
  uint32_t RDR_r, TDR_r, BRR_r, CR1_r, CR2_r, CR3_r, SR_r;

  /* Register Field Values */
  uint32_t SR_TXE, SR_TC, SR_RXNE, SR_ORE;
  uint32_t CR1_UE, CR1_TE, CR1_RE;

  uint32_t intEnable;
  uint32_t intFlags;

  bool sr_read_with_ore;

  //bool receiving; // Indicates whether the USART is currently receiving a byte.

  /* Timers used to simulate a delay corresponding to the baud rate. */
  //struct QEMUTimer *rx_timer;
  struct QEMUTimer *tx_timer;
  struct QEMUTimer *tx_dma_timer;

  qemu_irq irq;

  bool curr_irq_level;
  int number;
};

static void stm32_uart_baud_update( Stm32Uart *s ) // Update the baud rate based on the USART's peripheral clock frequency.
{
    uint32_t clk_freq = stm32_rcc_get_periph_freq( s->stm32_rcc, s->periph );

    if( s->BRR_r && clk_freq ) s->bits_per_sec = clk_freq / s->BRR_r;
    else                       s->bits_per_sec = 9600; //use 0 cause simulation freeze because use timer with 0 delay

    /* We assume 10 bits per character. This may not be exactly accurate depending on settings, but it should be good enough. */
    s->ns_per_char = 10 * 1000000000LL / s->bits_per_sec;

    if( !m_arena->running ) return;   // check if simulation is still running

    m_arena->simuAction = SIM_USART;
    m_arena->data8  = SIM_USART_BAUD;
    m_arena->data16 = s->number;          // Uart number
    m_arena->data32 = s->bits_per_sec;

    doAction();
}

static void stm32_uart_clk_irq_handler( void *opaque, int n, int level ) // Handle a change in the peripheral clock.
{
    Stm32Uart *s = (Stm32Uart *)opaque;

    assert(n == 0);

    if( level ) stm32_uart_baud_update(s); /* Only update the BAUD rate if the IRQ is being set. */
}

static void stm32_uart_update_irq( Stm32Uart *s ) // Called whenever an interrupt-related flag is updated.
{
    s->intFlags = s->SR_TC | s->SR_TXE | s->SR_RXNE | s->SR_ORE; // Interupts currently raised

    bool new_irq_level = (s->intEnable & s->intFlags) > 0;       // Filter interrupts enabled

    if( new_irq_level != s->curr_irq_level ) // Only trigger an interrupt if the IRQ level changes.
    {
        qemu_set_irq( s->irq, new_irq_level );
        s->curr_irq_level = new_irq_level;
    }
}

static void stm32_uart_start_tx( Stm32Uart *s );

static void stm32_uart_tx_complete( Stm32Uart *s ) /* Routine to be called when a transmit is complete. */
{
    if( s->SR_TXE ) {              // Buffer is empty
        s->SR_TC = 1<<SR_TC_BIT;   // Mark the transmit complete
    }
    else {                         // Buffer not empty.
        s->SR_TXE = 1<<SR_TXE_BIT; // Mark transmit buffer empty
        stm32_uart_start_tx( s );  // start transmitting
    }
    stm32_uart_update_irq(s);
}

static void stm32_uart_start_tx( Stm32Uart *s ) // Start transmitting a byte.
{
    if( !s->CR1_UE || !s->CR1_TE ) return; // Transmitter disabled

    s->SR_TC = 0; // Reset the Transmission Complete flag to indicate a transmit is in progress.

    if( !m_arena->running ) return;       // check if simulation is still running

    m_arena->simuAction = SIM_USART;
    m_arena->data8  = SIM_USART_WRITE;
    m_arena->data16 = s->number;        // Uart number
    m_arena->data32 = s->TDR_r;

    doAction();

    uint64_t time_ns = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
    timer_mod( s->tx_timer, time_ns + s->ns_per_char ); // Start the transmit delay timer.
}

//static void stm32_uart_rx_timer_expire( void *opaque ) // Receive delay complete, mark the receive as complete
//{
//    Stm32Uart *s = (Stm32Uart*)opaque;
//
//    //s->receiving = false;
//
//    uint64_t curr_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
//    timer_mod(s->rx_timer, curr_time + s->ns_per_char);
//}

static void stm32_uart_tx_timer_expire( void *opaque ) // Transmit delay complete, mark the transmit as complete
{
    Stm32Uart *s = (Stm32Uart*)opaque;
    stm32_uart_tx_complete( s );
}

static void stm32_uart_tx_dma_timer_expire( void *opaque ) /* DMA tx delay */
{
    Stm32Uart *s = (Stm32Uart*)opaque;
    uint64_t curr_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if( s->CR3_r & CR3_r_DMAT_BIT )
    {
        qemu_set_irq( *stm32_DMA1_irq, 0x00000011 );
        qemu_set_irq( *stm32_DMA1_irq, 0x00000000 );

        stm32_uart_tx_complete(s);

        timer_mod( s->tx_dma_timer, curr_time + s->ns_per_char );
    }
}

void stm32_uart_receive( Stm32Uart* s, const uint8_t data );

void stm32_uart_receive( Stm32Uart* s, const uint8_t data )
{
    printf("Qemu: stm32_uart_receive %u %u\n", s->number, data );
    printf("      at time %lu\n", getQemu_ps() ); fflush( stdout );

    if( s->CR1_UE && s->CR1_RE ) // Module is enabled
    {
        if( s->SR_RXNE ) // Set the overflow flag if buffer not empty
        {
            s->SR_ORE = 1<<SR_ORE_BIT;
            s->sr_read_with_ore = false;
        }
        s->RDR_r = data;             // Save data to buffer
        s->SR_RXNE = 1<<SR_RXNE_BIT; // Mark the buffer as not empty.
        stm32_uart_update_irq(s);
    }

    if( s->CR3_r & CR3_r_DMAR_BIT )  // DMA operation
    {
      qemu_set_irq(*stm32_DMA1_irq, 0x00000010);
      qemu_set_irq(*stm32_DMA1_irq, 0x00000000);
    }
}

/* REGISTER IMPLEMENTATION */

inline static void stm32_uart_SR_read( Stm32Uart *s )
{
    s->sr_read_with_ore = s->SR_ORE; // Clear ORE: SR read followed by a DR read.

    uint32_t mask = 1<<SR_TXE_BIT | 1<<SR_TC_BIT | 1<<SR_RXNE_BIT | 1<<SR_ORE_BIT;
    s->SR_r &= ~mask;
    s->SR_r |= s->SR_TXE | s->SR_TC | s->SR_RXNE | s->SR_ORE;
}

inline static void stm32_uart_SR_write( Stm32Uart *s, uint32_t new_SR )
{
    if( !(new_SR & 1<<SR_RXNE_BIT) ) s->SR_RXNE = 0;
    if( !(new_SR & 1<<SR_TC_BIT)   ) s->SR_TC   = 0;

    stm32_uart_update_irq(s);
}

inline static void stm32_uart_DR_read( Stm32Uart *s )
{
    if( s->sr_read_with_ore ) { // Clear ORE: SR read followed by a DR read.
        s->sr_read_with_ore = false;
        s->SR_ORE = 0;
    }
    s->SR_RXNE = 0;

    stm32_uart_update_irq(s);
}

inline static void stm32_uart_DR_write( Stm32Uart *s, uint32_t new_DR )
{
    s->TDR_r = new_DR & 0x000001ff;

    if( s->SR_TC ) stm32_uart_start_tx( s ); // Transmission Complete bit is set, transmission can immediately start.
    else           s->SR_TXE = 0;            // Mark buffer as not empty

    stm32_uart_update_irq(s);
}

inline static void stm32_uart_BRR_write( Stm32Uart *s, uint32_t new_BRR ) /* Update the Baud Rate Register. */
{
    s->BRR_r = new_BRR & 0x0000ffff;

    stm32_uart_baud_update(s);
}

inline static void stm32_uart_CR1_write( Stm32Uart *s, uint32_t new_CR1 )
{
    uint32_t enableMask = 1<<CR1_TXEIE_BIT | 1<<CR1_TCIE_BIT | 1<<CR1_RXNEIE_BIT;

    s->intEnable = new_CR1 & enableMask;
    if( s->intEnable & 1<<CR1_RXNEIE_BIT ) s->intFlags |= 1<<SR_ORE_BIT;

    s->CR1_UE = (new_CR1 & 1<<CR1_UE_BIT);
    s->CR1_TE = (new_CR1 & 1<<CR1_TE_BIT);
    s->CR1_RE = (new_CR1 & 1<<CR1_RE_BIT);

    s->CR1_r = new_CR1 & 0x00003fff;

    stm32_uart_update_irq( s );
}

inline static void stm32_uart_CR2_write( Stm32Uart *s, uint32_t new_CR2 )
{
    s->CR2_r = new_CR2 & 0x00007f7f;
}

inline static void stm32_uart_CR3_write( Stm32Uart *s, uint32_t new_CR3 )
{
    s->CR3_r = new_CR3 & 0x000007ff;

    if( s->CR3_r & CR3_r_DMAT_BIT ) {
      uint64_t curr_time = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
      timer_mod( s->tx_dma_timer, curr_time + s->ns_per_char );
    }
}

static void stm32_uart_reset( DeviceState *dev )
{
    Stm32Uart *s = STM32_UART(dev);

    s->SR_r = 0x00C0;

    s->intEnable = 0;
    s->intFlags  = 0;
    s->SR_TXE  = 1<<SR_TXE_BIT;
    s->SR_TC   = 1<<SR_TC_BIT;

    s->SR_RXNE = 0;
    s->SR_ORE  = 0;

    // USART_DR - undefined at reset.
    stm32_uart_BRR_write( s, 0 );
    stm32_uart_CR1_write( s, 0 );
    stm32_uart_CR2_write( s, 0 );
    stm32_uart_CR3_write( s, 0 );

    stm32_uart_update_irq(s);
}

static uint64_t stm32_uart_read( void *opaque, hwaddr offset, unsigned size )
{
    Stm32Uart *s = (Stm32Uart *)opaque;

    switch( offset & 0xfffffffc ) {
        case SR_OFFSET:  stm32_uart_SR_read(s); return s->SR_r; //extract64(stm32_uart_SR_read(s), start, length);
        case DR_OFFSET:  stm32_uart_DR_read(s); return s->RDR_r;
        case BRR_OFFSET: return s->BRR_r;
        case CR1_OFFSET: return s->CR1_r;
        case CR2_OFFSET: return s->CR2_r;
        case CR3_OFFSET: return s->CR3_r;
        case GTPR_OFFSET: STM32_NOT_IMPL_REG( offset, size ); return 0;
        default: STM32_BAD_REG( offset, size ); return 0;
    }
}

static void stm32_uart_write( void *opaque, hwaddr offset, uint64_t value, unsigned size )
{
    Stm32Uart *s = (Stm32Uart *)opaque;

    switch( offset & 0xfffffffc ) {
        case SR_OFFSET:  stm32_uart_SR_write( s, value ); break; //stm32_uart_SR_write( s, deposit64(stm32_uart_SR_read(s), start, length, value));
        case DR_OFFSET:  stm32_uart_DR_write( s, value ); break;
        case BRR_OFFSET: stm32_uart_BRR_write( s, value ); break;
        case CR1_OFFSET: stm32_uart_CR1_write( s, value ); break;
        case CR2_OFFSET: stm32_uart_CR2_write( s, value ); break;
        case CR3_OFFSET: stm32_uart_CR3_write( s, value ); break;
        case GTPR_OFFSET: STM32_NOT_IMPL_REG( offset, size ); break;
        default: STM32_BAD_REG( offset, size ); break;
    }
}

static const MemoryRegionOps stm32_uart_ops = {
    .read  = stm32_uart_read,
    .write = stm32_uart_write,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};

/* DEVICE INITIALIZATION */

static void stm32_uart_init( Object *obj )
{
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);
    Stm32Uart      *s = STM32_UART(dev);

    memory_region_init_io( &s->iomem, OBJECT(s), &stm32_uart_ops, s, "uart", 0x03ff );
    sysbus_init_mmio( dev, &s->iomem );

    sysbus_init_irq( dev, &s->irq );

    //s->rx_timer     = timer_new_ns( QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *)stm32_uart_rx_timer_expire, s);
    s->tx_timer     = timer_new_ns( QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *)stm32_uart_tx_timer_expire, s);
    s->tx_dma_timer = timer_new_ns( QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *)stm32_uart_tx_dma_timer_expire, s);

    s->curr_irq_level = 0;
}

static void stm32_uart_realize( DeviceState *dev, Error **errp )
{
    Stm32Uart *s = STM32_UART(dev);
    qemu_irq *clk_irq;

    clk_irq = qemu_allocate_irqs( stm32_uart_clk_irq_handler, (void *)s, 1 );
    stm32_rcc_set_periph_clk_irq( s->stm32_rcc, s->periph, clk_irq[0] );
    stm32_uart_reset( (DeviceState *)s );
}

void stm32_uart_set_rcc( Stm32Uart *uart, Stm32Rcc *rcc ) {
    uart->stm32_rcc = rcc;
}

void stm32_uart_set_number( Stm32Uart *uart, int uart_num ) {
    uart->number = uart_num -1;
}

static Property stm32_uart_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Uart, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_END_OF_LIST()};

static void stm32_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32_uart_realize;

    device_class_set_legacy_reset( dc,stm32_uart_reset );
    device_class_set_props( dc, stm32_uart_properties );
}

static TypeInfo stm32_uart_info = {
    .name = "stm32-uart",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32Uart),
    .instance_init = stm32_uart_init,
    .class_init = stm32_uart_class_init
};

static void stm32_uart_register_types(void) {
  type_register_static( &stm32_uart_info );
}

type_init( stm32_uart_register_types )
