/*
 * STM32 Microcontroller GPIO (General Purpose I/O) module
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Source code based on pl061.c
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
#include "hw/sysbus.h"
#include "hw/irq.h"

#include "../../system/simuliface.h"

#define GPIOx_CRL_OFFSET 0x00
#define GPIOx_CRH_OFFSET 0x04
#define GPIOx_IDR_OFFSET 0x08
#define GPIOx_ODR_OFFSET 0x0C
#define GPIOx_BSRR_OFFSET 0x10
#define GPIOx_BRR_OFFSET 0x14
#define GPIOx_LCKR_OFFSET 0x18

struct Stm32Gpio {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    //void *stm32_rcc_prop;

    /* Private */
    MemoryRegion iomem;

    Stm32Rcc *stm32_rcc;

    uint32_t GPIOx_CRL;
    uint32_t GPIOx_CRH;
    uint32_t GPIOx_ODR;

    uint16_t in;
    uint8_t number;

    qemu_irq in_irq[STM32_GPIO_PIN_COUNT]; /* IRQs which relay input pin changes to other STM32 peripherals */
};

/* CALLBACKs */

// Trigger fired when a GPIO input pin changes state (based on an external stimulus from the machine).

static void stm32_gpio_in_trigger( void *opaque, int irq, int level )
{
    Stm32Gpio *s = opaque;
    unsigned pin = irq;

    assert(pin < STM32_GPIO_PIN_COUNT);

    // Update internal pin state.
    s->in &= ~(1 << pin);
    s->in |= (level ? 1 : 0) << pin;
    //printf("stm32_gpio_in_trigger PORT%c %i %i\n", 'A'+s->number, pin, level );fflush( stdout );
    qemu_set_irq( s->in_irq[pin], level ); // Propagate the trigger to the input IRQs.
    //printf("stm32_gpio_in_trigger IRQs fired\n");fflush( stdout );
}

/* REGISTER IMPLEMENTATION */

static uint16_t readInputs( int32_t port )
{
    if( !m_arena->running ) return 0;

    m_arena->simuAction = ARM_GPIO_IN;
    m_arena->data8  = port;
    doAction();

    return m_arena->data16;
}

static void stm32_gpio_write_CRx( uint32_t value, uint8_t shift, uint8_t port )
{
    if( !m_arena->running ) return;

    m_arena->simuAction = ARM_GPIO_CRx;
    m_arena->data32 = value;
    m_arena->mask8  = shift;
    m_arena->data8  = port;     // We have to send Port number, PortA = 1

    doAction();
}

static void stm32_gpio_write_ODR( uint32_t new_value, uint8_t port )
{
    //printf("STM32 GPIO ODR Write %i %i\n", portNumber, new_value ); fflush( stdout );

    if( !m_arena->running ) return;        // Wait until SimulIDE execute previous action

    m_arena->simuAction = ARM_GPIO_OUT;
    m_arena->data16 = new_value;   // 1 bit per pin: 1 = High
    m_arena->data8  = port;  // We have to send Port number, PortA = 1

    doAction();
}

static uint64_t stm32_gpio_read( void *opaque, hwaddr offset, unsigned size )
{
    Stm32Gpio *s = (Stm32Gpio *)opaque;

    switch( offset ) {
        case GPIOx_CRL_OFFSET: return s->GPIOx_CRL;
        case GPIOx_CRH_OFFSET: return s->GPIOx_CRH;
        case GPIOx_IDR_OFFSET:
            s->in = readInputs( s->periph );   // Read pins
            return s->in;
        case GPIOx_ODR_OFFSET:  return s->GPIOx_ODR;
        case GPIOx_BSRR_OFFSET: return 0; /*STM32_WO_REG(offset);*/
        case GPIOx_BRR_OFFSET:  return 0; /*STM32_WO_REG(offset);*/
        case GPIOx_LCKR_OFFSET: return 0; /* Locking is not yet implemented */
        default:                return 0; //STM32_BAD_REG(offset, 4/*size*/);
    }
}

static void stm32_gpio_write( void *opaque, hwaddr offset, uint64_t value, unsigned size )
{
    Stm32Gpio *s = (Stm32Gpio *)opaque;

    /// Is this really needed? Maybe use an enabled flag
    /// stm32_rcc_check_periph_clk( (Stm32Rcc *)s->stm32_rcc, s->periph );

    switch( offset ) {
        case GPIOx_CRL_OFFSET:
            if( value == s->GPIOx_CRL ) break;
            s->GPIOx_CRL = value;
            stm32_gpio_write_CRx( value, 0, s->periph );
            break;
        case GPIOx_CRH_OFFSET:
            if( value == s->GPIOx_CRH ) break;
            s->GPIOx_CRH = value;
            stm32_gpio_write_CRx( value, 8, s->periph ); // shift Pin number by 8
            break;
        case GPIOx_IDR_OFFSET:
            STM32_RO_REG( offset );
            break;
        case GPIOx_ODR_OFFSET:
            if( value == s->GPIOx_ODR ) break;
            s->GPIOx_ODR = value;
            stm32_gpio_write_ODR( value, s->periph );
            break;
        case GPIOx_BSRR_OFFSET:{
            uint32_t set_mask = value & 0x0000FFFF;
            uint32_t reset_mask = ~(value >> 16) & 0x0000FFFF;
            value = (s->GPIOx_ODR & reset_mask) | set_mask; // Sets take priority over resets, so we do
            if( value == s->GPIOx_ODR ) break;
            s->GPIOx_ODR = value;
            stm32_gpio_write_ODR( value, s->periph );
            } break;
        case GPIOx_BRR_OFFSET:{
            uint32_t reset_mask = ~value & 0x0000FFFF;
            value = s->GPIOx_ODR & reset_mask;
            if( value == s->GPIOx_ODR ) break;
            s->GPIOx_ODR = value;
            stm32_gpio_write_ODR( value, s->periph );
            } break;
        case GPIOx_LCKR_OFFSET:
            /// TODO: Locking is not implemented
            break;
        default: break;
    }
}

static const MemoryRegionOps stm32_gpio_ops = {
    .read = stm32_gpio_read,
    .write = stm32_gpio_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_gpio_reset( DeviceState *dev ) // only outputs and config are affected by the GPIO reset.
{
    Stm32Gpio *s = STM32_GPIO(dev);

    s->GPIOx_CRL = 0x44444444;
    s->GPIOx_CRH = 0x44444444;
    s->GPIOx_ODR = 0;
}

/* DEVICE INITIALIZATION */

static void stm32_gpio_init( Object *obj )
{
    SysBusDevice *dev= SYS_BUS_DEVICE( obj );

    Stm32Gpio *s = STM32_GPIO(dev);

    //s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    memory_region_init_io(&s->iomem, OBJECT(s), &stm32_gpio_ops, s, "gpio", 0x03ff);
    sysbus_init_mmio(dev, &s->iomem);

    qdev_init_gpio_in(DEVICE(dev), stm32_gpio_in_trigger, STM32_GPIO_PIN_COUNT);

    for( int pin=0; pin<STM32_GPIO_PIN_COUNT; pin++ )
        sysbus_init_irq( dev, &s->in_irq[pin] );

    return ;
}

static void stm32_gpio_realize( DeviceState *dev, Error **errp )
{
    //Stm32Gpio *s = STM32_GPIO(dev);
}

void stm32_gpio_set_rcc( Stm32Gpio *gpio, Stm32Rcc* rcc )
{
    gpio->stm32_rcc = rcc;
}
void stm32_gpio_set_number(Stm32Gpio *gpio, int gpio_num) {
    gpio->number = gpio_num;
}

static Property stm32_gpio_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Gpio, periph, STM32_PERIPH_UNDEFINED),
    //DEFINE_PROP_PTR("stm32_rcc", Stm32Gpio, stm32_rcc_prop),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_gpio_class_init( ObjectClass *klass, void *data )
{
    DeviceClass *dc = DEVICE_CLASS( klass );

    device_class_set_legacy_reset( dc,stm32_gpio_reset);
    dc->realize = stm32_gpio_realize;
    //dc->props = stm32_gpio_properties;
    device_class_set_props( dc, stm32_gpio_properties );
}

static TypeInfo stm32_gpio_info = {
    .name  = TYPE_STM32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Gpio),
    .instance_init = stm32_gpio_init,
    .class_init = stm32_gpio_class_init
};

static void stm32_gpio_register_types(void)
{
    type_register_static( &stm32_gpio_info );
}

type_init( stm32_gpio_register_types )
