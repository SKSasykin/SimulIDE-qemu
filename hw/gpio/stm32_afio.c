/*
 * STM32 Microcontroller AFIO (Alternate Function I/O) module
 *
 * Copyright (C) 2010 Andre Beckus
 *
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
#include "qemu/bitops.h"
#include "hw/hw.h"
#include "qom/object.h"

#include "../../system/simuliface.h"

/* DEFINITIONS */

#define AFIO_EVCR_OFFSET 0x00
#define AFIO_MAPR_OFFSET 0x04
#define AFIO_EXTICR1_OFFSET 0x08
#define AFIO_EXTICR2_OFFSET 0x0c
#define AFIO_EXTICR3_OFFSET 0x10
#define AFIO_EXTICR4_OFFSET 0x14

#define AFIO_MAPR_TIM4_REMAP_BIT 12
#define AFIO_MAPR_TIM3_REMAP_START 10
#define AFIO_MAPR_TIM3_REMAP_MASK 0x00000C00
#define AFIO_MAPR_TIM2_REMAP_START 8
#define AFIO_MAPR_TIM2_REMAP_MASK 0x00000300
#define AFIO_MAPR_TIM1_REMAP_START 6
#define AFIO_MAPR_TIM1_REMAP_MASK 0x000000C0
#define AFIO_MAPR_USART3_REMAP_START 4
#define AFIO_MAPR_USART3_REMAP_MASK 0x00000030
#define AFIO_MAPR_USART2_REMAP_BIT 3
#define AFIO_MAPR_USART1_REMAP_BIT 2
#define AFIO_MAPR_I2C1_REMAP_BIT 1
#define AFIO_MAPR_SPI1_REMAP_BIT 0

#define AFIO_EXTICR_START 0x08
#define AFIO_EXTICR_COUNT 4

#define AFIO_EXTI_PER_CR 4

struct Stm32Afio {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    //void *stm32_rcc_prop;
    //void *stm32_exti_prop;

    /* Private */
    MemoryRegion iomem;

    Stm32Rcc *stm32_rcc;

    Stm32Gpio *gpio[STM32_GPIO_COUNT];
    Stm32Exti *exti;

    uint32_t AFIO_MAPR;
    uint16_t AFIO_EXTICR[AFIO_EXTICR_COUNT];
};


/* REGISTER IMPLEMENTATION */

static void stm32_afio_MAPR_write( Stm32Afio *s, uint32_t mapr, bool init )
{
    if( !init && s->AFIO_MAPR == mapr ) return;
    s->AFIO_MAPR = mapr;

    //s->Remap[STM32_SPI1]  = extract32(s->AFIO_MAPR, AFIO_MAPR_SPI1_REMAP_BIT, 1);
    //s->Remap[STM32_I2C1]  = extract32(s->AFIO_MAPR, AFIO_MAPR_I2C1_REMAP_BIT, 1);
    //s->Remap[STM32_UART1] = extract32(s->AFIO_MAPR, AFIO_MAPR_USART1_REMAP_BIT, 1);
    //s->Remap[STM32_UART2] = extract32(s->AFIO_MAPR, AFIO_MAPR_USART2_REMAP_BIT, 1);
    //s->Remap[STM32_UART3] = (s->AFIO_MAPR & AFIO_MAPR_USART3_REMAP_MASK) >> AFIO_MAPR_USART3_REMAP_START;

    stm32_timer_remap( 1, (s->AFIO_MAPR & AFIO_MAPR_TIM1_REMAP_MASK) >> AFIO_MAPR_TIM1_REMAP_START );
    stm32_timer_remap( 2, (s->AFIO_MAPR & AFIO_MAPR_TIM2_REMAP_MASK) >> AFIO_MAPR_TIM2_REMAP_START );
    stm32_timer_remap( 3, (s->AFIO_MAPR & AFIO_MAPR_TIM3_REMAP_MASK) >> AFIO_MAPR_TIM3_REMAP_START );
    stm32_timer_remap( 4, extract32(s->AFIO_MAPR, AFIO_MAPR_TIM4_REMAP_BIT, 1) );

    m_arena->simuAction = ARM_REMAP; // UART, I2C, SPI
    m_arena->data32 = mapr;

    doAction();
}

/* Write the External Interrupt Configuration Register.
 * There are four of these registers, each of which configures four EXTI interrupt lines.
 * Each line is represented by four bits, which indicate which GPIO the line is connected to.
 * When the register is written, the changes are propagated to the EXTI module.
 */
static void stm32_afio_EXTICR_write( Stm32Afio *s, unsigned index, uint16_t exticr, bool init )
{
    if( !init && s->AFIO_EXTICR[index] == exticr ) return;

    unsigned exti_line;
    unsigned start;
    unsigned old_gpio_index, new_gpio_index;

    assert( index < AFIO_EXTICR_COUNT );
//printf("stm32_afio_EXTICR_write %i %i\n", index, init );fflush( stdout );
    for( int i=0; i<AFIO_EXTI_PER_CR; i++ ) // Loop through the four EXTI lines controlled by this register.
    {
        exti_line = (index * AFIO_EXTI_PER_CR) + i;
        start = i * 4;

        old_gpio_index = (s->AFIO_EXTICR[index] >> start) & 0xF;
        new_gpio_index = (exticr >> start) & 0xF;
//printf("stm32_afio_EXTICR_write PORT%c EXTI%i\n", 'A'+new_gpio_index, exti_line );fflush( stdout );
        if( old_gpio_index != new_gpio_index ) // For each line that changed, notify the EXTI module.
        {
            sysbus_connect_irq( SYS_BUS_DEVICE(s->gpio[old_gpio_index]), exti_line, NULL );
            sysbus_connect_irq( SYS_BUS_DEVICE(s->gpio[new_gpio_index]), exti_line, qdev_get_gpio_in(DEVICE(s->exti), exti_line) );
        }
        else if( init ) // Initial configuration
            sysbus_connect_irq( SYS_BUS_DEVICE(s->gpio[new_gpio_index]), exti_line, qdev_get_gpio_in(DEVICE(s->exti), exti_line) );
    }
    s->AFIO_EXTICR[index] = exticr;
}

static uint64_t stm32_afio_read(void *opaque, hwaddr offset, unsigned size)
{
    Stm32Afio *s = (Stm32Afio *)opaque;

    switch( offset ) {
        case AFIO_EVCR_OFFSET:    STM32_NOT_IMPL_REG(offset, size); break;
        case AFIO_MAPR_OFFSET:    return s->AFIO_MAPR;
        case AFIO_EXTICR1_OFFSET: return s->AFIO_EXTICR[0];
        case AFIO_EXTICR2_OFFSET: return s->AFIO_EXTICR[1];
        case AFIO_EXTICR3_OFFSET: return s->AFIO_EXTICR[2];
        case AFIO_EXTICR4_OFFSET: return s->AFIO_EXTICR[3];
        default:                  STM32_BAD_REG(offset, size);
    }
    return 0;
}

static void stm32_afio_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    Stm32Afio *s = (Stm32Afio *)opaque;

    if( !stm32_rcc_check_periph_clk( s->stm32_rcc, STM32_AFIO_PERIPH) ) return;

    switch (offset) {
        case AFIO_EVCR_OFFSET:    STM32_NOT_IMPL_REG(offset, size); break;
        case AFIO_MAPR_OFFSET:    stm32_afio_MAPR_write( s, value & 0x71FFFFF, false ); break;
        case AFIO_EXTICR1_OFFSET: stm32_afio_EXTICR_write( s, 0, value, false ); break;
        case AFIO_EXTICR2_OFFSET: stm32_afio_EXTICR_write( s, 1, value, false ); break;
        case AFIO_EXTICR3_OFFSET: stm32_afio_EXTICR_write( s, 2, value, false ); break;
        case AFIO_EXTICR4_OFFSET: stm32_afio_EXTICR_write( s, 3, value, false ); break;
        default:                  STM32_BAD_REG(offset, size); break;
    }
}

static const MemoryRegionOps stm32_afio_ops = {
    .read = stm32_afio_read,
    .write = stm32_afio_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_afio_reset(DeviceState *dev)
{
    Stm32Afio *s = STM32_AFIO(dev);

    stm32_afio_MAPR_write( s, 0x00000000, true );
    for( int i=0; i<AFIO_EXTICR_COUNT; ++i ) stm32_afio_EXTICR_write( s, i, 0x00000000, true );
}

/* PUBLIC FUNCTIONS */

/* DEVICE INITIALIZATION */

static void stm32_afio_init(Object *obj)
{
    SysBusDevice *dev= SYS_BUS_DEVICE(obj);
    Stm32Afio *s = STM32_AFIO(dev);

    //s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    memory_region_init_io(&s->iomem, OBJECT(s), &stm32_afio_ops, s,"afio", 0x03ff);
    sysbus_init_mmio(dev, &s->iomem);

    return ;
}
/*
static Property stm32_afio_properties[] = {
    //DEFINE_PROP_PTR("stm32_rcc", Stm32Afio, stm32_rcc_prop),
    DEFINE_PROP_END_OF_LIST()
};
*/

void stm32_afio_set_rcc(Stm32Afio *afio, Stm32Rcc* rcc)
{
    afio->stm32_rcc = rcc;
}

static void add_gpio_link(Stm32Afio *s, int gpio_index, const char *link_name)
{
    object_property_add_link(OBJECT(s), link_name, TYPE_STM32_GPIO,
                                 (Object **)&s->gpio[gpio_index],
                                 object_property_allow_set_link,
                                 OBJ_PROP_LINK_STRONG);
}

static void stm32_afio_instance_init( Object *obj )
{
    Stm32Afio *s = STM32_AFIO(obj);

    stm32_afio_init(obj);
    
    add_gpio_link( s, 0, "gpio[a]");
    add_gpio_link( s, 1, "gpio[b]");
    add_gpio_link( s, 2, "gpio[c]");
    add_gpio_link( s, 3, "gpio[d]");
    add_gpio_link( s, 4, "gpio[e]");
    add_gpio_link( s, 5, "gpio[f]");
    add_gpio_link( s, 6, "gpio[g]");

    object_property_add_link(obj, "exti", TYPE_STM32_EXTI,
                                 (Object **)&s->exti,
                                 object_property_allow_set_link,
                                 OBJ_PROP_LINK_STRONG);

}

static void stm32_afio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    //SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    //k->init = stm32_afio_init;

    //dc->reset = stm32_afio_reset;
    device_class_set_legacy_reset( dc,stm32_afio_reset);
    //dc->props = stm32_afio_properties;
}

static TypeInfo stm32_afio_info = {
    .name  = TYPE_STM32_AFIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Afio),
    .instance_init = stm32_afio_instance_init,
    .class_init = stm32_afio_class_init
};

static void stm32_afio_register_types(void)
{
    type_register_static(&stm32_afio_info);
}

type_init(stm32_afio_register_types)
