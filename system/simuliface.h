/***************************************************************************
 *   Copyright (C) 2025 by Santiago González                               *
 *                                                                         *
 *  Modified by opencode 2026: struct matches SimulIDE master's qemudevice.h *
 *                                                                         *
 ***( see copyright.txt file at root folder )*******************************/

#ifndef QEMU_SIMULIFACE_H
#define QEMU_SIMULIFACE_H

#include <stdint.h>
#include <stdbool.h>

#include "qemu/typedefs.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------
// Struct layout MUST match SimulIDE master's qemuArena_t (src/microsim/cores/qemu/qemudevice.h)

typedef struct qemuArena{
    uint64_t simuTime;       // in ps
    uint64_t qemuTime;       // in ps
    uint64_t regData;
    uint64_t regAddr;
    uint64_t irqNumber;
    uint64_t irqLevel;
    uint64_t simuAction;
    uint64_t qemuAction;
    uint64_t running;
    int64_t  loop_timeout_ns;
    double   ps_per_inst;
} qemuArena_t;

enum simuAction{
    SIM_NONE = 0,
    SIM_READ,
    SIM_WRITE,
    SIM_FREQ,
    SIM_INTERRUPT,
    SIM_I2C=10,
    SIM_SPI,
    SIM_USART,
    SIM_TIMER,
    SIM_GPIO_IN,
    SIM_EVENT=1<<7,
};

// Legacy enums kept for compile-compat of dormant devices
enum esp32Actions{
    ESP_GPIO_OUT = 1,
    ESP_GPIO_DIR,
    ESP_GPIO_IN,
    ESP_IOMUX,
    ESP_MATRIX_IN,
    ESP_MATRIX_OUT
};

enum arm32Actions{
    ARM_GPIO_OUT = 1,
    ARM_GPIO_CRx,
    ARM_GPIO_IN,
    ARM_ALT_OUT,
    ARM_REMAP
};

// Legacy field aliases for dormant devices (old protocol). All mapped onto
// master fields so the old code compiles. These devices are shadowed by the
// SimulIDE bridge and are never actually reached.
#define data32 regData
#define mask32 regData
#define data16 regData
#define mask16 regData
#define data8  regData
#define mask8  regData
#define action regData
#define time   simuTime

extern volatile qemuArena_t* m_arena;
typedef void (*simulide_interrupt_handler)(uint64_t number, uint64_t level, void *opaque);
void simulide_set_interrupt_handler(simulide_interrupt_handler handler, void *opaque);
// ------------------------------------------------

extern uint64_t m_timeout;

uint64_t getQemu_ps(void);

void doAction(void);

int simuMain( int argc, char** argv );

uint32_t simulide_bridge_read( uint32_t addr );
void     simulide_bridge_write( uint32_t addr, uint32_t value );

#endif
