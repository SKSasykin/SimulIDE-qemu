/***************************************************************************
 *   Copyright (C) 2025 by Santiago González                               *
 *                                                                         *
 *  Modified by opencode 2026: master regAddr/regData protocol + macOS     *
 *                                                                         *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 or
 *  (at your option) any later version.
 */

#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#if defined( __linux__ ) || defined( __APPLE__ )
#include <sys/mman.h>
#include <sys/shm.h>
//#elif defined(_WIN32)
//#include <windows.h>
#endif

#include "simuliface.h"

#include "qemu/osdep.h"
#include "qemu-main.h"
#include "qemu/timer.h"
#include "sysemu/runstate.h"
#include "sysemu/sysemu.h"
#include "sysemu/cpu-timers.h"
#include "hw/irq.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------

volatile qemuArena_t* m_arena = NULL;

// ------------------------------------------------

uint64_t m_timeout;
uint64_t m_lastQemuTime;

QEMUTimer* qtimer;

static uint64_t s_simuTime;   // last time signaled to SimulIDE (monotonic)
static uint64_t s_nextEvent;  // next scheduled SIM_EVENT tick (ps)

#define SIMULIDE_IOMEM_BASE 0x3FF00000
#define SIMULIDE_TICK_PS   1000000ull // 1 us between SIM_EVENT ticks

uint64_t getQemu_ps(void)
{
    uint64_t qemuTime = icount_get_ps();
    return qemuTime;
}

// Monotonic view of QEMU time, never going backwards wrt SimulIDE
static uint64_t simulide_time(void)
{
    uint64_t now = getQemu_ps();
    if( now < s_simuTime ) now = s_simuTime;
    return now;
}

static void simulide_wait(void) // Wait for SimulIDE to consume the event (simuTime == 0)
{
    m_timeout = 0;
    while( m_arena->simuTime )
    {
        if( m_timeout++ > 1e9 ) break; // Terminate loop if timed out
    }
    m_timeout = 0;
}

void simulide_signal( uint64_t action, uint64_t time_ps )
{
    m_arena->simuAction = action;
    m_arena->simuTime   = time_ps;
    s_simuTime          = time_ps;
    simulide_wait();
}

uint32_t simulide_bridge_read( uint32_t addr )
{
    m_arena->regAddr = addr; // addr is already an IOMEM offset
    m_arena->regData = 0;
    simulide_signal( SIM_READ, simulide_time() );

    m_timeout = 0;
    while( m_arena->qemuAction != SIM_READ ) // Wait for SimulIDE answer
    {
        if( m_timeout++ > 1e9 ) break; // Terminate loop if timed out
    }
    m_timeout = 0;

    uint32_t val = (uint32_t)m_arena->regData;
    m_arena->qemuAction = 0;
    return val;
}

void simulide_bridge_write( uint32_t addr, uint32_t value )
{
    m_arena->regAddr = addr; // addr is already an IOMEM offset
    m_arena->regData = value;
    simulide_signal( SIM_WRITE, simulide_time() );
}

void doAction(void) // Legacy entry point for dormant devices
{
    simulide_signal( SIM_EVENT, simulide_time() );
}

static void scheduleNextEvent(void)
{
    s_nextEvent = s_simuTime + SIMULIDE_TICK_PS;
    timer_mod_ns( qtimer, s_nextEvent/1000 );
}

static void simu_event( void* opaque )
{
    if( !m_arena->running ) return;

    simulide_signal( SIM_EVENT, s_nextEvent );
    scheduleNextEvent();
}

int simuMain( int argc, char** argv )
{
    const int   shMemSize = sizeof( qemuArena_t );
    const char* shMemKey;

    if( argc > 2 ) // Check if there are any arguments
    {
        shMemKey = argv[1];
        argv = &argv[2];
        argc -= 2;
    } else {
        printf("Qemu Error: No arguments provided.\n");
        return 1;
    }

    void* arena = NULL;

#if defined( __linux__ ) || defined( __APPLE__ )
    int shMemId = shm_open( shMemKey, O_RDWR, 0666 ); // Open the shared memory object
    if( shMemId == -1 )
    {
        printf("Qemu: Error opening arena: %s\n", shMemKey );
        return 1;
    }
    else printf("Qemu: arena ok: %s\n", shMemKey );
    arena = mmap( 0, shMemSize, PROT_READ | PROT_WRITE, MAP_SHARED, shMemId, 0);
#elif defined(_WIN32)
    HANDLE hMapFile = OpenFileMapping( FILE_MAP_ALL_ACCESS,  FALSE, shMemKey );

    if( hMapFile == NULL ) {
        //std::cerr << "Could not create file mapping object: " << GetLastError() << std::endl;
        return 1;
    }
    arena = MapViewOfFile( hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, shMemSize );
#endif

    if( !arena )
    {
        printf("Qemu: Error mapping arena\n"); fflush( stdout );
        return 1;
    }
    else printf("Qemu: arena mapped %i bytes\n", shMemSize );

    //------------------------------------------------------------------

    m_arena = (qemuArena_t*)arena;

    // SimulIDE reads ps_per_inst (e.g. SPI clock): default ESP32 240 MHz
    m_arena->ps_per_inst = 1e12/240e6;

    //------------------------------------------------------------------

    printf("-----------------------------------\n");
    for( int i=0; i<argc; i++)
    {
        printf( "%s",argv[i] );
        if( !(i&1) ) printf("\n");
        else         printf(" ");
    }
    printf("-----------------------------------\n");
    fflush( stdout );

    qemu_init( argc, argv );

    qtimer = (QEMUTimer*)malloc( sizeof(QEMUTimer) );
    timer_init_full( qtimer, NULL, QEMU_CLOCK_VIRTUAL, 1, 0, simu_event, NULL );

    m_lastQemuTime = 0;
    s_simuTime     = 0;
    m_arena->running = true;

    printf("Qemu: initialized\n" );fflush( stdout );

    scheduleNextEvent();

    printf("Qemu: starting main loop\n");fflush( stdout );
    int status = qemu_main_loop();
    qemu_cleanup( status );

#if defined( __linux__ ) || defined( __APPLE__ )
    munmap( arena, shMemSize ); // Un-map shared memory
#elif defined(_WIN32)
    UnmapViewOfFile( arena );
    CloseHandle( hMapFile );
#endif

    printf("Qemu: process finished\n");fflush( stdout );

    return 0;
}
