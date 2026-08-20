/*
 * ESP8266 (Xtensa LX106) machine for SimulIDE
 *
 * SimulIDE uses a shared-memory bridge to access the UART and GPIO
 * peripherals. There is no boot ROM (the ESP8266 ROM is proprietary),
 * so the user firmware is loaded directly into IRAM.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/xtensa/xtensa_memory.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/char/esp32_uart.h"
#include "hw/gpio/esp8266_gpio.h"
#include "hw/timer/esp8266_frc1.h"
#include "hw/misc/esp32_simulide_bridge.h"
#include "hw/misc/unimp.h"
#include "core-lx106/core-isa.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/cpus.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "exec/exec-all.h"
#include "elf.h"

#define TYPE_ESP8266_CPU    XTENSA_CPU_TYPE_NAME("lx106")
#define TYPE_ESP8266_MACHINE MACHINE_TYPE_NAME("esp8266-simul")

OBJECT_DECLARE_SIMPLE_TYPE(Esp8266MachineState, ESP8266_MACHINE)

struct Esp8266MachineState {
    MachineState parent;
    XtensaCPU *cpu;
    DeviceState *uart[2];
    uint32_t fw_entry;
};

/* The user application is loaded at the start of the IRAM, there is
 * no boot ROM in the machine. */
#define ESP8266_IRAM_BASE   0x40000000
#define ESP8266_RAW_MAX     0x100000

#define ESP8266_UART0_BASE  0x60000000
#define ESP8266_UART1_BASE  0x60000f00

#define ESP8266_GPIO_BASE   0x60000300
#define ESP8266_IOMUX_BASE  0x60000800
#define ESP8266_FRC_BASE    0x60000600

static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}

static void esp8266_reset(void *opaque)
{
    Esp8266MachineState *ms = opaque;

    cpu_reset(CPU(ms->cpu));
    /* No boot ROM: jump straight into the user firmware. */
    ms->cpu->env.sregs[PS] = PS_UM;
    ms->cpu->env.pc = ms->fw_entry;
}

static bool esp8266_load_image(const uint8_t *image, size_t image_size,
                               AddressSpace *as, uint64_t *entry)
{
    if (image_size < 8 || image[0] != 0xe9) {
        return false;
    }

    uint8_t segment_count = image[1];
    if (!segment_count || segment_count > 16) {
        error_report("Invalid ESP8266 image segment count %u", segment_count);
        exit(1);
    }

    *entry = ldl_le_p(image + 4);
    size_t offset = 8;
    for (unsigned i = 0; i < segment_count; ++i) {
        if (offset + 8 > image_size) {
            error_report("Truncated ESP8266 image segment header");
            exit(1);
        }

        uint32_t address = ldl_le_p(image + offset);
        uint32_t length = ldl_le_p(image + offset + 4);
        offset += 8;
        if (length > 16 * MiB || offset + length > image_size) {
            error_report("Invalid ESP8266 image segment %u length %u", i,
                         length);
            exit(1);
        }

        char *name = g_strdup_printf("esp8266.segment.%u", i);
        rom_add_blob_fixed_as(name, image + offset, length, address, as);
        g_free(name);
        offset += length;
    }
    return true;
}

static void esp8266_machine_init(MachineState *machine)
{
    Esp8266MachineState *ms = ESP8266_MACHINE(machine);

    ms->cpu = XTENSA_CPU(cpu_create(machine->cpu_type));
    ms->cpu->env.sregs[PRID] = 0;
    ms->fw_entry = ESP8266_IRAM_BASE;

    qemu_register_reset(esp8266_reset, ms);
    /* Need MMU initialized prior to ELF loading,
     * so that ELF gets loaded into virtual addresses */
    esp8266_reset(ms);

    /* Create the local memories described by the LX106 core config. */
    xtensa_create_memory_regions(&ms->cpu->env.config->instrom, "esp8266.instrom",
                                 get_system_memory());
    xtensa_create_memory_regions(&ms->cpu->env.config->instram, "esp8266.instram",
                                 get_system_memory());
    xtensa_create_memory_regions(&ms->cpu->env.config->datarom, "esp8266.datarom",
                                 get_system_memory());
    xtensa_create_memory_regions(&ms->cpu->env.config->dataram, "esp8266.dataram",
                                 get_system_memory());
    xtensa_create_memory_regions(&ms->cpu->env.config->sysrom, "esp8266.sysrom",
                                 get_system_memory());
    xtensa_create_memory_regions(&ms->cpu->env.config->sysram, "esp8266.sysram",
                                 get_system_memory());

    /* UARTs: reuse the ESP32 UART model, register layout matches */
    for (int i = 0; i < 2; ++i) {
        char name[8];
        snprintf(name, sizeof(name), "uart%d", i);
        ms->uart[i] = qdev_new(TYPE_ESP32_UART);
        qdev_prop_set_chr(ms->uart[i], "chardev", serial_hd(i));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ms->uart[i]), &error_fatal);
        hwaddr base = (i == 0) ? ESP8266_UART0_BASE : ESP8266_UART1_BASE;
        memory_region_add_subregion(get_system_memory(), base,
                                    sysbus_mmio_get_region(SYS_BUS_DEVICE(ms->uart[i]), 0));
    }

    /* GPIO */
    DeviceState *gpio = qdev_new(TYPE_ESP8266_GPIO);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio), &error_fatal);
    memory_region_add_subregion(get_system_memory(), ESP8266_GPIO_BASE,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(gpio), 0));

    /* FRC1 is a 23-bit countdown timer; external input 7 maps to IRQ 9. */
    DeviceState *frc = qdev_new(TYPE_ESP8266_FRC1);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(frc), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(frc), 0,
                       xtensa_get_extints(&ms->cpu->env)[7]);
    /* Priority 2 so the FRC wins over the SimulIDE bridge (priority 1),
     * whose UART0 range (0x60000000..0x60000FFF) would otherwise shadow it */
    memory_region_add_subregion_overlap(get_system_memory(), ESP8266_FRC_BASE,
                                        sysbus_mmio_get_region(SYS_BUS_DEVICE(frc), 0), 2);

    /* IOMUX */
    create_unimplemented_device("esp8266.iomux", ESP8266_IOMUX_BASE, 0x1000);

    uint64_t entry = ESP8266_IRAM_BASE;

    /* Firmware may be an ELF, an Espressif 0xe9 image, or a self-contained
     * flat IRAM blob. */
    if (machine->firmware || machine->kernel_filename) {
        const char *fw = machine->firmware ? machine->firmware
                                           : machine->kernel_filename;
        int size = load_elf(fw, NULL, translate_phys_addr, ms->cpu,
                            &entry, NULL, NULL, NULL, TARGET_BIG_ENDIAN,
                            EM_XTENSA, 0, 0);
        if (size < 0) {
            g_autofree char *image = NULL;
            gsize image_size = 0;
            g_autoptr(GError) error = NULL;
            if (!g_file_get_contents(fw, &image, &image_size, &error)) {
                error_report("Error: could not read firmware '%s': %s", fw,
                             error->message);
                exit(1);
            }
            if (!esp8266_load_image((uint8_t *)image, image_size,
                                    CPU(ms->cpu)->as, &entry)) {
                if (image_size > ESP8266_RAW_MAX) {
                    error_report("Error: firmware image (%zu bytes) larger "
                                 "than IRAM (%d bytes)", image_size,
                                 ESP8266_RAW_MAX);
                    exit(1);
                }
                rom_add_blob_fixed_as("esp8266.raw", image, image_size,
                                      ESP8266_IRAM_BASE, CPU(ms->cpu)->as);
                entry = ESP8266_IRAM_BASE;
            }
        }
    } else {
        error_report("Error: no firmware specified, use -bios <fw>");
        exit(1);
    }

    ms->fw_entry = entry;
    ms->cpu->env.pc = entry;
    qemu_log("esp8266: loaded firmware, entry point 0x%08" PRIx64 "\n", entry);

    /* SimulIDE bridge shadows UART0/GPIO/UART1 (must be last, overlaps) */
    esp8266_simulide_bridge_create(get_system_memory());
}

static void esp8266_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Espressif ESP8266 machine";
    mc->init = esp8266_machine_init;
    mc->max_cpus = 1;
    mc->default_cpus = 1;
    mc->default_ram_size = 0;
    mc->default_cpu_type = TYPE_ESP8266_CPU;
}

static const TypeInfo esp8266_info = {
    .name = TYPE_ESP8266_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Esp8266MachineState),
    .class_init = esp8266_machine_class_init,
};

static void esp8266_machine_type_init(void)
{
    type_register_static(&esp8266_info);
}

type_init(esp8266_machine_type_init);
