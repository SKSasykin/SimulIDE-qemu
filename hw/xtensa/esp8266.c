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
#include "qemu/datadir.h"
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
    DeviceState *gpio;
    MemoryRegion dport;
    uint32_t dport_reg[0x40];
    uint32_t fw_entry;
    bool rom_loaded;
};

/* The user application is loaded at the start of the IRAM, there is
 * no boot ROM in the machine. */
#define ESP8266_IRAM_BASE   0x40000000
#define ESP8266_RAW_MAX     0x100000
#define ESP8266_IROM_BASE   0x40200000
#define ESP8266_IROM_SIZE   0x100000
#define ESP8266_ROM_BASE    0x40000000
#define ESP8266_ROM_SIZE    0x10000
#define ESP8266_ROM_NAME    "esp8266.rom"
#define ESP8266_CALL_USER_ROM_NAME "esp8266-call-user.rom"

#define ESP8266_UART0_BASE  0x60000000
#define ESP8266_UART1_BASE  0x60000f00

#define ESP8266_GPIO_BASE   0x60000300
#define ESP8266_IOMUX_BASE  0x60000800
#define ESP8266_FRC_BASE    0x60000600
#define ESP8266_SPI0_BASE   0x60000200
#define ESP8266_DPORT_BASE  0x3ff00000

#define ESP8266_DPORT_SPI       0x03
#define ESP8266_DPORT_MACADDR   0x14

/* SPI flash controller register layout. */
enum {
    ESP8266_SPI_FLASH_CMD,
    ESP8266_SPI_FLASH_ADDR,
    ESP8266_SPI_FLASH_CTRL,
    ESP8266_SPI_FLASH_CTRL1,
    ESP8266_SPI_FLASH_STATUS,
    ESP8266_SPI_FLASH_CTRL2,
    ESP8266_SPI_FLASH_CLOCK,
    ESP8266_SPI_FLASH_USER,
    ESP8266_SPI_FLASH_USER1,
    ESP8266_SPI_FLASH_USER2,
    ESP8266_SPI_FLASH_USER3,
    ESP8266_SPI_FLASH_PIN,
    ESP8266_SPI_FLASH_SLAVE,
    ESP8266_SPI_FLASH_SLAVE1,
    ESP8266_SPI_FLASH_SLAVE2,
    ESP8266_SPI_FLASH_SLAVE3,
    ESP8266_SPI_FLASH_C0,
    ESP8266_SPI_FLASH_C1,
    ESP8266_SPI_FLASH_C2,
    ESP8266_SPI_FLASH_C3,
    ESP8266_SPI_FLASH_C4,
    ESP8266_SPI_FLASH_C5,
    ESP8266_SPI_FLASH_C6,
    ESP8266_SPI_FLASH_C7,
    ESP8266_SPI_FLASH_EXT0 = 0x3c,
    ESP8266_SPI_FLASH_EXT1,
    ESP8266_SPI_FLASH_EXT2,
    ESP8266_SPI_FLASH_EXT3,
    ESP8266_SPI_MAX,
};

#define ESP8266_MAX_FLASH_SZ (1 << 24)

enum {
    ESP8266_SPI_CMD_USR_SHIFT = 18,
    ESP8266_SPI_CMD_WRDI_SHIFT = 29,
    ESP8266_SPI_CMD_WREN_SHIFT = 30,
    ESP8266_SPI_CMD_READ_SHIFT = 31,
    ESP8266_SPI_STATUS_WRENABLE_SHIFT = 1,
};

#define ESP8266_SPI_FLASH_CTRL_ENABLE_AHB BIT(17)
#define ESP8266_SPI_FLASH_USER_FLASH_MODE  BIT(2)

typedef struct Esp8266SpiState {
    MemoryRegion iomem;
    MemoryRegion cache;
    uint8_t *flash_image;
    uint32_t reg[ESP8266_SPI_MAX];
} Esp8266SpiState;

static uint64_t esp8266_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp8266SpiState *s = opaque;

    if (addr / 4 >= ESP8266_SPI_MAX || addr % 4 || size != 4) {
        return 0;
    }
    return s->reg[addr / 4];
}

static void esp8266_spi_cmd(Esp8266SpiState *s, uint64_t val)
{
    if (val & BIT(ESP8266_SPI_CMD_READ_SHIFT)) {
        if (s->reg[ESP8266_SPI_FLASH_USER] & ESP8266_SPI_FLASH_USER_FLASH_MODE
            && s->flash_image) {
            uint32_t offset = s->reg[ESP8266_SPI_FLASH_ADDR] & 0xffffff;
            uint32_t length = (s->reg[ESP8266_SPI_FLASH_ADDR] >> 24) & 0xff;
            memcpy(s->reg + ESP8266_SPI_FLASH_C0,
                   s->flash_image + offset, (length + 3) & ~3u);
        }
    }
    if (val & BIT(ESP8266_SPI_CMD_WRDI_SHIFT)) {
        s->reg[ESP8266_SPI_FLASH_STATUS] &= ~BIT(ESP8266_SPI_STATUS_WRENABLE_SHIFT);
    }
    if (val & BIT(ESP8266_SPI_CMD_WREN_SHIFT)) {
        s->reg[ESP8266_SPI_FLASH_STATUS] |= BIT(ESP8266_SPI_STATUS_WRENABLE_SHIFT);
    }
}

static void esp8266_spi_write_ctrl(Esp8266SpiState *s, uint64_t val)
{
    s->reg[ESP8266_SPI_FLASH_CTRL] = val;
    memory_region_set_enabled(&s->cache,
                              val & ESP8266_SPI_FLASH_CTRL_ENABLE_AHB);
}

static void esp8266_spi_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    Esp8266SpiState *s = opaque;

    if (addr / 4 >= ESP8266_SPI_MAX || addr % 4 || size != 4) {
        return;
    }
    unsigned reg = addr / 4;
    switch (reg) {
    case ESP8266_SPI_FLASH_CMD:
        esp8266_spi_cmd(s, val);
        break;
    case ESP8266_SPI_FLASH_CTRL:
        esp8266_spi_write_ctrl(s, val);
        break;
    case ESP8266_SPI_FLASH_STATUS:
        /* Read-only register. */
        break;
    default:
        s->reg[reg] = val;
        break;
    }
}

static uint64_t esp8266_spi_cache_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp8266SpiState *s = opaque;

    if (!s->flash_image) {
        return 0;
    }
    uint32_t off = addr & (ESP8266_MAX_FLASH_SZ - 1);

    switch (size) {
    case 1:
        return s->flash_image[off];
    case 2:
        return lduw_le_p(s->flash_image + off);
    case 4:
        return ldl_le_p(s->flash_image + off);
    default:
        return 0;
    }
}

static void esp8266_spi_cache_write(void *opaque, hwaddr addr, uint64_t val,
                                   unsigned size)
{
    /* The AHB cache window is a read-only view of the flash image. */
}

static const MemoryRegionOps esp8266_spi_cache_ops = {
    .read = esp8266_spi_cache_read,
    .write = esp8266_spi_cache_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void esp8266_spi_reset(void *opaque)
{
    Esp8266SpiState *s = opaque;

    memset(s->reg, 0, sizeof(s->reg));
    memory_region_set_enabled(&s->cache, false);
}

static const MemoryRegionOps esp8266_spi_ops = {
    .read = esp8266_spi_read,
    .write = esp8266_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t esp8266_dport_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp8266MachineState *ms = opaque;

    if ((addr & 3) || size != 4) {
        return 0;
    }
    return ms->dport_reg[(addr / 4) % ARRAY_SIZE(ms->dport_reg)];
}

static void esp8266_dport_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    Esp8266MachineState *ms = opaque;

    if ((addr & 3) || size != 4) {
        return;
    }

    unsigned reg = (addr / 4) % ARRAY_SIZE(ms->dport_reg);
    ms->dport_reg[reg] = value;
    if (reg == ESP8266_DPORT_SPI && (value & 1)) {
        /* The cache/SPI operation completes synchronously in this model. */
        ms->dport_reg[reg] |= 2;
    }
}

static const MemoryRegionOps esp8266_dport_ops = {
    .read = esp8266_dport_read,
    .write = esp8266_dport_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}

static void esp8266_reset(void *opaque)
{
    Esp8266MachineState *ms = opaque;

    cpu_reset(CPU(ms->cpu));
    memset(ms->dport_reg, 0, sizeof(ms->dport_reg));
    ms->dport_reg[ESP8266_DPORT_MACADDR + 0] = 0x01234567;
    ms->dport_reg[ESP8266_DPORT_MACADDR + 1] = 0x89abcdef;
    ms->dport_reg[ESP8266_DPORT_MACADDR + 2] = 0x00008000;

    if (ms->rom_loaded) {
        /* Boot ROM is present: cpu_reset() already set PC to the
         * ROM reset vector (VECBASE + 0x80).  The ROM will initialise
         * DRAM and peripherals, read the firmware entry from the GPIO
         * cheat, then jump into user code. */
        return;
    }
    /* No boot ROM: jump straight into the user firmware. */
    ms->cpu->env.sregs[PS] = PS_UM;
    ms->cpu->env.regs[1] = 0x3ffffff0;
    ms->cpu->env.pc = ms->fw_entry;
}

static bool esp8266_load_image(const uint8_t *image, size_t image_size,
                                AddressSpace *as, uint64_t *entry,
                                uint8_t *flash_image)
{
    if (image_size < 8 || image[0] != 0xe9) {
        return false;
    }

    /* Arduino's combined image stores the application E9 image at the next
     * flash sector after eboot. With no boot ROM, load and enter that image
     * directly instead of trying to execute eboot without a ROM stack. */
    size_t header_offset = 0;
    if (image_size > 0x1008 && image[0x1000] == 0xe9) {
        header_offset = 0x1000;
    }

    uint8_t segment_count = image[header_offset + 1];
    if (!segment_count || segment_count > 16) {
        error_report("Invalid ESP8266 image segment count %u", segment_count);
        exit(1);
    }

    *entry = ldl_le_p(image + header_offset + 4);

    /* ESP8266 flash cache maps flash offset N at 0x40200000 + N. Arduino
     * images contain eboot at offset zero and the application at 0x1000;
     * only the RAM segments are described by the first E9 header. The whole
     * image is copied into the SPI controller's AHB cache window so the ROM
     * bootloader (and the running firmware) can read it back via 0x40200000
     * or through SPI command-mode reads. */
    if (flash_image) {
        size_t mapped_size = MIN(image_size, (size_t)ESP8266_MAX_FLASH_SZ);
        memcpy(flash_image, image, mapped_size);
    }

    size_t offset = header_offset + 8;
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

        if (address < ESP8266_IROM_BASE ||
            address >= ESP8266_IROM_BASE + ESP8266_IROM_SIZE) {
            char *name = g_strdup_printf("esp8266.segment.%u", i);
            rom_add_blob_fixed_as(name, image + offset, length, address, as);
            g_free(name);
        }
        offset += length;
    }
    return true;
}

static void esp8266_machine_init(MachineState *machine)
{
    Esp8266MachineState *ms = ESP8266_MACHINE(machine);
    Esp8266SpiState *spi = NULL;

    ms->cpu = XTENSA_CPU(cpu_create(machine->cpu_type));
    ms->cpu->env.sregs[PRID] = 0;
    ms->fw_entry = ESP8266_IRAM_BASE;

    /* The lx106 config defines two reset vectors: RESET0 at 0x50000000
     * (unmapped) and RESET1 at 0x40000080 (ROM).  cpu_reset() selects
     * between them via env->static_vectors, so we must call this before
     * the first reset. */
    xtensa_select_static_vectors(&ms->cpu->env, 1);

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

    memory_region_init_io(&ms->dport, OBJECT(machine), &esp8266_dport_ops, ms,
                           "esp8266.dport", 0x10000);
    memory_region_add_subregion(get_system_memory(), ESP8266_DPORT_BASE,
                                &ms->dport);

    /* The LX106 core reports an XLMI data RAM at 0x3ff00000 (256 KiB).  The
     * lower 64 KiB of that range collides with the DPORT peripheral, so the
     * remaining gap (0x3ff10000..0x3ff3ffff) is mapped here as RAM.  The SDK
     * firmware accesses it early during startup (e.g. 0x3ff20c00) and stalls
     * with an unaligned/PIF error if it is left unmapped. */
    {
        static MemoryRegion xlmigap;
        memory_region_init_ram(&xlmigap, NULL, "esp8266.xlmi-gap", 0x30000,
                               &error_abort);
        memory_region_add_subregion(get_system_memory(), 0x3ff10000,
                                    &xlmigap);
    }

    bool have_fw = machine->firmware || machine->kernel_filename;
    const char *fw_path = have_fw
        ? (machine->firmware ? machine->firmware : machine->kernel_filename)
        : NULL;

    /* E9 images (the Arduino/SDK combined flash layout) rely on the ROM's
     * chip/DRAM initialisation and are entered through the "call-user" ROM
     * handoff.  Raw flat blobs and ELF images are self-contained and must be
     * entered directly (the original behaviour); otherwise a short example
     * such as blink.bin would be overwritten by the ROM at 0x40000000 and
     * never run. */
    bool is_e9 = false;
    if (fw_path) {
        g_autofree char *fw_buf = NULL;
        gsize fw_len = 0;
        if (g_file_get_contents(fw_path, &fw_buf, &fw_len, NULL)
            && fw_len >= 8 && (uint8_t)fw_buf[0] == 0xe9) {
            is_e9 = true;
        }
    }

    /* The mask ROM is proprietary and is therefore not bundled. When an E9
     * user firmware image is supplied, load the "call-user" variant of the
     * ROM: it performs the same chip initialisation but then reads the
     * firmware entry from the GPIO cheat register (offset 0x80, served by the
     * SimulIDE bridge) and jumps straight into user code, bypassing the
     * on-flash bootloader chain (which is not modelled).  Self-contained
     * firmware remains supported without it. */
    if (have_fw && is_e9) {
        const char *rom_name = ESP8266_CALL_USER_ROM_NAME;
        g_autofree char *rom_filename =
            qemu_find_file(QEMU_FILE_TYPE_BIOS, rom_name);
        if (!rom_filename) {
            rom_name = ESP8266_ROM_NAME;
            rom_filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, rom_name);
        }
        if (rom_filename) {
            gsize fsize = 0;
            g_autoptr(GError) gerr = NULL;
            g_autofree char *rom_data = NULL;

            if (!g_file_get_contents(rom_filename, &rom_data, &fsize, &gerr)) {
                error_report("Could not read ESP8266 ROM '%s': %s",
                             rom_filename, gerr->message);
                exit(1);
            }
            if (fsize < ESP8266_ROM_SIZE) {
                error_report("ESP8266 ROM '%s' is too small", rom_filename);
                exit(1);
            }

            rom_add_blob_fixed(rom_name, rom_data, ESP8266_ROM_SIZE,
                               ESP8266_ROM_BASE);
            ms->rom_loaded = true;
        }
    }

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
    ms->gpio = gpio;

    /* SPI flash controller: register interface plus an AHB cache window at
     * 0x40200000 that mirrors the flash image. The ROM enables it once it has
     * configured the SPI clock (CTRL.ENABLE_AHB).  The window is modelled as
     * an I/O region backed by a malloc'd copy of the flash image; this avoids
     * memory_region_init_ram(), which hangs machine initialisation in this
     * QEMU build. */
    spi = g_malloc(sizeof(*spi));
    memset(spi, 0, sizeof(*spi));
    spi->flash_image = g_malloc0(ESP8266_MAX_FLASH_SZ);
    memory_region_init_io(&spi->iomem, NULL, &esp8266_spi_ops, spi,
                          "esp8266.spi0", 0x100);
    memory_region_init_io(&spi->cache, NULL, &esp8266_spi_cache_ops, spi,
                          "esp8266.flash", ESP8266_MAX_FLASH_SZ);
    memory_region_add_subregion(get_system_memory(), ESP8266_SPI0_BASE,
                                &spi->iomem);
    memory_region_add_subregion(get_system_memory(), ESP8266_IROM_BASE,
                                &spi->cache);
    memory_region_set_enabled(&spi->cache, false);
    qemu_register_reset(esp8266_spi_reset, spi);

    /* FRC1 is a 23-bit countdown timer; external input 7 maps to IRQ 9. */
    DeviceState *frc = qdev_new(TYPE_ESP8266_FRC1);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(frc), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(frc), 0,
                       xtensa_get_extints(&ms->cpu->env)[7]);
    /* Priority 2 so the FRC wins over the SimulIDE bridge (priority 1),
     * whose UART0 range (0x60000000..0x60000FFF) would otherwise shadow it */
    memory_region_add_subregion_overlap(get_system_memory(), ESP8266_FRC_BASE,
                                        sysbus_mmio_get_region(SYS_BUS_DEVICE(frc), 0), 2);

    /* Peripheral register windows the ROM touches during init.  The
     * exact device behaviour is irrelevant for now; it is enough that
     * reads return zero instead of raising "unassigned" faults. */
    create_unimplemented_device("esp8266.iomux", ESP8266_IOMUX_BASE, 0x1000);
    create_unimplemented_device("esp8266.rtc",  0x60000700, 0x100);
    create_unimplemented_device("esp8266.i2c",  0x60000d00, 0x100);
    create_unimplemented_device("esp8266.sdio", 0x60000e00, 0x100);

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
                                    CPU(ms->cpu)->as, &entry,
                                    spi ? spi->flash_image : NULL)) {
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
    if (ms->rom_loaded) {
        Esp8266GpioState *gs = ESP8266_GPIO(ms->gpio);
        gs->user_entry = entry;
        /* ROM bootloader will initialise DRAM and peripherals, then
         * read the firmware entry from the GPIO cheat at offset 0x80.
         * Re-run the full reset so that cpu_reset() puts the CPU back
         * into kernel mode (the earlier esp8266_reset() call set
         * PS=PS_UM before the ROM was loaded). */
        esp8266_reset(ms);
    } else {
        ms->cpu->env.pc = entry;
    }
    qemu_log("esp8266: loaded firmware, entry point 0x%08" PRIx64 "\n", entry);

    /* SimulIDE bridge shadows UART0/GPIO/UART1 (must be last, overlaps) */
    esp8266_simulide_bridge_create(get_system_memory(),
                                   ms->rom_loaded ? ms->fw_entry : 0);
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
