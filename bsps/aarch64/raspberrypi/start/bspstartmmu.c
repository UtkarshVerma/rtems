#include <bsp/aarch64-mmu.h>

#include "bsp/raspberrypi/gic.h"
#include "bsp/raspberrypi/pl011.h"

BSP_START_DATA_SECTION static const aarch64_mmu_config_entry
    raspberrypi_4_mmu_config_table[] = {
        AARCH64_MMU_DEFAULT_SECTIONS,
        {
            .begin = PL011_UART0_BASE,
            .end   = PL011_UART0_BASE + PL011_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
        {
            .begin = GIC_BASE,
            .end   = GIC_BASE + GIC_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
};

// Make weak and let the user override
BSP_START_TEXT_SECTION void raspberrypi_4_setup_mmu_and_cache(void)
    __attribute__((weak));

BSP_START_TEXT_SECTION void raspberrypi_4_setup_mmu_and_cache(void) {
    aarch64_mmu_setup();

    aarch64_mmu_setup_translation_table(
        &raspberrypi_4_mmu_config_table[0],
        RTEMS_ARRAY_SIZE(raspberrypi_4_mmu_config_table));

    aarch64_mmu_enable();
}
