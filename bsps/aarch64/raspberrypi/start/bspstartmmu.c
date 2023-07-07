#include <bsp/aarch64-mmu.h>
#include <bsp/soc.h>

BSP_START_DATA_SECTION static const aarch64_mmu_config_entry
    raspberrypi_4_mmu_config_table[] = {
        AARCH64_MMU_DEFAULT_SECTIONS,
        {
            // Auxiliary peripherals: UART1, SPI1, SPI2
            .begin = SOC_AUX_BASE,
            .end   = SOC_AUX_BASE + SOC_AUX_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
        {
            // UART0
            .begin = SOC_UART0_BASE,
            .end   = SOC_UART0_BASE + SOC_PL011_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
        {
            // UART2
            .begin = SOC_UART2_BASE,
            .end   = SOC_UART2_BASE + SOC_PL011_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
        {
            // UART3
            .begin = SOC_UART3_BASE,
            .end   = SOC_UART3_BASE + SOC_PL011_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
        {
            // UART4
            .begin = SOC_UART4_BASE,
            .end   = SOC_UART4_BASE + SOC_PL011_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
        {
            // UART5
            .begin = SOC_UART5_BASE,
            .end   = SOC_UART5_BASE + SOC_PL011_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
        {
            // GPIO
            .begin = SOC_GPIO_BASE,
            .end   = SOC_GPIO_BASE + SOC_GPIO_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
        {
            // Interrupts
            .begin = SOC_GIC_BASE,
            .end   = SOC_GIC_BASE + SOC_GIC_SIZE,
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
