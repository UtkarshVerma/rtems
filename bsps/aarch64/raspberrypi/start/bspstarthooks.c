#include <bsp/irq-generic.h>

BSP_START_TEXT_SECTION void bsp_start_hook_0(void) {
    /* Do nothing */
}

BSP_START_TEXT_SECTION void bsp_start_hook_1(void) {
    AArch64_start_set_vector_base();
    bsp_start_copy_sections();
    raspberrypi_4_setup_mmu_and_cache();
    bsp_start_clear_bss();
}
