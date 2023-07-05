#include <bsp/bootcard.h>
#include <bsp/irq-generic.h>

void bsp_start(void) {
    bsp_interrupt_initialize();
    rtems_cache_coherent_add_area(bsp_section_nocacheheap_begin,
                                  (uintptr_t)bsp_section_nocacheheap_size);
}
