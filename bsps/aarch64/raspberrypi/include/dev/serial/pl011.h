#include <rtems/termiostypes.h>

typedef struct {
    rtems_termios_device_context context;
    uintptr_t base_reg;
    uint32_t clock;
    uint32_t initial_baud;
} pl011_context;

bool pl011_probe(rtems_termios_device_context *ctx);

void pl011_write_char_polled(rtems_termios_device_context *ctx, char ch);

extern const rtems_termios_device_handler pl011_handler;
