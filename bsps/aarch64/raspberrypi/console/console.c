#include <bsp/console-termios.h>
#include <dev/serial/arm-pl011.h>
#include <rtems/bspIo.h>

#include "bsp/raspberrypi/pl011.h"

arm_pl011_context uart0 = {
    .base         = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("PL011"),
    .regs         = (volatile pl011 *)PL011_UART0_BASE,
    .initial_baud = 115200,
};

const console_device console_device_table[] = {
    {
        .device_file = "/dev/ttyS0",
        .probe       = console_device_probe_default,
        .handler     = &arm_pl011_fns,
        .context     = &uart0.base,
    },
};

const size_t console_device_count = RTEMS_ARRAY_SIZE(console_device_table);

static void output_char(char c) {
    arm_pl011_write_polled(&uart0.base, c);
}

BSP_output_char_function_type BSP_output_char = output_char;

BSP_polling_getchar_function_type BSP_poll_char = NULL;
