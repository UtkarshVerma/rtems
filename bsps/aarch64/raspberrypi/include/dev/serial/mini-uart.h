#pragma once

#include <rtems/termiostypes.h>
#include <stdint.h>

typedef struct {
    rtems_termios_device_context context;
    uintptr_t base_reg;
    uint32_t clock;
    uint32_t initial_baud;
} mini_uart_context;

bool mini_uart_probe(rtems_termios_device_context* ctx);
void mini_uart_write_char_polled(rtems_termios_device_context* ctx, char ch);

extern const rtems_termios_device_handler mini_uart_handler;
