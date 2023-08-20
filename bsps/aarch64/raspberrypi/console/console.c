/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Console Configuration
 */

/*
 * Copyright (C) 2022 Mohd Noor Aman
 * Copyright (C) 2023 Utkarsh Verma
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "bsp/console.h"

#include <bsp/console-termios.h>
#include <bsp/fatal.h>
#include <bspopts.h>
#include <rtems/bspIo.h>
#include <rtems/console.h>
#include <stdint.h>

#include "bsp/aux.h"
#include "bsp/rpi-gpio.h"
#include "dev/serial/mini-uart.h"
#include "dev/serial/pl011.h"

#define UART_PORT_NAME(port) uart##port
#define UART_PORT_ENUM(port) UART##port

#define COMPARE_UART_PORT(port, file, base, size, tx, rx, alt_func, cmp_port) \
    cmp_port == UART_PORT_ENUM(port) ||
#define IS_MINI_UART_PORT(port) BSP_MINI_UARTS(COMPARE_UART_PORT, port) 0

#define UART_PORT(port, ...) UART_PORT_ENUM(port),
/* clang-format off */
typedef enum {
    BSP_PL011_UARTS(UART_PORT)
    BSP_MINI_UARTS(UART_PORT)

    UART_COUNT,
} pl01_uart_port;
/* clang-format on */

typedef struct {
    unsigned int tx_pin;
    unsigned int rx_pin;
    gpio_function gpio_func;
    console_device device;
} uart_config;

#define UART_DEVICE(port, file, base, size, tx, rx, alt_func, clock_hz)         \
    static pl011_context UART_PORT_NAME(port) = {                               \
        .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART" #port), \
        .base_reg     = base,                                                   \
        .clock        = clock_hz,                                               \
        .initial_baud = BSP_CONSOLE_BAUD,                                       \
    };

#define UART_CONFIG(port, file, base, size, tx, rx, alt_func, probe_func, \
                    handler_func)                                         \
    [UART_PORT_ENUM(port)] = {                                            \
        .tx_pin    = tx,                                                  \
        .rx_pin    = rx,                                                  \
        .gpio_func = alt_func,                                            \
        .device =                                                         \
            {                                                             \
                .device_file = file,                                      \
                .probe       = probe_func,                                \
                .context     = &UART_PORT_NAME(port).context,             \
                .handler     = handler_func,                              \
            },                                                            \
    },

BSP_PL011_UARTS(UART_DEVICE, BSP_PL011_CLOCK_FREQ)
BSP_MINI_UARTS(UART_DEVICE, BSP_SYSTEM_CLOCK_FREQ)

const uart_config configs[UART_COUNT] = {
    /* clang-format off */
    BSP_PL011_UARTS(UART_CONFIG, &pl011_probe, &pl011_handler)
    BSP_MINI_UARTS(UART_CONFIG, &mini_uart_probe, &mini_uart_handler)
    /* clang-format on */
};

static void output_char_pl011(char ch) {
    pl011_write_char_polled(configs[BSP_CONSOLE_PORT].device.context, ch);
}

static void output_char_mini_uart(char ch) {
    mini_uart_write_char_polled(configs[BSP_CONSOLE_PORT].device.context, ch);
}

rtems_device_driver console_initialize(rtems_device_major_number major,
                                       rtems_device_minor_number minor,
                                       void *arg) {
    rtems_termios_initialize();

    if (IS_MINI_UART_PORT(BSP_CONSOLE_PORT)) {
        aux_enable_mini_uart();
        BSP_output_char = output_char_mini_uart;
    }

    const uart_config *config    = &configs[BSP_CONSOLE_PORT];
    const console_device *device = &config->device;

    gpio_set_function(config->tx_pin, config->gpio_func);
    gpio_set_function(config->rx_pin, config->gpio_func);
    gpio_set_pull(config->tx_pin, GPIO_PULL_NONE);
    gpio_set_pull(config->rx_pin, GPIO_PULL_NONE);

    if ((*device->probe)(device->context)) {
        rtems_status_code status =
            rtems_termios_device_install(device->device_file, device->handler,
                                         device->flow, device->context);

        if (status != RTEMS_SUCCESSFUL) bsp_fatal(BSP_FATAL_CONSOLE_INSTALL_0);
    }

    if (link(config->device.device_file, CONSOLE_DEVICE_NAME) != 0) {
        bsp_fatal(BSP_FATAL_CONSOLE_INSTALL_1);
    }

    return RTEMS_SUCCESSFUL;
}

BSP_output_char_function_type BSP_output_char   = output_char_pl011;
BSP_polling_getchar_function_type BSP_poll_char = NULL;
