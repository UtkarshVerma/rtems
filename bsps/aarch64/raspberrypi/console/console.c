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
#include <dev/serial/pl011.h>
#include <rtems/bspIo.h>
#include <rtems/console.h>
#include <rtems/rtems/status.h>
#include <rtems/termiosdevice.h>
#include <stdint.h>

#include "bsp/rpi-gpio.h"

#define CONSOLE_DEVICE_CONTEXT_NAME(port_no) uart##port_no##_context

#define CONSOLE_DEVICE_CONTEXT(port_no, _file_name, base, _size, clock_freq,    \
                               irq_no, context_type, ...)                       \
    static context_type CONSOLE_DEVICE_CONTEXT_NAME(port_no) = {                \
        .context   = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART" #port_no), \
        .regs_base = base,                                                      \
        .clock     = clock_freq,                                                \
        .initial_baud = BSP_CONSOLE_BAUD,                                       \
        .irq          = irq_no,                                                 \
    };

#define CONSOLE_DEVICE(port_no, file_name, _base, _size, _clock, _irq,      \
                       _context_type, dev_handler, write_char_func, rx_pin, \
                       tx_pin, gpio_func, ...)                              \
    [CONSOLE_DEVICE_PORT2ENUM(port_no)] = {                                 \
        .file    = file_name,                                               \
        .context = &CONSOLE_DEVICE_CONTEXT_NAME(port_no).context,           \
        .gpio    = {.rx = rx_pin, .tx = tx_pin, .function = gpio_func},     \
        .handler = dev_handler,                                             \
        .write_char_polled = write_char_func,                               \
    },

typedef struct {
    const unsigned int rx;
    const unsigned int tx;
    const gpio_function function;
} bsp_console_device_gpio_config;

typedef struct {
    const char* file;
    rtems_termios_device_context* context;
    const bsp_console_device_gpio_config gpio;

    const rtems_termios_device_handler* handler;
    void (*write_char_polled)(const rtems_termios_device_context*, const char);
} bsp_console_device;

/* Initialize all console device contexts */
CONSOLE_DEVICES(CONSOLE_DEVICE_CONTEXT)

/* Initialize all device configurations */
static const bsp_console_device devices[CONSOLE_DEVICE_COUNT] = {
    /* clang-format off */
    CONSOLE_DEVICES(CONSOLE_DEVICE)
    /* clang-format on */
};

static rtems_status_code console_device_init_gpio(
    const bsp_console_device_gpio_config* gpio) {
    rtems_status_code status = gpio_set_function(gpio->rx, gpio->function);
    if (status != RTEMS_SUCCESSFUL)
        return status;

    status = gpio_set_function(gpio->tx, gpio->function);
    if (status != RTEMS_SUCCESSFUL)
        return status;

    status = gpio_set_pull(gpio->rx, GPIO_PULL_NONE);
    if (status != RTEMS_SUCCESSFUL)
        return status;

    status = gpio_set_pull(gpio->tx, GPIO_PULL_NONE);
    if (status != RTEMS_SUCCESSFUL)
        return status;

    return RTEMS_SUCCESSFUL;
}

static void output_char(const char ch) {
    const bsp_console_device* device = &devices[BSP_CONSOLE_PORT];

    device->write_char_polled(device->context, ch);
}

static int poll_char(void) {
    const bsp_console_device* device = &devices[BSP_CONSOLE_PORT];

    return device->handler->poll_read(device->context);
}

rtems_device_driver console_initialize(rtems_device_major_number major,
                                       rtems_device_minor_number minor,
                                       void* arg) {
    const bsp_console_device* device = &devices[BSP_CONSOLE_PORT];

    rtems_termios_initialize();

    rtems_status_code status = console_device_init_gpio(&device->gpio);
    if (status != RTEMS_SUCCESSFUL)
        bsp_fatal(BSP_FATAL_CONSOLE_REGISTER_DEV_0);

    status = rtems_termios_device_install(device->file, device->handler, NULL,
                                          device->context);
    if (status != RTEMS_SUCCESSFUL)
        bsp_fatal(BSP_FATAL_CONSOLE_INSTALL_0);

    if (link(device->file, CONSOLE_DEVICE_NAME) != 0)
        bsp_fatal(BSP_FATAL_CONSOLE_INSTALL_1);

    return RTEMS_SUCCESSFUL;
}

BSP_output_char_function_type BSP_output_char   = output_char;
BSP_polling_getchar_function_type BSP_poll_char = poll_char;
