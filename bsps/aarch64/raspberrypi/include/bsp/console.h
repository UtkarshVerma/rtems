/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Console Configuration
 */

/*
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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_CONSOLE_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_CONSOLE_H

#include <bspopts.h>
#include "bsp/aux.h"
#include "bsp/fatal.h"

#if RTEMS_BSP == raspberrypi4b
#include "console/raspberrypi4b.def"

#define CONSOLE_DEVICES RASPBERRYPI4B_CONSOLE_DEVICES
#endif /* raspberrypi4b */

#define CONSOLE_DEVICE_PORT2ENUM(port_no) UART##port_no
#define CONSOLE_DEVICE_ENUM(port_no, ...) CONSOLE_DEVICE_PORT2ENUM(port_no),
typedef enum {
    /* clang-format off */
    CONSOLE_DEVICES(CONSOLE_DEVICE_ENUM)
    /* clang-format on */

    CONSOLE_DEVICE_COUNT,
} bsp_console_device_port;
#undef CONSOLE_DEVICE_ENUM

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

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_CONSOLE_H */
