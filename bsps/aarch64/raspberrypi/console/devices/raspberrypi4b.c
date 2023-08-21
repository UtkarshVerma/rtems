/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Raspberry Pi 4B Console Devices
 */

/*
 * Copyright (c) 2023 Utkarsh Verma
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

#include <bspopts.h>
#include <rtems/termiosdevice.h>

#include "bsp/bcm2711.h"
#include "bsp/console/devices.h"
#include "dev/serial/mini-uart.h"
#include "dev/serial/pl011.h"

static pl011_context uart0 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("uart0"),
    .regs_base    = BCM2711_UART0_BASE,
    .clock        = BSP_PL011_CLOCK_FREQ,
    .initial_baud = BSP_CONSOLE_BAUD,
    .irq          = BCM2711_IRQ_PL011_UART,
};

static mini_uart_context uart1 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("uart1"),
    .regs_base    = BCM2711_UART1_BASE,
    .clock        = BSP_SYSTEM_CLOCK_FREQ,
    .initial_baud = BSP_CONSOLE_BAUD,
    .irq          = BCM2711_IRQ_AUX,
};

static pl011_context uart2 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("uart2"),
    .regs_base    = BCM2711_UART2_BASE,
    .clock        = BSP_PL011_CLOCK_FREQ,
    .initial_baud = BSP_CONSOLE_BAUD,
    .irq          = BCM2711_IRQ_PL011_UART,
};
static pl011_context uart3 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("uart3"),
    .regs_base    = BCM2711_UART3_BASE,
    .clock        = BSP_PL011_CLOCK_FREQ,
    .initial_baud = BSP_CONSOLE_BAUD,
    .irq          = BCM2711_IRQ_PL011_UART,
};
static pl011_context uart4 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("uart4"),
    .regs_base    = BCM2711_UART4_BASE,
    .clock        = BSP_PL011_CLOCK_FREQ,
    .initial_baud = BSP_CONSOLE_BAUD,
    .irq          = BCM2711_IRQ_PL011_UART,
};
static pl011_context uart5 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("uart5"),
    .regs_base    = BCM2711_UART5_BASE,
    .clock        = BSP_PL011_CLOCK_FREQ,
    .initial_baud = BSP_CONSOLE_BAUD,
    .irq          = BCM2711_IRQ_PL011_UART,
};

const bsp_console_device raspberrypi4b_console_devices[UART_COUNT] = {
    [UART0] =
        {
            .file    = "/dev/ttyAMA0",
            .type    = PL011_CONSOLE_DEVICE,
            .context = &uart0.context,
            .handler = &pl011_polled_handler,
            .gpio    = {14, 15, GPIO_AF0},
        },
    [UART1] =
        {
            .file    = "/dev/ttyS0",
            .type    = MINI_UART_CONSOLE_DEVICE,
            .context = &uart1.context,
            .handler = &mini_uart_polled_handler,
            .gpio    = {14, 15, GPIO_AF5},
        },

    [UART2] =
        {
            .file    = "/dev/ttyAMA1",
            .type    = PL011_CONSOLE_DEVICE,
            .context = &uart2.context,
            .handler = &pl011_polled_handler,
            .gpio    = {0, 1, GPIO_AF4},
        },

    [UART3] =
        {
            .file    = "/dev/ttyAMA2",
            .type    = PL011_CONSOLE_DEVICE,
            .context = &uart3.context,
            .handler = &pl011_polled_handler,
            .gpio    = {4, 5, GPIO_AF4},
        },

    [UART4] =
        {
            .file    = "/dev/ttyAMA3",
            .type    = PL011_CONSOLE_DEVICE,
            .context = &uart4.context,
            .handler = &pl011_polled_handler,
            .gpio    = {8, 9, GPIO_AF4},
        },

    [UART5] =
        {
            .file    = "/dev/ttyAMA4",
            .type    = PL011_CONSOLE_DEVICE,
            .context = &uart5.context,
            .handler = &pl011_polled_handler,
            .gpio    = {12, 13, GPIO_AF4},
        },
};
