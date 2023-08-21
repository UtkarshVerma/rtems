/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Console Devices
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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_CONSOLE_DEVICES_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_CONSOLE_DEVICES_H

#include <bspopts.h>
#include <stdint.h>

#include "bsp/rpi-gpio.h"
#include "rtems/rtems/status.h"
#include "rtems/termiosdevice.h"

typedef enum {
    PL011_CONSOLE_DEVICE,
    MINI_UART_CONSOLE_DEVICE,
} bsp_console_device_type;

typedef struct {
    unsigned int tx_pin;
    unsigned int rx_pin;
    gpio_function alt_func;
} bsp_console_device_gpio_metadata;

typedef struct {
    const char* file;
    const bsp_console_device_type type;
    rtems_termios_device_context* context;
    const rtems_termios_device_handler* handler;
    const bsp_console_device_gpio_metadata gpio;
} bsp_console_device;

rtems_status_code console_device_init_gpio(
    const bsp_console_device_gpio_metadata* metadata);

#if RTEMS_BSP == raspberrypi4b

typedef enum {
    UART0,
    UART1,
    UART2,
    UART3,
    UART4,
    UART5,

    UART_COUNT,
} bsp_console_device_port;

extern const bsp_console_device raspberrypi4b_console_devices[UART_COUNT];

#endif /* raspberrypi4b */

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_CONSOLE_DEVICES_H */
