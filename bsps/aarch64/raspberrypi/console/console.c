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

#include <bsp/console-termios.h>
#include <bsp/fatal.h>
#include <bspopts.h>
#include <rtems/bspIo.h>
#include <rtems/console.h>
#include <rtems/rtems/status.h>
#include <rtems/termiosdevice.h>
#include <stdint.h>

#include "bsp/aux.h"
#include "bsp/console/devices.h"

#if RTEMS_BSP == rasbperrypi4b

static const bsp_console_device* config =
    &raspberrypi4b_console_devices[BSP_CONSOLE_PORT];

#endif /* raspberrypi4b */

static void output_char(char ch) {
    config->handler->write(config->context, &ch, 1);
}

rtems_device_driver console_initialize(rtems_device_major_number major,
                                       rtems_device_minor_number minor,
                                       void* arg) {
    rtems_termios_initialize();

    if (config->type == MINI_UART_CONSOLE_DEVICE)
        aux_enable_mini_uart();

    rtems_status_code status = console_device_init_gpio(&config->gpio);
    if (status != RTEMS_SUCCESSFUL)
        bsp_fatal(BSP_FATAL_CONSOLE_INSTALL_0);

    status = rtems_termios_device_install(config->file, config->handler, NULL,
                                          config->context);
    if (status != RTEMS_SUCCESSFUL)
        bsp_fatal(BSP_FATAL_CONSOLE_INSTALL_0);

    if (link(config->file, CONSOLE_DEVICE_NAME) != 0) {
        bsp_fatal(BSP_FATAL_CONSOLE_INSTALL_1);
    }

    return RTEMS_SUCCESSFUL;
}

BSP_output_char_function_type BSP_output_char   = output_char;
BSP_polling_getchar_function_type BSP_poll_char = NULL;
