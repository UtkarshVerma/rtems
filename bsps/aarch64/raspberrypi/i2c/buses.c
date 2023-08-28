/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief I2C Buses
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

#include "bsp/i2c/buses.h"

#include <rtems/rtems/status.h>

#include "bsp/rpi-gpio.h"

rtems_status_code bsp_i2c_bus_init_gpio(
    const bsp_i2c_bus_gpio_metadata* metadata) {
    rtems_status_code status =
        gpio_set_function(metadata->sda, metadata->alt_func);
    if (status != RTEMS_SUCCESSFUL)
        return status;

    status = gpio_set_function(metadata->scl, metadata->alt_func);
    if (status != RTEMS_SUCCESSFUL)
        return status;

    status = gpio_set_pull(metadata->sda, GPIO_PULL_NONE);
    if (status != RTEMS_SUCCESSFUL)
        return status;

    status = gpio_set_pull(metadata->scl, GPIO_PULL_NONE);
    if (status != RTEMS_SUCCESSFUL)
        return status;

    return RTEMS_SUCCESSFUL;
}
