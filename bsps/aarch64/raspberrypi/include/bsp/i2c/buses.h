/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief I2C Devices
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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_I2C_BUSES_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_I2C_BUSES_H

#include <bspopts.h>
#include <dev/i2c/i2c.h>
#include <rtems/rtems/status.h>

#include "bsp/rpi-gpio.h"

typedef struct {
    unsigned int sda;
    unsigned int scl;
    gpio_function alt_func;
} bsp_i2c_bus_gpio_metadata;

typedef struct {
    const char* file;
    i2c_bus* bus;
    const bsp_i2c_bus_gpio_metadata gpio;
} bsp_i2c_bus;

int bsp_i2c_register_bus(bsp_i2c_bus* bus);

rtems_status_code bsp_i2c_bus_init_gpio(
    const bsp_i2c_bus_gpio_metadata* metadata);

#if RTEMS_BSP == raspberrypi4b

/*
 * NOTE:
 * I2C2 and I2C7 are for dedicated use by the HDMI interfaces and not
 * user-accessible.
 */
typedef enum {
    I2C0,
    I2C1,
    I2C3,
    I2C4,
    I2C5,
    I2C6,

    I2C_COUNT,
} bsp_i2c_bus_port;

extern const bsp_i2c_bus raspberrypi4b_i2c_buses[I2C_COUNT];

#endif /* raspberrypi4b */

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_I2C_BUSES_H */
