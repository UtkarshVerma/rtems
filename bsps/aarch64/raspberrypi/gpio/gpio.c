/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief GPIO Driver
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

#include <rtems/rtems/status.h>
#include <stdint.h>

#include "bsp/rpi-gpio.h"

#define REG(addr) *(volatile uint32_t*)(addr)

#define GPFSEL0                REG(BSP_GPIO_BASE + 0x00)
#define GPSET0                 REG(BSP_GPIO_BASE + 0x1c)
#define GPCLR0                 REG(BSP_GPIO_BASE + 0x28)
#define GPIO_PUP_PDN_CTRL_REG0 REG(BSP_GPIO_BASE + 0xe4)

static rtems_status_code gpio_set_reg(volatile uint32_t* base_reg,
                                      unsigned int pin, uint32_t value,
                                      unsigned int field_size) {
    if (pin > BSP_GPIO_PIN_COUNT) return RTEMS_INVALID_NUMBER;

    unsigned int field_mask = (1 << field_size) - 1;
    if (value > field_mask) return RTEMS_INVALID_NUMBER;

    // GPIO registers are uniformly subdivided
    unsigned int n_fields = sizeof(uint32_t) * 8 / field_size;

    // Registers are sequentially mapped for each `n_field` GPIOs
    volatile uint32_t* reg = base_reg + pin / n_fields;
    unsigned int shift     = (pin % n_fields) * field_size;

    unsigned int tmp = *reg;
    tmp &= ~(field_mask << shift);  // Clear the field
    tmp |= value << shift;          // Set value to the field
    *reg = tmp;

    return RTEMS_SUCCESSFUL;
}

rtems_status_code gpio_set_function(unsigned int pin, gpio_function value) {
    return gpio_set_reg(&GPFSEL0, pin, value, 3);
}

rtems_status_code gpio_clear_pin(unsigned int pin) {
    return gpio_set_reg(&GPCLR0, pin, 1, 1);
}

rtems_status_code gpio_set_pin(unsigned int pin) {
    return gpio_set_reg(&GPSET0, pin, 1, 1);
}

rtems_status_code gpio_set_pull(unsigned int pin, gpio_pull value) {
    return gpio_set_reg(&GPIO_PUP_PDN_CTRL_REG0, pin, value, 2);
}
