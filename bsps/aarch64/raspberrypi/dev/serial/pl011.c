/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief PL011 UART Device Driver
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

#include "dev/serial/pl011.h"

#include <bsp/utility.h>
#include <stdint.h>

#define REG(addr) *(volatile uint32_t *)(addr)

#define DR              REG(base + 0x00)
#define DR_DATA_MASK    BSP_MSK32(0, 7)
#define FR              REG(base + 0x18)
#define FR_BUSY         BSP_BIT32(3)
#define FR_RXFE         BSP_BIT32(4)
#define FR_TXFF         BSP_BIT32(5)
#define IBRD            REG(base + 0x24)
#define FBRD            REG(base + 0x28)
#define LCRH            REG(base + 0x2c)
#define LCRH_FEN        BSP_BIT32(4)
#define LCRH_WLEN_8BITS BSP_MSK32(5, 6)
#define CR              REG(base + 0x30)
#define CR_UARTEN       BSP_BIT32(0)
#define CR_TXE          BSP_BIT32(8)
#define CR_RXE          BSP_BIT32(9)
#define ICR             REG(base + 0x44)

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *ctx, struct termios *term,
                       rtems_libio_open_close_args_t *args);
static int read_char_polled(rtems_termios_device_context *ctx);
static void write_polled(rtems_termios_device_context *ctx, const char *buf,
                         size_t n);

const rtems_termios_device_handler pl011_handler = {
    .first_open = first_open,
    .poll_read  = read_char_polled,
    .write      = write_polled,
    .mode       = TERMIOS_POLLED,
};

bool pl011_probe(rtems_termios_device_context *ctx) {
    const pl011_context *dev_ctx = (pl011_context *)ctx;
    const uintptr_t base         = dev_ctx->base_reg;

    const uint32_t scalar = 1e5;
    uint64_t bauddiv =
        ((uint64_t)dev_ctx->clock * scalar) / (dev_ctx->initial_baud * 16);
    uint32_t ibrd = bauddiv / scalar;

    CR &= ~CR_UARTEN;

    /* Wait for pending transactions, then flush the FIFOs */
    while (FR & FR_BUSY)
        ;
    LCRH &= ~LCRH_FEN;

    /* Set the baudrate */
    IBRD = ibrd;
    FBRD = ((bauddiv - (uint64_t)ibrd * scalar) * 64 + scalar / 2) / scalar;

    LCRH |= LCRH_WLEN_8BITS;
    CR |= CR_UARTEN | CR_TXE | CR_RXE;

    return true;
}

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *ctx, struct termios *term,
                       rtems_libio_open_close_args_t *args) {
    pl011_context *dev_ctx = (pl011_context *)ctx;

    if (rtems_termios_set_initial_baud(tty, dev_ctx->initial_baud))
        return true;

    return true;
}

static int read_char_polled(rtems_termios_device_context *ctx) {
    const pl011_context *dev_ctx = (pl011_context *)ctx;
    const uintptr_t base         = dev_ctx->base_reg;

    if (FR & FR_RXFE) return -1;

    return DR & DR_DATA_MASK;
}

void pl011_write_char_polled(rtems_termios_device_context *ctx, char ch) {
    const pl011_context *dev_ctx = (pl011_context *)ctx;
    const uintptr_t base         = dev_ctx->base_reg;

    /* Wait until the FIFO is empty */
    while (FR & FR_TXFF)
        ;

    DR = ch;
}

static void write_polled(rtems_termios_device_context *ctx, const char *buf,
                         size_t n) {
    for (size_t i = 0; i < n; i++) {
        pl011_write_char_polled(ctx, buf[i]);
    }
}
