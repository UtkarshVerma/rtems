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

#include "dev/serial/mini-uart.h"

#include <bsp/utility.h>
#include <rtems/termiostypes.h>
#include <stdint.h>

#define REG(addr) *(volatile uint32_t*)(addr)

#define IO_REG             REG(base + 0x00)
#define IO_REG_DATA_MASK   0xff
#define IER_REG            REG(base + 0x04)
#define IER_REG_TXE        BSP_BIT32(0)
#define IER_REG_RXE        BSP_BIT32(1)
#define IIR_REG            REG(base + 0x08)
#define LCR_REG            REG(base + 0x0c)
#define LCR_REG_WLEN_8     BSP_BIT32(0)
#define MCR_REG            REG(base + 0x10)
#define MCR_REG_RTS_LOW    BSP_BIT32(1)
#define LSR_REG            REG(base + 0x14)
#define LSR_REG_DATA_READY BSP_BIT32(0)
#define LSR_REG_TXIDLE     BSP_BIT32(6)
#define CNTL_REG           REG(base + 0x20)
#define CNTL_REG_TXE       BSP_BIT32(0)
#define CNTL_REG_RXE       BSP_BIT32(1)
#define BAUD_REG           REG(base + 0x28)

static bool first_open(struct rtems_termios_tty* tty,
                       rtems_termios_device_context* ctx, struct termios* term,
                       rtems_libio_open_close_args_t* args);
static void write_polled(rtems_termios_device_context* ctx, const char* buf,
                         size_t n);
static int read_char_polled(rtems_termios_device_context* ctx);

const rtems_termios_device_handler mini_uart_handler = {
    .first_open = first_open,
    .poll_read  = read_char_polled,
    .write      = write_polled,
    .mode       = TERMIOS_POLLED,
};

bool mini_uart_probe(rtems_termios_device_context* ctx) {
    const mini_uart_context* dev_ctx = (mini_uart_context*)ctx;
    const uintptr_t base             = dev_ctx->base_reg;

    IER_REG &= ~(IER_REG_RXE | IER_REG_TXE);
    CNTL_REG &= ~(CNTL_REG_RXE | CNTL_REG_TXE);

    LCR_REG |= LCR_REG_WLEN_8;   /* Use 8-bit mode */
    MCR_REG &= ~MCR_REG_RTS_LOW; /* Assert RTS HIGH */

    BAUD_REG = dev_ctx->clock / (dev_ctx->initial_baud * 8) - 1;

    CNTL_REG |= CNTL_REG_TXE | CNTL_REG_RXE;

    return true;
}

static bool first_open(struct rtems_termios_tty* tty,
                       rtems_termios_device_context* ctx, struct termios* term,
                       rtems_libio_open_close_args_t* args) {
    const mini_uart_context* dev_ctx = (mini_uart_context*)ctx;

    if (rtems_termios_set_initial_baud(tty, dev_ctx->initial_baud))
        return false;

    return true;
}

static int read_char_polled(rtems_termios_device_context* ctx) {
    const mini_uart_context* dev_ctx = (mini_uart_context*)ctx;
    const uintptr_t base             = dev_ctx->base_reg;

    /* Wait until RXFIFO has some data */
    while (!(LSR_REG & LSR_REG_DATA_READY))
        ;

    return IO_REG & IO_REG_DATA_MASK;
}

void mini_uart_write_char_polled(rtems_termios_device_context* ctx, char ch) {
    const mini_uart_context* dev_ctx = (mini_uart_context*)ctx;
    const uintptr_t base             = dev_ctx->base_reg;

    /* Wait until TXFIFO is empty */
    while (!(LSR_REG & LSR_REG_TXIDLE))
        ;

    IO_REG = ch;
}

static void write_polled(rtems_termios_device_context* ctx, const char* buf,
                         size_t n) {
    for (size_t i = 0; i < n; i++) {
        mini_uart_write_char_polled(ctx, buf[i]);
    }
}
