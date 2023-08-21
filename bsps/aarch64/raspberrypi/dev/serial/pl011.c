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
#include <rtems/libio.h>
#include <rtems/termiosdevice.h>
#include <rtems/termiostypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "sys/_termios.h"

#define REG(addr) *(volatile uint32_t *)(addr)

#define DR(base)        REG(base + 0x00)
#define DR_DATA_MASK    BSP_MSK32(0, 7)
#define FR(base)        REG(base + 0x18)
#define FR_BUSY         BSP_BIT32(3)
#define FR_RXFE         BSP_BIT32(4)
#define FR_TXFF         BSP_BIT32(5)
#define IBRD(base)      REG(base + 0x24)
#define FBRD(base)      REG(base + 0x28)
#define LCRH(base)      REG(base + 0x2c)
#define LCRH_FEN        BSP_BIT32(4)
#define LCRH_WLEN_MASK  BSP_MSK32(5, 6)
#define LCRH_WLEN_5BITS BSP_FLD32(0, 5, 6)
#define LCRH_WLEN_6BITS BSP_FLD32(1, 5, 6)
#define LCRH_WLEN_7BITS BSP_FLD32(2, 5, 6)
#define LCRH_WLEN_8BITS BSP_FLD32(3, 5, 6)
#define CR(base)        REG(base + 0x30)
#define CR_UARTEN       BSP_BIT32(0)
#define CR_TXE          BSP_BIT32(8)
#define CR_RXE          BSP_BIT32(9)
#define CR_RTSEN        BSP_BIT32(14)
#define CR_CTSEN        BSP_BIT32(15)
/* #define ICR(base)       REG(base + 0x44) */

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args);

static int read_char_polled(rtems_termios_device_context *ctx);

static void write_polled(rtems_termios_device_context *base, const char *buf,
                         size_t n);

static void write_irq_driven(rtems_termios_device_context *base,
                             const char *buf, size_t n);

static bool set_attributes(rtems_termios_device_context *base,
                           const struct termios *term);

const rtems_termios_device_handler pl011_polled_handler = {
    .first_open     = first_open,
    .last_close     = NULL,
    .poll_read      = read_char_polled,
    .write          = write_polled,
    .set_attributes = set_attributes,
    .ioctl          = NULL,
    .mode           = TERMIOS_POLLED,
};

const rtems_termios_device_handler pl011_irq_driven_handler = {
    .first_open     = first_open,
    .last_close     = NULL,
    .poll_read      = NULL,
    .write          = write_irq_driven,
    .set_attributes = set_attributes,
    .ioctl          = NULL,
    .mode           = TERMIOS_IRQ_DRIVEN,
};

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args) {
    const pl011_context *ctx = (void *)base;

    if (rtems_termios_set_initial_baud(tty, ctx->initial_baud))
        return false;

    return set_attributes(base, term);
}

static inline int read_char(uintptr_t regs_base) {
    return DR(regs_base) & DR_DATA_MASK;
}

static int read_char_polled(rtems_termios_device_context *base) {
    const pl011_context *ctx  = (void *)base;
    const uintptr_t regs_base = ctx->regs_base;

    /* RXFIFO is empty */
    if (FR(regs_base) & FR_RXFE)
        return -1;

    return read_char(regs_base);
}

static size_t read_irq_driven(pl011_context *ctx, char *buffer, size_t size) {
    const uintptr_t regs_base = ctx->regs_base;

    unsigned int i = 0;
    while (i < size) {
        /* const uint32_t iir = IIR_REG(regs_base); */

        /* No interrupt has been raised */
        /* if (iir & IIR_REG_IRQ_NOT_PENDING) */
        /* break; */

        /* Data has been received */
        /* if (iir & IIR_REG_RXFIFO_HAS_DATA) { */
        /*     buffer[i] = read_char(regs_base); */
        /*     i++; */
        /* } */
    }

    return i;
}

static inline void write_char(uintptr_t regs_base, char ch) {
    DR(regs_base) = ch;
}

static void write_char_polled(rtems_termios_device_context *base, char ch) {
    const pl011_context *ctx  = (void *)base;
    const uintptr_t regs_base = ctx->regs_base;

    /* TODO: Wait until TXFIFO is empty */
    while (FR(regs_base) & FR_TXFF)
        ;

    write_char(regs_base, ch);
}

static void write_polled(rtems_termios_device_context *ctx, const char *buf,
                         size_t n) {
    for (size_t i = 0; i < n; i++)
        write_char_polled(ctx, buf[i]);
}

static void write_irq_driven(rtems_termios_device_context *base,
                             const char *buffer, size_t n) {
    pl011_context *ctx  = (void *)base;
    uintptr_t regs_base = ctx->regs_base;

    if (n == 0) {
        /*
         * Zero value indicates that the transmitter is now inactive. The
         * output buffer is empty in this case, so interrupts can be disabled.
         */
        /* IER_REG(regs_base) &= ~IER_REG_TXE; */

        return;
    }

    /*
     * Tell the device to transmit some characters from buf (less than
     * or equal to n).  When the device is finished it should raise an
     * interrupt.  The interrupt handler will notify Termios that these
     * characters have been transmitted and this may trigger this write
     * function again.  You may have to store the number of outstanding
     * characters in the device data structure.
     */
    for (unsigned int i = 0; i < 1; i++)
        write_char(regs_base, buffer[i]);
}

static bool set_attributes(rtems_termios_device_context *base,
                           const struct termios *term) {
    const pl011_context *ctx  = (void *)base;
    const uintptr_t regs_base = ctx->regs_base;

    /* Disable interrupts */
    /* IER_REG(regs_base) &= ~(IER_REG_RXE | IER_REG_TXE); */

    /* Disable data transfers */
    CR(regs_base) &= ~(CR_UARTEN | CR_RXE | CR_TXE);

    /* Wait for pending transactions, then flush the FIFOs */
    while (FR(regs_base) & FR_BUSY)
        ;
    LCRH(regs_base) &= ~LCRH_FEN;

    /* Configure flow control */
    CR(regs_base) &= ~(CR_CTSEN | CR_RTSEN);
    if (term->c_cflag & CCTS_OFLOW)
        CR(regs_base) |= CR_CTSEN;
    if (term->c_cflag & CRTS_IFLOW)
        CR(regs_base) |= CR_RTSEN;

    /* IIR_REG(regs_base) |= (IIR_REG_CLEAR_RXFIFO |
       IIR_REG_CLEAR_TXFIFO); */

    /* Set the baudrate */
    speed_t baud = rtems_termios_number_to_baud(term->c_ispeed);
    if (baud == B0)
        return false;

    /* Emulate floating-point division using integer arithmetic */
    const uint32_t scalar = 1e5; /* 5-digit decimal precision */
    uint64_t bauddiv      = ((uint64_t)ctx->clock * scalar) / (baud * 16);
    uint32_t ibrd         = bauddiv / scalar;

    IBRD(regs_base) = ibrd;
    FBRD(regs_base) =
        ((bauddiv - (uint64_t)ibrd * scalar) * 64 + scalar / 2) / scalar;

    /* Set the character size */
    LCRH(regs_base) &= ~LCRH_WLEN_MASK;
    switch (term->c_cflag & CSIZE) {
        case CS5:
            LCRH(regs_base) |= LCRH_WLEN_5BITS;
            break;
        case CS6:
            LCRH(regs_base) |= LCRH_WLEN_6BITS;
            break;
        case CS7:
            LCRH(regs_base) |= LCRH_WLEN_7BITS;
            break;
        case CS8:
            LCRH(regs_base) |= LCRH_WLEN_8BITS;
            break;
        default:
            return false;
    }

    /* Configure receiver */
    if (term->c_cflag & CREAD) {
        CR(regs_base) |= CR_RXE;
        /* if (ctx->irq_enabled) */
        /*     IER_REG(regs_base) |= IER_REG_RXE; */
    }

    /* Re-enable transmission */
    CR(regs_base) |= CR_UARTEN | CR_TXE;
    /* if (ctx->irq_enabled) */
    /*     IER_REG(regs_base) |= IER_REG_TXE; */

    return true;
}
