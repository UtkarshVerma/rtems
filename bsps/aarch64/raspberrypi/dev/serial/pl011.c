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
#include <bspopts.h>
#include <rtems/libio.h>
#include <rtems/termiosdevice.h>
#include <rtems/termiostypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/_termios.h>

#define REG(addr) *(volatile uint32_t *)(addr)

#define DR(base)                   REG(base + 0x00)
#define DR_DATA_MASK               BSP_MSK32(0, 7)
#define FR(base)                   REG(base + 0x18)
#define FR_BUSY                    BSP_BIT32(3)
#define FR_RXFE                    BSP_BIT32(4)
#define FR_TXFF                    BSP_BIT32(5)
#define FR_TXFE                    BSP_BIT32(7)
#define IBRD(base)                 REG(base + 0x24)
#define IBRD_BAUD_DIVINT_WIDTH     16
#define IBRD_BAUD_DIVINT_MASK      BSP_MSK32(0, IBRD_BAUD_DIVINT_WIDTH - 1)
#define FBRD(base)                 REG(base + 0x28)
#define FBRD_BAUD_DIVFRAC_WIDTH    6
#define FBRD_BAUD_DIVFRAC_MASK     BSP_MSK32(0, FBRD_BAUD_DIVFRAC_WIDTH - 1)
#define LCRH(base)                 REG(base + 0x2c)
#define LCRH_PEN                   BSP_BIT32(1)
#define LCRH_EPS                   BSP_BIT32(2)
#define LCRH_STP2                  BSP_BIT32(3)
#define LCRH_FEN                   BSP_BIT32(4)
#define LCRH_WLEN_MASK             BSP_MSK32(5, 6)
#define LCRH_WLEN_5BITS            BSP_FLD32(0, 5, 6)
#define LCRH_WLEN_6BITS            BSP_FLD32(1, 5, 6)
#define LCRH_WLEN_7BITS            BSP_FLD32(2, 5, 6)
#define LCRH_WLEN_8BITS            BSP_FLD32(3, 5, 6)
#define CR(base)                   REG(base + 0x30)
#define CR_UARTEN                  BSP_BIT32(0)
#define CR_TXE                     BSP_BIT32(8)
#define CR_RXE                     BSP_BIT32(9)
#define CR_RTSEN                   BSP_BIT32(14)
#define CR_CTSEN                   BSP_BIT32(15)
#define IFLS(base)                 REG(base + 0x34)
#define IFLS_TXIFLSEL_MASK         BSP_MSK32(0, 2)
#define IFLS_TXIFLSEL_ONE_EIGHTH   BSP_FLD32(0, 0, 2)
#define IFLS_TXIFLSEL_ONE_FOURTH   BSP_FLD32(1, 0, 2)
#define IFLS_TXIFLSEL_ONE_HALF     BSP_FLD32(2, 0, 2)
#define IFLS_TXIFLSEL_THREE_FOURTH BSP_FLD32(3, 0, 2)
#define IFLS_TXIFLSEL_SEVEN_EIGHTH BSP_FLD32(4, 0, 2)
#define IFLS_RXIFLSEL_MASK         BSP_MSK32(3, 5)
#define IFLS_RXIFLSEL_ONE_EIGHTH   BSP_FLD32(0, 3, 5)
#define IFLS_RXIFLSEL_ONE_FOURTH   BSP_FLD32(1, 3, 5)
#define IFLS_RXIFLSEL_ONE_HALF     BSP_FLD32(2, 3, 5)
#define IFLS_RXIFLSEL_THREE_FOURTH BSP_FLD32(3, 3, 5)
#define IFLS_RXIFLSEL_SEVEN_EIGHTH BSP_FLD32(4, 3, 5)
#define IMSC(base)                 REG(base + 0x38)
#define MIS(base)                  REG(base + 0x40)
#define ICR(base)                  REG(base + 0x44)

/* Applies to IMSC, ICR, and MIS */
#define IRQ_RX_BIT BSP_BIT32(4)
#define IRQ_RT_BIT BSP_BIT32(6)
#define IRQ_TX_BIT BSP_BIT32(5)
#define IRQ_FE_BIT BSP_BIT32(7)
#define IRQ_PE_BIT BSP_BIT32(8)
#define IRQ_BE_BIT BSP_BIT32(9)
#define IRQ_OE_BIT BSP_BIT32(10)
#define IRQ_MASK   BSP_MSK32(0, 10)

#define TX_IRQ BSP_BIT32(5)

#define FIFO_SIZE 32

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args);

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void last_close(rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       rtems_libio_open_close_args_t *args);
#else
static int read_char_polled(rtems_termios_device_context *ctx);
#endif

static void write_buffer(rtems_termios_device_context *base, const char *buf,
                         size_t n);

static bool set_attributes(rtems_termios_device_context *base,
                           const struct termios *term);

const rtems_termios_device_handler pl011_handler = {
    .first_open     = first_open,
    .write          = write_buffer,
    .set_attributes = set_attributes,
    .ioctl          = NULL,
#ifdef BSP_CONSOLE_USE_INTERRUPTS
    .last_close = last_close,
    .poll_read  = NULL,
    .mode       = TERMIOS_IRQ_DRIVEN,
#else
    .last_close = NULL,
    .poll_read  = read_char_polled,
    .mode       = TERMIOS_POLLED,
#endif
};

static inline char read_char(uintptr_t regs_base) {
    return DR(regs_base) & DR_DATA_MASK;
}

static inline void write_char(uintptr_t regs_base, char ch) {
    DR(regs_base) = ch;
}

static inline bool is_rxfifo_empty(uintptr_t regs_base) {
    return FR(regs_base) & FR_RXFE;
}

static inline bool is_txfifo_full(uintptr_t regs_base) {
    return FR(regs_base) & FR_TXFF;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static inline void clear_irq(uintptr_t regs_base, uint32_t irq_mask) {
    ICR(regs_base) |= irq_mask;
}

static void enable_irq(uintptr_t regs_base, uint32_t irq_mask) {
    IMSC(regs_base) |= irq_mask;
}
#endif

static inline void disable_irq(uintptr_t regs_base, uint32_t irq_mask) {
    IMSC(regs_base) &= ~irq_mask;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void irq_handler(void *arg) {
    rtems_termios_tty *tty = arg;
    pl011_context *ctx     = rtems_termios_get_device_context(tty);
    uintptr_t regs_base    = ctx->regs_base;

    uint32_t mis = MIS(regs_base);

    /* Clear all raised interrupts */
    clear_irq(regs_base, mis);

    /* RXFIFO got data */
    if (mis & (IRQ_RT_BIT | IRQ_RX_BIT)) {
        char buf[FIFO_SIZE];

        unsigned int i = 0;
        while (i < sizeof(buf) && !is_rxfifo_empty(regs_base))
            buf[i++] = read_char(regs_base);

        rtems_termios_enqueue_raw_characters(tty, buf, i);
    }

    /* Transmission was queued last time */
    if (ctx->transmitting) {
        /* Mark it as done */
        ctx->transmitting = false;
        int sent          = ctx->tx_queued;
        ctx->tx_queued    = 0;

        disable_irq(regs_base, IRQ_TX_BIT);
        rtems_termios_dequeue_characters(tty, sent);
    }
}
#endif

static void write_char_polled(rtems_termios_device_context *base, char ch) {
    const pl011_context *ctx = (void *)base;
    uintptr_t regs_base      = ctx->regs_base;

    /* Wait until TXFIFO has space */
    while (is_txfifo_full(regs_base))
        ;

    write_char(regs_base, ch);
}

static void flush_txfifo(rtems_termios_device_context *base) {
    const pl011_context *ctx = (void *)base;
    uintptr_t regs_base      = ctx->regs_base;

    int c = 4;
    while (c-- > 0)
        write_char_polled(base, '\r');

    while (!(FR(regs_base) & FR_TXFE)) {
        /* Wait for empty */
    }

    while ((FR(regs_base) & FR_BUSY) != 0) {
        /* Wait for empty */
    }
}

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args) {
    pl011_context *ctx = (void *)base;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    ctx->transmitting = false;
    ctx->tx_queued    = 0;
    ctx->first_send   = true;
#endif

    if (rtems_termios_set_initial_baud(tty, ctx->initial_baud))
        return false;

    flush_txfifo(base);

    if (!set_attributes(base, term))
        return false;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    uintptr_t regs_base = ctx->regs_base;

    IFLS(regs_base) &= ~(IFLS_RXIFLSEL_MASK | IFLS_TXIFLSEL_MASK);
    IFLS(regs_base) |= IFLS_RXIFLSEL_ONE_HALF | IFLS_TXIFLSEL_ONE_HALF;
    LCRH(regs_base) |= LCRH_FEN;

    /* Disable all interrupts */
    disable_irq(regs_base, IRQ_MASK);

    rtems_status_code sc = rtems_interrupt_handler_install(
        ctx->irq, "UART", RTEMS_INTERRUPT_SHARED, irq_handler, tty);
    if (sc != RTEMS_SUCCESSFUL)
        return false;

    clear_irq(regs_base, IRQ_MASK);
    enable_irq(regs_base, IRQ_RT_BIT | IRQ_RX_BIT);
#endif

    return true;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void last_close(rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       rtems_libio_open_close_args_t *args) {
    const pl011_context *ctx = (void *)base;
    rtems_interrupt_handler_remove(ctx->irq, irq_handler, tty);
}
#else
static int read_char_polled(rtems_termios_device_context *base) {
    const pl011_context *ctx  = (void *)base;
    const uintptr_t regs_base = ctx->regs_base;

    if (is_rxfifo_empty(regs_base))
        return -1;

    return read_char(regs_base);
}
#endif

static void write_buffer(rtems_termios_device_context *base, const char *buf,
                         size_t n) {
    pl011_context *ctx  = (void *)base;
    uintptr_t regs_base = ctx->regs_base;
#ifdef BSP_CONSOLE_USE_INTERRUPTS
    if (n > 0) {
        const char *p = buf;
        enable_irq(regs_base, IRQ_TX_BIT);

        /*
         * The PL011 IP in the Versal needs preloading the TX FIFO with
         * exactly 17 characters for the first TX interrupt to be
         * generated.
         */
        if (ctx->first_send) {
            ctx->first_send = false;
            for (int i = 0; i < 17; ++i)
                write_char(regs_base, '\r');
        }

        size_t remaining = n;
        while (!is_txfifo_full(regs_base) && remaining > 0) {
            write_char(regs_base, *p);
            p++;
            remaining--;
        }

        ctx->tx_queued    = n - remaining;
        ctx->transmitting = true;
    }

    // TODO: disable irq for n == 0?
#else
    for (size_t i = 0; i < n; i++) {
        /* Wait until TXFIFO is empty */
        while (is_txfifo_full(regs_base))
            ;

        write_char(regs_base, buf[i]);
    }
#endif
}

static int compute_baudrate_params(uint32_t *ibrd, uint32_t *fbrd,
                                   uint32_t baudrate, uint32_t clock,
                                   unsigned short max_error) {
    /*
     * The integer baudrate divisor, i.e. (clock / (baudrate * 16)), value
     * should lie on [1, 2^16 - 1]. To ensure this, clock and baudrate have to
     * be validated.
     */
    *ibrd = clock / 16 / baudrate;
    if (*ibrd < 1 || *ibrd > IBRD_BAUD_DIVINT_MASK)
        return 2;

    /* Find the fractional part */
    const uint16_t scalar    = 1 << (FBRD_BAUD_DIVFRAC_WIDTH + 1);
    uint64_t scaled_bauddiv  = ((uint64_t)clock * scalar) / 16 / baudrate;
    unsigned short round_off = scaled_bauddiv & 0x1;
    *fbrd = ((scaled_bauddiv >> 1) & FBRD_BAUD_DIVFRAC_MASK) + round_off;

    /* Calculate the baudrate and check if the error is too large */
    uint32_t computed_bauddiv = (*ibrd << FBRD_BAUD_DIVFRAC_WIDTH) | *fbrd;
    uint32_t computed_baudrate =
        ((uint64_t)clock << FBRD_BAUD_DIVFRAC_WIDTH) / 16 / computed_bauddiv;

    uint32_t baud_error = computed_baudrate - baudrate;
    if (baudrate > computed_baudrate)
        baud_error = baudrate - computed_baudrate;

    unsigned short percent_error = (baud_error * 100) / baudrate;
    if (percent_error >= max_error)
        return 1;

    return 0;
}

static bool set_attributes(rtems_termios_device_context *base,
                           const struct termios *term) {
    const pl011_context *ctx  = (void *)base;
    const uintptr_t regs_base = ctx->regs_base;

    /* Determine baudrate parameters */
    uint32_t baud = rtems_termios_number_to_baud(term->c_ospeed);
    if (baud == B0)
        return false;

    uint32_t ibrd, fbrd;
    if (compute_baudrate_params(&ibrd, &fbrd, baud, ctx->clock, 3))
        return false;

    /* Start mode configuration from a clean slate */
    uint32_t lcrh = LCRH(regs_base) & LCRH_FEN;

    /* Mode: parity */
    if (term->c_cflag & PARENB) {
        lcrh |= LCRH_PEN;

        if (!(term->c_cflag & PARODD))
            lcrh |= LCRH_EPS;
    }

    /* Mode: character size */
    switch (term->c_cflag & CSIZE) {
        case CS5:
            lcrh |= LCRH_WLEN_5BITS;
            break;
        case CS6:
            lcrh |= LCRH_WLEN_6BITS;
            break;
        case CS7:
            lcrh |= LCRH_WLEN_7BITS;
            break;
        case CS8:
        default:
            lcrh |= LCRH_WLEN_8BITS;
    }

    /* Mode: stop bits */
    if (term->c_cflag & CSTOPB)
        lcrh |= LCRH_STP2;

    /* Disable all interrupts */
    disable_irq(regs_base, IRQ_MASK);

    /* Disable UART */
    CR(regs_base) &= ~(CR_UARTEN | CR_RXE | CR_TXE);
    uint32_t cr = CR(regs_base);

    /* Wait for pending transactions, then flush the FIFOs */
    while (!(FR(regs_base) & FR_TXFE) || (FR(regs_base) & FR_BUSY))
        ;
    lcrh &= ~LCRH_FEN;

    /* Set the baudrate */
    IBRD(regs_base) = ibrd;
    FBRD(regs_base) = fbrd;
    LCRH(regs_base) = lcrh;

    /* Configure flow control */
    cr &= ~(CR_CTSEN | CR_RTSEN);
    if (term->c_cflag & CCTS_OFLOW)
        cr |= CR_CTSEN;
    if (term->c_cflag & CRTS_IFLOW)
        cr |= CR_RTSEN;

    /* Configure receiver */
    if (term->c_cflag & CREAD)
        cr |= CR_RXE;

    /* Re-enable UART */
    cr |= CR_UARTEN | CR_TXE;
    CR(regs_base) = cr;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    // TODO: respect CREAD

    /* Clear all interrupts  */
    clear_irq(regs_base, IRQ_MASK);

    /* Re-enable RX interrupts */
    enable_irq(regs_base, IRQ_RT_BIT | IRQ_RX_BIT);
#endif

    return true;
}
