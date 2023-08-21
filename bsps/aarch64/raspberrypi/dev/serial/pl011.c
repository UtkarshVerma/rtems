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

#include "sys/_termios.h"

#define REG(addr) *(volatile uint32_t *)(addr)

#define DR(base)                   REG(base + 0x00)
#define DR_DATA_MASK               BSP_MSK32(0, 7)
#define FR(base)                   REG(base + 0x18)
#define FR_BUSY                    BSP_BIT32(3)
#define FR_RXFE                    BSP_BIT32(4)
#define FR_TXFF                    BSP_BIT32(5)
#define IBRD(base)                 REG(base + 0x24)
#define FBRD(base)                 REG(base + 0x28)
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

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args);

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void last_close(rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       rtems_libio_open_close_args_t *args);
#endif

static int read_char_polled(rtems_termios_device_context *ctx);

static void write_buffer(rtems_termios_device_context *base, const char *buf,
                         size_t n);

static bool set_attributes(rtems_termios_device_context *base,
                           const struct termios *term);

const rtems_termios_device_handler pl011_handler = {
    .first_open = first_open,
    .write      = write_buffer,
    .poll_read  = read_char_polled,
#ifdef BSP_CONSOLE_USE_INTERRUPTS
    .last_close = last_close,
    .mode       = TERMIOS_IRQ_DRIVEN,
#else
    .last_close = NULL,
    .mode       = TERMIOS_POLLED,
#endif
    .set_attributes = set_attributes,
    .ioctl          = NULL,
};

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void interrupt_handler(void *arg) {
    rtems_termios_tty *tty = arg;
    pl011_context *ctx     = rtems_termios_get_device_context(tty);
    uintptr_t regs_base    = ctx->regs_base;
    uint32_t uartmis       = regs->uartmis;

    versal_uart_intr_clear(regs, uartmis);

    if ((uartmis & (VERSAL_UARTI_RTI | VERSAL_UARTI_RXI)) != 0) {
        char buf[32];
        int c = 0;
        while (c < sizeof(buf) &&
               versal_uart_flags_clear(regs, VERSAL_UARTFR_RXFE)) {
            buf[c++] = (char)VERSAL_UARTDR_DATA_GET(regs->uartdr);
        }
        rtems_termios_enqueue_raw_characters(tty, buf, c);
    }

    if (ctx->transmitting) {
        int sent          = ctx->tx_queued;
        ctx->transmitting = false;
        ctx->tx_queued    = 0;
        versal_uart_intr_disable(regs, VERSAL_UARTI_TXI);
        rtems_termios_dequeue_characters(tty, sent);
    }
}
#endif

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args) {
    const pl011_context *ctx = (void *)base;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    ctx->transmitting = false;
    ctx->tx_queued    = 0;
    ctx->first_send   = true;
#endif

    if (rtems_termios_set_initial_baud(tty, ctx->initial_baud))
        return false;

    if (set_attributes(base, term))
        return false;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    uintptr_t regs_base = ctx->regs_base;

    IFLS(regs_base) &= ~(IFLS_RXIFLSEL_MASK | IFLS_TXIFLSEL_MASK);
    IFLS(regs_base) = IFLS_RXIFLSEL_ONE_HALF | IFLS_TXIFLSEL_ONE_HALF;
    LCRH(regs_base) |= LCRH_FEN;

    versal_uart_intr_disableall(regs);
    rtems_status_code sc = rtems_interrupt_handler_install(
        ctx->irq, "UART", RTEMS_INTERRUPT_SHARED, interrupt_handler, tty);
    if (sc != RTEMS_SUCCESSFUL)
        return false;

    versal_uart_intr_clearall(regs);
    versal_uart_intr_enable(regs, VERSAL_UARTI_RTI | VERSAL_UARTI_RXI);
#endif

    return true;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void last_close(rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       rtems_libio_open_close_args_t *args) {
    pl011_context *ctx = (void *)base;
    rtems_interrupt_handler_remove(ctx->irq, interrupt_handler, tty);
}
#endif

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

static inline void write_char(uintptr_t regs_base, char ch) {
    DR(regs_base) = ch;
}

static void write_char_polled(pl011_context *ctx, char ch) {
    const uintptr_t regs_base = ctx->regs_base;

    /* Wait until TXFIFO is empty */
    while (FR(regs_base) & FR_TXFF)
        ;

    write_char(regs_base, ch);
}

static void write_buffer(rtems_termios_device_context *base, const char *buf,
                         size_t n) {
    pl011_context *ctx = (void *)base;
#ifdef BSP_CONSOLE_USE_INTERRUPTS
    pl011_context *ctx  = (void *)base;
    uintptr_t regs_base = ctx->regs_base;

    if (n > 0) {
        size_t len_remaining = len;
        const char *p        = buf;
        versal_uart_intr_enable(regs, VERSAL_UARTI_TXI);
        /*
         * The PL011 IP in the Versal needs preloading the TX FIFO with
         * exactly 17 characters for the first TX interrupt to be
         * generated.
         */
        if (ctx->first_send) {
            ctx->first_send = false;
            for (int i = 0; i < 17; ++i) {
                regs->uartdr = VERSAL_UARTDR_DATA('\r');
            }
        }

        while (versal_uart_flags_clear(regs, VERSAL_UARTFR_TXFF) &&
               len_remaining > 0) {
            regs->uartdr = VERSAL_UARTDR_DATA(*p++);
            --len_remaining;
        }

        ctx->tx_queued    = len - len_remaining;
        ctx->transmitting = true;
    }
#else
    for (size_t i = 0; i < n; i++)
        write_char_polled(ctx, buf[i]);
#endif
}

static int compute_baud_rate_params(unsigned int baudrate, unsigned int clock,
                                    unsigned int maxerror, uint32_t *ibrd,
                                    uint32_t *fbrd) {
    /* The baud rate cannot be larger than the UART clock / 16 */
    if (baudrate * 16 > clock)
        return 1;

    /*
     * The UART clock cannot be larger than 16*65535*baudrate.
     * Could maybe use an estimate (inputclk / 2**16) to save a division.
     * This invariant gets checked below, by ensuring ibdiv < 2**16.
     */

    /*
     * The UART clock cannot be more than 5/3 times faster than the
     * LPD_LSBUS_CLK
     *  - TODO?
     */

    /*
     * The baud rate divisor is a 16-bit integer and 6-bit fractional part.
     * It is equal to the UART clock / (16 * baudrate).
     */
    *ibrd = clock / 16 / baudrate;
    if (*ibrd > 1 << 16)
        return -1;

    /*
     * Find the fractional part. This can overflow with 32-bit arithmetic
     * if inputclk > 536870911, so use 64-bit. Unfortunately, this means we
     * have two 64-bit divisions here.
     */

    /* Emulate floating-point division using integer arithmetic */
    /* const uint32_t scalar = 1e5; */
    /* 5-digit decimal precision */
    /* uint64_t bauddiv      = ((uint64_t)ctx->clock * scalar) / (baud *
     * 16);
     */
    /* uint32_t ibrd         = bauddiv / scalar; */
    /* uint32_t fbrd = */
    /*     ((bauddiv - (uint64_t)ibrd * scalar) * 64 + scalar / 2) /
     * scalar; */

    uint32_t fbdivrnd;
    fbdivrnd = ((((uint64_t)clock / 16) << 7) / baudrate) & 0x1;
    *fbrd    = (((((uint64_t)clock / 16) << 6) / baudrate) & 0x3F) + fbdivrnd;

    /* Calculate the baudrate and check if the error is too large */
    uint32_t calculated_baudrate =
        (((uint64_t)clock / 16) << 6) / ((*ibrd << 6) | *fbrd);

    uint32_t bauderror = calculated_baudrate - baudrate;
    if (baudrate > calculated_baudrate)
        bauderror = baudrate - calculated_baudrate;

    uint32_t percenterror = (bauderror * 100) / baudrate;
    if (maxerror < percenterror)
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
    compute_baud_rate_params(baud, ctx->clock, 3, &ibrd, &fbrd);

    /* Configure the mode */
    uint32_t lcrh = LCRH(regs_base) & LCRH_FEN; /* Preserve both FIFO states */

    /* Parity */
    if (term->c_cflag & PARENB) {
        lcrh |= LCRH_PEN; /* Enable parity */
        if (!(term->c_cflag & PARODD))
            lcrh |= LCRH_EPS; /* Even parity */
    }

    /* Character size */
    LCRH(regs_base) &= ~LCRH_WLEN_MASK;
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

    /* Stop bits */
    if (term->c_cflag & CSTOPB)
        lcrh |= LCRH_STP2; /* 2 stop bits */

    /* Disable interrupts */
    /* IER_REG(regs_base) &= ~(IER_REG_RXE | IER_REG_TXE); */

    /* IIR_REG(regs_base) |= (IIR_REG_CLEAR_RXFIFO |
       IIR_REG_CLEAR_TXFIFO); */

    /* Disable UART */
    CR(regs_base) &= ~(CR_UARTEN | CR_RXE | CR_TXE);
    uint32_t cr = CR(regs_base);

    /* Wait for pending transactions, then flush the FIFOs */
    while (FR(regs_base) & FR_BUSY)
        ;
    lcrh &= ~LCRH_FEN;

    IBRD(regs_base) = ibrd;
    FBRD(regs_base) = fbrd;
    LCRH(regs_base) = lcrh;

    /* Configure flow control */
    cr &= ~(CR_CTSEN | CR_RTSEN);
    if (term->c_cflag & CCTS_OFLOW)
        cr |= CR_CTSEN; /* Enable CTS */
    if (term->c_cflag & CRTS_IFLOW)
        cr |= CR_RTSEN; /* Enable RTS */

    /* Configure receiver */
    if (term->c_cflag & CREAD) {
        cr |= CR_RXE;
        /* if (ctx->irq_enabled) */
        /*     IER_REG(regs_base) |= IER_REG_RXE; */
    }

    /* Re-enable UART */
    cr |= CR_UARTEN | CR_TXE;
    /* if (ctx->irq_enabled) */
    /*     IER_REG(regs_base) |= IER_REG_TXE; */

    CR(regs_base) = cr;

#ifdef BSPL_CONSOLE_USE_INTERRUPTS
    versal_uart_intr_clearall(regs);
    versal_uart_intr_enable(regs, VERSAL_UARTI_RTI | VERSAL_UARTI_RXI);
#endif

    return true;
}
