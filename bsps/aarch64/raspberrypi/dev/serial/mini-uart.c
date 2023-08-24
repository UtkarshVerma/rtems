/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Mini UART Device Driver
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
#include <bspopts.h>
#include <rtems/libio.h>
#include <rtems/termiosdevice.h>
#include <rtems/termiostypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/_termios.h>

#define REG(addr) *(volatile uint32_t *)(addr)

#define IO_REG(base)              REG(base + 0x00)
#define IO_REG_DATA_MASK          0xff
#define IER_REG(base)             REG(base + 0x04)
#define IER_REG_TXE               BSP_BIT32(0)
#define IER_REG_RXE               BSP_BIT32(1)
#define IER_REG_IRQ_MASK          BSP_MSK32(0, 1)
#define IIR_REG(base)             REG(base + 0x08)
#define IIR_REG_IRQ_NOT_PENDING   BSP_BIT32(0)
#define IIR_REG_TXFIFO_EMPTY      BSP_BIT32(1)
#define IIR_REG_RXFIFO_HAS_DATA   BSP_BIT32(2)
#define IIR_REG_CLEAR_RXFIFO      BSP_BIT32(1)
#define IIR_REG_CLEAR_TXFIFO      BSP_BIT32(2)
#define LCR_REG(base)             REG(base + 0x0c)
#define LCR_REG_WLEN_8            BSP_BIT32(0)
#define MCR_REG(base)             REG(base + 0x10)
#define MCR_REG_RTS_LOW           BSP_BIT32(1)
#define LSR_REG(base)             REG(base + 0x14)
#define LSR_REG_DATA_READY        BSP_BIT32(0)
#define LSR_REG_TXIDLE            BSP_BIT32(6)
#define CNTL_REG(base)            REG(base + 0x20)
#define CNTL_REG_TXE              BSP_BIT32(0)
#define CNTL_REG_RXE              BSP_BIT32(1)
#define STAT_REG(base)            REG(base + 0x24)
#define STAT_REG_RXFIFO_NOT_EMPTY BSP_BIT32(0)
#define STAT_REG_TXFIFO_NOT_FULL  BSP_BIT32(1)
#define BAUD_REG(base)            REG(base + 0x28)

#define FIFO_SIZE 8

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

const rtems_termios_device_handler mini_uart_handler = {
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
    return IO_REG(regs_base) & IO_REG_DATA_MASK;
}

static inline void write_char(uintptr_t regs_base, char ch) {
    IO_REG(regs_base) = ch;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static inline void enable_irq(uintptr_t regs_base, uint32_t irq_mask) {
    IER_REG(regs_base) |= irq_mask;
}
#endif

static inline void disable_irq(uintptr_t regs_base, uint32_t irq_mask) {
    IER_REG(regs_base) &= ~irq_mask;
}

static inline bool is_rxfifo_empty(uintptr_t regs_base) {
    return (STAT_REG(regs_base) & STAT_REG_RXFIFO_NOT_EMPTY) != 0;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS

static inline bool is_txfifo_full(uintptr_t regs_base) {
    return (STAT_REG(regs_base) & STAT_REG_TXFIFO_NOT_FULL) != 0;
}

static void irq_handler(void *arg) {
    rtems_termios_tty *tty = arg;
    mini_uart_context *ctx = rtems_termios_get_device_context(tty);
    uintptr_t regs_base    = ctx->regs_base;

    uint32_t irqs = IIR_REG(regs_base);

    /* RXFIFO got data */
    if ((irqs & IIR_REG_RXFIFO_HAS_DATA) != 0) {
        char buf[FIFO_SIZE];

        unsigned int i = 0;
        while (i < sizeof(buf) && !is_rxfifo_empty(regs_base))
            buf[i++] = read_char(regs_base);

        (void)rtems_termios_enqueue_raw_characters(tty, buf, i);
    }

    /*
     * Transmission was queued last time and TXFIFO is now empty, so mark the
     * transaction as done.
     */
    if ((irqs & IIR_REG_TXFIFO_EMPTY) != 0 && ctx->transmitting == true) {
        ctx->transmitting = false;
        int sent          = ctx->tx_queued;
        ctx->tx_queued    = 0;

        (void)rtems_termios_dequeue_characters(tty, sent);
    }
}
#endif

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args) {
    mini_uart_context *ctx = (void *)base;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    ctx->transmitting = false;
    ctx->tx_queued    = 0;
#endif

    if (rtems_termios_set_initial_baud(tty, ctx->initial_baud) != 0)
        return false;

    if (set_attributes(base, term) == false)
        return false;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    rtems_status_code status = rtems_interrupt_handler_install(
        ctx->irq, "UART", RTEMS_INTERRUPT_SHARED, irq_handler, tty);
    if (status != RTEMS_SUCCESSFUL)
        return false;
#endif

    return true;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void last_close(rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       rtems_libio_open_close_args_t *args) {
    const mini_uart_context *ctx = (void *)base;
    (void)rtems_interrupt_handler_remove(ctx->irq, irq_handler, tty);
}
#else
static int read_char_polled(rtems_termios_device_context *base) {
    const mini_uart_context *ctx = (void *)base;
    const uintptr_t regs_base    = ctx->regs_base;

    /* Data is available to be read */
    if (is_rxfifo_empty(regs_base) == false)
        return read_char(regs_base);

    /* There is no data to be read */
    return -1;
}
#endif

static void write_buffer(rtems_termios_device_context *base, const char *buf,
                         size_t n) {
#ifdef BSP_CONSOLE_USE_INTERRUPTS
    mini_uart_context *ctx = (void *)base;
    uintptr_t regs_base    = ctx->regs_base;

    if (n > 0) {
        const char *p = buf;
        enable_irq(regs_base, IER_REG_TXE);

        size_t remaining = n;
        while ((is_txfifo_full(regs_base) == false) && remaining > 0) {
            write_char(regs_base, *p);
            p++;
            remaining--;
        }

        ctx->tx_queued    = n - remaining;
        ctx->transmitting = true;

        return;
    }

    /*
     * Termios will set n to zero to indicate that the transmitter is
     * now inactive.  The output buffer is empty in this case.  The
     * driver may disable the transmit interrupts now.
     */
    disable_irq(regs_base, IER_REG_RXE);

#else
    const mini_uart_context *ctx = (void *)base;
    const uintptr_t regs_base    = ctx->regs_base;

    for (size_t i = 0; i < n; i++) {
        /* Wait until the transmitter is idle */
        while ((LSR_REG(regs_base) & LSR_REG_TXIDLE) == 0)
            ;

        write_char(regs_base, buf[i]);
    }
#endif
}

static bool set_attributes(rtems_termios_device_context *base,
                           const struct termios *term) {
    mini_uart_context *ctx    = (void *)base;
    const uintptr_t regs_base = ctx->regs_base;

    /* Disable interrupts */
    disable_irq(regs_base, IER_REG_IRQ_MASK);

    /* Disable data transfers */
    CNTL_REG(regs_base) &= ~(CNTL_REG_RXE | CNTL_REG_TXE);
    uint32_t cntl = CNTL_REG(regs_base);

    /* Clear both FIFOs */
    IIR_REG(regs_base) = (IIR_REG_CLEAR_RXFIFO | IIR_REG_CLEAR_TXFIFO);

    /*
     * Set the character size
     * Mini UART only supports 7-bit and 8-bit sizes
     */
    switch (term->c_cflag & CSIZE) {
        case CS7:
            LCR_REG(regs_base) &= ~LCR_REG_WLEN_8;
            break;
        case CS8:
        default:
            LCR_REG(regs_base) |= LCR_REG_WLEN_8;
    }

    /*
     * TODO:
     * Add hardware-flow control.
     * For now, this just asserts RTS HIGH all the time.
     */
    MCR_REG(regs_base) &= ~MCR_REG_RTS_LOW;

    /* Set the baudrate */
    speed_t baud = rtems_termios_number_to_baud(term->c_ospeed);
    if (baud == B0)
        return false;
    BAUD_REG(regs_base) = ctx->clock / 8 / baud - 1;

    /* Re-enable transmission */
    cntl |= CNTL_REG_TXE;
#ifdef BSP_CONSOLE_USE_INTERRUPTS
    uint32_t ier = IER_REG_TXE;
#endif

    /* Configure receiver */
    if ((term->c_cflag & CREAD) != 0) {
        cntl |= CNTL_REG_RXE;
#ifdef BSP_CONSOLE_USE_INTERRUPTS
        ier |= IER_REG_RXE;
#endif
    }

    /* Commit changes to the peripheral */
    CNTL_REG(regs_base) = cntl;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    IER_REG(regs_base) |= ier;
#endif

    return true;
}
