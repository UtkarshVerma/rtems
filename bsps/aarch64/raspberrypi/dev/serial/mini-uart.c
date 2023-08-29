/* SPDX-License-Identifier: BSD-2-Clause */

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
#include <stddef.h>
#include <stdint.h>
#include <sys/_termios.h>

#define REG(addr) *(volatile uint32_t *)(addr)

/*
 * NOTE:
 * The datasheet will specify swapped bits for RX and TX interrupts in IER_REG,
 * which is incorrect. The correct values are used here.
 */
#define IO_REG(base)              REG(base + 0x00)
#define IO_REG_DATA_MASK          BSP_MSK32(0, 7)
#define IER_REG(base)             REG(base + 0x04)
#define IER_REG_RXE               BSP_BIT32(0)
#define IER_REG_TXE               BSP_BIT32(1)
#define IER_REG_IRQ_MASK          BSP_MSK32(0, 1)
#define IIR_REG(base)             REG(base + 0x08)
#define IIR_REG_IRQ_NOT_PENDING   BSP_BIT32(0)
#define IIR_REG_TXFIFO_EMPTY      BSP_BIT32(1)
#define IIR_REG_CLEAR_RXFIFO      BSP_BIT32(1)
#define IIR_REG_RXFIFO_GOT_DATA   BSP_BIT32(2)
#define IIR_REG_CLEAR_TXFIFO      BSP_BIT32(2)
#define LCR_REG(base)             REG(base + 0x0c)
#define LCR_REG_WLEN_8            BSP_BIT32(0)
#define MCR_REG(base)             REG(base + 0x10)
#define MCR_REG_RTS_LOW           BSP_BIT32(1)
#define CNTL_REG(base)            REG(base + 0x20)
#define CNTL_REG_TXE              BSP_BIT32(0)
#define CNTL_REG_RXE              BSP_BIT32(1)
#define STAT_REG(base)            REG(base + 0x24)
#define STAT_REG_RXFIFO_NOT_EMPTY BSP_BIT32(0)
#define STAT_REG_TXFIFO_HAS_SPACE BSP_BIT32(1)
#define STAT_REG_TXFIFO_FULL      BSP_BIT32(5)
#define STAT_REG_TX_IDLE          BSP_BIT32(3)
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
static int read_char_polled(rtems_termios_device_context *base);
#endif

static void write_buffer(rtems_termios_device_context *base,
                         const char *buffer, size_t n);

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

static inline char read_char(const uintptr_t regs_base) {
    return IO_REG(regs_base) & IO_REG_DATA_MASK;
}

static inline void write_char(const uintptr_t regs_base, const char ch) {
    IO_REG(regs_base) = ch;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static inline void enable_irq(const uintptr_t regs_base,
                              const uint32_t irq_mask) {
    IER_REG(regs_base) |= irq_mask;
}
#endif

static inline void disable_irq(const uintptr_t regs_base,
                               const uint32_t irq_mask) {
    IER_REG(regs_base) &= ~irq_mask;
}

static inline bool is_rxfifo_empty(const uintptr_t regs_base) {
    return (STAT_REG(regs_base) & STAT_REG_RXFIFO_NOT_EMPTY) != 0;
}

static inline bool txfifo_has_space(const uintptr_t regs_base) {
    return (STAT_REG(regs_base) & STAT_REG_TXFIFO_HAS_SPACE) != 0;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void irq_handler(void *arg) {
    rtems_termios_tty *tty     = arg;
    mini_uart_context *context = rtems_termios_get_device_context(tty);
    const uintptr_t regs_base  = context->regs_base;
    const uint32_t irqs        = IIR_REG(regs_base);

    /* RXFIFO got data */
    if ((irqs & IIR_REG_RXFIFO_GOT_DATA) != 0) {
        char buffer[FIFO_SIZE];

        unsigned int i = 0;
        while (i < sizeof(buffer) && !is_rxfifo_empty(regs_base))
            buffer[i++] = read_char(regs_base);

        (void)rtems_termios_enqueue_raw_characters(tty, buffer, i);
    }

    /*
     * Transmission was queued last time and TXFIFO is now empty, so tell
     * Termios to dequeue the sent bytes.
     */
    if ((irqs & IIR_REG_TXFIFO_EMPTY) != 0)
        (void)rtems_termios_dequeue_characters(tty, context->tx_queued_chars);
}
#endif

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args) {
#ifndef BSP_CONSOLE_USE_INTERRUPTS
    const
#endif
        mini_uart_context *context = (void *)base;

    if (rtems_termios_set_initial_baud(tty, context->initial_baud) != 0)
        return false;

    if (!set_attributes(base, term))
        return false;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    const rtems_status_code status = rtems_interrupt_handler_install(
        context->irq, "UART", RTEMS_INTERRUPT_SHARED, irq_handler, tty);
    if (status != RTEMS_SUCCESSFUL)
        return false;
#endif

    return true;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void last_close(rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       rtems_libio_open_close_args_t *args) {
    const mini_uart_context *context = (void *)base;
    (void)rtems_interrupt_handler_remove(context->irq, irq_handler, tty);
}
#else
static int read_char_polled(rtems_termios_device_context *base) {
    const mini_uart_context *context = (void *)base;
    const uintptr_t regs_base        = context->regs_base;

    /* Data is available to be read */
    if (!is_rxfifo_empty(regs_base))
        return read_char(regs_base);

    /* There is no data to be read */
    return -1;
}
#endif

static void write_buffer(rtems_termios_device_context *base,
                         const char *buffer, size_t n) {
#ifdef BSP_CONSOLE_USE_INTERRUPTS
    mini_uart_context *context = (void *)base;
    const uintptr_t regs_base  = context->regs_base;

    if (n > 0) {
        size_t i = 0;
        while (txfifo_has_space(regs_base) && i < n) {
            write_char(regs_base, buffer[i]);
            i++;
        }

        context->tx_queued_chars = i;
        enable_irq(regs_base, IER_REG_TXE);

        return;
    }

    /*
     * Termios will set n to zero to indicate that the transmitter is now
     * inactive. The output buffer is empty in this case. The driver may
     * disable the transmit interrupts now.
     */
    disable_irq(regs_base, IER_REG_TXE);
#else
    for (size_t i = 0; i < n; i++)
        mini_uart_write_char_polled(base, buffer[i]);
#endif
}

static bool set_attributes(rtems_termios_device_context *base,
                           const struct termios *term) {
    const mini_uart_context *ctx = (void *)base;
    const uintptr_t regs_base    = ctx->regs_base;

    /* Disable interrupts */
    disable_irq(regs_base, IER_REG_IRQ_MASK);

    /* Disable data transfers */
    CNTL_REG(regs_base) &= ~(CNTL_REG_RXE | CNTL_REG_TXE);
    uint32_t cntl = CNTL_REG(regs_base);

    /*
     * Flush both FIFOs
     * NOTE: Only the FIFO bits support writing so direct assignment is safe.
     */
    IIR_REG(regs_base) = IIR_REG_CLEAR_RXFIFO | IIR_REG_CLEAR_TXFIFO;

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

    /* Configure receiver */
    const bool rx_enabled = (term->c_cflag & CREAD) != 0;
    if (rx_enabled)
        cntl |= CNTL_REG_RXE;

    /* Commit changes to the peripheral */
    CNTL_REG(regs_base) = cntl;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    /* Re-enable RX interrupts */
    if (rx_enabled)
        enable_irq(regs_base, IER_REG_RXE);
#endif

    return true;
}

void mini_uart_write_char_polled(const rtems_termios_device_context *base,
                                 const char ch) {
    const mini_uart_context *context = (void *)base;
    const uintptr_t regs_base        = context->regs_base;

    while (!txfifo_has_space(regs_base))
        ;

    write_char(regs_base, ch);
}
