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

#include "dev/serial/pl011.h"

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

#define DR(base)                REG(base + 0x00)
#define DR_DATA_MASK            BSP_MSK32(0, 7)
#define FR(base)                REG(base + 0x18)
#define FR_BUSY                 BSP_BIT32(3)
#define FR_RXFE                 BSP_BIT32(4)
#define FR_TXFF                 BSP_BIT32(5)
#define FR_TXFE                 BSP_BIT32(7)
#define IBRD(base)              REG(base + 0x24)
#define IBRD_BAUD_DIVINT_WIDTH  16
#define IBRD_BAUD_DIVINT_MASK   BSP_MSK32(0, IBRD_BAUD_DIVINT_WIDTH - 1)
#define FBRD(base)              REG(base + 0x28)
#define FBRD_BAUD_DIVFRAC_WIDTH 6
#define FBRD_BAUD_DIVFRAC_MASK  BSP_MSK32(0, FBRD_BAUD_DIVFRAC_WIDTH - 1)
#define LCRH(base)              REG(base + 0x2c)
#define LCRH_PEN                BSP_BIT32(1)
#define LCRH_EPS                BSP_BIT32(2)
#define LCRH_STP2               BSP_BIT32(3)
#define LCRH_FEN                BSP_BIT32(4)
#define LCRH_WLEN_MASK          BSP_MSK32(5, 6)
#define LCRH_WLEN_5BITS         BSP_FLD32(0, 5, 6)
#define LCRH_WLEN_6BITS         BSP_FLD32(1, 5, 6)
#define LCRH_WLEN_7BITS         BSP_FLD32(2, 5, 6)
#define LCRH_WLEN_8BITS         BSP_FLD32(3, 5, 6)
#define CR(base)                REG(base + 0x30)
#define CR_UARTEN               BSP_BIT32(0)
#define CR_TXE                  BSP_BIT32(8)
#define CR_RXE                  BSP_BIT32(9)
#define CR_RTSEN                BSP_BIT32(14)
#define CR_CTSEN                BSP_BIT32(15)
#define IFLS(base)              REG(base + 0x34)
#define IFLS_TXIFLSEL_MASK      BSP_MSK32(0, 2)
#define IFLS_TXIFLSEL(level)    BSP_FLD32(level, 0, 2)
#define IFLS_RXIFLSEL_MASK      BSP_MSK32(3, 5)
#define IFLS_RXIFLSEL(level)    BSP_FLD32(level, 3, 5)
#define IMSC(base)              REG(base + 0x38)
#define MIS(base)               REG(base + 0x40)
#define ICR(base)               REG(base + 0x44)

/* Applies to IMSC, ICR, and MIS */
#define IRQ_RX_BIT BSP_BIT32(4)
#define IRQ_RT_BIT BSP_BIT32(6)
#define IRQ_TX_BIT BSP_BIT32(5)
#define IRQ_FE_BIT BSP_BIT32(7)
#define IRQ_PE_BIT BSP_BIT32(8)
#define IRQ_BE_BIT BSP_BIT32(9)
#define IRQ_OE_BIT BSP_BIT32(10)
#define IRQ_MASK   BSP_MSK32(0, 10)

#ifdef BSP_CONSOLE_USE_INTERRUPTS
#define FIFO_SIZE                32
#define TXFIFO_IRQ_TRIGGER_LEVEL FIFO_LEVEL_ONE_EIGHTH
#define RXFIFO_IRQ_TRIGGER_LEVEL FIFO_LEVEL_ONE_HALF

enum fifo_trigger_level {
    FIFO_LEVEL_ONE_EIGHTH,
    FIFO_LEVEL_ONE_FOURTH,
    FIFO_LEVEL_ONE_HALF,
    FIFO_LEVEL_THREE_FOURTH,
    FIFO_LELEL_SEVEN_HALF,
};
#endif

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

static inline char read_char(const uintptr_t regs_base) {
    return DR(regs_base) & DR_DATA_MASK;
}

static inline void write_char(const uintptr_t regs_base, const char ch) {
    DR(regs_base) = ch;
}

static inline bool is_rxfifo_empty(const uintptr_t regs_base) {
    return (FR(regs_base) & FR_RXFE) != 0;
}

static inline bool is_txfifo_full(const uintptr_t regs_base) {
    return (FR(regs_base) & FR_TXFF) != 0;
}

static void flush_fifos(const pl011_context *context) {
    const uintptr_t regs_base = context->regs_base;
    
    LCRH(regs_base) &= ~LCRH_FEN;
    LCRH(regs_base) |= LCRH_FEN;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static inline void clear_irq(const uintptr_t regs_base,
                             const uint32_t irq_mask) {
    /* ICR is a write-only register */
    ICR(regs_base) = irq_mask;
}

static inline void enable_irq(const uintptr_t regs_base,
                              const uint32_t irq_mask) {
    IMSC(regs_base) |= irq_mask;
}
#endif

static inline void disable_irq(uintptr_t regs_base, uint32_t irq_mask) {
    IMSC(regs_base) &= ~irq_mask;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void irq_handler(void *arg) {
    pl011_context *context    = (void *)arg;
    const uintptr_t regs_base = context->regs_base;
    const uint32_t irqs       = MIS(regs_base);

    /* RXFIFO got data */
    const uint32_t rx_irq_mask = IRQ_RT_BIT | IRQ_RX_BIT;
    if ((irqs & rx_irq_mask) != 0) {
        char buffer[FIFO_SIZE];

        unsigned int i = 0;
        while (i < sizeof(buffer) && !is_rxfifo_empty(regs_base)) {
            buffer[i] = read_char(regs_base);
            i++;
        }

        (void)rtems_termios_enqueue_raw_characters(context->tty, buffer, i);

        /* Clear all interrupts */
        clear_irq(regs_base, rx_irq_mask);
    }

    /*
     * Some characters got queued in the TXFIFO, so dequeue them from Termios'
     * structures.
     */
    if ((irqs & IRQ_TX_BIT)!=0) {
        /*
         * First interrupt was raised, so no need to trigger the handler
         * through software anymore.
         */
        if (context->needs_sw_triggered_tx_irq == true)
            context->needs_sw_triggered_tx_irq = false;

        (void)rtems_termios_dequeue_characters(context->tty,
                                               context->tx_queued_chars);
        
        /* 
        * Explicitly clear the transmit interrupt.  This is necessary
        * because there may not be enough bytes in the output buffer to
        * fill the FIFO greater than the transmit interrupt trigger level.
        * If FIFOs are disabled, this applies if there are 0 bytes to
        * transmit and therefore nothing to fill the Tx holding register
        * with.  
        */
        clear_irq(regs_base, IRQ_TX_BIT);
    }
}
#endif

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args) {
#ifndef BSP_CONSOLE_USE_INTERRUPTS
    const
#endif
        pl011_context *context = (void *)base;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    context->tty = tty;
#endif

    if (rtems_termios_set_initial_baud(tty, context->initial_baud) != 0)
        return false;

    if (!set_attributes(base, term))
        return false;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    const uintptr_t regs_base = context->regs_base;

    /* Set FIFO trigger levels for interrupts */
    uint32_t ifls = IFLS(regs_base);
    ifls &= ~(IFLS_RXIFLSEL_MASK | IFLS_TXIFLSEL_MASK);
    ifls |= IFLS_TXIFLSEL(TXFIFO_IRQ_TRIGGER_LEVEL) |
            IFLS_RXIFLSEL(RXFIFO_IRQ_TRIGGER_LEVEL);
    IFLS(regs_base) = ifls;

    const rtems_status_code sc = rtems_interrupt_handler_install(
        context->irq, "UART", RTEMS_INTERRUPT_SHARED, irq_handler, context);
    if (sc != RTEMS_SUCCESSFUL)
        return false;
#endif

    return true;
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void last_close(rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       rtems_libio_open_close_args_t *args) {
    const pl011_context *context = (void *)base;
    (void)rtems_interrupt_handler_remove(context->irq, irq_handler, tty);
}
#else
static int read_char_polled(rtems_termios_device_context *base) {
    const pl011_context *context = (void *)base;
    const uintptr_t regs_base    = context->regs_base;

    if (is_rxfifo_empty(regs_base))
        return -1;

    return read_char(regs_base);
}
#endif

static void write_buffer(rtems_termios_device_context *base,
                         const char *buffer, size_t n) {
#ifdef BSP_CONSOLE_USE_INTERRUPTS
    pl011_context *context    = (void *)base;
    const uintptr_t regs_base = context->regs_base;

    enable_irq(regs_base, IRQ_TX_BIT);

    if (n > 0) {
        size_t i = 0;
        while(is_txfifo_full(regs_base)) ;
        while (!is_txfifo_full(regs_base) && i < n) {
            write_char(regs_base, buffer[i]);
            i++;
        }
        context->tx_queued_chars = i;
        
        if (context->needs_sw_triggered_tx_irq)
            (void)rtems_termios_dequeue_characters(context->tty,
                                               context->tx_queued_chars);
    }else {
        /*
        * Termios will set n to zero to indicate that the transmitter is now
        * inactive. The output buffer is empty in this case. The driver may
        * disable the transmit interrupts now.
        */
        disable_irq(regs_base, IRQ_TX_BIT);
    }
#else
    for (size_t i = 0; i < n; i++)
        pl011_write_char_polled(base, buffer[i]);
#endif
}

static int compute_baudrate_params(uint32_t *ibrd, uint32_t *fbrd,
                                   const uint32_t baudrate,
                                   const uint32_t clock,
                                   const unsigned short max_error) {
    /*
     * The integer baudrate divisor, i.e. (clock / (baudrate * 16)), value
     * should lie on [1, 2^16 - 1]. To ensure this, clock and baudrate have to
     * be validated.
     */
    *ibrd = clock / 16 / baudrate;
    if (*ibrd < 1 || *ibrd > IBRD_BAUD_DIVINT_MASK)
        return 2;

    /* Find the fractional part */
    const uint16_t scalar         = 1 << (FBRD_BAUD_DIVFRAC_WIDTH + 1);
    const uint64_t scaled_bauddiv = ((uint64_t)clock * scalar) / 16 / baudrate;
    const unsigned short round_off = scaled_bauddiv & 0x1;
    *fbrd = ((scaled_bauddiv >> 1) & FBRD_BAUD_DIVFRAC_MASK) + round_off;

    /* Calculate the baudrate and check if the error is too large */
    const uint32_t computed_bauddiv =
        (*ibrd << FBRD_BAUD_DIVFRAC_WIDTH) | *fbrd;
    const uint32_t computed_baudrate =
        ((uint64_t)clock << FBRD_BAUD_DIVFRAC_WIDTH) / 16 / computed_bauddiv;

    uint32_t baud_error = computed_baudrate - baudrate;
    if (baudrate > computed_baudrate)
        baud_error = baudrate - computed_baudrate;

    const unsigned short percent_error = (baud_error * 100) / baudrate;
    if (percent_error >= max_error)
        return 1;

    return 0;
}

static bool set_attributes(rtems_termios_device_context *base,
                           const struct termios *term) {
    pl011_context *context    = (void *)base;
    const uintptr_t regs_base = context->regs_base;

    /* Determine baudrate parameters */
    const uint32_t baud = rtems_termios_number_to_baud(term->c_ospeed);
    if (baud == B0)
        return false;

    uint32_t ibrd, fbrd;
    if (compute_baudrate_params(&ibrd, &fbrd, baud, context->clock, 3) != 0)
        return false;

    /* Start mode configuration from a clean slate */
    uint32_t lcrh = 0;

    /* Mode: parity */
    if ((term->c_cflag & PARENB) != 0) {
        lcrh |= LCRH_PEN;

        if ((term->c_cflag & PARODD) == 0)
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
    if ((term->c_cflag & CSTOPB) != 0)
        lcrh |= LCRH_STP2;

    /* Control: Disable UART */
    CR(regs_base) &= ~(CR_UARTEN | CR_RXE | CR_TXE);
    uint32_t cr = CR(regs_base);

    /*
     * Control: Configure flow control
     * NOTE: Flow control is untested
     */
    cr &= ~(CR_CTSEN | CR_RTSEN);
    if ((term->c_cflag & CCTS_OFLOW) != 0)
        cr |= CR_CTSEN;
    if ((term->c_cflag & CRTS_IFLOW) != 0)
        cr |= CR_RTSEN;

    /* Control: Configure receiver */
    const bool rx_enabled = (term->c_cflag & CREAD) != 0;
    if (rx_enabled)
        cr |= CR_RXE;

    /* Control: Re-enable UART */
    cr |= CR_UARTEN | CR_TXE;

    /* Disable all interrupts */
    disable_irq(regs_base, IRQ_MASK);

    flush_fifos(context);

    /* Set the baudrate */
    IBRD(regs_base) = ibrd;
    FBRD(regs_base) = fbrd;

    /* enable FIFO */
    lcrh = lcrh | LCRH_FEN;

    /*
     * Commit mode configurations
     * NOTE:
     * This has to happen after IBRD and FBRD as writing to LCRH is
     * required to trigger the baudrate update.
     */
    LCRH(regs_base) = lcrh;

    /* Commit changes to control register */
    CR(regs_base) = cr;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    /* Clear all interrupts */
    clear_irq(regs_base, IRQ_MASK);

    /* Re-enable RX interrupts */
    if (rx_enabled)
        enable_irq(regs_base, IRQ_RT_BIT | IRQ_RX_BIT);

    /*
     * UART is freshly enabled, TXFIFO is empty, and interrupt will be enabled,
     * so the next transmission will required software-trigger interrupt.
     */
    context->needs_sw_triggered_tx_irq = true;
#endif

    return true;
}

void pl011_write_char_polled(const rtems_termios_device_context *base,
                             const char ch) {
    const pl011_context *context = (void *)base;
    const uintptr_t regs_base    = context->regs_base;

    /* Wait until TXFIFO has space */
    while (is_txfifo_full(regs_base))
        ;

    write_char(regs_base, ch);
}
