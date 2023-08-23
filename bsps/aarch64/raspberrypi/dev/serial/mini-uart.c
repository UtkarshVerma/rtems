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
#include <bspopts.h>
#include <rtems/libio.h>
#include <rtems/termiosdevice.h>
#include <rtems/termiostypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "sys/_termios.h"

#define REG(addr) *(volatile uint32_t *)(addr)

#define IO_REG(base)            REG(base + 0x00)
#define IO_REG_DATA_MASK        0xff
#define IER_REG(base)           REG(base + 0x04)
#define IER_REG_TXE             BSP_BIT32(0)
#define IER_REG_RXE             BSP_BIT32(1)
#define IIR_REG(base)           REG(base + 0x08)
#define IIR_REG_TXFIFO_EMPTY    BSP_BIT32(1)
#define IIR_REG_RXFIFO_HAS_DATA BSP_BIT32(2)
#define IIR_REG_CLEAR_RXFIFO    BSP_BIT32(1)
#define IIR_REG_CLEAR_TXFIFO    BSP_BIT32(2)
#define IIR_REG_IRQ_NOT_PENDING BSP_BIT32(0)
#define LCR_REG(base)           REG(base + 0x0c)
#define LCR_REG_WLEN_8          BSP_BIT32(0)
#define MCR_REG(base)           REG(base + 0x10)
#define MCR_REG_RTS_LOW         BSP_BIT32(1)
#define LSR_REG(base)           REG(base + 0x14)
#define LSR_REG_DATA_READY      BSP_BIT32(0)
#define LSR_REG_TXIDLE          BSP_BIT32(6)
#define CNTL_REG(base)          REG(base + 0x20)
#define CNTL_REG_TXE            BSP_BIT32(0)
#define CNTL_REG_RXE            BSP_BIT32(1)
#define BAUD_REG(base)          REG(base + 0x28)

#define FIFO_SIZE 8

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

static void write_polled(rtems_termios_device_context *base, const char *buf,
                         size_t n);

static void write_irq_driven(rtems_termios_device_context *base,
                             const char *buf, size_t n);

static bool set_attributes(rtems_termios_device_context *base,
                           const struct termios *term);

const rtems_termios_device_handler mini_uart_handler = {
    .first_open = first_open,
#ifdef BSP_CONSOLE_USE_INTERRUPTS
    .last_close = last_close,
    .mode       = TERMIOS_IRQ_DRIVEN,
    .poll_read  = NULL,
    .write      = write_irq_driven,
#else
    .last_close = NULL,
    .mode       = TERMIOS_POLLED,
    .poll_read  = read_char_polled,
    .write      = write_polled,
#endif
    .set_attributes = set_attributes,
    .ioctl          = NULL,
};

static inline int read_char(uintptr_t regs_base) {
    return IO_REG(regs_base) & IO_REG_DATA_MASK;
}

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       struct termios *term,
                       rtems_libio_open_close_args_t *args) {
    const mini_uart_context *ctx = (void *)base;

    if (rtems_termios_set_initial_baud(tty, ctx->initial_baud))
        return false;

    return set_attributes(base, term);
}

#ifdef BSP_CONSOLE_USE_INTERRUPTS
static void last_close(rtems_termios_tty *tty,
                       rtems_termios_device_context *base,
                       rtems_libio_open_close_args_t *args) {
    const mini_uart_context *ctx = (void *)base;
    /* rtems_interrupt_handler_remove(ctx->irq, irq_handler, tty); */
}
#else
static int read_char_polled(rtems_termios_device_context *base) {
    const mini_uart_context *ctx = (void *)base;
    const uintptr_t regs_base    = ctx->regs_base;

    /* Data is available to be read */
    if (LSR_REG(regs_base) & LSR_REG_DATA_READY)
        return read_char(regs_base);

    /* There is no data to be read */
    return -1;
}
#endif

static size_t read_irq_driven(mini_uart_context *ctx, char *buffer,
                              size_t size) {
    const uintptr_t regs_base = ctx->regs_base;

    unsigned int i = 0;
    while (i < size) {
        const uint32_t iir = IIR_REG(regs_base);

        /* No interrupt has been raised */
        if (iir & IIR_REG_IRQ_NOT_PENDING)
            break;

        /* Data has been received */
        if (iir & IIR_REG_RXFIFO_HAS_DATA) {
            buffer[i] = read_char(regs_base);
            i++;
        }
    }

    return i;
}

static inline void write_char(uintptr_t regs_base, char ch) {
    IO_REG(regs_base) = ch;
}

static void write_char_polled(rtems_termios_device_context *base, char ch) {
    const mini_uart_context *ctx = (void *)base;
    const uintptr_t regs_base    = ctx->regs_base;

    /* Wait until the transmitter is idle */
    while (!(LSR_REG(regs_base) & LSR_REG_TXIDLE))
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
    mini_uart_context *ctx = (void *)base;
    uintptr_t regs_base    = ctx->regs_base;

    if (n == 0) {
        /*
         * Zero value indicates that the transmitter is now inactive. The
         * output buffer is empty in this case, so interrupts can be disabled.
         */
        IER_REG(regs_base) &= ~IER_REG_TXE;

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
    mini_uart_context *ctx    = (void *)base;
    const uintptr_t regs_base = ctx->regs_base;

    /* Disable interrupts */
    IER_REG(regs_base) &= ~(IER_REG_RXE | IER_REG_TXE);

    /* Disable data transfers */
    CNTL_REG(regs_base) &= ~(CNTL_REG_RXE | CNTL_REG_TXE);

    /* Set the character size */
    switch (term->c_cflag & CSIZE) {
        case CS8:
            LCR_REG(regs_base) |= LCR_REG_WLEN_8;
            break;
        case CS7:
            LCR_REG(regs_base) &= ~LCR_REG_WLEN_8;
        default:
            /* Mini UART supports only 7-bit and 8-bit character sizes. */
            return false;
    }

    /* Configure receiver */
    if (term->c_cflag & CREAD) {
        CNTL_REG(regs_base) |= CNTL_REG_RXE;
        /* if (ctx->irq_enabled) */
        /*     IER_REG(regs_base) |= IER_REG_RXE; */
    }

    /*
     * TODO:
     * Add hardware-flow control. For now, this just asserts RTS HIGH all the
     * time.
     */
    MCR_REG(regs_base) &= ~MCR_REG_RTS_LOW;

    /* Clear both FIFOs */
    IIR_REG(regs_base) |= (IIR_REG_CLEAR_RXFIFO | IIR_REG_CLEAR_TXFIFO);

    /* Set the baudrate */
    speed_t baud = rtems_termios_number_to_baud(term->c_ospeed);
    if (baud == B0)
        return false;
    BAUD_REG(regs_base) = ctx->clock / (baud * 8) - 1;

    /* Re-enable transmission */
    CNTL_REG(regs_base) |= CNTL_REG_TXE;
    /* if (ctx->irq_enabled) */
    /*     IER_REG(regs_base) |= IER_REG_TXE; */

    return true;
}

static void interrupt_handler(void *arg) {
    rtems_termios_tty *tty;
    mini_uart_context *ctx;

    tty = arg;
    ctx = rtems_termios_get_device_context(tty);

    char buffer[FIFO_SIZE];

    /*
     * Check if we have received something. The function reads the received
     * characters from the device and stores them in the buffer. It returns the
     * number of read characters.
     */
    size_t n = read_irq_driven(ctx, buffer, sizeof(buffer));
    if (n > 0)
        /* Hand the data over to the Termios infrastructure */
        rtems_termios_enqueue_raw_characters(tty, buffer, n);

    /*
     * Check if something got transmitted. The function returns the
     * number of transmitted characters since the last write to the
     * device.
     */
    /* n = my_driver_transmitted_chars(ctx); */
    if (n > 0)
        /*
         * Notify Termios that we have transmitted some characters.  It
         * will call now the interrupt write function if more characters
         * are ready for transmission.
         */
        rtems_termios_dequeue_characters(tty, n);
}

/* void uart_init(const unsigned baudrate, uart_newchar_t newchar) { */
/*   if (newchar != NULL) { */
/*     // Install the IRQ handler and enable read interrupts. */
/*     s_callback = newchar; */
/*     isr_addhandler(ISR_IRQ, uart_read_isr); */
/*     mem_write32(BASE_ADDR + AUX_MU_IER_REG, 5); */
/*     isr_enablebasic(29); */
/*   } */
/* } */
