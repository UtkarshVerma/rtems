/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Broadcom Serial Control (I2C) Device Driver
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

#include "dev/i2c/bsc.h"

#include <bsp.h>
#include <bsp/utility.h>
#include <dev/i2c/i2c.h>
#include <linux/i2c.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/errno.h>

#define REG(addr) *(volatile uint32_t *)(addr)

#define C_REG(base)    REG(base + 0x00)
#define C_REG_READ     BSP_BIT32(0)
#define C_REG_CLEAR    BSP_MSK32(4, 5)
#define C_REG_ST       BSP_BIT32(7)
#define C_REG_INTD     BSP_BIT32(8)
#define C_REG_I2CEN    BSP_BIT32(15)
#define S_REG(base)    REG(base + 0x04)
#define S_REG_TA       BSP_BIT32(0)
#define S_REG_DONE     BSP_BIT32(1)
#define S_REG_TXW      BSP_BIT32(2)
#define S_REG_RXD      BSP_BIT32(5)
#define S_REG_ERR      BSP_BIT32(8)
#define S_REG_CLKT     BSP_BIT32(9)
#define DLEN_REG(base) REG(base + 0x08)
#define DLEN_REG_DLEN  BSP_MSK32(0, 15)
#define A_REG(base)    REG(base + 0x0c)
#define A_REG_ADDR     BSP_MSK32(0, 6)
#define FIFO_REG(base) REG(base + 0x10)
#define FIFO_REG_DATA  BSP_MSK32(0, 7)
#define DIV_REG(base)  REG(base + 0x14)
#define DIV_REG_CDIV   BSP_MSK32(0, 15)
#define DEL_REG(base)  REG(base + 0x18)
#define CLKT_REG(base) REG(base + 0x1c)

#define TRANSFER_SIZE_MAX DLEN_REG_DLEN

/* TODO:
 * - 10-bit addressing
 * - IRQ support
 */

static int set_clock_speed(i2c_bus *bus, unsigned long clock) {
    const bsc_context *ctx    = (void *)bus;
    const uintptr_t regs_base = ctx->regs_base;

    DIV_REG(regs_base) = ctx->clock / clock;

    return 0;
}

static bool transfer_failed(const uintptr_t regs_base) {
    /* Check for acknowledgment or clock stretching errors. */
    return (S_REG(regs_base) & (S_REG_ERR | S_REG_CLKT)) != 0;
}

static int read_char_polled(const uintptr_t regs_base) {
    /* Wait for data to arrive */
    while ((S_REG(regs_base) & S_REG_RXD) == 0)
        ;

    /* Read data from the RX FIFO. */
    const char ch = FIFO_REG(regs_base) & FIFO_REG_DATA;

    if (transfer_failed(regs_base))
        return -EINVAL;

    return ch;
}

static int write_char_polled(const uintptr_t regs_base, const char ch) {
    /* Wait until there is space */
    while ((S_REG(regs_base) & S_REG_TXW) == 0)
        ;

    /* Write data to the TX FIFO. */
    FIFO_REG(regs_base) = ch & FIFO_REG_DATA;

    if (transfer_failed(regs_base))
        return -1;

    return 0;
}

static int init_transfer(const uintptr_t regs_base,
                         const uint16_t transfer_size, const bool must_read) {
    /* Wait for pending transfers */
    while ((S_REG(regs_base) & S_REG_DONE) == 0)
        ;

    /* Specify the transfer size */
    DLEN_REG(regs_base) = transfer_size & DLEN_REG_DLEN;

    /* Clear the acknowledgment and clock stretching error status */
    S_REG(regs_base) |= (S_REG_ERR | S_REG_CLKT);

    uint32_t c = C_REG(regs_base);

    /* Start the transmission */
    c |= C_REG_ST;

    /* Configure read/write mode */
    c &= ~C_REG_READ;
    if (must_read)
        c |= C_REG_READ;

    /* Commit changes to the control register */
    C_REG(regs_base) = c;

    /* Check if we got an ACK */
    if ((S_REG(regs_base) & S_REG_ERR) != 0)
        return EIO;

    return 0;
}

static int read_polled(const uintptr_t regs_base, uint8_t *buffer,
                       const size_t size) {
    init_transfer(regs_base, size, true);

    for (size_t i = 0; i < size; i++) {
        int ch = read_char_polled(regs_base);
        if (ch < 0)
            return ch;

        buffer[i] = (char)ch;
    }

    return 0;
}

static int write_polled(const uintptr_t regs_base, const uint8_t *buffer,
                        const size_t size) {
    init_transfer(regs_base, size, false);

    for (size_t i = 0; i < size; i++) {
        int status = write_char_polled(regs_base, buffer[i]);
        if (status != 0)
            return status;
    }

    return 0;
}

static int transfer(i2c_bus *bus, i2c_msg *messages,
                    const uint32_t message_count) {
    bsc_context *ctx          = (void *)bus;
    const uintptr_t regs_base = ctx->regs_base;

    /* Perform an initial parse through the messages for the I2C_M_RECV_LEN
     * flag, which the Pi seems to not support and the I2C framework expects
     * the bus to provide as part of the I2C_FUNC_I2C functionality.
     *
     * It states that the slave device sends an initial byte containing the
     * size of the transfer, and for this to work the Pi will likely require
     * two transfers, with a stop-start condition in-between. */
    for (uint32_t i = 0; i < message_count; ++i) {
        if (messages[i].flags & I2C_M_RECV_LEN)
            return -EINVAL;
    }

    for (uint32_t i = 0; i < message_count; ++i) {
        const i2c_msg *message = &messages[i];

        /* Clear FIFOs */
        C_REG(regs_base) |= C_REG_CLEAR;

        /* Specify the slave address */
        A_REG(regs_base) = messages[i].addr;

        const bool must_read = (message->flags & I2C_M_RD) != 0;
        if (must_read || message->flags == 0) {
            /*
             * NOTE:
             * BSC transmissions have 16-bit sizes. Hence, to transmit data
             * with bigger sizes, we'll have to chunk the data into transfers
             * of size 0xffff each.
             */
            const unsigned int transfer_count =
                message->len / TRANSFER_SIZE_MAX + 1;

            for (unsigned int i = 0; i < transfer_count; i++) {
                uint16_t transfer_size = TRANSFER_SIZE_MAX;

                /* Last transfer contains the remaining bytes */
                if (i == transfer_count - 1)
                    transfer_size = message->len % TRANSFER_SIZE_MAX;

                if (must_read) {
                    read_polled(regs_base, message->buf, transfer_size);
                } else {
                    write_polled(regs_base, message->buf, transfer_size);
                }
            }
        }
    }

    return 0;
}

int bsc_init(i2c_bus *bus) {
    const bsc_context *ctx    = (void *)bus;
    const uintptr_t regs_base = ctx->regs_base;

    int error = i2c_bus_init(bus);
    if (error != 0)
        return error;

    bus->transfer      = transfer;
    bus->set_clock     = set_clock_speed;
    bus->destroy       = i2c_bus_destroy;
    bus->functionality = I2C_FUNC_I2C;

    /* Enable the peripheral */
    C_REG(regs_base) |= C_REG_I2CEN;

    return 0;
}
