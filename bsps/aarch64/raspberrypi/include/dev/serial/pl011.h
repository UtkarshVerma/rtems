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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_DEV_SERIAL_PL011_H
#define LIBBSP_AARCH64_RASPBERRYPI_DEV_SERIAL_PL011_H

#include <bspopts.h>
#include <rtems/rtems/intr.h>
#include <rtems/termiosdevice.h>
#include <rtems/termiostypes.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    rtems_termios_device_context context;
    uintptr_t regs_base;
    uint32_t clock;
    const uint32_t initial_baud;
    const rtems_vector_number irq;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    bool is_tx_queued;
    volatile size_t tx_queued_chars;
    volatile bool is_txfifo_primed;
    rtems_termios_tty* tty;
#endif
} pl011_context;

extern const rtems_termios_device_handler pl011_handler;

void pl011_write_char_polled(rtems_termios_device_context* base, char ch);

#endif /* LIBBSP_AARCH64_RASPBERRYPI_DEV_SERIAL_PL011_H */
