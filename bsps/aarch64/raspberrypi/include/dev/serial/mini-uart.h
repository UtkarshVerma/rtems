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

#ifndef LIBBSP_SHARED_DEV_SERIAL_MINI_UART_H
#define LIBBSP_SHARED_DEV_SERIAL_MINI_UART_H

#include <bspopts.h>
#include <rtems/rtems/intr.h>
#include <rtems/termiosdevice.h>
#include <stdint.h>

typedef struct {
    rtems_termios_device_context context;
    const uintptr_t regs_base;
    const uint32_t clock;
    const uint32_t initial_baud;
    const rtems_vector_number irq;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    volatile int tx_queued_chars;
#endif
} mini_uart_context;

extern const rtems_termios_device_handler mini_uart_handler;

void mini_uart_write_char_polled(const rtems_termios_device_context* context,
                                 const char ch);

#endif /* LIBBSP_SHARED_DEV_SERIAL_MINI_UART_H */
