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

#ifndef LIBBSP_SHARED_DEV_SERIAL_PL011_H
#define LIBBSP_SHARED_DEV_SERIAL_PL011_H

#include <bspopts.h>
#include <rtems/rtems/intr.h>
#include <rtems/termiosdevice.h>
#include <rtems/termiostypes.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    rtems_termios_device_context context;
    const uintptr_t regs_base;
    const uint32_t clock;
    const uint32_t initial_baud;
    const rtems_vector_number irq;

#ifdef BSP_CONSOLE_USE_INTERRUPTS
    /*
     * Due to HW limitation, the first TX interrupt should be triggered by the
     * software. This is because TX interrupts are based on transition through
     * a level, rather than on the level itself. When the UART interrupt and
     * UART is enabled before any data is written to the TXFIFO, the interrupt
     * is not set. The interrupt is only set once the TXFIFO becomes empty
     * after being filled to the trigger level. Until then, this flag variable
     * ensures that the interrupt handler is software triggered.
     */
    volatile bool needs_sw_triggered_tx_irq;

    volatile int tx_queued_chars;
    rtems_termios_tty* tty;
#endif
} pl011_context;

extern const rtems_termios_device_handler pl011_handler;

void pl011_write_char_polled(const rtems_termios_device_context* base,
                             const char ch);

#endif /* LIBBSP_SHARED_DEV_SERIAL_PL011_H */
