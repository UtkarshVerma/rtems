/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Mailbox Driver
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

#include "bsp/mbox.h"

#include <bsp/utility.h>
#include <rtems/rtems/cache.h>
#include <stdint.h>

#include "bsp.h"

#define REG(addr) *(volatile uint32_t*)(addr)

#define MBOX_READ         REG(BSP_MBOX_BASE + 0x00)
#define MBOX_STATUS       REG(BSP_MBOX_BASE + 0x18)
#define MBOX_STATUS_EMPTY BSP_BIT32(30)
#define MBOX_STATUS_FULL  BSP_BIT32(31)
#define MBOX_WRITE        REG(BSP_MBOX_BASE + 0x20)

#define MBOX_MESSAGE_CHANNEL_MASK BSP_MSK32(0, 3)
#define MBOX_MESSAGE_DATA_MASK    ~MBOX_MESSAGE_CHANNEL_MASK

uint32_t mbox_read(mbox_channel channel) {
    while (1) {
        // Wait until there is data to be read
        while (MBOX_STATUS & MBOX_STATUS_EMPTY)
            ;

        // Verify that the received data is for the requested channel
        uint32_t message = MBOX_READ;
        if ((message & MBOX_MESSAGE_CHANNEL_MASK) == channel)
            return message & MBOX_MESSAGE_DATA_MASK;
    }
}

void mbox_write(mbox_channel channel, void* buffer) {
    // Wait until the mailbox is empty
    while (MBOX_STATUS & MBOX_STATUS_FULL)
        ;

    uint32_t message = (uintptr_t)buffer & MBOX_MESSAGE_DATA_MASK;
    message |= channel & MBOX_MESSAGE_CHANNEL_MASK;
    MBOX_WRITE = message;
}
