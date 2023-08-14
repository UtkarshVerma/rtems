/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Property Message
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

#include "bsp/mbox/property/message.h"

#include <bsp/utility.h>
#include <rtems/rtems/cache.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "bsp/mbox/property/tags.h"

#define NULL_TAG_ID 0

#define STATUS_CODE_PROCESS_REQUEST  0
#define STATUS_CODE_RESPONSE_MASK    BIT32(31) | BIT32(0)
#define STATUS_CODE_RESPONSE_SUCCESS RESPONSE_CODE
#define STATUS_CODE_RESPONSE_ERROR   RESPONSE_CODE | BIT32(0)

mbox_property_message *mbox_property_message_new(size_t size) {
    /*
     * NOTE:
     * This has to be 16-byte aligned as the least significant nibble is
     * discarded from the pointer and assumed to be zero while making the
     * mailbox call.
     */
    void *buffer = rtems_cache_coherent_allocate(size, 16, 0);
    return buffer;
}

void mbox_property_message_destroy(mbox_property_message *message) {
    rtems_cache_coherent_free(message);
}

int mbox_property_message_init(mbox_property_message *message, size_t size,
                               mbox_property_tag_metadata *tag_metadata,
                               unsigned int tag_count) {
    uintptr_t buffer_end   = (uintptr_t)message + size;
    mbox_property_tag *tag = message->buffer;
    for (unsigned int i = 0; i < tag_count; i++) {
        if (mbox_property_tag_init(tag, buffer_end - (uintptr_t)tag,
                                   &tag_metadata[i]))
            return 1;

        /* Tags should be 32-bit aligned */
        uintptr_t next_tag_addr =
            (uintptr_t)(tag) + sizeof(tag->header) + tag_metadata[i].size;
        next_tag_addr += sizeof(uint32_t) - next_tag_addr % sizeof(uint32_t);
        tag = (mbox_property_tag *)next_tag_addr;
    }

    uint32_t *null_tag = (uint32_t *)tag;
    *null_tag          = NULL_TAG_ID;

    message->header.size   = (uintptr_t)(null_tag + 1) - (uintptr_t)message;
    message->header.status = STATUS_CODE_PROCESS_REQUEST;

    return 0;
}
