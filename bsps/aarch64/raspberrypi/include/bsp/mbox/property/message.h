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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_MESSAGE_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_MESSAGE_H

#include <stdint.h>

#include "tags.h"

/*
 * NOTE:
 * This has to be 16-byte aligned as the least significant nibble is discarded
 * from the pointer and assumed to be zero while making the mailbox call.
 */
#define MBOX_PROPERTY_MESSAGE_BUFFER(name, size) \
    uint32_t name[size] __attribute__((aligned(16)))

#define MBOX_PROPERTY_MESSAGE_BUFFER_INIT(name, TAGS, dest) \
    struct {                                                \
        uint32_t size;                                      \
        volatile message_buffer_status_code status;         \
        TAGS(MBOX_PROPERTY_TAG_TYPE, name)                  \
        uint32_t null_tag;                                  \
    } *name = (void *)dest;                                 \
                                                            \
    TAGS(MBOX_PROPERTY_TAG_INIT, name)                      \
    name->size                = sizeof(*name);              \
    name->status.request_code = PROCESS_REQUEST;            \
    name->null_tag            = 0

// TODO: Make this internal to the init function
typedef union {
    enum {
        PROCESS_REQUEST = 0,
    } request_code;

    enum {
        RESPONSE_SUCCESSFUL = 0x80000000,
        REPSONSE_ERROR      = 0x80000001,
    } response_code;
} message_buffer_status_code;

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_MESSAGE_H */
