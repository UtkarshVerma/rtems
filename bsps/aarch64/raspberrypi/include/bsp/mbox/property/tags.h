
/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Property Tags
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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_TAGS_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_TAGS_H

#include <stdint.h>

/*
 * NOTE:
 * A tag has to include padding in the end to make it 32-bit aligned. This is
 * done implicity here due to other members being uint32_t.
 */
#define MBOX_PROPERTY_TAG_TYPE(...) MBOX_PROPERTY_TAG_TYPE_(__VA_ARGS__)
#define MBOX_PROPERTY_TAG_TYPE_(type, tag_id, name, ...) \
    struct {                                             \
        uint32_t id;                                     \
        uint32_t buffer_size;                            \
        volatile uint32_t status_code;                   \
        volatile type buffer;                            \
    } name;

#define MBOX_PROPERTY_TAG_INIT(...) MBOX_PROPERTY_TAG_INIT_(__VA_ARGS__)
#define MBOX_PROPERTY_TAG_INIT_(type, tag_id, name, buffer) \
    buffer->name.buffer_size = sizeof(type);                \
    buffer->name.id          = tag_id;

#define MBOX_PROPERTY_TAG_REQUEST(buffer_name, child) \
    buffer_name->child.buffer.request

#define MBOX_PROPERTY_TAG_RESPONSE(buffer_name, child) \
    buffer_name->child.buffer.response

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_TAGS_H */
