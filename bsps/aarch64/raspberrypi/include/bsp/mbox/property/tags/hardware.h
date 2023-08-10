/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Hardware Property Tags
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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_TAGS_HARDWARE_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_TAGS_HARDWARE_H

#include <stdint.h>

#define HARDWARE_GET_BOARD_MODEL_TAG \
    hardware_get_board_model_tag_buffer, 0x00010001
#define HARDWARE_GET_BOARD_REVISION_TAG \
    hardware_get_board_revision_tag_buffer, 0x00010002
#define HARDWARE_GET_BOARD_MAC_ADDRESS_TAG \
    hardware_get_board_mac_address_tag_buffer, 0x00010003
#define HARDWARE_GET_BOARD_SERIAL_TAG \
    hardware_get_board_serial_tag_buffer, 0x00010004
#define HARDWARE_GET_ARM_MEMORY_TAG \
    hardware_get_arm_memory_tag_buffer, 0x00010005
#define HARDWARE_GET_VC_MEMORY_TAG \
    hardware_get_vc_memory_tag_buffer, 0x00010006
#define HARDWARE_GET_CLOCKS_TAG hardware_get_clocks_tag_buffer, 0x00010007

typedef union {
    struct {
        uint32_t board_model;
    } response;
} hardware_get_board_model_tag_buffer;

typedef union {
    struct {
        uint32_t board_revision;
    } response;
} hardware_get_board_revision_tag_buffer;

typedef union {
    struct {
        uint8_t board_mac_address[6];
    } response;
} hardware_get_board_mac_address_tag_buffer;

typedef union {
    struct {
        uint64_t board_serial;
    } response;
} hardware_get_board_serial_tag_buffer;

typedef union {
    struct {
        uint32_t base_addr;
        uint32_t size;
    } response;
} hardware_get_arm_memory_tag_buffer;

typedef union {
    struct {
        uint32_t base_addr;
        uint32_t size;
    } response;
} hardware_get_vc_memory_tag_buffer;

/*
 * NOTE: The docs mention 15 kinds of clocks, but the 15 array length might
 * need further investigation.
 */
typedef union {
    struct {
        struct {
            uint32_t parent_id;
            uint32_t id;
        } clocks[15];
    } response;
} hardware_get_clocks_tag_buffer;

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_MBOX_PROPERTY_TAGS_HARDWARE_H */
