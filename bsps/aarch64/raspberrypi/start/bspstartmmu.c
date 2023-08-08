/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief This source file contains the default MMU tables and setup.
 */

/*
 * Copyright (C) 2022 Mohd Noor Aman
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

#include "bsp/start/bspstartmmu.h"

#include <bsp/aarch64-mmu.h>

#include "bsp.h"

BSP_START_DATA_SECTION static const aarch64_mmu_config_entry
    bsp_mmu_config_table[] = {
        AARCH64_MMU_DEFAULT_SECTIONS,
        {
            /* UART0 */
            .begin = BSP_UART0_BASE,
            .end   = BSP_UART0_BASE + BSP_UART0_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
        {
            /* Interrupts */
            .begin = BSP_GIC_BASE,
            .end   = BSP_GIC_BASE + BSP_GIC_SIZE,
            .flags = AARCH64_MMU_DEVICE,
        },
};

BSP_START_TEXT_SECTION void bsp_start_mmu_setup(void) {
    aarch64_mmu_setup();

    aarch64_mmu_setup_translation_table(
        &bsp_mmu_config_table[0], RTEMS_ARRAY_SIZE(bsp_mmu_config_table));

    aarch64_mmu_enable();
}
