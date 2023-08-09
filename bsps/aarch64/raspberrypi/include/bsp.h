/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64RaspberryPi
 *
 * @brief Core BSP Definitions
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

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_H

/**
 * @addtogroup RTEMSBSPsAArch64
 *
 * @{
 */

#include <bspopts.h>

#ifndef ASM

#include <bsp/default-initial-extension.h>

#if RTEMS_BSP == raspberrypi4b
#include "bsp/bcm2711.h"

#define BSP_AUX_BASE BCM2711_AUX_BASE
#define BSP_AUX_SIZE BCM2711_AUX_SIZE

#define BSP_GPIO_BASE      BCM2711_GPIO_BASE
#define BSP_GPIO_SIZE      BCM2711_GPIO_SIZE
#define BSP_GPIO_PIN_COUNT BCM2711_GPIO_PIN_COUNT

#define BSP_GIC_BASE           BCM2711_GIC_BASE
#define BSP_GIC_SIZE           BCM2711_GIC_SIZE
#define BSP_ARM_GIC_CPUIF_BASE BCM2711_GIC_CPUIF_BASE
#define BSP_ARM_GIC_DIST_BASE  BCM2711_GIC_DIST_BASE

#define BSP_UART0_BASE BCM2711_UART0_BASE
#define BSP_UART0_SIZE BCM2711_UART0_SIZE
#endif /* raspberrypi4b */

#endif /* ASM */

/** @} */

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_H */
