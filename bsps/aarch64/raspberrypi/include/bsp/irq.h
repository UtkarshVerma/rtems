/**
 * @file
 *
 * @ingroup raspberrypi_interrupt
 *
 * @brief Interrupt Definitions
 */

/**
 * Copyright (C) 2013 Alan Cudmore
 * Copyright (C) 2022 Mohd Noor Aman
 * Copyright (C) 2023 Utkarsh Verma
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *
 *  http://www.rtems.org/license/LICENSE
 *
 */

#ifndef LIBBSP_AARCH64_RASPBERRYPI_BSP_IRQ_H
#define LIBBSP_AARCH64_RASPBERRYPI_BSP_IRQ_H

#include <bspopts.h>

/**
 * @defgroup raspberrypi_interrupt Interrrupt Support
 *
 * @ingroup RTEMSBSPsAAarch64RaspberryPi
 *
 * @brief Interrupt Support
 */

#if RTEMS_BSP == raspberrypi4b
#include "bsp/bcm2711.h"

#define BSP_GIC_BASE           BCM2711_GIC_BASE
#define BSP_GIC_SIZE           BCM2711_GIC_SIZE
#define BSP_ARM_GIC_CPUIF_BASE BCM2711_GIC_CPUIF_BASE
#define BSP_ARM_GIC_DIST_BASE  BCM2711_GIC_DIST_BASE

#define BSP_TIMER_VIRT_PPI    BCM2711_IRQ_V_TIMER
#define BSP_TIMER_PHYS_S_PPI  BCM2711_IRQ_PS_TIMER
#define BSP_TIMER_PHYS_NS_PPI BCM2711_IRQ_PNS_TIMER
#define BSP_IRQ_PL011_UART    BCM2711_IRQ_PL011_UART
#define BSP_IRQ_MINI_UART     BCM2711_IRQ_AUX

#define BSP_INTERRUPT_VECTOR_COUNT BCM2711_GIC_IRQ_COUNT

#endif /* raspberrypi4b */

#if defined(RTEMS_SMP)
#include <rtems/rtems/intr.h>
#include <rtems/score/processormask.h>

static inline rtems_status_code bsp_interrupt_set_affinity(
    rtems_vector_number vector, const Processor_mask *affinity) {
    (void)vector;
    (void)affinity;
    return RTEMS_UNSATISFIED;
}

static inline rtems_status_code bsp_interrupt_get_affinity(
    rtems_vector_number vector, Processor_mask *affinity) {
    (void)vector;
    _Processor_mask_From_index(affinity, 0);
    return RTEMS_UNSATISFIED;
}
#endif

#endif /* LIBBSP_AARCH64_RASPBERRYPI_BSP_IRQ_H */
