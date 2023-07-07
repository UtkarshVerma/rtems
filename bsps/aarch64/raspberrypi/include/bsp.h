#pragma once

#ifndef ASM

// TODO: Implement an initial extension
#include <bsp/default-initial-extension.h>
#include <bsp/start.h>

#include "bsp/soc.h"

#define BSP_ARM_GIC_CPUIF_BASE SOC_GIC_CPUIF_BASE
#define BSP_ARM_GIC_DIST_BASE  SOC_GIC_DIST_BASE

// MMU initialization
BSP_START_TEXT_SECTION void raspberrypi_4_setup_mmu_and_cache(void);

#endif /* ASM */
