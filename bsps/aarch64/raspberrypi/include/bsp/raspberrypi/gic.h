#pragma once

#include "mmu.h"

#define GIC_BASE (MMU_ARM_LOCAL_BASE + 0x40000)
#define GIC_SIZE 0x8000

#define GIC_DIST_BASE  MMU_ADDR(GIC, 0x1000)
#define GIC_CPUIF_BASE MMU_ADDR(GIC, 0x2000)
