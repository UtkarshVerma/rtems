#pragma once

#include "mem.h"

#define MMU_ADDR(peripheral, offset) (peripheral##_BASE + offset)

#define MMU_PERIPHERAL_BASE 0xfc000000LL
#define MMU_PERIPHERAL_SIZE 0x3800000

#define MMU_ARM_LOCAL_BASE 0xff800000LL
#define MMU_ARM_LOCAL_SIZE 0x800000
