#pragma once
#include <stdint.h>

typedef volatile uint32_t soc_reg;
#define SOC_REG(addr) (*(soc_reg*)(addr))

#define SOC_PERIPHERAL_BASE 0xfc000000LL
#define SOC_PERIPHERAL_SIZE 0x3800000

// GPIO
#define SOC_GPIO_BASE (SOC_PERIPHERAL_BASE + 0x2200000)
#define SOC_GPIO_SIZE 0xf4

// Auxiliary peripherals
#define SOC_AUX_BASE (SOC_PERIPHERAL_BASE + 0x2215000)
#define SOC_AUX_SIZE 0x100

#define SOC_AUX_ENABLES           SOC_REG(SOC_AUX_BASE + 0x04)
#define SOC_AUX_ENABLES_MINI_UART (1 << 0)
#define SOC_AUX_ENABLES_SPI1      (1 << 1)
#define SOC_AUX_ENABLES_SPI2      (1 << 2)

// AUX: Mini UART
#define SOC_AUX_MU_BASE (SOC_AUX_BASE + 0x40)
#define SOC_AUX_MU_SIZE 0x40

#define SOC_UART1_BASE SOC_AUX_MU_BASE

// PL011 UARTs
#define SOC_PL011_BASE (SOC_PERIPHERAL_BASE + 0x2201000)
#define SOC_PL011_SIZE 0x200

#define SOC_UART0_BASE (SOC_PL011_BASE + 0x000)
#define SOC_UART2_BASE (SOC_PL011_BASE + 0x400)
#define SOC_UART3_BASE (SOC_PL011_BASE + 0x600)
#define SOC_UART4_BASE (SOC_PL011_BASE + 0x800)
#define SOC_UART5_BASE (SOC_PL011_BASE + 0xA00)

#define SOC_ARM_LOCAL_BASE 0xff800000LL
#define SOC_ARM_LOCAL_SIZE 0x800000

// Generic interrupt controller
#define SOC_GIC_BASE (SOC_ARM_LOCAL_BASE + 0x40000)
#define SOC_GIC_SIZE 0x8000

#define SOC_GIC_DIST_BASE  (SOC_GIC_BASE + 0x1000)
#define SOC_GIC_CPUIF_BASE (SOC_GIC_BASE + 0x2000)
