#pragma once

#include <rtems/rtems/status.h>

#include "mmu.h"

#define GPIO_BASE (MMU_PERIPHERAL_BASE + 0x200000)
#define GPIO_SIZE 0xf4

typedef enum {
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_AF5,
    GPIO_AF4,
    GPIO_AF0,
    GPIO_AF1,
    GPIO_AF2,
    GPIO_AF3,
} gpio_function;

typedef unsigned int gpio_pin;

rtems_status_code gpio_set_function(gpio_pin pin, gpio_function value);
rtems_status_code gpio_set_pin(gpio_pin pin);
rtems_status_code gpio_clear_pin(gpio_pin pin);
