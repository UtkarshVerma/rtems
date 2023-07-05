#include "bsp/raspberrypi/gpio.h"

#include "bsp/raspberrypi/mmu.h"

#define GPIO_MAX_PIN 53

#define GPFSEL0                MEM_REG(MMU_ADDR(GPIO, 0x00))
#define GPSET0                 MEM_REG(MMU_ADDR(GPIO, 0x1c))
#define GPCLR0                 MEM_REG(MMU_ADDR(GPIO, 0x28))
#define GPIO_PUP_PDN_CTRL_REG0 MEM_REG(MMU_ADDR(GPIO, 0xe4))

static rtems_status_code gpio_set_reg(mem_reg* base_reg, unsigned int pin,
                                      mem_reg value, unsigned int field_size) {
    if (pin > GPIO_MAX_PIN) return RTEMS_INVALID_NUMBER;

    unsigned int field_mask = (1 << field_size) - 1;
    if (value > field_mask) return RTEMS_INVALID_NUMBER;

    // GPIO registers are uniformly subdivided
    unsigned int n_fields = sizeof(mem_reg) * 8 / field_size;

    // Registers are sequentially mapped for each `n_field` GPIOs
    mem_reg* reg       = base_reg + pin / n_fields;
    unsigned int shift = (pin % n_fields) * field_size;

    unsigned int tmp = *reg;
    tmp &= ~(field_mask << shift);  // Clear the field
    tmp |= value << shift;          // Set value to the field
    *reg = tmp;

    return RTEMS_SUCCESSFUL;
}

rtems_status_code gpio_set_function(unsigned int pin, gpio_function value) {
    return gpio_set_reg(&GPFSEL0, pin, value, 3);
}

rtems_status_code gpio_clear_pin(unsigned int pin) {
    return gpio_set_reg(&GPCLR0, pin, 1, 1);
}

rtems_status_code gpio_set_pin(unsigned int pin) {
    return gpio_set_reg(&GPSET0, pin, 1, 1);
}
