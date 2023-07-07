#include <bsp/rpi-gpio.h>
#include <bsp/soc.h>
#include <rtems/rtems/status.h>
#include <stdint.h>

#define GPIO_MAX_PIN 53

#define GPFSEL0                SOC_REG(SOC_GPIO_BASE + 0x00)
#define GPSET0                 SOC_REG(SOC_GPIO_BASE + 0x1c)
#define GPCLR0                 SOC_REG(SOC_GPIO_BASE + 0x28)
#define GPIO_PUP_PDN_CTRL_REG0 SOC_REG(SOC_GPIO_BASE + 0xe4)

static rtems_status_code gpio_set_reg(soc_reg* base_reg, unsigned int pin,
                                      soc_reg value, unsigned int field_size) {
    if (pin > GPIO_MAX_PIN) return RTEMS_INVALID_NUMBER;

    unsigned int field_mask = (1 << field_size) - 1;
    if (value > field_mask) return RTEMS_INVALID_NUMBER;

    // GPIO registers are uniformly subdivided
    unsigned int n_fields = sizeof(soc_reg) * 8 / field_size;

    // Registers are sequentially mapped for each `n_field` GPIOs
    soc_reg* reg       = base_reg + pin / n_fields;
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

rtems_status_code gpio_set_pull(unsigned int pin, gpio_pull value) {
    return gpio_set_reg(&GPIO_PUP_PDN_CTRL_REG0, pin, value, 2);
}
