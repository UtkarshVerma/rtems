#include <bsp/console-termios.h>
#include <bsp/fatal.h>
#include <bsp/raspberrypi.h>
#include <bsp/rpi-gpio.h>
#include <bsp/soc.h>
#include <dev/serial/mini-uart.h>
#include <dev/serial/pl011.h>
#include <rtems/bspIo.h>
#include <rtems/console.h>

#define CONSOLE_PORT UART1

enum uart_port {
    UART0,
    UART1,
    UART2,
    UART3,
    UART4,
    UART5,

    UART_COUNT,
};

typedef struct {
    unsigned int tx_pin;
    unsigned int rx_pin;
    gpio_function gpio_func;

    console_device device;
} uart_config;

pl011_context uart0 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART0"),
    .base_reg     = SOC_UART0_BASE,
    .clock        = RPI_UART_CLOCK_FREQ,
    .initial_baud = RPI_UART_BAUDRATE,
};

mini_uart_context uart1 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART1"),
    .base_reg     = SOC_UART1_BASE,
    .clock        = RPI_SYSTEM_CLOCK_FREQ,
    .initial_baud = RPI_UART_BAUDRATE,
};

pl011_context uart2 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART2"),
    .base_reg     = SOC_UART2_BASE,
    .clock        = RPI_UART_CLOCK_FREQ,
    .initial_baud = RPI_UART_BAUDRATE,
};

pl011_context uart3 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART3"),
    .base_reg     = SOC_UART3_BASE,
    .clock        = RPI_UART_CLOCK_FREQ,
    .initial_baud = RPI_UART_BAUDRATE,
};

pl011_context uart4 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART4"),
    .base_reg     = SOC_UART4_BASE,
    .clock        = RPI_UART_CLOCK_FREQ,
    .initial_baud = RPI_UART_BAUDRATE,
};

pl011_context uart5 = {
    .context      = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART5"),
    .base_reg     = SOC_UART5_BASE,
    .clock        = RPI_UART_CLOCK_FREQ,
    .initial_baud = RPI_UART_BAUDRATE,
};

const uart_config configs[UART_COUNT] = {
    [UART0] =
        {
            .tx_pin    = 14,
            .rx_pin    = 15,
            .gpio_func = GPIO_AF0,

            .device =
                {
                    .device_file = "/dev/ttyAMA0",
                    .probe       = pl011_probe,
                    .context     = &uart0.context,
                    .handler     = &pl011_handler,
                },
        },
    [UART1] =
        {
            .tx_pin    = 14,
            .rx_pin    = 15,
            .gpio_func = GPIO_AF5,

            .device =
                {
                    .device_file = "/dev/ttyS0",
                    .probe       = mini_uart_probe,
                    .context     = &uart1.context,
                    .handler     = &mini_uart_handler,
                },
        },
    [UART2] =
        {
            .tx_pin    = 0,
            .rx_pin    = 1,
            .gpio_func = GPIO_AF4,

            .device =
                {
                    .device_file = "/dev/ttyAMA1",
                    .probe       = pl011_probe,
                    .context     = &uart2.context,
                    .handler     = &pl011_handler,
                },
        },
    [UART3] =
        {
            .tx_pin    = 4,
            .rx_pin    = 5,
            .gpio_func = GPIO_AF4,

            .device =
                {
                    .device_file = "/dev/ttyAMA2",
                    .probe       = pl011_probe,
                    .context     = &uart3.context,
                    .handler     = &pl011_handler,
                },
        },
    [UART4] =
        {
            .tx_pin    = 8,
            .rx_pin    = 9,
            .gpio_func = GPIO_AF4,

            .device =
                {
                    .device_file = "/dev/ttyAMA3",
                    .probe       = pl011_probe,
                    .context     = &uart4.context,
                    .handler     = &pl011_handler,
                },
        },
    [UART5] =
        {
            .tx_pin    = 12,
            .rx_pin    = 13,
            .gpio_func = GPIO_AF4,

            .device =
                {
                    .device_file = "/dev/ttyAMA4",
                    .probe       = pl011_probe,
                    .context     = &uart5.context,
                    .handler     = &pl011_handler,
                },
        },
};

static void output_char_pl011(char ch) {
    pl011_write_char_polled(configs[CONSOLE_PORT].device.context, ch);
}

static void output_char_mini_uart(char ch) {
    mini_uart_write_char_polled(configs[CONSOLE_PORT].device.context, ch);
}

rtems_device_driver console_initialize(rtems_device_major_number major,
                                       rtems_device_minor_number minor,
                                       void *arg) {
    rtems_termios_initialize();

    SOC_AUX_ENABLES |= SOC_AUX_ENABLES_MINI_UART;

    for (unsigned int i = 0; i < RTEMS_ARRAY_SIZE(configs); i++) {
        const uart_config *config    = &configs[i];
        const console_device *device = &config->device;

        gpio_set_function(config->tx_pin, config->gpio_func);
        gpio_set_function(config->rx_pin, config->gpio_func);
        gpio_set_pull(config->tx_pin, GPIO_PULL_NONE);
        gpio_set_pull(config->rx_pin, GPIO_PULL_NONE);

        if ((*device->probe)(device->context)) {
            rtems_status_code status = rtems_termios_device_install(
                device->device_file, device->handler, device->flow,
                device->context);

            if (status != RTEMS_SUCCESSFUL)
                bsp_fatal(BSP_FATAL_CONSOLE_INSTALL_0);
        }
    }

    if (link(configs[CONSOLE_PORT].device.device_file, CONSOLE_DEVICE_NAME) !=
        0) {
        bsp_fatal(BSP_FATAL_CONSOLE_INSTALL_1);
    }

    BSP_output_char =
        CONSOLE_PORT == UART1 ? output_char_mini_uart : output_char_pl011;

    return RTEMS_SUCCESSFUL;
}

BSP_output_char_function_type BSP_output_char   = output_char_mini_uart;
BSP_polling_getchar_function_type BSP_poll_char = NULL;
