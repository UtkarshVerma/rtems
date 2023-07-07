#include <dev/serial/mini-uart.h>
#include <stdint.h>

#define MEM_REG(addr) (*(volatile uint32_t*)(addr))

#define IO_REG             MEM_REG(base + 0x00)
#define IO_REG_DATA_MASK   0xff
#define IER_REG            MEM_REG(base + 0x04)
#define IER_REG_TXE        (1 << 0)
#define IER_REG_RXE        (1 << 1)
#define IIR_REG            MEM_REG(base + 0x08)
#define LCR_REG            MEM_REG(base + 0x0c)
#define LCR_REG_WLEN_8     (1 << 0)
#define MCR_REG            MEM_REG(base + 0x10)
#define MCR_REG_RTS_LOW    (1 << 1)
#define LSR_REG            MEM_REG(base + 0x14)
#define LSR_REG_DATA_READY (1 << 0)
#define LSR_REG_TXIDLE     (1 << 6)
#define CNTL_REG           MEM_REG(base + 0x20)
#define CNTL_REG_TXE       (1 << 0)
#define CNTL_REG_RXE       (1 << 1)
#define BAUD_REG           MEM_REG(base + 0x28)

static bool first_open(struct rtems_termios_tty* tty,
                       rtems_termios_device_context* ctx, struct termios* term,
                       rtems_libio_open_close_args_t* args);
static void write_polled(rtems_termios_device_context* ctx, const char* buf,
                         size_t n);
static int read_char_polled(rtems_termios_device_context* ctx);

const rtems_termios_device_handler mini_uart_handler = {
    .first_open = first_open,
    .poll_read  = read_char_polled,
    .write      = write_polled,
    .mode       = TERMIOS_POLLED,
};

bool mini_uart_probe(rtems_termios_device_context* ctx) {
    const mini_uart_context* dev_ctx = (mini_uart_context*)ctx;
    const uintptr_t base             = dev_ctx->base_reg;

    IER_REG &= ~(IER_REG_RXE | IER_REG_TXE);
    CNTL_REG &= ~(CNTL_REG_RXE | CNTL_REG_TXE);

    LCR_REG |= LCR_REG_WLEN_8;    // Use 8-bit mode
    MCR_REG &= ~MCR_REG_RTS_LOW;  // Assert RTS HIGH

    BAUD_REG = dev_ctx->clock / (dev_ctx->initial_baud * 8) - 1;

    CNTL_REG |= CNTL_REG_TXE | CNTL_REG_RXE;

    return true;
}

static bool first_open(struct rtems_termios_tty* tty,
                       rtems_termios_device_context* ctx, struct termios* term,
                       rtems_libio_open_close_args_t* args) {
    const mini_uart_context* dev_ctx = (mini_uart_context*)ctx;

    if (rtems_termios_set_initial_baud(tty, dev_ctx->initial_baud))
        return false;

    return true;
}

static int read_char_polled(rtems_termios_device_context* ctx) {
    const mini_uart_context* dev_ctx = (mini_uart_context*)ctx;
    const uintptr_t base             = dev_ctx->base_reg;

    // Wait until RXFIFO has some data
    while (!(LSR_REG & LSR_REG_DATA_READY))
        ;

    return IO_REG & IO_REG_DATA_MASK;
}

void mini_uart_write_char_polled(rtems_termios_device_context* ctx, char ch) {
    const mini_uart_context* dev_ctx = (mini_uart_context*)ctx;
    const uintptr_t base             = dev_ctx->base_reg;

    // Wait until TXFIFO is empty
    while (!(LSR_REG & LSR_REG_TXIDLE))
        ;

    IO_REG = ch;
}

static void write_polled(rtems_termios_device_context* ctx, const char* buf,
                         size_t n) {
    for (size_t i = 0; i < n; i++) {
        mini_uart_write_char_polled(ctx, buf[i]);
    }
}
