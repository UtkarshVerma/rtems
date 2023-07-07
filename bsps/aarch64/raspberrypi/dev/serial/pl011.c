#include <dev/serial/pl011.h>
#include <stdint.h>

#define MEM_REG(addr) (*(volatile uint32_t *)(addr))

#define DR              MEM_REG(base + 0x00)
#define DR_DATA_MASK    ((1 << 8) - 1)
#define FR              MEM_REG(base + 0x18)
#define FR_BUSY         (1 << 3)
#define FR_RXFE         (1 << 4)
#define FR_TXFF         (1 << 5)
#define IBRD            MEM_REG(base + 0x24)
#define FBRD            MEM_REG(base + 0x28)
#define LCRH            MEM_REG(base + 0x2c)
#define LCRH_FEN        (1 << 4)
#define LCRH_WLEN_8BITS (0b11 << 5)
#define CR              MEM_REG(base + 0x30)
#define CR_UARTEN       (1 << 0)
#define CR_TXE          (1 << 8)
#define CR_RXE          (1 << 9)
#define ICR             MEM_REG(base + 0x44)

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *ctx, struct termios *term,
                       rtems_libio_open_close_args_t *args);
static int read_char_polled(rtems_termios_device_context *ctx);
static void write_polled(rtems_termios_device_context *ctx, const char *buf,
                         size_t n);

const rtems_termios_device_handler pl011_handler = {
    .first_open = first_open,
    .poll_read  = read_char_polled,
    .write      = write_polled,
    .mode       = TERMIOS_POLLED,
};

bool pl011_probe(rtems_termios_device_context *ctx) {
    const pl011_context *dev_ctx = (pl011_context *)ctx;
    const uintptr_t base         = dev_ctx->base_reg;

    const uint32_t scalar = 1e5;
    uint64_t bauddiv =
        ((uint64_t)dev_ctx->clock * scalar) / (dev_ctx->initial_baud * 16);
    uint32_t ibrd = bauddiv / scalar;

    CR &= ~CR_UARTEN;

    // Wait for pending transactions, then flush the FIFOs
    while (FR & FR_BUSY)
        ;
    LCRH &= ~LCRH_FEN;

    // Set the baudrate
    IBRD = ibrd;
    FBRD = ((bauddiv - (uint64_t)ibrd * scalar) * 64 + scalar / 2) / scalar;

    LCRH = LCRH_WLEN_8BITS;
    CR   = CR_UARTEN | CR_TXE | CR_RXE;

    return true;
}

static bool first_open(struct rtems_termios_tty *tty,
                       rtems_termios_device_context *ctx, struct termios *term,
                       rtems_libio_open_close_args_t *args) {
    pl011_context *dev_ctx = (pl011_context *)ctx;

    if (rtems_termios_set_initial_baud(tty, dev_ctx->initial_baud))
        return true;

    return true;
}

static int read_char_polled(rtems_termios_device_context *ctx) {
    const pl011_context *dev_ctx = (pl011_context *)ctx;
    const uintptr_t base         = dev_ctx->base_reg;

    if (FR & FR_RXFE) return -1;

    return DR & DR_DATA_MASK;
}

void pl011_write_char_polled(rtems_termios_device_context *ctx, char ch) {
    const pl011_context *dev_ctx = (pl011_context *)ctx;
    const uintptr_t base         = dev_ctx->base_reg;

    // Wait until the FIFO is empty
    while (FR & FR_TXFF)
        ;

    DR = ch;
}

static void write_polled(rtems_termios_device_context *ctx, const char *buf,
                         size_t n) {
    for (size_t i = 0; i < n; i++) {
        pl011_write_char_polled(ctx, buf[i]);
    }
}
