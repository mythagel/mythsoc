// board_demo.c -- mythrv32 on-hardware demo (Colorlight 5a-75b v8.2).
//
// Not a test battery (those are t01..t07, sim-gated): this is the ROM image
// the board boots by default so that `make flash` produces something you can
// SEE on the UART (/dev/ttyUSB0 per design.md sec.1) with zero tooling.
//
// Behavior (115200 8N1 -- RTL reset defaults, rtl/16550.v:213):
//   1. banner: ISA/mode/clock line (misa read + M-bit check);
//   2. heartbeat: one '.' per ~0.5 s (calibrated loop, not mtime: keeps
//      the demo free of device assumptions beyond the UART);
//   3. echo: every char arriving on rxd is re-sent (polling, no PLIC/IE --
//      proves RX sampling on the real 25 MHz clock too).
//
// The fw framework gives us: _start (M mode), .data copy + .bss zero via
// the SDRAM (this IS the board bring-up test of wb_sdram), link.lds
// layout (dram[] at 0x8000_0000), trap shims (linked; nothing traps).
#include "lib.h"

// framework contract (see t05_soc.c): dram[] at 0x8000_0000 via .data.dram,
// stacks for _start + the S shim (linked though never entered here).
__attribute__((section(".data.dram"), aligned(16)))
volatile uint32_t dram[1024];
uint32_t _stk_main[1024] __attribute__((aligned(16)));
uint32_t _stk_m[1024]    __attribute__((aligned(16)));
uint32_t _stk_s[1024]    __attribute__((aligned(16)));
uint32_t _sav_m[16]      __attribute__((aligned(16)));
uint32_t _sav_s[16]      __attribute__((aligned(16)));

#define UART_BASE 0x10000000u
#define U_RBR     (UART_BASE + 0x0u)   // read = pop RX FIFO, write = THR
#define U_LSR     (UART_BASE + 0x7u)   // bit0 DR, bit5 TEMT, bit6 THRE

static void tx(uint8_t c)
{
    while (!(rd32(U_LSR) & 0x20u))     // wait TEMT (t05 lesson: THR writes
        ;                              // while busy are dropped by the RTL)
    wr8(U_RBR, (uint32_t)c);
}

static void puts_(const char *s)
{
    while (*s) {
        if (*s == '\n') tx('\r');
        tx((uint8_t)*s++);
    }
}

static void put_hex(uint32_t v)
{
    for (int i = 28; i >= 0; i -= 4) {
        uint32_t n = (v >> i) & 0xFu;
        tx((uint8_t)(n < 10 ? '0' + n : 'a' + n - 10));
    }
}

void m_hnd_c(void) { }                  // shims reference these; unused
void s_hnd_c(void) { }

int main(void)
{
    dram[0] = 0xDEADBEEFu;              // .bss zero already exercised SDRAM
    uint32_t isa = csr_read(0x301);     // misa
    puts_("mythrv32 rv32im @ 25 MHz (LFE5U-25F)\r\n");
    puts_("misa=0x");
    put_hex(isa);
    puts_(isa & 0x1000u ? "  M-ext ok\r\n" : "  M-ext MISSING\r\n");
    puts_("echo on; '.' = heartbeat\r\n");

    uint32_t beats = 0;
    for (;;) {
        for (volatile uint32_t i = 0; i < 1000000u; i++)   // ~0.3 s @25 MHz
            ;
        tx('.');
        if (++beats % 20u == 0u) {     // periodic sanity line
            dram[0] = beats;
            puts_("\r\nalive:");
            put_hex(beats);
            tx('\n');
        }
        while (rd32(U_LSR) & 0x1u)     // drain RX -> echo
            tx((uint8_t)(rd32(U_RBR) & 0xFFu));
    }
}
