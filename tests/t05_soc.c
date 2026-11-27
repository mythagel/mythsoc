// t05_soc.c -- interconnect/SoC battery (design.md sec.10 E)
//
// Everything on the LIVE SoC: core -> wb_ic (3 masters x 5 slaves) -> ROM,
// DRAM, 16550, CLINT, PLIC. Module benches proved each block standalone;
// this battery proves the SYSTEM paths:
//   E1  ROM reads: signature words live in the ROM image; read via D path
//   E2  DRAM stress: linear fill + LCG random walk + byte/half SEL lanes
//   E3  null slave: unmapped read = 0, unmapped write dropped (design sec.2)
//   E4  decode boundaries of all five regions (wb_ic.v 91-95 / tools/addr.py)
//   E5  16550 register scan (byte-lane aligned-window decode, sel-gated)
//   E6  UART loopback roundtrip + FIFO order + RX trigger via PLIC (+LOOP)
//   E7  CLINT reset state, mtime monotonic+RO, sip regs, 0x4004 overlap
//   E8  PLIC prio/thr/en/reserved + claim/complete gateway on live levels
//   E9  fence.i I$ coherence (self-mod code) + mret invalidate + FENCE nop
//
// Register semantics per RTL (not memory): rtl/16550.v (window/lane decode,
// IER mask, LSR/MSR bits), rtl/clint.v (mtime RO, 0x4004 overlap),
// rtl/plic.v (prio src 1..16 only, d_en ADR[6:2]==0, claim ignores
// threshold, complete re-arms level gateway), rtl/wb_ic.v (region decode).
//
// UART pacing (tb_fw DIV = 100/20 = 5 cyc/bit): roundtrip ~60 cyc/byte;
// THR writes are gated on TEMT -- the RTL drops writes while tx_busy
// (16550.v:173), and TEMT is true but pending for <=1 tick after a write,
// so a TEMT poll always follows the previous roundtrip (>=50 cyc) first.
//
// ecall at E9b: mepc = ecall pc (same convention t04 verified) -> the
// handler adds 4. MIE/mie/sie stay 0: the interrupt fabric is exercised at
// the PLIC/register level; the hart never takes one (meip visible via mip).
//
// DRAM scoreboard: dram[16]=M trap count, [17]=mcause, [18]=mepc, [19]=S count.
#include "lib.h"

#define CSR_MIE     0x304
#define CSR_MTVEC   0x305
#define CSR_MCAUSE  0x342
#define CSR_MEPC    0x341
#define CSR_MIP     0x344

#define CLINT_BASE  0x02000000u
#define PLIC_BASE   0x0C000000u
#define UART_BASE   0x10000000u

#define P_PRIO0     (PLIC_BASE + 0x000000u)   // +4*src
#define P_PENDING   (PLIC_BASE + 0x00001000u) // RO; bit N = src N
#define P_EN0       (PLIC_BASE + 0x00002000u)
#define P_EN1       (PLIC_BASE + 0x00002080u)
#define P_THR0      (PLIC_BASE + 0x00200000u)
#define P_CLAIM0    (PLIC_BASE + 0x00200004u)

#define U_RBR       (UART_BASE + 0x0u)   // read = pop FIFO; write = THR
#define U_IER       (UART_BASE + 0x1u)
#define U_IIR       (UART_BASE + 0x2u)
#define U_FCR       (UART_BASE + 0x2u)
#define U_LCR       (UART_BASE + 0x3u)
#define U_MSR       (UART_BASE + 0x4u)   // RO 0x9F
#define U_SCR       (UART_BASE + 0x5u)
#define U_LSR       (UART_BASE + 0x7u)

static uint32_t rd8(uint32_t a) { return *(volatile uint8_t *)a; }

// poll until (rd(a) & mask) == want, bounded (fail-fast, never livelock)
static int poll8(uint32_t a, uint32_t mask, uint32_t want, uint32_t budget)
{
    for (uint32_t i = 0; i < budget; i++)
        if ((rd8(a) & mask) == want)
            return 1;
    return 0;
}
static int poll32(uint32_t a, uint32_t mask, uint32_t want, uint32_t budget)
{
    for (uint32_t i = 0; i < budget; i++)
        if ((rd32(a) & mask) == want)
            return 1;
    return 0;
}

// ROM signature block: .rodata lands in the .text output section = ROM
// (scripts/link.lds; scripts/fw_build.sh --only-section=.text). Values
// stride 0x3C3C3C3C (tool-verified, see notes) -- read ONLY through
// volatile so clang cannot fold the known constants (E1 must be a REAL
// bus+cache read).
static const uint32_t rom_sig[4] = {
    0x0F1E2D3Cu, 0x4B5A6978u, 0x8796A5B4u, 0xC3D2E1F0u,
};
#define ROM_SIG(i) (*(volatile const uint32_t *)&rom_sig[i])

static uint32_t buf[4096] __attribute__((aligned(16)));
static uint32_t smc[4]    __attribute__((aligned(16)));

// xorshift32 deterministic random walk
static uint32_t xs(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return *s = x;
}

__attribute__((section(".data.dram"), aligned(16)))
volatile uint32_t dram[1024];
uint32_t _stk_main[1024] __attribute__((aligned(16)));
uint32_t _stk_m[1024]    __attribute__((aligned(16)));
uint32_t _stk_s[1024]    __attribute__((aligned(16)));
uint32_t _sav_m[16]      __attribute__((aligned(16)));
uint32_t _sav_s[16]      __attribute__((aligned(16)));

void m_hnd_c(void)
{
    dram[16] = dram[16] + 1;
    dram[17] = csr_read(CSR_MCAUSE);
    dram[18] = csr_read(CSR_MEPC);
    csr_write(CSR_MEPC, dram[18] + 4);      // return past the ecall
}
void s_hnd_c(void) { dram[19]++; }          // must never run

static void fencei(void) { asm volatile("fence.i" ::: "memory"); }

int main(void)
{
    uint32_t bad = 0, seed, v, t1, t2, t3, i;

    // wb_ram is X until written (rtl/wb_ram.v header): zero every word that
    // is later read or RMW'd while still .bss. touched[] X -> the replay
    // branch would compare on X (t01 lesson: X-propagates-through-pipe).
    dram[16] = dram[17] = dram[18] = dram[19] = 0u;
    static uint32_t touched[16];
    for (i = 0; i < 16; i++)
        wr32((uint32_t)&touched[i], 0u);

    csr_write(CSR_MTVEC, (uint32_t)(uintptr_t)m_shim);   // E9b ecall target

    // ================================================================= E1
    // ROM reads: D path = mmu identity (M) -> l1_d CACHEABLE ROM window
    // (PA[31:22]==0) -> m1 -> wb_ic -> s0 wb_rom
    chk(1, ROM_SIG(0) == 0x0F1E2D3Cu);
    chk(2, ROM_SIG(1) == 0x4B5A6978u);
    chk(3, ROM_SIG(2) == 0x8796A5B4u);
    chk(4, ROM_SIG(3) == 0xC3D2E1F0u);
    chk(5, ROM_SIG(2) == 0x8796A5B4u);      // re-read: cached hit, stable

    // ================================================================= E2
    for (i = 0; i < 512; i++)
        wr32((uint32_t)&buf[i], 0x80000000u ^ (i * 0x9E3779B9u));
    for (i = 0; i < 512; i++)
        if (rd32((uint32_t)&buf[i]) != (0x80000000u ^ (i * 0x9E3779B9u)))
            bad++;
    chk(6, bad == 0);

    // LCG random write walk; shadow copy in the upper half (last writer
    // wins in both -> replay-order verification is exact). collisions:
    // 400 writes / 512 slots -> ~150 repeated indices actually exercised.
    seed = 0xC0FFEE11u;
    for (i = 0; i < 400; i++) {
        uint32_t k = xs(&seed) % 512, d = xs(&seed);
        wr32((uint32_t)&buf[k], d);
        wr32((uint32_t)&buf[512 + k], d);
        touched[k >> 5] |= 1u << (k & 31);
    }
    for (i = 0; i < 512; i++)
        if ((touched[i >> 5] >> (i & 31)) & 1u)
            if (rd32((uint32_t)&buf[i]) != rd32((uint32_t)&buf[512 + i]))
                bad++;
    chk(7, bad == 0);

    seed = 0x5EED1234u;                     // interleaved write+readback
    for (i = 0; i < 400; i++) {             // (store fwd + WB order RAW)
        uint32_t k = xs(&seed) % 512, d = xs(&seed);
        wr32((uint32_t)&buf[k], d);
        if (rd32((uint32_t)&buf[k]) != d) bad++;
    }
    chk(8, bad == 0);

    volatile uint8_t *bp = (volatile uint8_t *)&buf[100];
    bp[0] = 0x11u; bp[1] = 0x22u; bp[2] = 0x33u; bp[3] = 0x44u;
    chk(9, rd32((uint32_t)&buf[100]) == 0x44332211u);
    volatile uint16_t *hp = (volatile uint16_t *)&buf[101];
    hp[0] = 0xA5A5u; hp[1] = 0x5A5Au;
    chk(10, rd32((uint32_t)&buf[101]) == 0x5A5AA5A5u);
    chk(11, bp[2] == 0x33u && hp[1] == 0x5A5Au);

    // ================================================================= E3
    chk(12, rd32(0x04000000u) == 0u);
    wr32(0x04000000u, 0xDEADBEEFu);
    chk(13, rd32(0x04000000u) == 0u);
    wr32(0x08000000u, 0x12345678u);
    chk(14, rd32(0x08000000u) == 0u);

    // ================================================================= E4
    chk(15, rd32(CLINT_BASE) == 0u);                // msip reset 0
    chk(16, rd32(CLINT_BASE + 0x00008000u) == 0u);  // ALIGNED unmapped reg -> 0
    // (unaligned rd32 at +0xFFFF was the old WIP: core traps cause-4 correctly
    // -- load-address-misaligned -- so the check must use a word-aligned addr)
    chk(17, rd32(CLINT_BASE + 0x00010000u) == 0u);  // outside -> null
    chk(18, rd32(PLIC_BASE) == 0u);                 // prio word 0 reserved
    chk(19, rd32(PLIC_BASE + 0x003FFFFCu) == 0u);   // last PLIC word reserved
    wr32(PLIC_BASE + 0x00400000u, 7u);              // outside PLIC -> null
    chk(20, rd32(PLIC_BASE + 0x00400000u) == 0u);
    chk(21, (rd32(UART_BASE + 0xFFCu) >> 24) == (rd8(U_LSR) & 0xFFu));
    // same LSR via word lane 3 (grp4 window) and byte lane read
    chk(22, rd32(UART_BASE + 0x1000u) == 0u);       // outside UART -> null
    chk(23, rd32(0x80400000u) == 0u);               // outside DRAM -> null

    // ================================================================= E5
    // 16550 register scan (byte-lane ops; no RBR access until E6)
    wr8(U_SCR, 0x5Au);
    chk(24, rd8(U_SCR) == 0x5Au);
    wr8(U_SCR, 0x00u);
    chk(25, rd8(U_SCR) == 0x00u);
    chk(26, rd8(U_MSR) == 0x9Fu);                   // RO: THD|TEMT pattern
    wr8(U_IER, 0xFFu);                              // bits [1:0] only exist
    chk(27, rd8(U_IER) == 0x03u);
    wr8(U_IER, 0x00u);
    chk(28, rd8(U_IER) == 0x00u);
    wr8(U_LCR, 0xABu);
    chk(29, rd8(U_LCR) == 0xABu);                   // latched, readable
    chk(30, (rd8(U_LSR) & 0x60u) == 0x60u);         // THRE+TEMT idle
    chk(31, (rd8(U_LSR) & 0x01u) == 0u);            // DR clear (FIFO empty)
    chk(32, (rd8(U_IIR) & 0x07u) == 1u);            // "no interrupt" id
    wr8(U_SCR, 0x77u);
    (void)rd32(UART_BASE + 4u);                     // window4 read: no clobber
    chk(33, rd8(U_SCR) == 0x77u);

    // ================================================================= E6a
    // loopback (harness +LOOP muxes txd->rxd; DIV=5 -> ~55 cyc/byte)
    chk(34, poll8(U_LSR, 0x20u, 0x20u, 4000u));     // TEMT idle
    wr8(U_RBR, 0xA5u);                              // THR: start TX
    chk(35, poll8(U_LSR, 0x01u, 0x01u, 4000u));     // byte came back
    chk(36, rd8(U_RBR) == 0xA5u);
    chk(37, (rd8(U_LSR) & 0x01u) == 0u);            // DR clears when drained

    static const uint32_t msg[3] = {0x52u, 0x56u, 0x57u};   // 'R','V','W'
    for (i = 0; i < 3; i++) {
        chk(38 + i, poll8(U_LSR, 0x20u, 0x20u, 4000u));     // TX free
        wr8(U_RBR, msg[i]);
        // Gate on TEMT (full stop-bit tick done), NOT DR: DR has been 1 since
        // byte 0 (no pops in this loop) and cannot prove THIS byte arrived.
        // TEMT sets on the same tick edge as rx_push (16550.v:100/165) ->
        // after TEMT, the loopback byte is guaranteed in the RX FIFO.
        chk(41 + i, poll8(U_LSR, 0x20u, 0x20u, 4000u) && (rd8(U_LSR) & 0x01u));
    }
    chk(44, rd8(U_RBR) == 0x52u);                   // FIFO in-order pop
    chk(45, rd8(U_RBR) == 0x56u);
    chk(46, rd8(U_RBR) == 0x57u);

    // ================================================================= E6b
    // RX trigger level gates the uart->PLIC irq (src1): trig=2
    wr8(U_FCR, 0x40u);                              // fcr[7:6]=01 -> level 2
    wr8(U_IER, 0x01u);                              // DR irq enable
    wr32(P_PRIO0 + 4u, 1u);                         // prio[1] = 1
    wr32(P_EN0, 2u);                                // ctx0 enable src1
    wr32(P_THR0, 0u);
    // Drain residual IP1: E5's IER=0xFF blip (ier[1] -> irq level high, see
    // 16550.v:203) latched src1 -- gateways latch IP regardless of enable,
    // and IP is held until claimed (PLIC spec 4). Without this, chk48's
    // "no irq at fcnt=1" can never see PENDING clear.
    if (rd32(P_CLAIM0) == 1u)
        wr32(P_CLAIM0, 1u);                         // complete: level low now
    wr8(U_RBR, 'a');
    chk(47, poll8(U_LSR, 0x01u, 0x01u, 4000u));     // 1st char in FIFO
    chk(48, poll32(P_PENDING, 2u, 0u, 2000u));      // fcnt=1 < trig: NO irq
    chk(49, poll8(U_LSR, 0x20u, 0x20u, 4000u));     // TX free
    wr8(U_RBR, 'b');
    chk(50, poll32(P_PENDING, 2u, 2u, 4000u));      // fcnt=2 >= trig: IP1
    chk(51, rd32(P_CLAIM0) == 1u);                  // claim src1
    chk(52, (rd32(P_PENDING) & 2u) == 0u);          // claim cleared + disarmed
    wr32(P_CLAIM0, 1u);                             // complete -> re-arm
    chk(53, poll32(P_PENDING, 2u, 2u, 2000u));      // level high -> re-pend
    wr8(U_FCR, 0x01u);                              // RX FIFO reset (IIR bit0
    chk(54, poll8(U_LSR, 0x01u, 0x00u, 2000u));     // was 1: mode on) -> DR 0
    chk(55, rd32(P_CLAIM0) == 1u);                  // claim the latched IP
    wr32(P_CLAIM0, 1u);                             // complete; level now low
    chk(56, poll32(P_PENDING, 2u, 0u, 2000u) && rd32(P_CLAIM0) == 0u);
    wr8(U_IER, 0x00u);
    wr32(P_EN0, 0u);
    wr32(P_PRIO0 + 4u, 0u);

    // ================================================================= E7
    chk(57, rd32(CLINT_BASE + 0x4000u) == 0xFFFFFFFFu &&
             rd32(CLINT_BASE + 0x4004u) == 0xFFFFFFFFu);   // mtimecmp all1
    chk(58, rd32(CLINT_BASE + 0x4008u) == 0xFFFFFFFFu);    // stimecmp hi all1
    chk(59, rd32(CLINT_BASE) == 0u && rd32(CLINT_BASE + 4u) == 0u &&
             rd32(CLINT_BASE + 0x7000u) == 0u);            // sips all 0
    t1 = rd32(CLINT_BASE + 0xBFF8u);
    busy(200);
    t2 = rd32(CLINT_BASE + 0xBFF8u);
    chk(60, t2 > t1);                               // mtime counts
    wr32(CLINT_BASE + 0xBFF8u, 0u);                 // writes must NOT land
    wr32(CLINT_BASE + 0xBFFCu, 0u);
    t3 = rd32(CLINT_BASE + 0xBFF8u);
    chk(61, t3 > t2);                               // mtime is RO
    wr32(CLINT_BASE, 1u);
    v = rd32(CLINT_BASE + 4u);
    wr32(CLINT_BASE + 4u, 1u);
    chk(62, rd32(CLINT_BASE) == 1u && v == 0u);     // msip RW
    wr32(CLINT_BASE + 0x7000u, 1u);
    chk(63, rd32(CLINT_BASE + 0x7000u) == 1u &&
             rd32(CLINT_BASE + 4u) == 1u);          // mtip+sstip RW
    chk(64, poll32(P_PENDING, 0xC04u, 0xC04u, 2000u));
    // fan-out: mtip -> src10+src11, sstip -> src2 (design sec.3 table)
    wr32(CLINT_BASE, 0u); wr32(CLINT_BASE + 4u, 0u);
    wr32(CLINT_BASE + 0x7000u, 0u);
    chk(65, rd32(CLINT_BASE) == 0u && rd32(CLINT_BASE + 4u) == 0u &&
             rd32(CLINT_BASE + 0x7000u) == 0u);
    wr32(CLINT_BASE + 0x4004u, 0x0000ABCDu);        // overlap: mtc hi = stc lo
    chk(66, rd32(CLINT_BASE + 0x4004u) == 0x0000ABCDu);   // read = mtc hi
    wr32(CLINT_BASE + 0x4000u, 0xFFFFFFF0u);        // mtc lo far future
    wr32(CLINT_BASE + 0x4004u, 0xFFFFFFFFu);        // mtc hi all1 (stc lo too)
    wr32(CLINT_BASE + 0x4008u, 0xFFFFFFFFu);        // stc hi all1
    chk(67, poll32(P_PENDING, 0x0C0u, 0u, 2000u));  // src6/7 stay silent

    // ================================================================= E8
    chk(68, rd32(P_PRIO0) == 0u);                   // src0 word reserved
    wr32(P_PRIO0, 0xFFu);
    chk(69, rd32(P_PRIO0) == 0u);                   // write ignored
    wr32(P_PRIO0 + 0x14u, 3u);                      // prio[5]
    chk(70, rd32(P_PRIO0 + 0x14u) == 3u);
    wr32(P_PRIO0 + 0x44u, 9u);                      // prio[17]: src>16 resv
    chk(71, rd32(P_PRIO0 + 0x44u) == 0u);
    wr32(P_EN1, 0x0001FFFFu);                       // ctx1 en: bits16:1 RW;
    chk(72, rd32(P_EN1) == 0x0001FFFEu);            // bit0=src0 hardwired 0 (PLIC ch.6)
    wr32(P_EN1 + 4u, 0xFFu);                        // enable word 1 undecoded
    chk(73, rd32(P_EN1 + 4u) == 0u);
    wr32(P_EN1, 0u);
    wr32(P_THR0, 0xDEADBEEFu);                      // threshold 32b RW
    chk(74, rd32(P_THR0) == 0xDEADBEEFu);
    wr32(P_THR0, 0u);
    chk(75, rd32(P_CLAIM0) == 0u);                  // nothing enabled -> 0

    // live msip -> src3 gateway cycle. Drain the residual IP3 latched in
    // E7 first (level gateways latch even when disabled at the contexts).
    wr32(P_PRIO0 + 0x0Cu, 4u);                      // prio[3] = 4
    wr32(P_EN0, 8u);                                // ctx0 en src3
    chk(76, rd32(P_CLAIM0) == 3u);                  // drains E7 residual
    wr32(P_CLAIM0, 3u);                             // complete
    chk(77, poll32(P_PENDING, 8u, 0u, 2000u) && rd32(P_CLAIM0) == 0u);
    wr32(CLINT_BASE, 1u);                           // msip level up
    chk(78, poll32(P_PENDING, 8u, 8u, 2000u));      // gateway latches IP3
    wr32(P_THR0, 4u);                               // thr == prio -> masked
    chk(79, (csr_read(CSR_MIP) & 0x800u) == 0u);    // EIP0 masked -> pin 0
    chk(80, rd32(P_CLAIM0) == 3u);                  // claim IGNORES threshold
    chk(81, (csr_read(CSR_MIP) & 0x800u) == 0u);    // IP cleared + disarmed
    wr32(P_THR0, 0u);
    wr32(P_CLAIM0, 3u);                             // complete -> re-arm
    chk(82, poll32(P_PENDING, 8u, 8u, 2000u));      // level high -> re-pend
    chk(83, (csr_read(CSR_MIP) & 0x800u) != 0u);    // EIP0 -> hart pin
    wr32(CLINT_BASE, 0u);                           // level down
    chk(84, rd32(P_CLAIM0) == 3u);                  // IP latched, claimable
    wr32(P_CLAIM0, 3u);                             // complete: level gone
    chk(85, poll32(P_PENDING, 8u, 0u, 2000u) && rd32(P_CLAIM0) == 0u);
    chk(86, (csr_read(CSR_MIP) & 0x800u) == 0u);
    wr32(P_CLAIM0, 0u);                             // complete bad IDs: inert
    wr32(P_CLAIM0, 17u);
    wr32(P_EN0, 0u);
    wr32(P_PRIO0 + 0x0Cu, 0u);
    wr32(P_PRIO0 + 0x14u, 0u);

    // ================================================================= E9
    // fence.i: store-visible code becomes fetch-visible ONLY after fence.i
    // (l1_i INVAL held until idle; the commit self-redirect drains fetch)
    smc[0] = 0x00B00513u;                           // addi a0,x0,11
    smc[1] = 0x00008067u;                           // ret  (tools/instr_enc)
    fencei();
    uint32_t (*fn)(void) = (uint32_t (*)(void))&smc[0];
    chk(87, fn() == 11u);                           // cold fetch runs v1
    smc[0] = 0x01600513u;                           // addi a0,x0,22 (patch)
    chk(88, fn() == 11u);                           // STALE I$ line: v1 runs
    fencei();
    chk(89, fn() == 22u);                           // fence.i -> coherence

    // mret also invalidates the I$ (design sec.7): patch, ecall trap, the
    // mret must make the new code fetch-visible.
    smc[0] = 0x02D00513u;                           // addi a0,x0,45
    asm volatile("ecall" ::: "memory");
    chk(90, fn() == 45u);                           // mret did the invalidate
    chk(91, dram[16] == 1u && dram[17] == 11u);     // ecall-from-M, cause 11
    chk(92, dram[19] == 0u);                        // S handler never ran

    // FENCE (ioco) is a legal NOP: reaching here proves decode + retire
    asm volatile("fence rw, rw" ::: "memory");
    chk(93, dram[16] == 1u);

    // devices/null region still consistent after all of the above
    chk(94, rd32(0x04000000u) == 0u && rd32(0x80400000u) == 0u);
    fw_done();
    for (;;) ;
}
