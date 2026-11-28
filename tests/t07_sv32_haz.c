// t07_sv32_haz.c -- C8 hazard/forwarding battery under LIVE Sv32 translation
// (design.md sec.10 C-first policy; closes the "C8 Sv32 on-core" gap in sec.12).
//
// The C8 patterns (ALU d=1..4, load-use per width, st->ld / st->st b2b,
// ld->branch, ld->jalr, MUL/DIV/REM b2b + div edge semantics, CSR d=1,
// backward-loop accum, same-reg b2b overwrite) are re-expressed as C
// dataflow with volatile operands, so every RAW consumer is a REAL pipeline
// hazard (LLVM cannot fold a volatile load into its stored constant), and
// each pattern runs with the D-TLB cold: sfence.vma between items forces a
// 4K walk (5-cyc MA park) or 4M walk (3-cyc) exactly under the stall/fwd
// window that the forwarding logic must survive.
// Divisors/edge operands load from volatile slots -- a literal `-1` or `7`
// divisor gets strength-reduced by LLVM to neg/mulhi and would test no DIV.
//
// MMU-specific passes beyond C8-Bare:
//   - alias triangle: same PA via identity VA, 4K-alias VA, 4M-mega alias VA
//     interleaved b2b (PA-keyed D$: coherence + per-VA TLB keys)
//   - two cold 4K pages as back-to-back loads -> two serialized m2 walks with
//     a dependent op pending in EX
//   - I-side: call ROM leaves through 4K alias VAs -> I-TLB walk + I$ refill
//     mid-call (link/return in flight while IF is stalled on the walk)
//   - asid1 re-run of the subset after a satp asid switch (re-key + re-walk)
//
// Traps: NONE expected. medeleg/mideleg = 0, interrupts fully masked. The M
// handler records (cause,tval,epc) to dram[16..19] and skips (+4) purely so
// a regression FAILS with evidence instead of hanging; the final checks
// require BOTH handler counts == 0 (chk 45). S-mode, satp on, after mret.
//
// VA map (pt0; pt1 = identical clone used with satp.asid=1):
//  0x00000000 4M -> PA 0          R W X A D   identity ROM+CLINT+PLIC+UART
//  0x60000000 4M -> PA 0x80000000 R W X A D   DRAM mega alias
//  0x80000000 4M -> PA 0x80000000 R W X A D   DRAM identity (shims/PTs)
//  0xA0000000 ptr -> l0s[i] 4K -> scratch[i]  R W X A D
//  0xC0000000 ptr -> l0r[j] 4K -> PA j<<12    R X   A D   (ROM code alias)
//
// Protocol (tests/lib.h): dram[0]/[1] result+marker via chk()/fw_done.
#include "lib.h"

#define CSR_SSTATUS  0x100
#define CSR_STVEC    0x105
#define CSR_SEPC     0x141
#define CSR_SCAUSE   0x142
#define CSR_STVAL    0x143
#define CSR_SSCRATCH 0x140
#define CSR_SATP     0x180
#define CSR_MSTATUS  0x300
#define CSR_MEDELEG  0x302
#define CSR_MIDELEG  0x303
#define CSR_MIE      0x304
#define CSR_MTVEC    0x305
#define CSR_MEPC     0x341
#define CSR_MCAUSE   0x342
#define CSR_MTVAL    0x343

#define MS_MPP_M  0x1800u
#define MS_MPP_S  0x0800u
#define SATP_MODE 0x80000000u

#define VA_MEGA 0x60000000u     // 4M -> DRAM PA 0x8000_0000
#define VA_SC   0xA0000000u     // 8 x 4K -> scratch[0..7]
#define VA_ROM  0xC0000000u     // 4 x 4K -> ROM PA 0x00000..0x03FFFF

#define PPN(pa) ((((uint32_t)(pa)) >> 12) & 0x3FFFFFu)
#define F_V 1u
#define F_R 2u
#define F_W 4u
#define F_X 8u
#define F_A 64u
#define F_D 128u
static uint32_t mkpte(uint32_t ppn, uint32_t f) { return (ppn << 10) | f; }

// sfence.vma has no C spelling -- inline asm (necessary-minimum surface, t04 idiom)
static void sf_all(void) { asm volatile("sfence.vma" ::: "memory"); }

// ---- objects (PA == identity VA; 4K-aligned => in-page offset 0) ----
uint32_t _stk_main[1024] __attribute__((aligned(16)));
uint32_t _stk_m[1024]    __attribute__((aligned(16)));
uint32_t _stk_s[1024]    __attribute__((aligned(16)));
uint32_t _sav_m[16]      __attribute__((aligned(16)));
uint32_t _sav_s[16]      __attribute__((aligned(16)));

__attribute__((section(".data.dram"))) volatile uint32_t dram[1024];

uint32_t pt0[1024]      __attribute__((aligned(4096)));
uint32_t pt1[1024]      __attribute__((aligned(4096)));
uint32_t l0s[1024]      __attribute__((aligned(4096)));   // VA_SC 4K pages
uint32_t l0r[1024]      __attribute__((aligned(4096)));   // VA_ROM 4K pages
volatile uint32_t scratch[8][1024] __attribute__((aligned(4096)));

// pass results (asid0 vs asid1) + opaque operand slots
static volatile uint32_t res1[8];
static volatile uint32_t res2[8];
static volatile uint32_t fptr_v;      // func ptr slot (ld->jalr)
static volatile uint32_t zero_v;      // live 0        (div-by-0)
static volatile uint32_t seven_v;     // live 7        (div/rem by const)
static volatile uint32_t neg1_v;      // live 0xFFFFFFFF (divu edge / -1 signed)

static void build_tables(void)
{
    uint32_t i;
    for (i = 0; i < 1024; i++) { pt0[i] = 0; l0s[i] = 0; l0r[i] = 0; }
    pt0[0x000] = mkpte(0x00000, F_V | F_R | F_W | F_X | F_A | F_D);
    pt0[0x180] = mkpte(PPN(0x80000000u), F_V | F_R | F_W | F_X | F_A | F_D);
    pt0[0x200] = mkpte(PPN(0x80000000u), F_V | F_R | F_W | F_X | F_A | F_D);
    pt0[0x280] = mkpte(PPN(l0s), F_V);
    pt0[0x300] = mkpte(PPN(l0r), F_V);
    for (i = 0; i < 8; i++)
        l0s[i] = mkpte(PPN(&scratch[i][0]), F_V | F_R | F_W | F_X | F_A | F_D);
    for (i = 0; i < 4; i++)
        l0r[i] = mkpte(i, F_V | F_R | F_X | F_A | F_D);
    for (i = 0; i < 1024; i++) pt1[i] = pt0[i];
}

static volatile uint32_t *AP(uint32_t pg, uint32_t idx)   // 4K alias window
{ return (volatile uint32_t *)(VA_SC + pg * 0x1000u + idx * 4u); }
static volatile uint32_t *MP(uint32_t pg, uint32_t idx)   // 4M mega alias
{ return (volatile uint32_t *)(VA_MEGA + ((uint32_t)&scratch[pg][0]
                                          - 0x80000000u) + idx * 4u); }

// ============================================================ handlers
// record + skip; counts MUST stay zero (chk 43)
void m_hnd_c(void)
{
    dram[16] = dram[16] + 1;
    dram[17] = csr_read(CSR_MCAUSE);
    dram[18] = csr_read(CSR_MTVAL);
    dram[19] = csr_read(CSR_MEPC);
    csr_write(CSR_MEPC, dram[19] + 4);
}

void s_hnd_c(void)
{
    dram[20] = dram[20] + 1;
    dram[21] = csr_read(CSR_SCAUSE);
    dram[22] = csr_read(CSR_STVAL);
    dram[23] = csr_read(CSR_SEPC);
    csr_write(CSR_SEPC, dram[23] + 4);
}

// ============================================================ noinline leaves
static uint32_t __attribute__((noinline)) leaf1(uint32_t x) { return x * 3u + 1u; }
static uint32_t __attribute__((noinline)) leaf2(uint32_t x) { return leaf1(x) ^ 0xA5A5A5A5u; }
static uint32_t __attribute__((noinline)) leaf3(uint32_t x) { return leaf2(x + 1u) - 2u; }
static uint32_t __attribute__((noinline)) leaf4(uint32_t x)
{ uint32_t s = 0; for (uint32_t i = 1; i <= x; i++) s += i * i; return s; }

// ROM alias entry: identity PA == VA in the low 4 MiB -> alias = VA_ROM + pa
static uint32_t __attribute__((noinline)) rom_alias(uint32_t f)
{ return VA_ROM + (f & 0x003FFFFFu); }

// ============================================================ C8 subset pass
// Runs the core C8 chains cold; results -> res[] (asid0 pass: page set
// 0/1/2, asid1 pass MUST consume identically-seeded cells -> equal res).
static void pass(volatile uint32_t *res, uint32_t pg, uint32_t plus)
{
    // ---- B: ALU d=1..4 from two cold loads (2 serialized walks, pending ALU)
    sf_all();
    uint32_t a = *AP(pg, 0);
    uint32_t b = *AP(pg, 1);
    uint32_t r1 = a + b;                    // d=1 (consumer of parked load)
    uint32_t r2 = r1 ^ a;                   // d=2
    uint32_t r3 = r2 - b;                   // d=3
    res[0] = r3 | 0x5A5A0000u;              // d=4

    // ---- C: load-use per width, each cold (walk parks MA mid-extract)
    sf_all();
    res[1] = *AP(pg, 2) + 1u;                       // LW d=1
    sf_all();
    res[2] = (uint32_t)(int32_t)(*(volatile int8_t *)
                                 ((uint32_t)AP(pg, 3))) + 1u;              // LB
    sf_all();
    res[3] = (uint32_t)(int32_t)(*(volatile int16_t *)
                                 ((uint32_t)AP(pg, 3) + 2u)) * 2u;         // LH
    sf_all();
    res[4] = (uint32_t)(*(volatile uint8_t *)
                         ((uint32_t)AP(pg, 3))) + 7u;                     // LBU
    sf_all();
    res[5] = (uint32_t)(*(volatile uint16_t *)
                         ((uint32_t)AP(pg, 3) + 2u)) + 7u;                // LHU

    // ---- H: b2b cold loads on ADJACENT pages -> walk, walk, dependent op
    sf_all();
    res[6] = *AP(pg, 4) ^ *AP((pg + 1) & 7u, 4);

    // ---- F: CSR write/read + consumer d=1 (S-mode sscratch)
    sf_all();
    csr_write(CSR_SSCRATCH, *AP(pg, 5));
    res[7] = csr_read(CSR_SSCRATCH) + plus;         // right after csrr
}

// ============================================================ s_main (S)
static void s_main(void)
{
    uint32_t i, v, acc, arg;

    // ---- A: identity + alias windows live (first translations) ----
    chk(1, scratch[0][0] == 0xB0B00000u);           // main's identity store
    chk(2, *AP(0, 0) == 0xB0B00000u);               // same PA via 4K alias
    chk(3, *MP(0, 0) == 0xB0B00000u);               // same PA via 4M mega

    // ---- seed pass cells (pages 0/1 identical; page2[4] closes the b2b pair) ----
    for (i = 0; i < 8; i++) { v = 0xB0B00000u + i; *AP(0, i) = v; *AP(1, i) = v; }
    *AP(0, 4) = 0x11110000u;  *AP(1, 4) = 0x00002222u;  *AP(2, 4) = 0x11110000u;
    *AP(0, 5) = 0x0F0F0F0Fu;  *AP(1, 5) = 0x0F0F0F0Fu;
    for (i = 0; i < 8; i++) *AP(i, 6) = (i == 0) ? 0xC0C00000u : 0xC0C00004u + i;
    for (i = 0; i < 8; i++) *AP(i, 7) = i + 1u;
    *AP(4, 0) = 6u;   *AP(4, 1) = 7u;               // G operands
    *AP(4, 2) = 84u;  *AP(4, 3) = 85u;
    *AP(4, 4) = 0x80000000u;
    *AP(6, 0) = 0xD0D00000u;  *AP(6, 1) = 0x11111111u;   // K cells
    *AP(7, 0) = 0x12345678u;  *AP(7, 1) = 0x9ABCDEF0u;   // L cells
    // byte/half geometry @ cell (pg,3): byte0=0x80, half@+2=0xC0FF
    *(volatile uint8_t *)AP(0, 3) = 0x80u;
    *(volatile uint16_t *)((uint32_t)AP(0, 3) + 2u) = 0xC0FFu;
    *(volatile uint8_t *)AP(1, 3) = 0x80u;
    *(volatile uint16_t *)((uint32_t)AP(1, 3) + 2u) = 0xC0FFu;

    // ---- 4..11: C8 subset pass (results mirror C8-T1..T5/T19/T22 classes) ----
    pass(res1, 0, 0x20u);
    for (i = 0; i < 8; i++) dram[24 + i] = res1[i];   // visible in TB dump
    {
        uint32_t ea = 0xB0B00000u, eb = 0xB0B00001u;
        chk(4, res1[0] == ((((ea + eb) ^ ea) - eb) | 0x5A5A0000u));
    }
    chk(5,  res1[1] == 0xB0B00003u);                // LW d=1 (0xB0B00002 + 1)
    chk(6,  res1[2] == 0xFFFFFF81u);                // LB d=1 (sign-extend)
    chk(7,  res1[3] == 0xFFFF81FEu);                // LH d=1 (sx16 0xC0FF *2)
    chk(8,  res1[4] == 0x00000087u);                // LBU d=1
    chk(9,  res1[5] == 0x0000C106u);                // LHU d=1
    chk(10, res1[6] == 0x11112222u);                // two cold walks b2b
    chk(11, res1[7] == 0x0F0F0F0Fu + 0x20u);        // CSR d=1

    // ---- D: store->load and store->store b2b, cold page ----
    sf_all();
    *AP(2, 0) = 0xAABBCCDDu;
    chk(12, *AP(2, 0) + 1u == 0xAABBCCDEu);         // st->ld d=1
    *AP(2, 0) = 1u;
    *AP(2, 0) = 2u;
    chk(13, *AP(2, 0) == 2u);                       // st->st same word

    // ---- E: ld->branch across 8 cold pages ----
    acc = 0;
    for (i = 0; i < 8; i++) {
        sf_all();
        v = *AP(i, 6);
        if (v == 0xC0C00000u) acc += 1u;            // only pg0 matches
        else if (v & 1u) acc += 100u;               // pg1,3,5,7
        else acc += 10u;                            // pg2,4,6
    }
    chk(14, acc == 1u + 400u + 30u);

    // ---- F: ld->jalr (function pointer from cold volatile slot) ----
    fptr_v = (uint32_t)&leaf1;
    sf_all();
    {
        uint32_t (*fp)(uint32_t) = (uint32_t (*)(uint32_t))fptr_v;
        v = *AP(3, 7);                              // cold d=1 arg
        chk(15, fp(v) == v * 3u + 1u);
    }

    // ---- G: M-ext chains on cold operands (volatile divisors) ----
    sf_all();
    {
        uint32_t x = *AP(4, 0), y = *AP(4, 1);      // 6, 7 (both cold)
        uint32_t m1 = x * y, m2 = m1 * x, m3 = m2 * y;
        chk(16, m3 == 6u * 7u * 6u * 7u);
        sf_all();
        uint32_t d7 = seven_v;
        uint32_t q = *AP(4, 2) / d7;                // 84 / 7 (real DIVU)
        uint32_t r = *AP(4, 3) % *AP(4, 1);         // 85 % 7 (real REMU)
        chk(17, q == 12u && r == 1u);
        // RISC-V div edge semantics (unpriv 7.2/Table 11: div-by-0 -> all-ones
        // quotient / dividend remainder; DIV overflow -> dividend / 0) are C UB
        // -- LLVM FOLDS them away (verified: no DIV emitted via C operators).
        // asm-where-required: real DIV/DIVU/REM/REMU through pinned regs.
        sf_all();
        uint32_t v = *AP(4, 4);                     // 0x80000000 (cold)
        uint32_t n1 = neg1_v, z0 = zero_v;
        uint32_t q1, r1, q2, r2, q3, r3, q4, r4, q5, r5, q6, r6;
        asm volatile("div  %0, %2, %3\n\t"  "rem  %1, %2, %3"
                     : "=&r"(q1), "=&r"(r1) : "r"(v), "r"(z0));
        asm volatile("div  %0, %2, %3\n\t"  "rem  %1, %2, %3"
                     : "=&r"(q2), "=&r"(r2) : "r"(v), "r"(n1));
        asm volatile("divu %0, %2, %3\n\t"  "remu %1, %2, %3"
                     : "=&r"(q3), "=&r"(r3) : "r"(v), "r"(z0));
        asm volatile("divu %0, %2, %3\n\t"  "remu %1, %2, %3"
                     : "=&r"(q4), "=&r"(r4) : "r"(v), "r"(n1));
        asm volatile("div  %0, %2, %3\n\t"  "rem  %1, %2, %3"
                     : "=&r"(q5), "=&r"(r5) : "r"(d7), "r"(n1));
        asm volatile("div  %0, %2, %3\n\t"  "rem  %1, %2, %3"
                     : "=&r"(q6), "=&r"(r6) : "r"(z0), "r"(d7));
        chk(18, q1 == 0xFFFFFFFFu);                 // DIV by 0: all-ones
        chk(19, r1 == v);                           // DIV by 0: dividend
        chk(20, q2 == 0x80000000u && r2 == 0u);     // DIV overflow edge
        chk(21, q3 == 0xFFFFFFFFu && r3 == v);      // DIVU/REMU by 0
        chk(22, q4 == 0u && r4 == v);               // DIVU 2^31 / (2^32-1)
        chk(23, q5 == 0u - 7u && r5 == 0u);         // signed 7/-1 (normal path)
        chk(24, q6 == 0u && r6 == 0u);              // 0/7
    }

    // ---- I: backward loop, accum under repeated walks (C8-T20 class) ----
    acc = 0;
    for (i = 0; i < 32; i++) {
        uint32_t pg = i & 7u;
        if ((i & 3u) == 0) sf_all();                // walk every 4th load
        acc += *AP(pg, 7);
        acc ^= (acc << 1);                          // dependent ALU
    }
    {
        uint32_t a2 = 0;
        for (i = 0; i < 32; i++) { a2 += (i & 7u) + 1u; a2 ^= (a2 << 1); }
        chk(25, acc == a2);
    }

    // ---- J: alias triangle coherence (identity / 4K / mega, b2b) ----
    sf_all();
    scratch[5][9] = 0xDEAD0001u;                    // identity write
    chk(26, *AP(5, 9) == 0xDEAD0001u && *MP(5, 9) == 0xDEAD0001u);
    *MP(5, 9) = 0xC0FFEE02u;                        // mega alias write
    chk(27, scratch[5][9] == 0xC0FFEE02u && *AP(5, 9) == 0xC0FFEE02u);
    v = *AP(5, 9);                                  // 4K read
    scratch[5][9] = v + 1u;                         // identity d=1 RMW mix
    chk(28, *MP(5, 9) == 0xC0FFEE03u);

    // ---- K: same-reg b2b overwrite chain (C8-T21) ----
    sf_all();
    v = *AP(6, 0);
    v = v + 3u; v = v ^ 0xF0F0F0F0u; v = v << 2u; v = v | (*AP(6, 1) & 1u);
    chk(29, v == ((((0xD0D00000u + 3u) ^ 0xF0F0F0F0u) << 2u) | 1u));

    // ---- L: varied ALU d=1 mix (C8-T22) ----
    sf_all();
    {
        uint32_t p = *AP(7, 0), q = *AP(7, 1);
        uint32_t t = p - q; t ^= q; t += p; t >>= 3; t |= (q << 1);
        chk(30, t == (((((0x12345678u - 0x9ABCDEF0u) ^ 0x9ABCDEF0u)
                       + 0x12345678u) >> 3) | (0x9ABCDEF0u << 1)));
    }

    // ---- M/N: I-side ROM alias calls (I-walk + refill mid call-chain) ----
    {
        uint32_t (*g1)(uint32_t) = (uint32_t (*)(uint32_t))rom_alias((uint32_t)&leaf1);
        uint32_t (*g2)(uint32_t) = (uint32_t (*)(uint32_t))rom_alias((uint32_t)&leaf2);
        uint32_t (*g3)(uint32_t) = (uint32_t (*)(uint32_t))rom_alias((uint32_t)&leaf3);
        uint32_t (*g4)(uint32_t) = (uint32_t (*)(uint32_t))rom_alias((uint32_t)&leaf4);
        arg = *AP(2, 5) & 0xFFu;                    // arbitrary stable arg
        sf_all();
        chk(31, g1(arg) == arg * 3u + 1u);
        sf_all();
        chk(32, g2(arg) == ((arg * 3u + 1u) ^ 0xA5A5A5A5u));
        sf_all();
        chk(33, g4(5u) == 55u);                     // mul+add loop fetched aliased
        sf_all();
        chk(34, g3(arg) == (((arg + 1u) * 3u + 1u) ^ 0xA5A5A5A5u) - 2u);
        sf_all();
        chk(35, g2(g1(arg)) == (((arg * 3u + 1u) * 3u + 1u) ^ 0xA5A5A5A5u));
    }

    // ---- O: asid1 re-run (satp switch, TLB re-key, same maps) ----
    sf_all();
    csr_write(CSR_SATP, SATP_MODE | (1u << 22) | PPN(pt1));
    pass(res2, 1, 0x20u);
    for (i = 0; i < 8; i++) chk(36 + i, res2[i] == res1[i]);    // 36..43
    csr_write(CSR_SATP, SATP_MODE | PPN(pt0));
    sf_all();

    // ---- post-asid warm access ----
    chk(44, *AP(1, 0) == 0xB0B00000u && res1[0] == res2[0]);

    // ---- no traps were taken ----
    chk(45, dram[16] == 0u && dram[20] == 0u);

    fw_done();
    for (;;) ;
}

// ============================================================ main (M)
int main(void)
{
    uint32_t i;

    for (i = 16; i <= 23; i++) dram[i] = 0;         // belt (X-init discipline)
    fptr_v = 0; zero_v = 0; seven_v = 7; neg1_v = 0xFFFFFFFFu;
    for (i = 0; i < 8; i++) { res1[i] = 0; res2[i] = 0; }
    scratch[0][0] = 0xB0B00000u;                    // seed chk1/2/3

    build_tables();

    csr_write(CSR_MTVEC, (uint32_t)&m_shim);
    csr_write(CSR_STVEC, (uint32_t)&s_shim);
    csr_write(CSR_MEDELEG, 0);
    csr_write(CSR_MIDELEG, 0);
    csr_write(CSR_MIE, 0);

    csr_write(CSR_SATP, SATP_MODE | PPN(pt0));      // M ignores satp
    chk(1, scratch[0][0] == 0xB0B00000u);           // ... identity still works

    csr_write(CSR_MEPC, (uint32_t)s_main);
    csr_write(CSR_MSTATUS, (csr_read(CSR_MSTATUS) & ~MS_MPP_M) | MS_MPP_S);
    mret();
    for (;;) ;
    return 0;
}
