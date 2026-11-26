// t03_intr.c -- interrupts through the real PLIC/CLINT (design.md sec.10 C)
//
// Delivery path under test: CLINT level -> PLIC gateway (level-latched IP)
// -> per-ctx enable/priority/threshold -> EIP -> core meip/seip -> handler
// (claim / clear source / complete). Real PLIC + CLINT in the SoC; the TB
// only provides the UART txd->rxd loopback (run with +LOOP).
//
// Cause is constant per context (ALL sources route via PLIC):
//   M: 0x8000_000B (machine external)    S: 0x8000_0009 (supervisor ext.)
// Source identity = claimed ID: 1=UART-RX 2=S-sw(sstip) 3=M-sw(msip)
//   6=S-timer(stimecmp) 7=M-timer(mtimecmp) 10=S-ext 11=M-ext (CLINT mtip
// fans out to BOTH 10 and 11).
//
// PLIC enable words: bit N = source N. M sources {1,3,7,11} = 0x88A;
// S sources {2,6,10} = 0x444. Prio: p1=1 p2=2 p3=5 p6=1 p7=1 p10=1 p11=8.
//
// DRAM word map (dram[] = 0x8000_0000):
//   [16] mcause_last  [17] mepc_last  [18] mtval_last  [19] m_count
//   [20] m_hist_n;    M hist entry n: [22+3n] = id, [23+3n] = epc
//   [100] s_count  [101] s_hist_n;  S hist entry n: [103+3n] = id,
//          [104+3n] = epc
//   [98] UART RX byte  [99] msip_mode: 0=normal 1=no-clear 2=S-return
//   [102] mtime_mode: 3=mret-into(msip at mret) 4=mret-M(MPP=M, after_s)
//
// C framework: trap entry = fw_traps.S (m_shim/s_shim) -> m_hnd_c/s_hnd_c
// (full C ABI; the interrupted context is restored bit-exactly, so being
// interrupted mid-sequence is safe -- the retired .S version preserved
// only x1/x2/x3 and its P9 bug was exactly a clobbered x15).
//
// Each waited-on interrupt is taken inside the phase function's OWN spin
// loop (macro WM, noinline phases) so an epc check is a range [fn, fn+SPAN)
// around that function -- a shared wait helper would put every epc inside
// the helper, making the range checks meaningless. The .S observation
// points are preserved: e.g. P1 = p1a (arm, WM(1), settle busy) then
// checks, then p1b (rearm, WM(2)). Same 61-check sequence as the .S
// version; two deliberate deltas, same coverage:
//   - P9 S window rebuilt for determinism: stale ctx1 pending drained via
//     CLAIM1 in main; msip armed inside s_phase (M#12 epc there, mode-2
//     mret re-executes the SIE csrs); S timer + MTIP armed INSIDE s_wait2
//     -> S-hist {2,6,10}: S1 sstip, S2/S3 = src6/src10 pending together at
//     the claim (prio 1 tie -> low ID first), epc checks prove timing.
//   - epc point-checks (+4/+8 of the .S wait loops) become these ranges.
//     SPANs are verified non-overlapping against the linked ELF (see
//     scripts/fw_ranges.py -- rule 2: tool-check the layout, never assume).
#include "lib.h"

#define PLIC_EN0    0x0C002000u
#define PLIC_EN1    0x0C002080u
#define PLIC_THR0   0x0C200000u
#define PLIC_THR1   0x0C201000u
#define PLIC_CLAIM0 0x0C200004u
#define PLIC_CLAIM1 0x0C201004u
#define CL_MSIP      0x02000000u
#define CL_SSTIP    0x02000004u
#define CL_MTC_LO   0x02004000u  // mtimecmp[31:0]
#define CL_MTC_OV   0x02004004u  // mtimecmp[63:32] | stimecmp[31:0] (overlap)
#define CL_STC_HI   0x02004008u  // stimecmp[63:32]
#define CL_MTIP     0x02007000u
#define CL_MTIME    0x0200BFF8u
#define UART_THR    0x10000000u
#define UART_IER    0x10000001u

// Stacks/save-areas (layout/offsets fixed in fw_entry.S / fw_traps.S)
uint32_t _stk_main[1024] __attribute__((aligned(16)));
uint32_t _stk_m[1024]    __attribute__((aligned(16)));
uint32_t _stk_s[1024]    __attribute__((aligned(16)));
uint32_t _sav_m[16]     __attribute__((aligned(16)));
uint32_t _sav_s[16]     __attribute__((aligned(16)));

// The DRAM scoreboard, linked FIRST at exactly 0x8000_0000 (scripts/link.lds)
__attribute__((section(".data.dram"))) volatile uint32_t dram[1024];

static void s_phase(void);
static void after_s(void) __attribute__((noinline));
static void p10(void);
static void p1a(void) __attribute__((noinline));
static void p1b(void) __attribute__((noinline));
static void s_wait2(void) __attribute__((noinline));
static void s_wait3(void) __attribute__((noinline));

// Spin for m_count == T INSIDE the calling phase (epc range = that fn).
#define WM(T) do { for (;;) if (dram[19] == (T)) break; } while (0)

// ============================================================ handlers
// m_hnd_c: log, claim(CL0), dispatch by id, complete; the shim does mret.
void m_hnd_c(void)
{
    uint32_t mcause = csr_read(0x342);
    uint32_t mepc   = csr_read(0x341);
    uint32_t mtval  = csr_read(0x343);
    uint32_t n, id;

    dram[16] = mcause;
    dram[17] = mepc;
    dram[18] = mtval;
    dram[19] = dram[19] + 1;
    n = dram[20];
    dram[20] = n + 1;

    id = rd32(PLIC_CLAIM0);                 // claim -> ID (clears IP)
    dram[22 + 3 * n] = id;
    dram[23 + 3 * n] = mepc;
    switch (id) {
    case 1:                                 // UART RX: pop one byte
        dram[98] = rd32(UART_THR) & 0xFFu;
        wr32(PLIC_CLAIM0, 1);              // complete(1)
        break;
    case 3: {                              // M software (msip)
        uint32_t mode = dram[99];
        if (mode == 2) {                   // S-return: MPP=S, mret to the S PC
            dram[99] = 0;
            csr_set(0x300, 0x800);
            wr32(CL_MSIP, 0);
            wr32(PLIC_CLAIM0, 3);
        } else if (mode == 1) {            // no-clear: msip LEFT -> re-arm
            dram[99] = 0;
            wr32(PLIC_CLAIM0, 3);
        } else {
            wr32(CL_MSIP, 0);
            wr32(PLIC_CLAIM0, 3);
        }
        break;
    }
    case 7: {                              // M timer (mtimecmp)
        uint32_t mode = dram[102];
        if (mode == 3) {                   // mret-into-interrupt
            wr32(CL_MTC_LO, ~0u);          // clear (hi=0)
            wr32(PLIC_CLAIM0, 7);
            dram[102] = 0;
            wr32(CL_MSIP, 1);              // pending at the mret
            // NB: do NOT set MIE here -- mid-handler re-entrance corrupts
            // the shim (mepc overwritten). mret restores MIE<-MPIE (MPIE=1
            // from entry), so the src3 trap lands on the first p10 retire.
        } else if (mode == 4) {            // mret-M: MPP=M, mepc=after_s
            wr32(CL_MTC_LO, ~0u);
            wr32(PLIC_CLAIM0, 7);
            dram[102] = 0;
            csr_set(0x300, 0x1800);
            csr_write(0x341, (uint32_t)after_s);
        } else {
            wr32(CL_MTC_LO, ~0u);
            wr32(PLIC_CLAIM0, 7);
            dram[102] = 0;
        }
        break;
    }
    case 11:                               // M external (mtip)
        wr32(CL_MTIP, 0);
        wr32(PLIC_CLAIM0, 11);
        break;
    default:
        break;
    }
}

// s_hnd_c: log, claim(CL1), dispatch by id (2/6/10), complete; shim srets.
void s_hnd_c(void)
{
    uint32_t sepc = csr_read(0x141);
    uint32_t n, id;

    csr_read(0x142);                      // scause (read for completeness)
    csr_read(0x143);                      // stval

    dram[100] = dram[100] + 1;
    n = dram[101];
    dram[101] = n + 1;

    id = rd32(PLIC_CLAIM1);               // claim ctx1 -> ID
    dram[103 + 3 * n] = id;
    dram[104 + 3 * n] = sepc;
    switch (id) {
    case 2:
        wr32(CL_SSTIP, 0);
        wr32(PLIC_CLAIM1, 2);
        break;
    case 6:
        wr32(CL_STC_HI, ~0u);             // stimecmp_hi = -1 (clear)
        wr32(PLIC_CLAIM1, 6);
        break;
    case 10:
        wr32(CL_MTIP, 0);
        wr32(PLIC_CLAIM1, 10);
        break;
    default:
        break;
    }
}

// Wait loops are inline in the phases (WM). S2+S3 land in s_wait2: src10
// IP-latches at the MTIP WB (~t+21), src6 at stimecmp (t+60). Both are
// pending when the trap is taken -> prio tie (1=1) -> claim returns LOW ID
// 6 first, then 10: hist {2,6,10}. Tie-break itself is proven in P3.
static void s_wait2(void)
{
    uint32_t t = rd32(CL_MTIME);
    uint32_t i;
    wr32(CL_MTC_OV, t + 60);              // stimecmp_lo = t+60; mtimecmp_hi = t+60 (M far)
    wr32(CL_STC_HI, 0);                   // stimecmp DONE -> src6 fires ~60 ticks later
    wr32(CL_MTIP, 1);                     // src10(S)+src11(M, ctx0-disabled) IP latch
    // 3-instr loop (addi; lw; bne): the lw retires on a non-branch cycle,
    // the only kind int_fire accepts (see s_phase NB).
    for (i = 0; i != 0xFFFFFFFu; i++)
        if (dram[100] >= 3) return;
}

// s_wait3: spin that nothing terminates -- the M-timer entry (src7, mode 4)
// diverts it. Data-dependent loop (NOT `for(;;);` = a bare j . whose mepc
// would sit one past the fn): epc lands on loop+4/+8, inside the fn.
static void s_wait3(void)
{ for (;;) if (dram[19] == 0xFFFFu) break; }

// S-mode phase (entered via mret from main; MPP=S). msip is set HERE: M#12
// (src3) fires as the next commit (M beats S) with epc = csrs+4 -> mode 2
// mret re-executes the csrs; its retire (SIE=1, no branch) is the S1 entry.
// -> wait S1 -> s_wait2 (arms stimecmp + MTIP) -> arm M timer (mtimecmp =
// mtime+30; mode 4 retargets to after_s in M mode) -> s_wait3: diverted.
// NB: s_wait2's spin must be 3 distinct instructions (lw; op; bne): int_fire
// needs a NON-branch clean retire (spec: no branch that cycle); a 2-instr
// lw+bnez loop only commits branches, which masks seip for a whole deadline
// window (observed: src10 latched at t+21, S entry delayed to t+60).
static void s_phase(void)
{
    uint32_t t;

    csr_set(0x100, 2);                   // sstatus.SIE = 1 (SIE gate)
    wr32(CL_MSIP, 1);                    // M-from-S (src3): taken right here
    while (dram[100] < 1) ;             // S1 (sstip) consumed
    s_wait2();                          // S2+S3: src10 & src6 (order = timing)
    // Arm M timer (S->M return): mtimecmp = mtime + 30
    t = rd32(CL_MTIME);
    wr32(CL_MTC_OV, 0);                 // mtimecmp_hi=0 + stimecmp_lo=0 (S stays clear)
    wr32(CL_MTC_LO, t + 30);
    s_wait3();                          // (diverted by M entry, src7)
}

// M mode after the S window (entered via mret from the M-timer handler,
// mode 4). Restore EN0 -> the src11 IP latched while ctx0-disabled claims
// once (n=14). Then P10: mret-into-interrupt.
static void after_s(void)
{
    wr32(PLIC_EN0, 0x88Au);
    WM(15);

    chk(38, dram[100] == 3);                              // s_count == 3
    chk(39, dram[103] == 2);
    chk(40, dram[106] == 6);      // S2: stimecmp src6 (prio tie w/ src10 ->
    chk(41, dram[109] == 10);     //        lower ID first; timing proven by epc)
    chk(42, in_range(dram[104], (uint32_t)s_phase, 0x44));   // S1 after SIE=1
    chk(43, in_range(dram[107], (uint32_t)s_wait2, 0x38));   // S2 in s_wait2
    chk(44, in_range(dram[110], (uint32_t)s_wait2, 0x38));   // S3 in s_wait2
    chk(45, dram[19] == 15);
    chk(46, dram[22 + 3 * 12] == 3);                    // M-from-S: taken in S
    chk(47, in_range(dram[23 + 3 * 12], (uint32_t)s_phase, 0x44));
    chk(48, dram[22 + 3 * 13] == 7);                    // S->M via M timer
    chk(49, in_range(dram[23 + 3 * 13], (uint32_t)s_wait3, 0x18));
    chk(50, dram[22 + 3 * 14] == 11);                  // IP latched while dis
    chk(51, in_range(dram[23 + 3 * 14], (uint32_t)after_s, 0x60));
    busy(200);
    chk(52, dram[100] == 3);                           // S sources: stable
    chk(53, dram[19] == 15);

    p10();
    busy(200);
    chk(59, dram[19] == 17);                          // no refire

    // ---- final: nothing pending anywhere
    chk(60, rd32(PLIC_CLAIM0) == 0);                 // claim ctx0 empty
    chk(61, rd32(PLIC_CLAIM1) == 0);                 // claim ctx1 empty
    fw_done();
}

// P10: mret-into-interrupt (msip pending at the mret of the src7 handler).
static void p10(void)
{
    uint32_t t3;

    dram[102] = 3;
    t3 = rd32(CL_MTIME);
    wr32(CL_MTC_OV, 0);                               // S stays clear (stc_lo=0)
    wr32(CL_MTC_LO, t3 + 100);
    WM(17);
    chk(54, dram[19] == 17);
    chk(55, dram[22 + 3 * 15] == 7);
    chk(56, dram[22 + 3 * 16] == 3);                 // taken right after mret
    chk(57, in_range(dram[23 + 3 * 15], (uint32_t)p10, 0x60));
    chk(58, in_range(dram[23 + 3 * 16], (uint32_t)p10, 0x60));
}

// Phase arms: each fn does arm + wait (+settle) so epc checks are ranges
// around the fn. (chk() of results happens in main/after_s afterwards.)
// P1 needs two observation points (fire #1 + no-refire, then rearm fire #2)
// -> split p1a/p1b so each wait loop lives in its own noinline fn.
static void p1a(void)
{
    uint32_t t0 = rd32(CL_MTIME);
    wr32(CL_MTC_OV, 0);                              // stimecmp_lo=0 (S stays clear)
    wr32(CL_MTC_LO, t0 + 100);
    WM(1);
    busy(200);                                       // no refire (cleared)
}
static void p1b(void)
{
    uint32_t t0 = rd32(CL_MTIME);
    wr32(CL_MTC_LO, t0 + 100);                       // rearm
    WM(2);
}
static void p2(void)
{
    wr32(CL_MSIP, 1);
    WM(3);
    busy(200);
}
static void p3(void)                                 // tie -> low ID (3,11)
{
    csr_clear(0x300, 8);                            // MIE=0: arm atomically
    wr32(0x0C00000Cu, 5);                           // prio(3)=5
    wr32(0x0C00002Cu, 5);                           // prio(11)=5 (tie)
    wr32(CL_MSIP, 1);
    wr32(CL_MTIP, 1);
    csr_set(0x300, 8);
    WM(5);
    busy(200);
}
static void p4(void)                                 // prio desc (11,3)
{
    csr_clear(0x300, 8);
    wr32(0x0C00000Cu, 5);
    wr32(0x0C00002Cu, 8);                           // prio(11)=8
    wr32(CL_MSIP, 1);
    wr32(CL_MTIP, 1);
    csr_set(0x300, 8);
    WM(7);
    busy(200);
}
static void p5(void)                                 // threshold
{
    wr32(PLIC_THR0, 5);
    wr32(CL_MSIP, 1);                               // prio 5 <= 5: not notified
    wr32(CL_MTIP, 1);                               // prio 8 > 5: notified
    WM(8);
    busy(200);
    chk(23, rd32(PLIC_CLAIM0) == 3);                // claim ignores threshold
    wr32(CL_MSIP, 0);                               // level low BEFORE complete
    wr32(PLIC_CLAIM0, 3);                          // complete(3): no re-pend
    wr32(PLIC_THR0, 0);
    busy(200);
}
// p6 SPLIT at the .S check interleave: chk25 observes count==8 BETWEEN the
// settle (disabled, no fire) and the re-enable (entry8, id 11).
static void p6a(void)
{
    csr_clear(0x300, 8);                           // MIE=0: latch+disable atomic
    wr32(CL_MTIP, 1);                               // src11 level -> IP latches
    wr32(PLIC_EN0, 0x8Au);                         // EN0 without bit11
    busy(50);                                      // MIE still 0: EN0 writeback MUST
                                                   //   land before MIE=1 (IP11 latches
                                                   //   at the MTIP WB with bit11 still
                                                   //   enabled; .S didn't race: its
                                                   //   csrrsi retired after both WBs)
    csr_set(0x300, 8);
    busy(200);                                     // disabled: no fire
}
static void p6b(void)
{
    wr32(PLIC_EN0, 0x88Au);                         // re-enable -> fires (entry8)
    WM(9);
    busy(200);
}
static void p7(void)                                 // complete re-arms (level held)
{
    dram[99] = 1;                                 // mode 1: no clear of msip
    wr32(CL_MSIP, 1);
    WM(11);
    busy(200);
}
static void p8(void)                                 // UART RX irq (loopback)
{
    wr8(UART_THR, 0x2Au);                         // TX -> TB loops txd->rxd -> RX
    WM(12);
    busy(200);
}

int main(void)
{
    uint32_t i;

    // ---- zero init (wb_ram is X at reset) ----
    // MUST cover every watched word: s_count [100] X makes "< 1" false and
    // ">= 3" false -> waits race/silently-hang (livelock, no fw_fail).
    for (i = 16; i <= 115; i++) dram[i] = 0;

    // ---- vectors ----
    csr_write(0x305, (uint32_t)&m_shim);          // mtvec
    csr_write(0x105, (uint32_t)&s_shim);          // stvec

    // ---- PLIC: priorities (word N @ 0x0C00_0004*N) ----
    wr32(0x0C000004u, 1);                          // prio(1)=1
    wr32(0x0C000008u, 2);                          // prio(2)=2
    wr32(0x0C00000Cu, 5);                          // prio(3)=5
    wr32(0x0C000018u, 1);                          // prio(6)=1
    wr32(0x0C00001Cu, 1);                          // prio(7)=1
    wr32(0x0C000028u, 1);                          // prio(10)=1
    wr32(0x0C00002Cu, 8);                          // prio(11)=8
    // ---- PLIC: enables (bit N = source N) + thresholds ----
    wr32(PLIC_EN0, 0x88Au);                       // ctx0=M: src 1,3,7,11
    wr32(PLIC_EN1, 0x444u);                       // ctx1=S: src 2,6,10
    wr32(PLIC_THR0, 0);
    wr32(PLIC_THR1, 0);
    // ---- UART: IER = 1 (DR irq; bit1 THRE never set) ----
    wr8(UART_IER, 1);
    // ---- CSR: enables, then MIE=1 (intake from here on) ----
    csr_write(0x304, 0xA00u);                     // mie: MEIP(b11)|SEIP(b9)
    csr_write(0x303, 0x200u);                     // mideleg: SEIP -> S
    csr_write(0x104, 0x200u);                     // sie: SEIE
    csr_set(0x300, 8);                            // mstatus.MIE = 1

    // ================================================= P1: M timer (src7)
    // (p1a: fire #1 + settle; checks 1-5 observe count==1 before the rearm)
    p1a();
    chk(1, dram[19] == 1);
    chk(2, dram[16] == 0x8000000Bu);
    chk(3, in_range(dram[17], (uint32_t)p1a, 0x40));   // epc in p1a wait
    chk(4, dram[18] == 0);
    chk(5, dram[19] == 1);                              // no refire (cleared)
    p1b();
    chk(6, dram[19] == 2);
    chk(7, in_range(dram[17], (uint32_t)p1b, 0x28));   // 2nd fire in p1b wait

    // ============================================ P2: M software (src3 msip)
    p2();
    chk(8, dram[19] == 3);
    chk(9, dram[16] == 0x8000000Bu);
    chk(10, dram[22 + 3 * 2] == 3);
    chk(11, dram[19] == 3);                            // no refire (cleared)

    // ============================ P3: tie -> lower ID first (3 before 11)
    p3();
    chk(12, dram[19] == 5);
    chk(13, dram[22 + 3 * 3] == 3);
    chk(14, dram[22 + 3 * 4] == 11);
    chk(15, dram[19] == 5);

    // ============================================ P4: priority desc (11,3)
    p4();
    chk(16, dram[19] == 7);
    chk(17, dram[22 + 3 * 5] == 11);                  // higher prio first
    chk(18, dram[22 + 3 * 6] == 3);
    chk(19, dram[19] == 7);

    // ===================== P5: threshold: EIP needs prio>thr (20-22 in p5)
    p5();
    chk(20, dram[19] == 8);                           // only src11 notified
    chk(21, dram[22 + 3 * 7] == 11);
    chk(22, dram[19] == 8);                          // src3 pending, not notf
    chk(24, dram[19] == 8);                          // stable after claim

    // ============================ P6: per-ctx enable gate (src11 ctx0)
    p6a();
    chk(25, dram[19] == 8);                          // disabled: no fire
    p6b();
    chk(26, dram[19] == 9);
    chk(27, dram[22 + 3 * 8] == 11);
    chk(28, dram[19] == 9);

    // ====================== P7: complete re-arms gateway (level held)
    p7();
    chk(29, dram[19] == 11);                         // two entries (re-arm)
    chk(30, dram[22 + 3 * 9] == 3);
    chk(31, dram[22 + 3 * 10] == 3);
    chk(32, dram[19] == 11);

    // ======================================= P8: UART RX irq (src1, loopback)
    p8();
    chk(33, dram[19] == 12);
    chk(34, dram[16] == 0x8000000Bu);
    chk(35, dram[22 + 3 * 11] == 1);
    chk(36, dram[98] == 0x2Au);
    chk(37, dram[19] == 12);                        // deasserts when empty

    // ================================================= P9: S phase (nested)
    // Drain stale ctx1 pending (src10 IP latched by P3-P6 MTIP arms; S was
    // never entered so they were never claimed) BEFORE arming anything.
    while ((i = rd32(PLIC_CLAIM1)) != 0) wr32(PLIC_CLAIM1, i);
    // Disable src11 in ctx0 for the S window (its IP latches anyway from
    // s_wait2's MTIP: the gateway is per-source; claimed after the window
    // -> proof, chk50). msip is armed INSIDE s_phase (M#12, epc there).
    wr32(PLIC_EN0, 0x8Au);
    wr32(CL_SSTIP, 1);                               // sstip = 1 (src2)
    dram[99] = 2;                                    // msip_mode: S-return
    dram[102] = 4;                                   // mtime_mode: mret-M
    csr_clear(0x100, 2);                            // sstatus.SIE = 0 at S entry
    csr_set(0x300, 0x800);                          // MPP = S
    csr_write(0x341, (uint32_t)s_phase);
    mret();                                         // -> s_phase (S mode)
    for (;;) ;                                      // unreachable
    return 0;
}
