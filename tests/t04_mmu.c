// t04_mmu.c -- Sv32 MMU on the full SoC (design.md sec.10 D)
//
// Real page walks through wb_ic master m2 against wb_ram, I+D translation in
// the live pipeline, TLB (L1 4-way set VA[13:12] / L2 8-way set VA[16:12],
// asid+global) behavior, sfence.vma classes, MPRV, S/U privilege matrix,
// permission + A/D + structural faults, 4K / 4M pages, alias-coherent PA
// caches, satp enable/clear. (Perms/TLB unit-verified in tb_mmu M1-M13 and
// core C7; this is the integration battery.)
//
// Traps: page faults (12/13/15) + ebreak(3) are medeleg'd to S. The S
// handler records (cause, stval, sepc) then: 13/15 -> sepc += 4 (skips the
// faulting load/store); 12 -> sepc = g_abort ? g_abort : _sav_s[1] (saved ra
// of the faulting CALL -> returns to caller); 3 -> sepc = g_abort + SPP <- S
// (U-mode exit). ecall stays in M (cause 8 from U -> mret back to U; 9 from
// S -> mret to M). mie/sie CSRs = 0 throughout (synchronous-only); mstatus.MIE
// does become 1 via the mret chain (MIE<-MPIE, spec 3.1.7) -- harmless, nothing
// is enabled in mie/sie.
// MPRV: trap entry overwrites MPP<-M and mret clears MPRV -> EVERY translated
// M access re-arms (MPP=S,MPRV) first; the m_shim itself runs with MPP=M so
// its DRAM save-area writes bypass translation (never re-fault).
//
// VA map (root pt0 @ dynamic PA; pt1 = asid=1 clone, differs at [0x1C0]):
//   0x00000000 4M -> PA 0 R W X A D   identity: ROM+CLINT+PLIC+UART
//   0x40000000     V=0                pf battery + satp=0 identity read (null)
//   0x50000000 ptr -> l0a (perm matrix)
//   0x60000000 4M -> PA 0x80000000 R W X A D   (DRAM mega alias)
//   0x70000000 ptr -> l0b / l0b1 (alias + sfence/asid batteries)
//   0x80000000 4M -> PA 0x80000000 R W X A D   (DRAM identity: stacks/PTs)
//   0x90000000 4M leaf PPN[9:0]!=0             (structural fault)
//   0xC0000000     V=0                M null read 0; S/MPRV pf 13
// L1-set spread (set = VA[13:12]) keeps the asid-survival entry (VA_AL3,
// set 3) from being evicted by the asid1 fill: set3 4K entries = {U,RF,AL3}.
// l0a: [0]RO-A-D [1]RW no-D [2]RWX no-A [3]RWXUAD [4]X-only-AD [5]RWXAD
//      [6]XU-AD blob [7]RO-AD (fetch tgt) [8]RWXAD-G [9]nonleaf+A [10]R0W1
// l0b: [1]=pgB [3]=pgA [6]=pgC2; l0b1 (asid1): [3]=pgF.
// U blob offsets: lw-S@0x10, ecall@0x14, ebreak@0x24 (llvm-mc encoded).
//
// DRAM scoreboard: [16]=m_n [17]=m_cause [18]=m_tval [19]=m_epc
//                  [20]=s_n [21]=s_cause [22]=s_tval [23]=s_epc
//                  [24]=0 (asid-operand source) [30]=abort target (vis)
#include "lib.h"

#define CSR_SSTATUS  0x100
#define CSR_STVEC    0x105
#define CSR_SEPC     0x141
#define CSR_SCAUSE   0x142
#define CSR_STVAL    0x143
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
#define MS_MPRV   0x20000u
#define MS_SUM    0x40000u
#define MS_MXR    0x80000u
#define SS_SPP    0x100u

#define SATP_MODE 0x80000000u

// VA bases (all target arrays aligned(4096) => in-page offset 0)
#define VA_BAD   0x40000000u
#define VA_MEGA  0x60000000u
#define VA_BDM   0x90000000u
#define VA_NULL  0xC0000000u
#define VA_RO    0x50000000u
#define VA_D0    0x50001000u
#define VA_A0    0x50002000u
#define VA_U     0x50003000u
#define VA_XC    0x50004000u
#define VA_S     0x50005000u
#define VA_UC    0x50006000u
#define VA_RF    0x50007000u
#define VA_G     0x50008000u
#define VA_NL    0x50009000u
#define VA_R0W1  0x5000A000u
#define VA_AL1   0x70001000u
#define VA_AL3   0x70003000u
#define VA_AL6   0x70006000u

#define PPN(pa)   ((((uint32_t)(pa)) >> 12) & 0x3FFFFFu)
#define F_V 1u
#define F_R 2u
#define F_W 4u
#define F_X 8u
#define F_U 16u
#define F_G 32u
#define F_A 64u
#define F_D 128u
static uint32_t mkpte(uint32_t ppn, uint32_t f) { return (ppn << 10) | f; }

// Stacks/save-areas (layout fixed in fw_entry.S / fw_traps.S)
uint32_t _stk_main[1024] __attribute__((aligned(16)));
uint32_t _stk_m[1024]    __attribute__((aligned(16)));
uint32_t _stk_s[1024]    __attribute__((aligned(16)));
uint32_t _sav_m[16]     __attribute__((aligned(16)));
uint32_t _sav_s[16]     __attribute__((aligned(16)));

__attribute__((section(".data.dram"))) volatile uint32_t dram[1024];

// page tables
uint32_t pt0[1024]  __attribute__((aligned(4096)));
uint32_t pt1[1024]  __attribute__((aligned(4096)));
uint32_t l0a[1024]  __attribute__((aligned(4096)));
uint32_t l0b[1024]  __attribute__((aligned(4096)));
uint32_t l0b1[1024] __attribute__((aligned(4096)));

// data/code pages
uint32_t pgRO[2]  __attribute__((aligned(4096)));
uint32_t pgD0[2]  __attribute__((aligned(4096)));
uint32_t pgA0[2]  __attribute__((aligned(4096)));
uint32_t pgU[4]   __attribute__((aligned(4096)));
uint32_t pgS[4]   __attribute__((aligned(4096)));
uint32_t pgXC[4]  __attribute__((aligned(4096)));
uint32_t pgUC[16] __attribute__((aligned(4096)));
uint32_t pgG1[2]  __attribute__((aligned(4096)));
uint32_t pgG2[2]  __attribute__((aligned(4096)));
uint32_t pgA[2]   __attribute__((aligned(4096)));
uint32_t pgB[2]   __attribute__((aligned(4096)));
uint32_t pgC2[2]  __attribute__((aligned(4096)));
uint32_t pgD[2]   __attribute__((aligned(4096)));
uint32_t pgF[2]   __attribute__((aligned(4096)));
uint32_t pgNL[2]  __attribute__((aligned(4096)));

static volatile uint32_t g_abort;   // U-abort / ebreak exit target

// U blob (llvm-mc encoded, see tb note below): offsets fixed
static const uint32_t uc_blob[11] = {
    0x50003537u,  // lui   a0, 0x50003
    0x5A500593u,  // li    a1, 0x5a5
    0x00B52023u,  // sw    a1, 0(a0)     U store -> pgU[0]
    0x50005637u,  // lui   a2, 0x50005
    0x00062683u,  // lw    a3, 0(a2)     -> pf 13 (U loads S page), skip
    0x00000073u,  // ecall               -> M (+4, mret back to U)
    0x00052783u,  // lw    a5, 0(a0)
    0x00178793u,  // addi  a5, a5, 1
    0x00F52223u,  // sw    a5, 4(a0)     -> pgU[1]
    0x00100073u,  // ebreak              -> S abort-exit
    0x0000006Fu,  // j     .             (unreachable)
};
#define UC_ECALL  0x14u
#define UC_EBREAK 0x24u

// sfence.vma has no C spelling -- inline asm (necessary-minimum .S surface).
// rs1/rs2 are REGISTER slots: pass runtime-opaque values so a 0 never lands
// in x0 (which the RTL decodes as the all-/asid-class selector).
static void sf_all(void) { asm volatile("sfence.vma" ::: "memory"); }
static void sf_asid(uint32_t a) { asm volatile("sfence.vma x0, %0" :: "r"(a) : "memory"); }
static void sf_page(uint32_t v) { asm volatile("sfence.vma %0, x0" :: "r"(v) : "memory"); }

static void s_main(void) __attribute__((noreturn));

// ============================================================ handlers
void m_hnd_c(void)
{
    uint32_t c = csr_read(CSR_MCAUSE);
    uint32_t e = csr_read(CSR_MEPC);

    dram[16] = dram[16] + 1;
    dram[17] = c;
    dram[18] = csr_read(CSR_MTVAL);
    dram[19] = e;

    if (c == 9) {                       // ecall from S -> exit to M at +4
        csr_write(CSR_MEPC, e + 4);
        csr_set(CSR_MSTATUS, MS_MPP_M);
    } else {                            // 8 (U ecall) / 12,13,15 (MPRV): +4
        csr_write(CSR_MEPC, e + 4);     // MPP holds the trapped mode already
    }
}

void s_hnd_c(void)
{
    uint32_t c = csr_read(CSR_SCAUSE);
    uint32_t e = csr_read(CSR_SEPC);

    dram[20] = dram[20] + 1;
    dram[21] = c;
    dram[22] = csr_read(CSR_STVAL);
    dram[23] = e;

    if (c == 12) {                      // fetch pf
        uint32_t t = g_abort ? g_abort : _sav_s[1];   // abort label, else caller ra
        csr_write(CSR_SEPC, t);
        csr_set(CSR_SSTATUS, SS_SPP);   // return to S
        g_abort = 0;
    } else if (c == 3) {                // ebreak (U blob exit -> S)
        csr_write(CSR_SEPC, g_abort);
        csr_set(CSR_SSTATUS, SS_SPP);
        g_abort = 0;
    } else {                            // 13/15: skip the faulting access
        csr_write(CSR_SEPC, e + 4);
    }
}

// enter U at pc; execution never falls out (sret jumps away) and resumes at
// `abort` via the S handler (sepc = g_abort = abort). `abort` MUST be the
// address of a real function symbol (u_resume_probe/u_resume_done below):
// a `&&label` here is misfolded by clang(18) -O2 -- the blockaddress of a
// label sitting right after enter_u constant-folds to a wrong value, so the
// handler's `sepc = g_abort` lands outside .text and the core re-fetches
// garbage / restarts the test. A function symbol always materialises.
static void u_resume_probe(void);
static void u_resume_done(void);
static void enter_u(uint32_t pc, uint32_t abort);
static void enter_u(uint32_t pc, uint32_t abort)
{
    g_abort = abort;
    dram[30] = abort;
    csr_write(CSR_SEPC, pc);
    csr_clear(CSR_SSTATUS, SS_SPP);     // SPP = U
    sret();
}

static uint32_t call_va(uint32_t va) __attribute__((noinline));
static uint32_t call_va(uint32_t va)
{
    uint32_t (*f)(void) = (uint32_t (*)(void))va;
    return f();
}

// ============================================================ setup (M)
static void build_tables(void)
{
    uint32_t i;

    for (i = 0; i < 1024; i++) {
        pt0[i] = 0; pt1[i] = 0; l0a[i] = 0; l0b[i] = 0; l0b1[i] = 0;
    }

    // roots (asid0/asid1 identical except the alias table pointer)
    pt0[0x000] = mkpte(0x00000, F_V | F_R | F_W | F_X | F_A | F_D);
    pt0[0x140] = mkpte(PPN(l0a), F_V);
    pt0[0x180] = mkpte(0x80000, F_V | F_R | F_W | F_X | F_A | F_D);   // mega alias
    pt0[0x1C0] = mkpte(PPN(l0b), F_V);
    pt0[0x200] = mkpte(0x80000, F_V | F_R | F_W | F_X | F_A | F_D);   // ident DRAM
    pt0[0x240] = mkpte(0x80001, F_V | F_R | F_W | F_X | F_A | F_D);   // PPN[9:0]!=0 BAD
    for (i = 0; i < 1024; i++) pt1[i] = pt0[i];
    pt1[0x1C0] = mkpte(PPN(l0b1), F_V);

    // l0a: permission matrix
    l0a[0]  = mkpte(PPN(pgRO), F_V | F_R | F_A | F_D);
    l0a[1]  = mkpte(PPN(pgD0), F_V | F_R | F_W | F_A);                    // D=0
    l0a[2]  = mkpte(PPN(pgA0), F_V | F_R | F_W | F_X | F_D);              // A=0
    l0a[3]  = mkpte(PPN(pgU),  F_V | F_R | F_W | F_X | F_U | F_A | F_D);
    l0a[4]  = mkpte(PPN(pgXC), F_V | F_X | F_A | F_D);                    // X-only
    l0a[5]  = mkpte(PPN(pgS),  F_V | F_R | F_W | F_X | F_A | F_D);
    l0a[6]  = mkpte(PPN(pgUC), F_V | F_X | F_U | F_A | F_D);              // U code
    l0a[7]  = mkpte(PPN(pgRO), F_V | F_R | F_A | F_D);                    // fetch tgt
    l0a[8]  = mkpte(PPN(pgG1), F_V | F_R | F_W | F_X | F_A | F_D | F_G);  // global
    l0a[9]  = mkpte(PPN(pgNL), F_V | F_A);                                // nonleaf+A
    l0a[10] = mkpte(PPN(pgNL), F_V | F_W | F_A | F_D);                    // R=0,W=1

    // l0b / l0b1: alias + sfence + asid batteries
    l0b[1]  = mkpte(PPN(pgB),  F_V | F_R | F_W | F_A | F_D);
    l0b[3]  = mkpte(PPN(pgA),  F_V | F_R | F_W | F_A | F_D);
    l0b[6]  = mkpte(PPN(pgC2), F_V | F_R | F_W | F_A | F_D);
    l0b1[1] = l0b[1];
    l0b1[3] = mkpte(PPN(pgF),  F_V | F_R | F_W | F_A | F_D);              // asid1: pgF
    l0b1[6] = l0b[6];

    // markers (identity/Bare writes before satp takes effect for S)
    pgRO[0] = 0x0F0F0051u; pgD0[0] = 0x0D0D0052u; pgA0[0] = 0x0A0A0053u;
    pgU[0] = 0x00550054u;  pgU[1] = 0; pgU[2] = 0; pgU[3] = 0;
    pgS[0] = 0x0550E5u;    pgS[2] = 0xABCD1234u; pgS[3] = 0;
    pgG1[0] = 0x01610071u; pgG2[0] = 0x02620072u;
    pgA[0] = 0x0A0A0A0Au;  pgB[0] = 0x0B0B0B0Bu; pgC2[0] = 0x0C0C0C0Cu;
    pgD[0] = 0x0D0D0D0Du;  pgF[0] = 0x0F0F0F0Fu; pgNL[0] = 0x0E0E0E0Eu;
    pgA[1] = 0; pgB[1] = 0;

    // code blobs
    pgXC[0] = 0x02A00513u;   // li a0, 42
    pgXC[1] = 0x00008067u;   // ret
    for (i = 0; i < 11; i++) pgUC[i] = uc_blob[i];
}

// re-arm MPRV with MPP=S (trap entry clobbers MPP, mret clears MPRV)
static void mprs(void)
{
    csr_write(CSR_MSTATUS,
              (csr_read(CSR_MSTATUS) & ~MS_MPP_M) | MS_MPP_S | MS_MPRV);
}

// ============================================================ main (M)
int main(void)
{
    uint32_t i, v;

    for (i = 16; i <= 63; i++) dram[i] = 0;      // scoreboard (wb_ram boots X)
    g_abort = 0;                                  // .bss is NOT zeroed by
                                                  // fw_entry -- must init or
                                                  // s_hnd_c's beqz X-poisons
                                                  // the fetch engine

    build_tables();

    csr_write(CSR_MTVEC, (uint32_t)&m_shim);
    csr_write(CSR_STVEC, (uint32_t)&s_shim);
    csr_write(CSR_MEDELEG, (1u << 3) | (1u << 12) | (1u << 13) | (1u << 15));
    csr_write(CSR_MIDELEG, 0);
    csr_write(CSR_MIE, 0);
    csr_write(CSR_SATP, SATP_MODE | PPN(pt0));

    // ---- M ignores satp ----
    chk(1, pgRO[0] == 0x0F0F0051u && rd32(VA_RO) == 0);   // raw M: 0x50000000 null
    chk(2, rd32(VA_NULL) == 0);                            // null read 0 (no ERR)
    wr32(0x80000400u, 0x11223344u);
    chk(3, rd32(0x80000400u) == 0x11223344u);              // DRAM ok with satp loaded

    // ---- MPRV battery ----
    mprs(); v = rd32(VA_NULL);
    chk(4, dram[16] == 1 && dram[17] == 13 && dram[18] == VA_NULL);
    (void)v;
    mprs(); wr32(VA_RO, 0xFFFF0000u);                      // S view: RO -> 15
    chk(5, dram[16] == 2 && dram[17] == 15 && dram[18] == VA_RO);
    chk(6, pgRO[0] == 0x0F0F0051u);                        // no write reached RAM
    mprs(); v = rd32(VA_S);                                // translated load, ok
    chk(7, v == 0x0550E5u && dram[16] == 2);
    (void)v;
    mprs(); v = rd32(VA_BAD);
    chk(8, dram[16] == 3 && dram[17] == 13 && dram[18] == VA_BAD);
    (void)v;
    chk(9, (csr_read(CSR_MSTATUS) & MS_MPRV) == 0);        // mret cleared MPRV
    mprs(); v = rd32(VA_U);                                // MPP=S SUM=0 -> 13
    chk(52, dram[16] == 4 && dram[17] == 13 && dram[18] == VA_U);
    (void)v;
    csr_set(CSR_MSTATUS, MS_SUM);
    mprs(); v = rd32(VA_U);                                // SUM=1 -> allowed
    chk(53, v == 0x00550054u && dram[16] == 4);
    (void)v;
    csr_clear(CSR_MSTATUS, MS_SUM);                        // clean mstatus for S

    // ---- to S ----
    csr_write(CSR_MEPC, (uint32_t)s_main);
    csr_set(CSR_MSTATUS, MS_MPP_S);
    mret();
    for (;;) ;
    return 0;
}

// ============================================================ s_main (S)
static void s_main(void)
{
    uint32_t v, n0, a;

    // ---- identity / basic translation ----
    chk(10, rd32(VA_RO) == 0x0F0F0051u);                   // 4M identity from S
    chk(11, rd32(VA_S) == 0x0550E5u && rd32(VA_G) == 0x01610071u);
    // (VA_U must NOT be read here: U page without SUM would fault)

    // ---- 4M mega alias ----
    a = VA_MEGA + ((uint32_t)&pgS[2] - 0x80000000u);
    chk(12, rd32(a) == 0xABCD1234u);
    wr32(a + 4, 0x77770001u);
    chk(13, pgS[3] == 0x77770001u);                        // alias wr -> direct rd
    dram[96] = 0;
    wr32(VA_MEGA + ((uint32_t)dram - 0x80000000u) + 384, 0x5A5ABEEFu);
    chk(14, dram[96] == 0x5A5ABEEFu);
    dram[96] = 0;

    // ---- 4K alias (l0b[1] -> pgB) ----
    chk(15, rd32(VA_AL1) == 0x0B0B0B0Bu);
    wr32(VA_AL1 + 4, 0x99009900u);
    chk(16, pgB[1] == 0x99009900u);

    // ---- permission fault battery (S handler; n0 = s trap count) ----
    n0 = dram[20];
    wr32(VA_RO, 0xDEADu);                                  // RO store -> 15
    chk(17, dram[20] == n0 + 1 && dram[21] == 15 && dram[22] == VA_RO);
    chk(18, pgRO[0] == 0x0F0F0051u && rd32(VA_RO) == 0x0F0F0051u);

    chk(19, rd32(VA_D0) == 0x0D0D0052u);                   // D=0: load ok
    wr32(VA_D0, 0xBAADu);                                  // D=0: store -> 15
    chk(20, dram[20] == n0 + 2 && dram[21] == 15 && dram[22] == VA_D0 &&
             pgD0[0] == 0x0D0D0052u);

    v = rd32(VA_A0);                                       // A=0: load -> 13
    chk(21, dram[20] == n0 + 3 && dram[21] == 13 && dram[22] == VA_A0);
    (void)v;
    wr32(VA_A0, 1);                                        // A=0: store -> 15
    chk(22, dram[20] == n0 + 4 && dram[21] == 15 && dram[22] == VA_A0);

    // ---- U page from S (SUM matrix) ----
    v = rd32(VA_U);
    chk(23, dram[20] == n0 + 5 && dram[21] == 13 && dram[22] == VA_U);
    (void)v;
    csr_set(CSR_SSTATUS, MS_SUM);                          // bit18 in sstatus
    chk(24, rd32(VA_U) == 0x00550054u && dram[20] == n0 + 5);
    wr32(VA_U + 8, 0x13579BDFu);
    chk(25, pgU[2] == 0x13579BDFu);                        // SUM store ok
    csr_clear(CSR_SSTATUS, MS_SUM);

    // ---- X-only page: execute, load fault, MXR ----
    v = call_va(VA_XC);
    chk(26, v == 42);                                      // blob ran via I-side walk
    v = rd32(VA_XC);                                       // R=0 -> 13
    chk(27, dram[20] == n0 + 6 && dram[21] == 13 && dram[22] == VA_XC);
    (void)v;
    csr_set(CSR_SSTATUS, MS_MXR);                          // bit19
    chk(28, rd32(VA_XC) == 0x02A00513u);                   // MXR: X readable = code word
    csr_clear(CSR_SSTATUS, MS_MXR);

    // ---- fetch page fault (call into R-only page) ----
    v = call_va(VA_RF);                                    // 12 -> ra-skip back here
    chk(29, dram[20] == n0 + 7 && dram[21] == 12 &&
             dram[22] == VA_RF && dram[23] == VA_RF);
    (void)v;

    // ---- structural faults ----
    v = rd32(VA_BDM);                                      // 4M PPN[9:0]!=0 -> 13
    chk(30, dram[20] == n0 + 8 && dram[21] == 13 && dram[22] == VA_BDM);
    (void)v;
    v = rd32(VA_NL);                                       // nonleaf+A -> 13
    chk(31, dram[20] == n0 + 9 && dram[21] == 13 && dram[22] == VA_NL);
    (void)v;
    v = rd32(VA_R0W1);                                     // R=0,W=1 -> 13
    chk(32, dram[20] == n0 + 10 && dram[21] == 13 && dram[22] == VA_R0W1);
    (void)v;

    // ---- sfence page-specific (pair VA_AL1 / VA_AL6) ----
    sf_all();
    chk(33, rd32(VA_AL1) == 0x0B0B0B0Bu && rd32(VA_AL6) == 0x0C0C0C0Cu);
    l0b[1] = mkpte(PPN(pgD), F_V | F_R | F_W | F_A | F_D); // retarget BOTH
    l0b[6] = mkpte(PPN(pgD), F_V | F_R | F_W | F_A | F_D);
    chk(34, rd32(VA_AL1) == 0x0B0B0B0Bu && rd32(VA_AL6) == 0x0C0C0C0Cu); // stale TLB
    sf_page(VA_AL1);                                       // + runtime rs1
    chk(35, rd32(VA_AL1) == 0x0D0D0D0Du && rd32(VA_AL6) == 0x0C0C0C0Cu); // only VA_AL1 gone
    sf_all();
    chk(36, rd32(VA_AL6) == 0x0D0D0D0Du);

    // ---- global entry vs asid-specified sfence ----
    l0a[8] = mkpte(PPN(pgG1), F_V | F_R | F_W | F_X | F_A | F_D | F_G);
    sf_all();
    chk(37, rd32(VA_G) == 0x01610071u);
    l0a[8] = mkpte(PPN(pgG2), F_V | F_R | F_W | F_X | F_A | F_D | F_G);
    sf_asid(dram[24]);                                     // asid 0 via register
    chk(38, rd32(VA_G) == 0x01610071u);                    // G survives asid-specified
    sf_all();
    chk(39, rd32(VA_G) == 0x02620072u);                    // sf_all kills G

    // ---- asid keying (pt1 asid=1 -> pgF at VA_AL3) ----
    l0b[3] = mkpte(PPN(pgA), F_V | F_R | F_W | F_A | F_D);
    sf_all();
    chk(40, rd32(VA_AL3) == 0x0A0A0A0Au);                  // asid0 walk -> pgA
    l0b[3] = mkpte(PPN(pgD), F_V | F_R | F_W | F_A | F_D); // asid0 retarget (hidden)
    csr_write(CSR_SATP, SATP_MODE | (1u << 22) | PPN(pt1));
    chk(41, rd32(VA_AL3) == 0x0F0F0F0Fu);                  // asid1: fresh walk -> pgF
    csr_write(CSR_SATP, SATP_MODE | PPN(pt0));             // back to asid0, no sfence
    chk(42, rd32(VA_AL3) == 0x0A0A0A0Au);                  // asid0 entry SURVIVED
    sf_all();
    chk(43, rd32(VA_AL3) == 0x0D0D0D0Du);                  // flushed -> pgD

    // ---- satp clear in S: translation OFF -> identity null reads ----
    n0 = dram[20];
    csr_write(CSR_SATP, 0);
    chk(44, rd32(VA_BAD) == 0 && rd32(VA_NULL) == 0 && dram[20] == n0);
    csr_write(CSR_SATP, SATP_MODE | PPN(pt0));
    sf_all();
    v = rd32(VA_BAD);                                      // invalid again -> 13
    chk(45, dram[20] == n0 + 1 && dram[21] == 13 && dram[22] == VA_BAD);
    (void)v;

    // ---- U mode: fetch of S page -> 12 -> abort-exit ----
    // enter_u never falls out; the S handler resumes in u_resume_probe
    // (sepc = g_abort = u_resume_probe, a real function symbol).
    enter_u(VA_S, (uint32_t)u_resume_probe);
    for (;;) ;
}

// U-mode resume trampolines, entered by s_hnd_c via sepc = g_abort. Plain
// static functions so their addresses are real symbols (see enter_u note:
// a &&label here would be misfolded by clang(18) -O2).
static void u_resume_probe(void)
{
    chk(46, dram[21] == 12 && dram[22] == VA_S);

    // ---- U mode: blob full run ----
    enter_u(VA_UC, (uint32_t)u_resume_done);
    for (;;) ;
}

static void u_resume_done(void)
{
    chk(47, pgU[0] == 0x5A5u && pgU[1] == 0x5A6u);         // both stores ran
    chk(48, dram[21] == 3 && dram[23] == VA_UC + UC_EBREAK);// ebreak -> S record
    chk(49, dram[16] == 5 && dram[17] == 8 &&               // ecall -> M record
             dram[19] == VA_UC + UC_ECALL);                 // (4 MPRV + this)

    // ---- exit S -> M ----
    asm volatile("ecall" ::: "memory");                    // 9 -> M, mret to M
    chk(50, dram[17] == 9 && dram[16] == 6);
    // now in M: reading mstatus (0x300) is legal (from S it would trap illegal
    // -> m_n becomes 7). Do NOT assert mstatus.MIE==0 here: the last mret
    // loaded MIE<-MPIE (spec 3.1.7; MPIE was set by the earlier mret chain) --
    // with mie==0 interrupts stay masked either way. Success == legal read
    // (no new M trap) and MPP==U after mret (RTL zeroes MPP on mret).
    {
        uint32_t n0 = dram[16];
        uint32_t v = csr_read(CSR_MSTATUS);
        chk(51, dram[16] == n0 && (v & MS_MPP_M) == 0u);
        (void)v;
    }

    fw_done();
    for (;;) ;
}
