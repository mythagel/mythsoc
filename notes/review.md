# RTL correctness / spec-conformance review (session 23, 2026-11-27)

Fresh read of all rtl/*.v against specs/ (extracted text /tmp/specs_txt).
Issues verified against spec text, then fixed. Baseline suite GREEN before edits.

## CORE (rtl/core.v)

F1. FETCH FAULT CAN BE SILENTLY DROPPED (correctness).
    req_l1 = !flush && !syn && t_pa_v && l1_free && space && !inval_p
    had NO !t_fault gate. A faulted translation (held pav_f, or fresh DONE
    w/ mmu_fault) arriving while stall=1 (syn is gated by !stall) or while
    syn's space term fails -> req_l1 issued an l1_i READ of the faulted PA;
    the response later enqueued as a NORMAL instruction ({pf=0}), the
    fetch-page-fault marker never entered the FIFO -> cause-12 trap lost,
    garbage from an unrelated PA executed. Same 1-ahead invariant as the
    t05 inval_p lesson: a held-fault translation must PAUSE fetch (syn
    consumes it), not be fetched-through.
    FIX: req_l1 &&= !t_fault. (pav_f then stays held; req_mmu blocked by
    !pav; syn fires when !stall && FIFO space -> marker enqueued, fault
    delivered.) Verified all syn-blocked windows now leave the fault held.

F2. INTERRUPT CAUSE CODE HARDCODED 11 / 9 (priv spec 3.1.9 + tables 16/37).
    int entry wrote mcause=0x8000_000B / scause=0x8000_0009 regardless of
    which pending bit fired. Implemented mip bits: MEIP(11), SEIP(9),
    SSIP(1, CSR-writable). Spec mcause/scause tables: 1=SSI, 5=STI,
    9=SEI, 11=MEI (priv.txt 5539-5564); simultaneous M-destined interrupts
    priority "MEI, MSI, MTI, SEI, SSI, STI" (priv.txt 5164), S-destined
    "SEI, SSI, STI, LCOFI" (priv.txt 11251). A non-delegated SSIP trapped
    to M must report mcause=1; delegated SSIP to S -> scause=1; non-
    delegated SEIP -> M cause 9. FIX: cause chosen from the pending vector
    with the spec priority chains (int_m_cause/int_s_cause wires).

F3. CSR WRITE-ATTEMPT CONDITION (unpriv 6.1, unpriv.txt 5922-5940 + Table 7).
    "For both CSRRS and CSRRC, if rs1=x0, ... will not write ... nor raise
    illegal-instruction exceptions on accesses to read-only CSRs. ... if
    rs1 specifies a register other than x0, and that register holds a zero
    value, the instruction ... will action any side effects caused by
    writing to the entire CSR." CSRRSI/CSRRCI: uimm=0 -> no write.
    RTL keyed the reg-form condition on the forwarded VALUE (x_src!=0), so
    CSRRS rd,RO_csr,x5 with x5==0 wrongly skipped the RO-write illegal
    trap. FIX: reg RS/RC write-attempt iff instr[19:15]!=0; imm iff
    uimm!=0; RW/RWI always.

F4. SYSTEM funct3=111 DECODED AS CSR (unpriv CSR table: f3=111 reserved).
    sys_csr list included 3'b111 -> it executed as CSRRCI. FIX: removed;
    f3=111 now illegal. (f3=100 already correctly illegal.)

F5. SFENCE.VMA EXECUTABLE IN U-MODE (no privilege check). SFENCE.VMA is a
    supervisor instruction; U-mode attempt must raise illegal. RTL flushed
    the TLB and retired silently. FIX: sf_bad_priv = x_sfence && priv==PU
    -> trap_illegal; the sf pulse suppressed when bad.

F6. SFENCE.VMA DID NOT REFETCH YOUNGER INSTRUCTIONS (ordering guarantee).
    priv.txt 11994: sfence "guarantees that any previous stores already
    visible ... are ordered before certain implicit references by
    SUBSEQUENT instructions"; 12067: an implementation may use any
    translation "valid at any time since the most recent SFENCE.VMA that
    subsumes that address". Instructions already sitting in the fetch FIFO
    / in-flight fetches were translated BEFORE the sfence -> executing them
    uses a sub-sfence (stale) mapping with no ordering vs the PTE stores.
    FIX: sfence now self-redirects to pc+4 like fence.i (flush drains FIFO
    + discard in-flight), WITHOUT I$ INVAL (I$ is PA-keyed; refetch via the
    flushed TLB walks fresh). Deferred if an interrupt fires that cycle
    (same rule as fi_now; sfence re-executes after the handler).

F7. MTVEC/STVEC MODE BITS: RTL write {wv[31:1],1'b0} preserved bit1, so
    MODE=2/3 (reserved) could be stored & read back; design.md sec.7 says
    direct-only, MODE=0 always. FIX: {wv[31:2],2'b00}. (BASE field itself
    is WARL, may be non-4B-aligned in the low bits -> trap vector always
    uses {mtvec[31:2],2'b00} already, per priv.txt 4755-4757.)

F8. MSTATUS BIT 9 WRITABLE. RV32 mstatus bit 9 (SPV) belongs to the H
    extension; without H it is WPRI, and "implementations that do not
    furnish these fields must make them read-only zero" (priv.txt 3386-
    3390). MSTATUS_WMASK 0xe1baa / SSTATUS_MASK 0xc0322 both included bit9.
    FIX: masks -> 0x000E19AA / 0x000C0122 (tools: bitlist). Trap entry to
    S never wrote bit9 anyway; only explicit csrw could.

F9. xRET MPRV CLEARING (priv.txt 4162-4171: "supposing xPP holds y ...
    If y != M, xRET also sets MPRV=0").
    mret: RTL cleared MPRV unconditionally; must clear ONLY when MPP != M
    (mret returning to M keeps MPRV). sret: y=SPP is always < M, so sret
    must ALWAYS clear MPRV; RTL never did. FIX both.

F10. sip (0x144) WRITE DROPPED. priv.txt 11223-11225: "SSIP is writable in
    sip" (when delegated; if not delegated sip.SSIP reads 0 and writes
    ignored). RTL had no 0x144 case -> S-mode could not self-IPI via
    csrw sip (S-mode route for MSIP-equivalent). FIX: 12'h144 case writes
    ssip_r gated by mideleg[1] (mirrors emu32, which already modelled it).

F12. satp PPN WARL: SoC physical space is 32-bit; satp.PPN[21:20] cannot be
    represented ("The satp.PPN field might not be capable of holding all
    physical page numbers ... platform standards might place constraints",
    priv.txt 11772). FIX: satp write forces PPN[21:20]=0 (WARL-legal).

## MMU (rtl/mmu.v)

F11. OUT-OF-RANGE PPN WRAPPED INSTEAD OF FAULTING (design.md sec.7 itself
    documents "PPN[21:20]!=0 -> PTE addr escapes 32b SoC -> null read 0 ->
    v=0 fault"). The 34-bit sums a1 (satp base) and a2c (L0 base) were
    truncated [31:0] -> address WRAPPED into real memory and could build
    bogus mappings; 4K/4M LEAF PPN[21:20]!=0 likewise silently dropped
    PA[33:32]. FIX: L1-PTE decode: DAT_I[31:30]!=0 in w1_bad (terminate;
    no continue into a wrapped S_W2S) and pi_ppn[21:20]!=0 in r_bad_i
    (covers W2W 4K leaf + both states' structural check) -> page fault,
    no TLB fill. REQ-path: SATP overflow guard faulting directly (belt &
    braces behind F12 mask; tb_mmu drives SATP_I without CSR masking).
    Also fixed header comment: 4M hit PA = {pa12[19:10], VA[21:0]}.

## PLIC (rtl/plic.v)

F13. ENABLE BIT 0 must be hardwired 0: plic.txt ch.6 "Bit 0 of enable
    register 0 represents the non-existent interrupt ID 0 and is hardwired
    to 0" (pending bit 0 already correct). FIX: en write {DAT[16:1],1'b0}.

## CLINT (rtl/clint.v)

F14. msip/sstip/mtip BIT-FORMAT CSRs: priv.txt 5120: "A hart's memory-
    mapped msip register is a 32-bit read/write register, where bits 31-1
    read as zero and bit 0 contains the MSIP bit" (one bit per hart; 1-hart
    design). sstip/mtip per design.md sec.4 are the same format. RTL let
    all 32 bits store/read. FIX: writes keep bit0 only. tb_clint group 11
    retargeted: unaligned/lane-merge checks move to mtimecmp (real 32-bit
    regs); msip/sstip keep bit0 + upper-reads-zero checks.

## WB SLAVES (rtl/wb_rom.v, rtl/wb_ram.v)

F15. WB B4 3.1.1: "All hardware interfaces are initialized to a
    pre-defined state" via RST_I. rom/ram ACK_O/DAT_O had no reset (X
    until first STB; rst port unused). FIX: async rst clears both
    (style-matches plic/clint/16550).

## COMMENT-ONLY (no behavior change)

F16. l1_d header lists ROM among never-cache devices -> matches code only
    if ROM is called cacheable (code is right, session-18 decision); fixed
    text. 16550 header "byte offset = ADR[5:3] mod 8" -> actual decode
    ADR[2:0] with grp=ADR[2] (session-17 rewrite).

F17. SFENCE.VMA CLASSIFICATION KEYED ON OPERAND VALUES, NOT FIELDS
    (core.v sf_typ). priv.txt 12030-12039: the four cases are defined by
    WHICH rs FIELD IS x0 ("If rs1=x0 and rs2!=x0, ... address space
    identified by integer register rs2; accesses to global mappings are
    NOT affected"). `sfence.vma x0, x15` with x15=0 (t04 sf_asid(dram[24]))
    was value-decoded -> treated as sf_all -> killed global entries.
    Baseline t04 passed chk38 only because L1D dirty-PTE writeback timing
    made the (wrongly) re-walked entry return the OLD mapping - the test
    accidentally masked the RTL bug. Fixed: sf_typ from exma_instr fields.
    Same rs-field-vs-value distinction as F3 (CSR write-attempt).

F18. FETCH HOLE AFTER FLUSH WHILE AN I-MMU WALK IS IN FLIGHT (core.v
    req_mmu). flush clears mio (line 230) but the walk continues inside
    mmu_i; mmu.v (header protocol) DROPS a REQ asserted mid-walk (accepts
    only in S_IDLE or on the DONE cycle). Old gate let req_mmu fire the
    cycle after a trap redirect (pav=0, mio=0, d_mio=1): request dropped,
    fpc advanced -> that instruction NEVER FETCHED. Observed: s_shim
    entered WITHOUT its first auipc (t2 garbage -> handler stores to
    wrong PAs -> trap storm). D-side req_md already guards this with
    !d_md (C8); added the symmetric !d_mio gate to req_mmu. Exposed by
    the F1b pause (held-fault responses lengthen the in-flight window
    across redirect points) - latent before, not introduced.
    (Earlier "Fetch 1-ahead pairing CONFORMING" bullet below: amended -
    pairing algebra was clean, accept-window was not.)

## Reviewed and found CONFORMING (no change)

- wb_ic: grant/pop/bypass/return timing per design sec.6; no-ERR/1-out
  model; STALL; null slave.
- Fetch 1-ahead {pc,data} pairing incl. inval_p gating (t05 fix intact);
  FIFO depth accounting (count_next + in-flight <= 4).
- Forwarding priority idex/exma/mawb + lf/cs gates (t02/t03 fixes intact).
- Misaligned-before-translation ordering (causes 4/6, tval=VA); ebreak
  tval=VA permitted (priv.txt 5715); ecall tval=0; illegal tval=0 permitted.
- Trap-to-S field set (SPP/SPIE/SIE; mepc/mcause/mtval untouched) ✓.
- mret MPP<-U (least supported), MPIE<-1, MIE<-MPIE ✓; sret same ✓.
- MIDELEG/MEDELEG/MIE masks (medeleg subset is WARL-permitted, priv.txt
  4820 "implementation can choose to subset the delegatable traps").
- mideleg[9]=0 non-delegated SEIP -> M pending (m_pend = mie&mip&~mideleg)
  and sgate/mgate rules ✓ (priv.txt 4.2.x pending formulas).
- MMU perms (U/S/SUM/X-only-fetch/4M-align/non-leaf D|A|U/RW reserved),
  Svade A/D fault scheme, G-bit match, sfence classes incl. page flush
  over-flush, speculative-walk sf_p ordering vs fill, fill policy
  (invalid-first FIFO), 1-AHEAD S_RESP accept + GRANT arbiter.
- PLIC: gateway latch/disarm/re-arm, claim atomics, threshold strictness,
  tie->low ID, multicast first-claim-wins, reserved offsets.
- 16550: lane-gated decode (session-17), trigger levels, FIFO pop,
  LSR/MSR/IIR simplified-but-documented model (no 16550 datasheet in
  specs/; design.md sec.5 governs), irq = ier[0]&rx_active | ier[1].
- wb_ic_mfifo push/pop/simultaneous cases.
- emu32 remains the golden: updated in lockstep (F2,F3,F4,F8,F9,F10,F12 +
  mip/sip reads include pins to match RTL).

## Accepted deviations (documented, spec-permissive or test-neutral)

- ECALL/EBREAK/WFI/xret: rd and rs1 fields NOT required zero (spike
  masks them; spec prose silent -> reserved-field convention ambiguous).
  Left as-is.
- No Zicntr counters / mcounteren -> csrr cycle/time traps illegal
  ("should" in spec, design omits counters; noted in design.md).
- Interrupts may be deferred across taken branches (int_fire gate);
  spec allows bounded deferral, and pure taken-branch loops never take
  interrupts (documented lesson, session notes t05/E).
- satp MODE: no TLB auto-flush on satp write; software sfence.vma
  (spec-permitted; design sec.7; immediate-mode-change text 12.3.x: MODE
  Bare<->Sv32 changes take effect immediately - they do, no cache used
  in Bare ✓).
- medeleg bits 10/14 writable: WARL subset choice permitted.
