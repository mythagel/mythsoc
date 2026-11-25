# progress

## 2026-11-25 (session 19)
- "Investigate and fix the hang" (t03): COMBINATIONAL LOOP FOUND + FIXED,
  residual t03 issue handed off (see bottom).
- Symptom: t03 (first fw test with PLIC meip/seip LIVE) -> vvp 100% CPU,
  sim TIME frozen (not just slow): first ~10k cyc at normal rate, then zero
  progress forever; RC=124 on every timeout. Loopback (+LOOP) NOT involved
  (no-LOOP run freezes identically). Pipe was advancing right up to the
  freeze (~cyc 13681, P9 S-phase) -> zero-delay delta storm, not a logical
  deadlock (re-verify: HB deltas + VCD, don't assume).
- Detection: `verilator --lint-only --timing -Wall -Wno-fatal -top soc
  rtl/soc.v rtl/core.v rtl/mmu.v rtl/l1_i.v rtl/l1_d.v rtl/wb_ic.v
  rtl/wb_ic_mfifo.v rtl/wb_rom.v rtl/wb_ram.v rtl/16550.v rtl/clint.v
  rtl/plic.v` -> `%Warning-UNOPTFLAT ... Circular combinational logic:
  'soc.core0.stall'` (rtl/core.v:96). Verilator's static comb-loop detector;
  iverilog has no equivalent (a loop only "works" if it happens to settle;
  odd-inverter loops oscillate when a state-dependent input goes live).
  Pre-existing benign notes: BLKLOOPINIT "delayed assignment to array
  inside for loops" on mmu/l1 arrays (keep -Wno-fatal).
- THE LOOP (rtl/core.v): stall -> pa_wait -> !pa_now_c -> fresh_md ->
  !flush -> flush -> int_fire -> !stall. Closes ONLY when int_fire is live
  (pending PLIC int + MIE/SIE) -> why t01/t02/soc (no live ints) were
  green. FIX: fresh_md = mmd_done && !d_md (drop !flush). A flush's dead
  op is discarded by flush-clearing md_pv/d_md/exma_mem_q (exma_df_q
  gated by idex_v); fresh_md needs no !flush. Flush-cycle behavior: 1
  fewer pa_wait cycle on a dead op (harmless, fq flushed anyway).
- Forensics tools (now in tree): tb/tb_fw.v +VCD ($dumpfile
  /tmp/opencode/tb_fw.vcd + $dumpvars core0/ic0/s2uart/s3clint/s4plic) +
  tools/vcd_last.py <vcd> <keep> [substr...]: per-sample changes +
  cumulative value table at the (frozen) last timestamp. VCD parse gotchas
  baked into the tool: $var fields = [type width id name] (id may be a
  bare '#'), do NOT partition header on first '#' char, 1-bit changes are
  "<v><id>" without space.
- LESSON: on "hang" check sim-time progress FIRST (VCD last-# or HB wall
  time). Rate crawl vs frozen-time vs zero-delay storm are different
  beasts; this one was storm #3 (sessions 16/17 had storm+crawl classes).
  Verilator lint is cheap and should be run on any stall-class hang.
- HANDOFF (next agent, t03): rtl fix is in; full suite green except t03
  (ran: mmu l1_i l1_d wb_ic 16550 clint plic core c8 t01 t02 soc). t03
  now PROGRESSES (P1-P8 checks all pass; no fw_fail) but livelocks in the
  P9 S-phase: fpc repeats an 11-HB (~22000 cyc) period of handler hops
  (0xa74/0xbf4/0x75c/0xab8 = m_hnd/h_mret/s_hnd/s_code region, priv
  S/M bouncing) at ~1.4k cyc/s, no WDT marker. P9 arms: stimecmp via
  0x02004004 overlap word, sstip, mtip fan-out 10/11, msip M-from-S,
  sret/mret MPP juggling, SIE=0->1 gate, EN0 0x88A->0x8A restore, and
  expects s_count=3 (ids 2,6,10) + m_count 14 + 1 more M entry (11,
  latched while ctx0-disabled). Suspects: (a) h_stmr clears
  stimecmp_hi(0x4008) with -1 -> word 0x1002 = stimecmp_hi ONLY? (it is:
  ADR word idx 0x4008>>2=0x1002; OK) but stimecmp_LO (0x4004) still
  T2+60 with mtime>>T2+60 -> src6 REFIRES forever if the S handler never
  runs after SIE=1 ordering slips; (b) mret-into at sret of s_code
  (MPP=M set at 0x740?) re-arms msip at the sret; (c) PLIC ctx1 claim
  vs complete ID. Repro: scripts/regress.sh t03 (or vvp +LOOP +HB);
  instrument s_hnd entries (s_count/s_hist) via VCD or DRAM.
- Files: rtl/core.v (1-line fix + comment), tb/tb_fw.v (+VCD; +HB/+LOOP/
  +DUMP already WIP), tools/vcd_last.py (new), tests/t03_intr.S +
  tb/t_fw_t03.hex + scripts/regress.sh t03 target (WIP, committed for
  handoff).

## 2026-11-23 (session 18)
- tb_soc 16/16 GREEN. Two real RTL bugs found by the end-to-end p2 echo phase:
  (1) l1_d CACHED DEVICE READS (load-miss filled any line incl. PLIC/UART/
      CLINT): stale claim skipped the atomic IP-clear -> interrupt storm,
      DRAM[6]/[8] clobber, sim crawl (X-poison + storm). FIX: cacheable
      window = DRAM PA[31:22]==10'h200 or ROM ==10'h000 ONLY; devices are
      strict pass-through (never hit, never fill). design.md sec.7 updated.
      tb_l1_d +D12 group (dev ld on bus twice w/ changed ram value, no-evict
      on set collision, dev st to bus, uart/clint on bus): l1_d now 31 checks.
      (2) 16550 WRITE DECODE not SEL-lane gated: sb to THR also wrote
      IER<=DAT[15:8] (replicated 0x5A -> ier=0b10 -> THRE irq STUCK HIGH);
      byte_ofs>=5 writes could spuriously hit THR (TX start). FIX: register
      x accessed iff x[2]==ADR[2] && SEL_I[x[1:0]] (aligned 4-reg window,
      lane L -> reg {grp4,L}); read side rebuilt as lane-mux rb0..rb3 (same
      aligned-window semantics the core uses: loads sel=1111 + lane extract).
      tb_16550 rewritten to core-protocol access helpers wr_reg/rd_reg +
      new U17 (THR st keeps IER/FCR, irq stays low) + U18 (SCR st starts no
      TX): 21/21 green.
- t02 irq-cnt expectation: handler counter is shared -> 3 (uart,echo,msip).
- scripts/regress.sh covers everything; FULL SUITE GREEN: mmu l1_i l1_d(31)
  wb_ic 16550(21) clint plic core(54) c8(33) t01(378) t02(33) soc(16).
- lesson (worth repeating): "hang" was really 100x slowdown from storm+X;
  add sim-time heartbeat BEFORE theorizing; probe observable state per-cycle
  (WB transaction log + exma instr) localized both bugs in ~2 runs.
- next: t03_intr.S (sec.10 C: PLIC priority/threshold/claim-complete, CLINT
  mtimecmp M+S timer, sstip->S sw, mtip fan-in (10->S,11->M), nested M->S),
  t04_mmu.S (sec.10 D on SoC via tools/ptmap.py), t05_soc.S (sec.10 E),
  dhrystone (F), C8-Sv32, synth+bitstream, board lpf, README.

## 2026-11-23 (session 17)
- CSR-FORWARDING BUG FOUND + FIXED (rtl/core.v): csrr rd,csr (rd!=0) consumed
  by the NEXT instruction (d=1) or one-after (d=2) latched alu_res/exma_res =
  the CSR op's ALU PASS output (rs1, usually x0), NOT csr_rdata (which only
  materializes at MA->mawb). t02's interleaved csrr/sw handler exposed it:
  mepc/mtval stored 0. Repro probe: /tmp/opencode/t02p.S (trace: producer at
  idex on consumer's operand-latch edge -> d=1 term picks alu_res).
  FIX mirrors the load path: (a) lu_ld_ex extended to idex CSR writers with
  rd==rs1/rs2 (hold consumer 1 cyc in ID, bubble idex); (b) new cs_a/cs_b
  terms forward csr_rdata directly from exma (checked BEFORE alu_res term).
  CSR ops never stall EX (no div/pa/mem), so exma CSR rd == "writeback source"
  for exactly one cycle: the consumer's ID->EX edge. t02 C5-style passes BOTH
  handler styles now; T9 added (mscratch d=1 store, d=2 ALU, d=2 store).
- scripts/regress.sh: full-suite runner (mmu l1_i l1_d wb_ic 16550 clint plic
  core c8 t01 t02 soc). GREEN: c8 33, core 54, t01 378, t02 33, all modules.
  scripts/core_asm.sh: skip empty --arm-m/--arm-s (c8 has no irq labels).
  tb_fw.v: +DUMP plusarg + ALL TESTS PASSED marker (regress.sh grep).
- tb_soc status: boot phase p1 (ier, RX 0x2A -> claim -> DRAM[6], count=1)
  PASSes; then ZERO sim-time progress after `loop = 1` (TXD->RXD mux): HB
  (#100us sim) never fires, clock edges stop -> zero-delay delta storm in the
  loopback path (not OOM; vvp 100% CPU; also killed an earlier t01 loopback
  run the same way). tb/tb_soc.v now has PROBE blocks on txd/rxd_t changes.
  NOTE earlier theory (TX-on-read) REJECTED: reg0_wr already WE-gated.
  Next: read PROBE log -> find the oscillating signal (suspect 16550 irq->
  PLIC ip edge-triggered re-pend loop, or uart TX start/tick interplay with
  rxd sampling the *same* wire txd drives).
- TB checkpoint lesson: iverilog $display cannot print a whole memory
  (vpiMemory error at RUNTIME, silent if never hit); print elements.

## 2026-11-23 (session 16)
- Firmware suite started (design.md sec.10). Framework `tests/fw_tb.S`:
  _start sets x1=DRAM base; j _fw_test; fw_done writes DRAM[0]=result,
  DRAM[1]=0x5A5A0000 marker LAST; fw_fail: result=0x8000_000N. GOTCHA:
  bare labels in LLVM .S are LOCAL -> `.globl fw_done/fw_fail` required.
  Harness `tb/tb_fw.v`: -DROM_HEX='"<hex>"' per test; -DWDT=N overrides
  watchdog (default 300000; a 1M-cycle WDT is minutes of wall time -
  bounded wait + dump is the way); timeout dump prints
  fpc/exma_pc/mcause/mepc/priv + DRAM[0..31].
- t01_isa.S (sec.10 A): GENERATED by tools/isa_gen.py (378 checks):
  ALU/shift/I-type/branches/jal/jalr/loads/stores/M-ext/forwarding.
  ALL 378 GREEN on the SoC via tb_fw (~9.5k cycles).
  Every failure this session was a GENERATOR bug, not core (rule 2):
  * sgn(): `v>>W` checked bit 32 (never set) -> must be `v>>(W-1)`.
  * c_sll/c_srl/c_sra given signed Python ints: >> on negative =
    arithmetic; normalize w32() first.
  * mulh/mulhsu/mulhu must be (prod)>>32, not low-32.
  * div/divu b==0 -> quotient ALL-1 for ANY dividend (unpriv Table 11:
    "quotient of division by zero has all bits set"); rem->dividend.
  * AUIPC: my expected 0x104 was wrong; core correct: U-type imm sits in
    insn[31:12] BIT POSITION; added value = insn&0xFFFFF000. Probe
    dram[16]=0x100004 == (0x100<<12)+4 confirmed. A transient "fix" to
    core.v was REVERTED; rtl/core.v has zero diff vs HEAD.
  * load tests read offsets 0..3 = RESULT BLOCK (X!). Reading X ->
    beq taken=X -> flush=X -> whole pipe latches X forever (proven with
    per-cycle TT/MA traces). Fixed: data words at 64/68/72 (w16..18);
    tests never read the result block; handler never reads unwritten mem.
  * final `j fw_done` must `li x2,0` first (chk leaves x2 nonzero ->
    result=0x1234 looked like failure).
- TB-side X-init gotcha: `integer pc;` is X at t=0 -> `pc % 250 == 0`
  is X -> checkpoint always-block NEVER PRINTS (earlier "silent 90 s"
  mystery = dead print block + long watchdogs, NOT vvp slowness; real
  speed ~20M ps/s). Init TB counters (`integer pc = 0;`). Same lesson
  class as C7's pram-X note.
- tb_soc.v rewritten: bounded waits (ier / mem[9]==1 / sentinel, caps
  200k/200k/400k) + dump_state() on stuck + fpc/exma_pc in timeout print.
  tb_soc_fw.S: boot checks + IRQ phases (PLIC src1 UART RX irq with TB
  injecting 0x2A on rxd, loopback txd->rxd for the 0x5A echo, CLINT
  msip->src3, mtime monotonic; handler latches mcause/mepc; results
  DRAM[5..13]). RUN NOT YET CONFIRMED: last launch was detached and got
  killed; log /tmp/opencode/tb_soc_run.log was empty.
  => RESUME FIRST: rebuild + run tb_soc; expect 14 checks green
  (SOC-sentinel, dram0..5, rx1-byte, loop-echo, irq-cnt, msip-done,
  mtime-mono, mcause, mepc). Firmware/hex already built OK
  (scripts/soc_asm.sh; 109 words).
- t02_traps.S (sec.10 B) WRITTEN (30 checks: ebreak/M-ecall/.word 0/
  csrw misa(RO)/mis ld/mis st/delegation S via medeleg=1<<3 + mret->S +
  ebreak->s_hnd + ecall->m_hnd returning to M via MPP<-M).
  CURRENT FAILURE: chk3 - m_hnd records mcause=3 OK, m_count=1 OK, but
  mepc and mtval read 0 (dram[17]/[18]). mret worked (test continued).
  KEY DIFF vs C5's m_hnd in tb_core.S (49 green checks there): C5 does 3
  BACK-TO-BACK csrrs then stores; t02 interleaves `sw` (first-ever stores
  to w16..19 = l1_d MISS + 3-cycle MA stall) BETWEEN the csrrs.
  Next steps (in order):
   (1) restructure t02 m_hnd to C5 style (3 consecutive csrr into x19/
       x20/x21, stores after) and rerun. If green -> real RTL bug:
       csrr result corrupted when an earlier store's l1_d miss stalls
       MA (suspect: rd/csr_rdata sampling on the stall/done cycle, or
       the stalled csrr re-latching while mld_done pulses). Then fix
       core + re-verify C5/C8/t01 regressions.
   (2) if still failing: TB trace of mepc write at trap entry (exma_pc
       value at the entry edge) vs the csrr read cycles.
- Files this session: tools/isa_gen.py (new), tests/fw_tb.S,
  tests/t01_isa.S (generated), tests/t02_traps.S, tb/tb_fw.v,
  tb/tb_soc.v, tb/tb_soc_fw.S(+hex), scripts/{soc_asm.sh,fw_asm.sh}.
  rtl/ UNCHANGED (git diff clean for rtl/core.v).
- AFTER t02: t03_intr.S (sec.10 C, real PLIC/CLINT in SoC - pins are
  driven by HW now, no +define arming), t04_mmu.S (sec.10 D, use
  tools/ptmap.py), t05_soc.S (sec.10 E), dhrystone (sec.10 F), full
  suite regression, then synth/board/docs as before.

## 2026-11-23 (session 15)
- SoC peripheral tests (next from session 14): tb/tb_soc_fw.S rewritten +
  tb/tb_soc.v extended (14 checks). End-to-end interrupt paths through the
  SoC (M-mode, PLIC ctx0):
  - p1: TB drives 0x2A on rxd -> 16550 RX -> irq -> PLIC src1 -> EIP[0] ->
    meip -> core take (mcause 0x8000_000B) -> m_hnd: PLIC claim (read),
    pop FIFO byte -> DRAM[6], complete (write).
  - p2: TB loops txd->rxd once p1 consumed; firmware TX 0x5A -> RX echo ->
    DRAM[8] (verifies TX engine + loopback RX through the same irq path).
  - CLINT msip -> PLIC src3 -> handler clears msip, DRAM[10]=1.
  - mtime monotonic (DRAM[7] < DRAM[11] across a delay loop).
  - m_hnd latches mcause/mepc -> DRAM[12]/[13] on every take.
  - scripts/soc_asm.sh: clang -c + ld.lld (-Ttext=0 -e _start) + objcopy
    + bin->hex python (1 word/line).
  - Gotchas: wb_ram mem NOT pre-zeroed (X) -> firmware must zero-init
    handler-written words (count/marker) before IER/mie/MIE; handler must
    store full words (sb leaves X lanes -> TB full-word checks fail).
    LLVM: csrsi GAS 2-op form rejected (use csrrsi rd,csr,uimm); bnez
    label with a REGISTER NAME (s1/s2/x1) mis-parses (use l1/... names).
- next: firmware suite tests/*.S (lib.S framework + per-section tests A-F,
  generic tb/tb_fw.v: plusarg ROM hex, DRAM result block protocol),
  full suite, synth (yosys/nextpnr/ecppack), board lpf, docs.

## 2026-11-23 (session 14)
- C8 Bare GREEN (33 checks): dense hazard/forwarding regression. tb/tb_c8.S
  (clang) + tb/tb_c8.v, 23 controlled RAW patterns, DRAM scoreboard
  (words 0..31) + final RF checked vs tools/emu32.py golden (tb/tb_c8_exp.vh).
- CAUGHT + FIXED a real d=1 load-use bug (the reason C8 exists): the consumer
  immediately after a load got the load's ADDRESS (alu_res = idex_a), not the
  loaded data. For d=1 the consumer is in EX when the load reaches MA, so it
  latched operands one cycle too early from the load-in-EX (whose alu_res is
  the address); lf_a+ma_wait only covers d=2 (consumer in ID when load in MA).
  FIX: lu_stall = consumer(ID) reads a load in EX(idex). Hold consumer in ID
  (stall_if = stall | lu_stall, used by ifid + deq + bypass) and bubble idex
  (idex_v<=0) so it re-latches when the load is in MA w/ mld_done (lf_a). The
  producer load keeps advancing (stall EXCLUDES lu_stall -> no deadlock).
  tb_core (C1-C7, 54 checks) re-run green after the fix.
- IPC measured (probe benches, probe_*.v in /tmp/opencode/c8): frontend
  delivers 1 instr/cyc on hits (the "1/cyc on hits" claim holds -- l1_i is
  1-WORD lines, so a cold straight-line program is all MISSES, not the hit
  path; a tight loop that revisits lines shows steady-state hit delivery
  1/cyc). Steady-state tight-loop COMMIT IPC = 0.667 (taken-branch redirect
  overhead on a no-prediction in-order core; expected).
- SoC top DONE (sim): rtl/wb_rom.v + rtl/wb_ram.v (WB B4 RAM/ROM slaves) +
  rtl/soc.v (core -> wb_ic -> {ROM,DRAM,16550,CLINT,PLIC}; PLIC IRQ fan-in +
  EIP->core meip/seip per design.md sec.3). tb/tb_soc.v + tb/tb_soc_fw.S:
  firmware boots from ROM, uses DRAM (store/load/mul/div/loop), writes a
  sentinel; TB polls the sentinel then checks DRAM 0..5 (7 checks green).
  Proves the full SoC datapath (ROM fetch -> wb_ic -> DRAM) works.
- next: SoC peripheral tests (UART loopback, CLINT mtime, PLIC irq->core
  meip/seip), then firmware test suite (tests/*.S), full suite, synth.

## 2026-11-22 (session 10)
- C2 GREEN (20 checks): M-ext single-cycle mul (dedicated signed-product
  wires - iverilog ?: signed pitfall) + 34-cyc restoring divider (EX stall;
  1 load + 32 iter + 1 D_DONE; D_DONE cycle alu_res = final -> EX/MA capture).
  Div-zero/overflow = 1-cycle special path (Table 11: quo all1/dividend).
- FETCH redesign: 1-slot resp buffer insufficient (DONE pulse overwritten
  during long stalls -> lost instructions, proven by trace). Now 4-deep
  in-order fetch FIFO {pc,data} w/ bypass + space-gated issue; sustained
  1 instr/cyc on hits; guard check C-fifo-no-ovf.
- FORWARDING redesign (the big one): EX-input forwarding ONLY works for
  fixed 1-cyc spacing. With fetch bubbles (every cold miss) the producer is
  in mawb at the consumer's ID->EX latch edge -> rf sample = pre-write value
  (write is the SAME edge) -> stale operand, no forward source left.
  FIX = ID-side mux at operand latch, sources youngest-first:
  idex(alu_res, d=1; safe for mul/div: latch only on !stall edges) /
  exma(d=2) / mawb(d=3) / rf(d>=4). C1 never caught this (dist>=4 + x0s).
- next: C3 (branches/jal/jalr, MA redirect, fetch discard on flush).

## 2026-11-22 (session 11)
- C3 GREEN (27 checks): branch/jal/jalr resolved in MA (no prediction);
  redirect loads fpc + flush kills younger (ifid/idex) + fetch-discard
  flags d_mio/d_lio drop stale in-flight mmu/cache responses; FIFO emptied;
  divider aborts if flushed while running (stale-D_DONE would capture
  garbage into EX/MA - guard `if (flush && is_div) ds<=D_IDLE`).
  C3 bugs found via trace: (a) req_l1 fired with STALE pav on the flush
  cycle -> gated !flush; (b) d_lio missing `&& !l1_done` clear -> stuck
  discard, fetch dead after 2nd loop iteration.
- BUILD FIX: clang -c leaves R_RISCV_BRANCH relocs ZERO-FILLED in .o bytes;
  objcopy -O binary without linking => every branch offset 0 (self-loop!).
  scripts/core_asm.sh now links with ld.lld (-nostdlib -Ttext=0 -e _start).
- emu32: BRANCH/JAL/JALR + halt-on-self-loop exit.
## 2026-11-22 (session 12)
- C5 GREEN (49): CSR+trap+delegation+S-mode. Bugs: mscratch/mepc reg collision
  (0x340 wrote the mepc reg - split regs); fetch FIFO fq_w/fq_r NOT reset on
  flush (only fq_c) -> post-trap stream corruption; CSRRS/RC merge missing in
  RTL MA write path (wrote raw src) - masked earlier by zero-old bits.
- C6 GREEN (54): interrupts: meip/seip pins armed by TB on retire-PC match
  (PC_ARM_M/S from llvm-nm -> tb_core_irq.vh), mirrored by emu --arm-m/-s;
  take at MA on clean sequential retire (epc=pc+4); M>S priority; mideleg
  routing; MIE/SIE gates; handler consume=csrw mie,0 (polarity bug found via
  trace); s_hnd csrc sie/sstatus. emu bug: int_deliver SPP write clobbered by
  base-m reuse.
- Golden window now 128 DRAM words (trap records + C4 data).
- next: C7 Sv32 on: fetch fault->synthetic-cause path (cause rides fq/idex/
  exma), mmu_d fault -> 13/15, sfence.vma SF pulse + 1-cyc req-hold, MPRV
  eff-mode mux, tools/ptmap.py page tables, emu Sv32 via mmu_addr.translate.

## 2026-11-22 (session 13)
- C5 (49) + C6 (54) + C7 (54) GREEN, full suite green. Core now RV32IM
  + M/S/U + traps + delegation + PLIC-pin interrupts + Sv32 (4K/4M, asid,
  sfence, A/D-faults, SUM/MXR/MPRV) — RTL == emu32 golden bit-exact on a
  618-instruction bare-metal program with 18 synchronous + 2 interrupt
  traps and 12 translated accesses.
- C7 bugs: pram 1MB + zero-init ($readmemh leaves X holes -> phantom X
  faults); mmu_addr bits() PPN mask 18b->22b (PA>=0x4000_0000 truncated);
  fetch syn-marker path: pav/pav_f replacement algebra (held-vs-fresh);
  Verilog has NO 0x literals (32'h..); md_pv_f missing reset entry.
- emu32 = full arch model now (CSR/trap/int windows --arm-m/s, Sv32 --pt,
  records via mexp window). scripts/core_asm.sh: clang+ld.lld+llvm-nm+ptmap.
- next: C8 (IPC measurement + dense RAW chains + final regression pass).

  entry; load data forward direct from l1_d DATA_O at DONE; width/sel).

## 2026-11-22 (session 9)
- C1 GREEN: rtl/core.v (IF/ID/EX/MA/WB, fetch=mmu_i->l1_i 1-ahead ctrl,
  regfile 2R+2W, EX/MA+MA/WB forwarding, RV32I straight-line ALU) +
  tb/tb_core.v (7 checks: 31-reg golden vs tools/emu32.py of clang-assembled
  tb/tb_core.S, x0, WB protocol/idle). Flow: scripts/core_asm.sh =
  clang -c -target riscv32-unknown-elf -march=rv32i -> llvm-objcopy -O
  binary -> emu32 golden .vh + $readmemh hex.
- FIXED l1_i/l1_d: miss DONE sat in W_RESP -> REQ on a miss's DONE cycle was
  IGNORED (core lost a request: fetch deadlock). W_RESP folded into W_IDLE
  (DONE=1cyc pulse via default-0 reg; DATA_O held until next REQ). Both TBs
  re-run green. l1_d had the same latent bug (would hit it in C4).
- iverilog -g2005 GOTCHA (probe /tmp/opencode/sra_probe.v): `$signed(x)>>>sh`
  INSIDE a `?:` chain evaluates WRONG (logical not arithmetic); keep signed
  ops on dedicated wires, ternary selects only.
- iverilog gotchas hit again: task $monitor hierarchical refs; use dut.sig.
- emu32.py = reference model (sign-extend 12-bit imm; mask every w on store;
  python `not x & 1` precedence trap -> use (x & 1) == 0).
- LESSON: golden-vs-DUT mismatches localized via one python tool check,
  never mental math (caught emu s() double-sign-extend + RTL SRA via probe).
- next: C2 (M-ext: single-cycle mul via dedicated signed-product wires;
  32-cycle iterative div w/ EX stall; Table 11 div-zero/overflow semantics).

## 2026-11-21 (session 8)
- core task decomposed -> notes/core.md (8 ordered pieces C1-C8, each RTL
  increment + TB group, green-before-next). Strategy: full datapath built
  once (mmu x2 + l1_i + l1_d + regfile + 5-stage pipe + csr/trap/int state),
  C1-C6 tested M-mode Bare (MMU identity), C7 Sv32, C8 regression.
- Grounded in spec via new tools:
  - tools/extract_tables.py (pdfplumber): clean RV32I Table 13 (40 instr),
    RV32M Table 15 (8 ops), mcause Table 16 / scause Table 37 cause codes.
  - tools/csr_list.py: parses priv Ch.19 CSR listing -> addr/access/name.
    Captured machine+supervisor CSR set for C5.
  - tools/misa.py: misa reset for RV32IM = 0x40001100 (MXL=1 bits[31:30],
    M=bit12, I=bit8; spec 3.1.1 alpha-index A=0..Z=25).
- Resolved spec questions for C5: mtval (priv 3.1.16): pf/misaligned/
  access-fault on fetch/load/store -> VA; ebreak -> VA; illegal -> 0 (instr
  bits optional, skip v1); ecall/other -> 0. Reset (3.4): mode=M,
  MIE=MPRV=MBE=0, PC=0, mcause=0, rest 0.
- design.md sec.12: core line expanded to C1-C8 sub-checklist (notes/core.md).
- next: C1 (if/decode/ALU straight-line) + tb group.

## 2026-11-21 (session 7)
- mmu + tb: ALL TESTS PASSED (59 checks M1-M12). Fixes this session:
  - TB base map was missing 2 PTEs vs tools/mmu_addr.py output:
    0x000087FC (B_root_hi, M5 4M new-set) and 0x00020020 (C_l0[8], M7/M10
    VA 0x8004). Verified by set-diff of tool output vs TB ram_set lines
    (python).
  - TB init loop: 32768 -> 65536 words (word 0x8008 = 0x20020 was X ->
    PA=xxxxx004 FAU=x on M7/M10).
  - Test-expectation fix: M7 "S load U=0 OK" expected lat7 but the L2 entry
    (set8, tag VA[31:17]=0, asid0) filled by the earlier "U load U=0 13"
    request survives (L1 was evicted by the G-page fill, L2 set8 untouched)
    -> correct behavior is an L2 hit, lat2. RTL verified correct via S_R
    fill probes (fe1/fe2/fl_ok) + REQ match-wire probes; probes stripped
    after green (kept rlog).
- LESSONS:
  - Always set-diff the tool-generated constant map vs the TB's copy; both
    0x87FC and 0x20020 were dropped when the map was first written by hand.
  - Hex literal width: 0x2001C is 32'h0002001C (8 digits), not 32'h00002001C.
  - When a "hit" is observed where none should exist, first check the second
    TLB level: L1 eviction does NOT evict L2 (separate sets).
- next: l1_i/l1_d + tb, then core, soc, firmware (tests/*.S + clang build),
  full suite, synth (yosys/nextpnr/ecppack + fmax), board (lpf/openocd/boot
  stub), README

## 2026-11-21 (session 6)
- mmu TB status: 53 pass / 6 fail (cyc=292). M1-M4, M6, M8, M9, M11, M12 all green
  (incl. M12 mid-walk sfence flush-after-fill). Fails: M5 "new set miss",
  M7 x3, M10 x2.
- Fixed this session:
  - TB RAM slave index bug: `ram[a[14:2]]` (13b index) silently wrapped word
    addresses >= 0x8000 (0xC000->idx 4096 etc.). Max word idx = 0x8008
    (map C L0 @0x20020) -> index [17:2], array 65536. Repro: /tmp/t_task.v.
  - TB mmu_req race: requests issued while prior DONE still high consumed the
    stale response (every later phase desynced). Fix: wait DONE low (negedge)
    before asserting REQ.
  - RTL: l1_pa12/l2_pa12/h_pa12 12b->20b (entry stores PA[31:12] per
    design.md sec.7). 4M hit PA = {pa12[19:10], VA[21:0]}. (M5 "4M hit PA"
    was failing: PA[31:22] truncated.)
  - RTL: sfence page flush compared WAY bits of entry idx: L1 i[1:0]->i[3:2],
    L2 i[4:0]->i[7:3] (set bits). M12 now passes.
  - tools/mmu_addr.py: +satp C asid=1 word (S_C1=0x8040000C), +B_root_hi PTE
    @0x87FC = 0x008000CF (VA 0x7FFFFFFF 4M region), +4M 0x7FFFFFFF check.
- Remaining 6 fails + evidence (per-req DBG + TLB dump live in TB):
  - M5 "4M new set miss": DUT reads 0x87FC (rlog) -> 0 -> fault 12. TB base
    map LACKS ram_set(0x87FC, 0x008000CF) (earlier edit: wrong addr 0x807C +
    failed on 2 matches). Known fix: add the tool's line to the base map.
  - M7 x3 (U load U=0 / v=0 unmapped / S load U=0): PA=xxxxx004 FAU=x
    (X-propagation). Suspect: TB init loop zeroes only words 0..32767 of the
    65536 array (map C L0 idx 9 @0x20024 -> word 0x8009 = X). Fix: init all
    65536.
  - M10 x2 (C asid1 NG fill / NG flushed asid): same X-PA pattern; likely same
    root cause. Re-verify after the two fixes.
- OPEN RTL suspicion (unverified): reset zeroes only l1_v/l2_v; l1_st/l2_st/
  l1_pa12/... left X. fw1/fw2 min-stamp scan over X l1_st -> X way -> fill
  index X (l1_v[fe1] with fe1=X). Next probe: dump lru/l1_st[0..3]/fe1/fw1
  at fill (hierarchical or RTL DBG) — do NOT hand-reason iverilog X-index NBA
  write semantics.
- On green: remove TB instrumentation (tlb_dump task, per-req DBG, dn,
  DBG lines), keep rlog; mark mmu done in design.md sec.12.
- LESSONS:
  - iverilog -g2005: part-select index width silently wraps (13b index on a
    32768-word array for addrs needing 16b) — NO warning, silent corruption.
    Derive index width from the tool's max address, never eyeball.
  - iverilog -g2005: no `break` (elab error "unknown task") — use while.
  - TB protocol: never issue a request while prior DONE is high; gate on
    negedge for both edges.
  - sfence page flush must compare SET bits of {set,way}: L1 i[3:2],
    L2 i[7:3] (ways are not part of the page index).

## 2026-11-20 (session 5)
- plic + tb: ALL TESTS PASSED (24 checks P1-P13: reset, prio RW/reserved words,
  level-latch no-retraction, claim-needs-enable, per-ctx threshold
  (strict >), per-ctx enable, multicast first-claim-wins (shared IP),
  prio-desc/tie->low-ID, prio0=never, claim-ignores-threshold, gateway
  re-arm/complete (bad ID 0/17 ignored; no level = no re-pend), reserved
  offsets inert, SEL ignored (word only), ACK=1cyc, clean shutdown).
- rtl/plic.v: WB B4 slave, 16 level gateways, claim = max(prio desc, ID asc)
  among IP&en[c]&prio>0 (clears IP, returns ID, threshold-independent),
  complete re-arms gateway (ID not checked vs last claim), EIP[c]=|IP&en[c]
  &prio>thr[c]. Decode from tools/plic_addr.py (1M-word brute force).
- LESSONS:
  - Decode base-page: base 0x0C00_0000 has base[21:12]==0x000, NOT 0x0C0
    (the 0xC sits in bits 27-24). Off-by-0x0C0 decode made every access fall
    to reserved. Always take decode constants from the addr tool, never
    mental math.
  - iverilog -g2005: out-of-range part-select on a var (`en[16:1]` on
    `reg [15:0]`) silently X/no-ops, no error. en must be [16:0] (bit16=src16).
  - iverilog -g2005: Verilog FUNCTION called as a wire, with an `integer`
    loop index indexing reg arrays, mis-evaluated (claim always 0). Fixed:
    always @(*) with fixed-width (5-bit) loop index.
  - Convention: bit N = source N (enable val src1 = 2, srcs 1..5 = 0x3E).
  - TB: deasserted level does NOT re-pend after complete; TB must re-assert
    IRQ before expecting re-pend (P5).

## 2026-11-19 (session 4)
- wb_ic + tb: ALL TESTS PASSED (25 checks: reset, 1cyc/3cyc slave push->ack latency,
  cross-master order, decode routing, null slave, byte/SEL merge, 8-read burst
  pipelining, no-ACK slave + per-master STALL isolation, reset mid-flight + slave
  revive, 3cyc-slave parallelism, 450-op concurrent stress (3 LCG engines), exact
  region re-verify + ack/push balance, FIFO unit).
- rtl/wb_ic.v rewrite: combinational present-at-grant (slave STB+data at grant,
  FIFO pop tied to grant; empty-FIFO STB grants the pushed word directly: push@P,
  pop@P+1), combinational return (m_k_ACK = s_ACK & s_busy, same cycle as slave
  ACK). Latency per design.md sec.6: 1cyc slave STB@c -> m_ACK@c+1; same-master
  re-grant period off+2 (3 for 1cyc slave) -> 8-push burst total 24.
- tb_wb_slv: NOACK is now a runtime input (T9 revive); DELAY=1 now ACKs at c+1
  (sec.6: slave ACK >=1 cycle after STB; old same-cycle path violated protocol).
  Measured: 1cyc push->ack dt=3, 3cyc dt=4, burst=24 (tools/wb_ic_latency.py).
- tools/wb_ic_latency.py: cycle-accurate push->ack/burst/FIFO model; anchors are
  sec.6 latency + measured dt.
- iverilog gotcha: string labels via `task ok(input [255:0] s)` + %s print only
  32 chars; longer literals truncate (high end) SILENTLY. All TB labels <=32;
  guard comment at ok/err in all TBs. (Found: 4 PASS lines printed truncated.)
- notes/design.md sec.12: 16550/clint/wb_ic marked done.

## next (done in session 7)
- mmu finished: ALL TESTS PASSED (59). See session 7 entry above.

## 2026-11-18 (session 3)
- tools/addr.py (region/regmap modes). Caught: mtime regs at word 0x2FFE/0x2FFF (not 0x2FFF/0x3000).
  Verified decodes: CLINT ADR[31:16]==16'h0200, PLIC ADR[31:22]==10'h030, word idx ADR[15:2]/ADR[21:2].
- rtl/clint.v rewritten: lane b = byte b of register at byte addr ADR+b (word regs); per-register
  byte-merge fold (multi-lane writes to same word); 0x4004 overlap (read=mtc hi, write=both);
  mtime RO; reset cmp=all1 (no boot pending); +1/cyc.
- tb/tb_clint.v: ALL TESTS PASSED (36 checks: reset state, mtime +1/cyc deltas, msip/sstip/mtip
  edges, stc/mtc set->poll->sticky->clear, cmp0 both-pending, overlap, RO mtime, aligned+unaligned
  LW/SW, byte lanes (data in lane b pos), ACK timing, OOB no-ACK).
- NEW iverilog -g2005 gotcha: UNSIZED hex literal (0x...) as arg to task-with-output-port call
  -> "Syntax error in task arguments". Always use sized (32'h...).

## next
- wb_ic + tb (me), then mmu/tlb, l1_i/l1_d, core, soc, firmware, synth, board

## 2026-11-18 (session 2)
- u16550 (rtl/16550.v) + tb (tb/tb_16550.v): ALL TESTS PASSED (16 checks).
  Bugs found/fixed via TB: ADR[5:3]->ADR[2:0] decode; genvar in proc block;
  variable part-select chain (iverilog); 1-pop-per-R0-read semantics;
  tx start tick-align; LSR bit order (bit0 DR,1 OE,3 FE,5 TEMT,6 THRE + DR bit);
  TB: task `input` default 1-bit width trap (wait_ticks); 120ns bit period;
  decoder must sample once per tick (tick_ff); badstop must return line high.

## 2026-11-17 (session 1)
- Rules reiterated (AGENTS.md).
- Recon done: toolchain (yosys 0.70, iverilog 12, clang 18 w/ riscv32-unknown-elf target,
  nextpnr-ecp5, ecppack, openocd). No riscv-tests available; tests hand-written .S
  (user decision). No webfetch (user directive).
- Board: Colorlight 5a-75b v8.2 (LFE5U-25F BG256, 25MHz P6, SDRAM M12L64322A 2Mx32,
  SPI flash W25Q32J, no UART header — 16550 exposed on pins; LED T6, key R7,
  JTAG J27/J31/J32/J30). lpf base: ~/dev/project/chubby75/5a-75b/lpf/cl-5a-75v-v80.lpf
  (has SDRAM/flash/phy pins; needs JTAG/clock/LED additions -> lpf/).
- wb_sdram_ctrl (local, ~/dev/project/wb_sdram_ctrl) reusable for board DRAM.
- Read PLIC spec (v1.0.0 ratified) fully: NO U context (M/S only), register map
  +0x1000 pending, +0x2000 enables (ctx stride 0x80), +0x200000 threshold,
  +0x200004 claim/complete (ctx stride 0x1000); prio 0=never; ties->low ID;
  claim unaffected by threshold; complete re-arms level gateways.
- WB B4: classic STB/ACK/ERR/RTY/CYC + pipeline STALL + LOCK; registered-feedback
  (pipelined) mode per ch.4/5. Our interconnect: per-master in-order queues,
  1 outstanding per master, 3 pipeline stages (decode/slave-queue/return).
- Wrote notes/design.md (full architecture, memory map, protocols, test plan).

## next
- rtl/16550.v + tb, rtl/clint.v + tb, rtl/plic.v + tb  (THIS TURN if possible)
