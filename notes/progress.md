# progress

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
- next: C4 loads/stores (mmu_d pre-issue in EX + pa hold; l1_d REQ on MA
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
