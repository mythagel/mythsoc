# progress

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
