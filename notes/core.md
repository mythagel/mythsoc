# core (mythrv32) — task breakdown

Goal: `rtl/core.v` + `tb/tb_core.v` — the 5-stage in-order RV32IM core, "RV32IM
w/o MMU first, then MMU on" (design.md §7, §12).

Strategy: build the **full** datapath once (mmu×2 + l1_i + l1_d + regfile +
5-stage pipeline + csr/trap/interrupt state), then feature-increment it.
C1..C6 run in **M-mode Bare** (MMU present, satp.MODE=0 → identity, no fault).
C7 flips to Sv32. C8 is the regression. Each piece = RTL increment + labeled
check group in tb/tb_core.v, green before the next.

Spec (authoritative, /tmp/specs_txt): unpriv = RV32I/M unpriv, priv = privileged.
Reference models in tools/: mmu_addr.py (Sv32 walk), addr.py (WB decode),
csr_list.py (CSR addresses), misa.py (misa reset).

## Already-tested blocks (interfaces fixed; core only consumes, do not re-derive)
- **rtl/mmu.v** — Sv32 walker+TLB, WB RO master (SoC m2). 59 checks.
  In : REQ, VA, ACC[1:0] (0=fetch 1=load 2=store 3=amo), MODE[1:0] (0=M 1=S 2=U),
        SUM, MXR, SATP, SF[1:0], SFASID
  Out: DONE, PA, FAULT, CAUSE[3:0] (12/13/15)
  Latency from REQ: hit 2 cyc; 4M miss 5; 4K miss 7 (1-cyc slave).
- **rtl/l1_i.v** — 4 KiB I-cache, WB RO master (m0). 23 checks.
  In : REQ, PA, INVAL   Out: DONE, DATA   hit 1 cyc; miss 3; INVAL flushes all.
- **rtl/l1_d.v** — 4 KiB D-cache, write-through no-allocate, WB RW master (m1).
  20 checks. In : REQ, PA, WEN, WDAT   Out: DONE, DATA
  load hit 1; load miss / store 3.

## Core structure — one module rtl/core.v, sub-blocks
1. **regfile** — 32×32; 3 read (ID) + 2 write (MA, WB); same-cycle conflict → MA (older) wins. x0 hardwired 0.
2. **if_stage** — PC reg; drives mmu_i (VA=PC) → l1_i (PA) → fetch word; PC update
   (seq / branch / jal/jalr / mret-sret / trap-redirect).
3. **id_stage** — decode 32-bit instr → op+fields; read regfile rs1/rs2; CSR read;
   forwarding mux (from EX/MA/WB).
4. **ex_stage** — ALU (R/I ops, shifts, compares, LUI/AUIPC); M-ext (mul/div);
   branch compare; PC-target + immediate generation.
5. **ma_stage** — memory: drive mmu_d (VA=eff) → l1_d (PA); load byte/half
   extract+sign-ext; store SEL/align; branch/jump commit (redirect); trap +
   interrupt commit point.
6. **wb_stage** — regfile write (ALU result from EX, load data from MA).
7. **csr_trap** — CSR state (set below); trap entry (epc/cause/tval/tvec);
   mret/sret; delegation (medeleg→S); mtvec MODE=0 direct-only.
8. **intc** — interrupt pending/enable (mip/mie; sstatus.SIE, mstatus.MIE gate);
   priority M>S; mret-into-interrupt; nested M→S. Sources: PLIC eip_m/eip_s → meip/seip.
9. **mmu_i / mmu_d** — two instances of rtl/mmu.v (per-side TLB, design.md §7).
   I-side ACC=0 (fetch); D-side ACC=1/2/3 by op.

Core module ports (SoC top §9 wires these): clk, rst; WB masters
m0(I:STB/SEL/ADR/WE/ACK/DAT), m1(D:+WDAT/WDAT_O), m2(MMU:RO); irq in: meip, seip.
(No direct UART/LED — MMIO via m1.)

## Ordered pieces (each: RTL increment + TB group, green before next)

### C1. fetch + decode + R/I ALU  (+ LUI/AUIPC)
Straight-line only (no control flow / memory yet; PC=pc+4).
Ops (unpriv Table 13): ADD SUB SLL SLT SLTU XOR SRL SRA OR AND;
ADDI SLTI SLTIU XORI ORI ANDI SLLI SRLI SRAI; LUI AUIPC.
Done: every op, all rd; sign/zero correct; x0 stays 0.
Spec: unpriv Ch.2 (RV32I, Table 13).

### C2. M extension
MUL MULH MULHSU MULHU DIV DIVU REM REMU.
Done: 8 ops; div-by-zero (quotient=all-ones, rem=dividend);
overflow INT_MIN/−1 (quotient=0, rem=0); unsigned 0x80000000 edges.
Spec: unpriv Ch.12 (Table 15 RV32M).

### C3. branches + jumps + flush
BEQ BNE BLT BGE BLTU BGEU JAL JALR.
Done: taken/not-taken; forward/backward; ra link value; branch NOT predicted →
younger IF/ID/EX flushed at MA (no stale commit); JALR target=rs1+imm (bit0 masked).
Spec: unpriv Ch.2.

### C4. load/store via l1_d  (+ wire l1_i into IF)
LB LH LW LBU LHU SB SH SW.
Done: each width sign/zero-ext; store→load round-trip; byte/half lane (SEL);
misaligned (test plan A: must trap → causes 0/4/6); l1_i hit/miss;
fence.i / mret / sret I-cache flush.
Spec: unpriv Ch.2; design.md §7 caches.

### C5. CSR + synchronous exceptions  (no interrupts yet)
Ops: CSRRW CSRRS CSRRC CSRRWI CSRRSI CSRRCI; ECALL EBREAK; illegal instruction.
CSR set to implement (addresses from tools/csr_list.py):
  machine : mstatus mstatush misa mvendorid marchid mimpid mhartid mconfigptr
            mscratch mepc mcause mtval mtvec medeleg mideleg mie mip
            mcycle minstret (+*h)   [counters optional; mcycle/minstret = 0xB00/0xB02]
  superv  : sstatus stvec sepc scause stval sie sip sscratch scounteren
            stimecmp stimecmph satp
Done: csrr* rd/rs semantics (x0 rules); ECALL (8/9/11), EBREAK (3); illegal (2);
mtvec direct MODE=0 (vectored → reserved); mepc/mcause/mtval contents;
mret (xepc/xie/xpp/mode per spec); delegation medeleg→S (scause/sepc/stval).
mtval encoding (priv 3.1.16): page-fault/misaligned/access-fault on fetch/load/store
→ faulting VA; EBREAK → 0 or VA (pick VA, redundant with epc); illegal → 0 (instr-bits
optional, skip for v1); ecall/other → 0.
Reset (priv 3.4): mode=M; mstatus.MIE=MPRV=MBE=0; misa=0x40001100 (tools/misa.py);
PC=reset vector (0x0000_0000 ROM); mcause=0; other state 0 (determinism).
Spec: priv Ch.2 (CSR), Ch.3.1 (mstatus/mtvec/mcause/mtval), Ch.12.1 (s*), Ch.3.4 (reset);
unpriv Ch.2 (CSR instrs); cause codes = priv Table 16/37 (see tools/extract_tables.py).

### C6. interrupts
Done: mip/mie pending/enable; mstatus.MIE gates M, sstatus.SIE gates S;
priority M>S; sources: timer (M/S via CLINT mtimecmp/stimecmp), software
(msip/sstip), external (PLIC eip_m/eip_s → meip/seip); mret-into-interrupt;
nested M→S.
Spec: priv Ch.3 (machine interrupts), Ch.12 (supervisor); plic.txt; CLINT/PLIC already TB-green.

### C7. MMU on  (I + D side; satp Bare → Sv32)
Done: identity map; swap mapping; 4K + 4M; R/W/X permission faults (12/13/15);
A/D enforcement (Svade fault scheme); U-bit (S-vs-U); M-mode bypass
(identity, no fault); SUM/MXR; MPRV (effective mode = MPRV?MPP:mode);
sfence.vma (all/asid/page); TLB flush via satp clear; I-side fetch fault;
mret with satp change; I/D side independence.
Spec: priv Ch.12 (Sv32, 12.3); unpriv Ch.2; ref model tools/mmu_addr.py.

### C8. hazards + full regression  (Bare, then Sv32)
Done: load-use (MA→ID 1-cyc stall + forward); forwarding chains (EX/MA/WB→ID);
back-to-back; misaligned traps; full RV32IM regression in both modes.
Spec: unpriv Ch.2/12.

## TB notes
- One growing testbench: tb/tb_core.v, labeled check groups (C1-…, C2-…, …)
  matching the other TBs' "I1-I10"/"M1-M12" convention; ok/err labels ≤32 chars.
- TB provides 3 WB slaves for the core's master ports: m0 (instruction fetch),
  m1 (data/MMIO), m2 (page-table walk). In Bare pieces C1..C6, m2 is idle
  (identity, no walk). Reuse the WB slave/RAM model from tb_l1_i/tb_l1_d/tb_mmu.
- Program + data live in TB memory (m0/m1 slaves); mmu page tables in m2 slave
  for C7 (use tools/mmu_addr.py to generate the PTE map, as tb_mmu does).
- Compile: iverilog -g2005 (no `break`; sized hex literals; part-select index
  width from the tool's max address — see progress.md lessons).

## Verify before/during (spec, not memory)
- mtvec MODE handling: implement MODE=0 direct only; confirm reserved/illegal
  behavior for MODE=1 writes (design.md §7 says MODE=0 only).
- misa: implement as RO 0x40001100 (or WARL); confirm reset = maximal (3.1.1/3.4).
- CSR reset values beyond 3.4 (e.g. mtvec/mepc/mcause) → "all other hart state
  UNSPECIFIED" → zero for determinism; document.
- Counter CSRs (mcycle/minstret/cycle/instret): optional; if implemented, wire
  mcycle=minstret=1/instr, cycle/instret alias. Confirm mcountinhibit presence.
- Unaligned: trap (causes 0/4/6) per test plan A — confirm vs support.
- mtvec/scause vectored mode: out of scope (direct only) — document.
