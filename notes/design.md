# mythrv32 SoC — design notes (source of truth for this repo)

Specs in `specs/` (extracted text under /tmp/specs_txt) are authoritative:
unpriv = RV32I/M unprivileged, priv = privileged (CSR/trap/MMU/Sv32),
plic = RISC-V PLIC v1.0.0, wb = Wishbone B4.

## 1. System overview

mythrv32: RV32IM, M+S+U, 5-stage in-order, Sv32 MMU (L1/L2 TLB + page walk),
4 KiB direct-mapped L1 I/D caches (D: write-through, no-allocate), 1 hart.
SoC: pipelined Wishbone B4 interconnect (3 masters x 5 slaves), 16550 UART,
CLINT, PLIC (16 sources, 2 contexts: M=0, S=1 — ratified PLIC spec has no U context),
ROM (boot), DRAM (sim: WB RAM; board: SDRAM via existing wb_sdram_ctrl).

Target: Lattice ECP5 LFE5U-25F (BG256) on Colorlight 5a-75b v8.2, 25 MHz sysclk.
Performance goal: synth/P&R closure >= 25 MHz (min), target 60 MHz, ideal 80 MHz.

## 2. Memory map (32-bit VA/PA, 4 KiB pages)

| region   | base        | size    | device |
|----------|-------------|---------|--------|
| ROM      | 0x0000_0000 | 32 KiB  | boot image (sim: hex; board: BRAM boot stub) |
| CLINT    | 0x0200_0000 | 64 KiB  | msip/sstip/mtip/mtimecmp/stimecmp/mtime |
| PLIC     | 0x0C00_0000 | 4 MiB   | 16 sources, contexts 0=M, 1=S |
| UART16550| 0x1000_0000 | 4 KiB   | 16550-compatible |
| DRAM     | 0x8000_0000 | 4 MiB   | sim RAM / board SDRAM (8 MB part, 4 MiB mapped) |

Unmapped reads return 0 (no ERR, documented). Writes to unmapped regions dropped.
(Keeps WB simple; all tests use mapped regions.)

## 3. PLIC (spec v1.0.0, 16 sources, contexts: 0=M, 1=S)

Sources (level-sensitive gateways, latched IP; completion re-arms level inputs):
  1  = 16550 UART RX (FIFO nonempty)
  2  = S software  (CLINT sstip bit0)   [design extension: sstip reg]
  3  = M software  (CLINT msip bit0)
  6  = S timer     (mtime >= stimecmp)
  7  = M timer     (mtime >= mtimecmp)
 10  = S external  (CLINT mtip bit0)
 11  = M external  (CLINT mtip bit0)
 4,5,8,9,12..16 reserved (no input, never pending)

Register map (per spec ch.3):
  +0x000000  reserved (src 0)
  +0x000004  source 1 priority ... +0x00003C source 16 priority  (4B stride)
  +0x001000  pending word 0 = sources 1..31 bits (bit N = src 32k+N) [16 src -> 1 word]
  +0x002000  enables ctx0: word0 = src 1..31 (16 bits used)
  +0x002080  enables ctx1 (same layout)            (ctx stride 0x80, 2 words each)
  +0x200000  threshold ctx0   +0x201000 threshold ctx1
  +0x200004  claim/complete ctx0  +0x201004 claim/complete ctx1
Semantics:
  priority 0 = never; higher = higher priority; ties -> lower ID wins.
  EIP(ctx) = OR over sources: IP & enable(ctx) & prio>0 & prio>threshold(ctx).
  claim (read): atomically selects highest (prio desc, ID asc) among IP&enable,
  clears that IP, returns ID (0 if none). Not affected by threshold.
  complete (write ID): gateway re-arm (level re-evaluated).
  Writes to claim reg of 0 or to priority/pending ignored appropriately:
    claim write only acts as complete; read = claim.
Notification: eip_m -> hart meip, eip_s -> hart seip (mip/sip bits), 1 bit each.

## 4. CLINT (base 0x0200_0000)

  0x0000  msip[0]  (bit 0 = hart 0)
  0x0004  sstip[0] (bit 0 = hart 0)   [extension]
  0x7000  mtip[0]  (bit 0 = hart 0)
  0x4000  mtimecmp[0] low, 0x4004 high (64-bit)
  0x4004+0x8=0x400C? -> stimecmp[0] low 0x400C? NO: standard: mtimecmp i at 0x4000+i*8,
         stimecmp i at 0x4004+i*8. So: mtimecmp 0x4000/0x4004, stimecmp 0x4004/0x4008.
         (Both 64-bit, little-endian lo/hi.)
  0xBFF8  mtime low, 0xBFFC mtime high (64-bit free-running, +1 per clk cycle default;
          parameter MTIME_PERIOD (default 1))
Interrupts to PLIC sources 3,2,7,6,11,10 per table above. All level-driven.

## 5. 16550 UART (base 0x1000_0000), 115200 8N1 default

Standard 16550 registers (offset 0, byte addressing; LW/SW OK, unaligned OK via sel):
  0 RX FIFO (64) / 1 THRU IER / 2 IIR+FIFO (read), LCR (write) / 3 SCR /
  4 MSR (read-only: THD=1, TEMT=1 always) / 5 MSR scratch (RW) /
  7 LSR: bit0 DR, bit5 TEMT, bit6 THRE (always 1)
  FIFO enable via FCR (offset 2 write): bit0 reset, bit1 tx empty, bit2 rx reset,
  bits[7:6] rx trigger (1/2/3/4), [5:4] tx empty level (1/8/16/28)
IRQ: level = (FIFO enabled & rx count > 0) | (IER bit1 THRE & THRE) | (IER bit0 DR & DR).
UART->PLIC source 1. RX: async sampling, 3-vote. TX: shift, TXE/THRE levels.
Parameters: BAUD, UART_CLK_DIV (clk = sysclk). Sim default: div=1, baud=1 (fast);
board: div for 115200 @ 25 MHz? board clock 25 MHz: div 217 -> use 115200/25e6.
TB: div=1, bit time 16.

## 6. Wishbone B4 interconnect (wb_ic) — FINAL protocol

Masters (3): m0 = I-cache (read only), m1 = D-cache (read/write), m2 = MMU walk (read only).
Slaves (5): s0 ROM, s1 DRAM, s2 UART, s3 CLINT, s4 PLIC. + virtual s5 "null" (unmapped).
DRAM sim = wb_ram (BRAM, 1-cycle). DRAM board = dram_adapter -> wb_sdram_ctrl (WB_PORTS=1):
  its WB port requires CYC & STB HELD until ACK (PicoRV32 style, no STALL); adapter converts
  wb_ic slave protocol (single STB, wait for ACK) to held STB+CYC. (Board stage; sim never needs it.)

Master-facing protocol (IC as slave to master m), per master:
- Master asserts STB[m]=1 for exactly 1 cycle per transfer; {WE,SEL,ADR,DAT} valid on that cycle.
- STALL[m]=1 while master m has 4 queued transfers (FIFO full). Master must not STB while STALL.
- IC asserts ACK[m]=1 for exactly 1 cycle per accepted transfer, strictly in issue order;
  DAT[m] valid on ACK cycles for reads. No ERR (everything completes; unmapped read returns 0).
- Master may assert STB[m] on the same cycle it latches ACK[m].

Slave-facing protocol (IC as master to slave s), per slave:
- IC asserts STB[s]=1 for exactly 1 cycle per transfer (never to a busy slave; 1 outstanding max).
- Slave asserts ACK[s]=1 for exactly 1 cycle per accepted transfer, >=1 cycle later; DAT[s]
  valid on that cycle for reads. No STALL, no ERR, no CYC (IC sets nothing else; the held-ACK
  slaves that need it live behind an adapter).
- Address decode (verified by tools/addr.py; mutually exclusive):
    s0 ROM  : ADR[31:15] == 17'h00000   (0x0000_0000, 32 KiB)
    s1 DRAM : ADR[31:22] == 10'h200     (0x8000_0000, 4 MiB)
    s2 UART : ADR[31:12] == 20'h10000   (0x1000_0000, 4 KiB)
    s3 CLINT: ADR[31:16] == 16'h0200    (0x0200_0000, 64 KiB)
    s4 PLIC : ADR[31:22] == 10'h030     (0x0C00_0000, 4 MiB)
    s5 null : otherwise (read->0, write dropped, 1-cycle ACK)

IC internal design (pipelined; up to 3 transfers in flight, one per master):
- Per master: shift FIFO depth 4, entry = {we, sel[3:0], adr[31:0], dat[31:0]} (69b);
  state[m] = IDLE | ATSLAVE (1 outstanding per master => per-master in-order by construction).
- Grant: m in IDLE, fifo nonempty, slave s=decode(head.adr) not busy (and s5 always ok).
  Fixed priority m0 > m1 > m2, one grant/cycle. Grant = present (STB[s]=1 with FIFO head data,
  popped), m -> ATSLAVE, busy[s]=1, sl_master[s]=m. (Since IC never presents to a busy slave and
  all slaves accept a presented transfer, grant == accepted; no retry/hold path.)
- Return (combinational, same cycle as slave ACK): ACK[sl_master[s]]=1,
  DAT[sl_master[s]] = slave_DAT[s] (reads); then m -> IDLE, busy[s]=0.
  Null slave: grant -> 1 internal cycle -> "ACK" with DAT=0 (same latency as 1-cycle slaves).
- Latency: 1-cycle slave => master STB at c, master ACK at c+1 (1 cycle total).
  Multi-cycle slave => master ACK at slave's ACK cycle.

## 7. Core (mythrv32) — 5-stage in-order RV32IM

IF: PC -> MMU-I (VA->PA, TLB) -> I-cache -> fetch word. 1 instr/cycle (32-bit).
ID: decode, 2 regfile reads (3 R ports), CSR read. Forwarding from EX/MA/WB.
EX: ALU (add/sub/and/or/xor/sll/srl/sra, SLT/SLTU, MULDIV mul/div/divu/rem/remu),
    branch compare, PC targets (branch: pc+imm; jal/jalr: imm/rd; csrr: pc+4).
MA: load/store via D-cache (VA->PA via MMU-D; PA to cache), branch/jump commit,
    ALU result held; trap/interrupt commit point (see below).
WB: register file write (ALU/branch results), load data written in MA (2 regfile
    write ports: MA port + WB port; same-target conflict -> MA (older) wins).
Hazard control:
  - load-use (MA->EX/ID dependency): 1-cycle stall, data forwarded after.
  - regfile: 3 read (ID) + 2 write (MA, WB).
  - branches not predicted; flush younger (IF/ID/EX) on MA resolution.
Interrupts/traps (checked at MA, commit boundary, in-order):
  traps from current instr: instruction page fault (2), illegal (1),
    misaligned (0), load page fault (4), store page fault (6), ecall (8/9/11), ebreak (3).
  mtval: page fault VA / illegal instr / ecall? (mtval=VA for pf, =instr for illegal? no:
  mtval: instruction pf=VA; illegal=0; misaligned=VA; load pf=VA; store pf=VA; ecall=instr; ebreak=VA).
  (Per priv spec: load/store/instr page fault -> VA; misaligned -> VA; illegal -> 0? ->
  see priv spec section 17.4.1; implement exactly per spec text.)
  Delegation: mideleg bit(c) -> S-mode: scause/sepc/stval; else M: mcause/mepc/mtval.
  Interrupts: pending = (mip/mie & mstatus.MIE) for M, (sip/sie & sstatus.SIE) for S.
  mstatus.MIE gates all; sstatus.SIE gates S-context only. Priority M>S.
  On trap: save epc/mcause/mtval, set xie=0, pc = xtvec (direct only; vectored not
  implemented -> mtvec.MODE=0 always; vectored = reserved->trap to 0? we set MODE=0 only,
  writes with MODE=1 ignored).
  mret/sret: pc=xepc, xie=1, xpp=cur mode, cur=xpp, sstatus/sie etc per spec.
  wfi = nop.

MMU (Sv32, 4-level):
  mode: satp.MODE=8 (Sv32) enables translation for all modes (M/S/U).
  walk: level 3..0, index bits VA[38:30]/[29:21]/[20:12]/[11:3], base = pgd<<12.
  PTE: [31]D [30]A [22:10]PPN [3]U [2]X [1]W [0]R.
    leaf types: 4K (R|W set, or R&W&X? R=1,W=1 is PTD -> table; 4K leaf: exactly one of
    R/W or X with (R|W|X)!=0 and not (R&W)); superpage 4M: X=1,R=0,W=0 at level 1;
    1G: X=1,R=0,W=0 at level 2; PTD: R=1,W=1 (X don't care -> treated as X=0); invalid: all 0 or W=1,R=0 (with X=0?)
    -> follow priv spec exactly (Sv32 PTE validity rules).
  A/D: NO hardware update; PTE with A=0 or (W access & D=0) -> page fault.
    (Firmware must set A|D bits. Documented.)
  Privilege checks: M: none; S: PTE.U=0 required; U: PTE.U=1 required.
  Perms: fetch needs X (and R for M/S/U all), load R, store W.
  TLB: L1 I: 16-entry 4x4, L1 D: 16-entry 4x4, L2: 256-entry 32x8 (VA[11:8]), LRU.
    Entry: va[31:0], pa[31:12] (+page size), perms[2:0], u. On walk done: fill L1+L2.
    Walk: MMU m2 WB master; 4 reads; I-side walk stalls IF, D-side stalls MA;
    walk result used by both (serialized single walk unit).
  Page sizes: 4K/4M/1G supported.

I-cache: 4 KiB, direct-mapped, 1-word lines (1024 x 32b), tag PA[31:12].
  Miss: stall IF, refill from memory (m0). fence.i & mret/sret: invalidate all.
D-cache: 4 KiB, direct-mapped, 1-word lines, write-through, no-allocate on miss.
  Load: hit->data 1 cyc (MA); miss->m1 read (stall MA, flush pipeline).
  Store: hit->write-through m1 (stall MA); miss->direct m1 write.
  fence: pipeline flush (1 cycle). No coherence issue (single core, ROM code only
  modified via DRAM when I-cache disabled? -> code in ROM or invalid I-cache line:
  firmware places self-mod code in DRAM, uses fence.i; I-cache lines for DRAM
  invalidated by fence.i. OK.)

## 8. Caches note: tag compare, single-cycle; no ECC (F4K has no ECC on BRAM reads?
ECP5 BRAM: no ECC. OK.)

## 9. SoC top (sim) ports: clk, rst, uart_tx/uart_rx (16550), rom hex load, RAM init hex,
JTAG: none in sim. Board top: +sdram pads (wb_sdram_ctrl), led (T6), key (R7), clk P6 25 MHz,
jtag header (J27/J31/J32/J30) for openocd bit loading only (no in-design JTAG).

## 10. Test plan (tests/, hand-written .S, compiled by clang)

lib.S: framework (pass/fail via UART "PASS/FAIL name", trap catcher printing mcause).
A. ISA: all RV32I instructions (reg/imm, all ALU, branches, jumps, all loads/stores,
   CSR rd/rs/csr/wfi, fence), M ext (8 ops incl. div/rem by zero, 0x80000000 edge cases),
   forwarding chains, back-to-back hazards, misaligned (must trap) checks.
B. Traps: each mcause value, mepc/mtval contents, mtvec, ecall from U/S/M, ebreak,
   illegal instr, instruction/load/store page fault causes + VA, delegation to S,
   mret/sret return, interrupt disable/enable.
C. Interrupts: CLINT mtimecmp (M+S), msip/sstip (M+S), mtip (M+S), mtimecmp clear/re-arm,
   PLIC: priorities (tie-break by ID), threshold, enables, claim/complete, UART RX irq,
   mret into interrupt, nested (M->S).
D. MMU: Sv32 identity map, swap mapping, R/W/X permission faults, A/D enforcement,
   U-bit checks (U-mode page in S, S page in U, M bypass), 4K/4M/1G pages,
   TLB invalidation via satp clear, mret with satp change, I/D side, page walk
   multi-level (all 4 levels used), PTE invalid (all-zero), misaligned VA->pf.
E. Interconnect/SoC: ROM read, DRAM stress (random walk pattern), UART loopback
   (TB injects chars), 16550 reg scan, CLINT mtime monotonic, PLIC regs.
F. Perf: Dhrystone (embedded) cycle count -> DMIPS/MHz via TB cycle count;
   also yosys/nextpnr fmax (25/60/80 MHz gate).

## 11. Build (Makefile)

- sim: iverilog -g2005 (yosys ecp5 cells not needed for sim top; board top uses
  yosys cells_sim.v for ecp5 RAMs — or sim with generic RAM impl).
- synth: yosys -p 'read_verilog ...; synth_ecp5 -top soc/top -abc9 -json build/soc.json'
- pnr: nextpnr-ecp5 --25k --package CABGA256 --speed 6 --lpf lpf/mythrv32-5a-75b-v82.lpf
  --json build/top.json --textcfg build/top.config --freq 25
- bit: ecppack --compress --input build/top.config --bit build/top.bit
- openocd: ftdi digilent? board JTAG via 4-pin header; script in board/.

## 12. Status tracker
- [x] scaffold, board/spec recon
- [ ] 16550 + tb
- [ ] clint + tb
- [ ] plic + tb
- [ ] wb_ic + tb (with WB slave models)
- [ ] mmu + tb (walker + TLB, with WB ram)
- [ ] l1_i/l1_d + tb
- [ ] core + tb (RV32IM w/o MMU first, then MMU on)
- [ ] soc + tb (hex firmware)
- [ ] firmware: tests/*.S + build (clang)
- [ ] full suite green
- [ ] yosys synth + nextpnr + ecppack (bitstream) + fmax report
- [ ] board lpf + openocd + boot stub (BRAM)
- [ ] docs (README)
