# mythrv32 SoC — design notes (source of truth for this repo)

Specs in `specs/` (extracted text under /tmp/specs_txt) are authoritative:
unpriv = RV32I/M unprivileged, priv = privileged (CSR/trap/MMU/Sv32),
plic = RISC-V PLIC v1.0.0, wb = Wishbone B4.

Load bitstream: "openFPGALoader --cable dirtyJtag --freq 1000000"
Reset fpga: "openFPGALoader --cable dirtyJtag --reset"
UART FPGA pin C4 connected to USB-Serial adapter /dev/ttyUSB0
5a-75b (V8.2) board specs in ~/dev/project/chubby75/5a-75b/

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
  +0x002000  enables ctx0: word0 = src 1..31 (16 bits used; bit 0 = src 0
             does not exist -> hardwired 0, spec ch.6)
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

  0x0000  msip[0]  (bit 0 = hart 0; bits 31:1 read 0 - priv 3.2 flag format)
  0x0004  sstip[0] (bit 0 = hart 0; same bit format)   [extension]
  0x7000  mtip[0]  (bit 0 = hart 0; same bit format)
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
  Bypass: STB with empty fifo grants the pushed word directly (push@P, pop@P+1).
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
  Interrupt CAUSE code = highest-priority set bit of mie&mip[&mideleg sel]
  (priv 3.1.9: M-destined MEI11>MSI3>MTI7>SEI9>SSI1>STI5; S-destined
  SEI9>SSI1>STI5; this hart: bits 11/9 pins + 1 CSR) — session 23 review.
  On trap: save epc/mcause/mtval, set xie=0, pc = xtvec (direct only; vectored not
  implemented -> mtvec/stvec write forces MODE[1:0]=00; trap vector uses
  BASE[31:2]).
  mret/sret: pc=xepc, xie=1, xpp=cur mode, cur=xpp, sstatus/sie etc per spec.
  MPRV: cleared when xRET's target y != M (mret MPP=M keeps MPRV; sret always
  clears since SPP<M). mstatus bit 9 (SPV, H-ext) WPRI -> read-only 0 (not
  writable; masks: MSTATUS_WMASK 0x000e19aa, SSTATUS 0x000c0122).
  sip(0x144).SSIP writable when mideleg[1]=1 (priv 5.1); sie(0x104) aliases
  mie&mideleg. CSRRS/CSRRC write-attempt keyed on rs1 FIELD != x0 (unpriv
  6.1: value 0 in a real register still writes -> RO write still traps);
  CSRRSI/CSRRCI: uimm=0 no write; SYSTEM f3=100/111 reserved -> illegal.
  wfi = nop.

MMU (Sv32; spec 12.3, satp 12.1.11; addresses verified by tools/mmu_addr.py):
  satp (RV32): MODE[31] ASID[30:22](9b WARL, ASIDLEN=9) PPN[21:0].
    Table 40 (SXLEN=32): MODE 0=Bare, 1=Sv32. Bare: write satp=0.
    PPN[21:20] WARL 0: platform PA space is 32-bit; satp write masks them.
  Translation active ONLY in S/U mode; M-mode: always identity (pa=va), no MMU fault
    (spec 12.3.2 step 1: "satp must be active, i.e. effective mode S or U").
    MPRV handled at core (effective mode in = MPRV ? MPP : mode).
  walk: LEVELS=2 (Fig 65/66). L1 idx = VA[31:22] (10b), L0 idx = VA[21:12] (10b);
    PAGESIZE=4096 PTESIZE=4.
    L1 PTE addr = satp.ppn*4096 + VA[31:22]*4; L0 PTE addr = L1.pte.ppn*4096 + VA[21:12]*4.
    PA = 34b: PPN[1]=PA[33:22] (12b, from PTE[31:20]), PPN[0]=PA[21:12] (10b, from
    PTE[19:10]); PA[11:0]=VA[11:0]. SoC wires PA[31:0];
    PPN[21:20]!=0 -> derived PTE addr / leaf PA escapes the 32b SoC bus:
    the walker FAULTS directly (root: satp guard + CSR WARL mask; any PTE
    with PPN[21:20]!=0 -> structural page fault, no continue, no fill).
    NEVER truncate-and-wrap (would alias real memory; session 23 review F11).
    4K leaf: PA[31:12] = {PPN1[9:0], PPN0[9:0]}; 4M leaf: PA[31:0] = PPN1[9:0]<<22 | VA[21:0].
  PTE bits (Fig 67): V[0] R[1] W[2] X[3] U[4] G[5] A[6] D[7] RSW[9:8] PPN0[19:10] PPN1[31:20].
    validity: v=0 fault; r=0&w=1 fault; non-leaf (r=x=0): d/a/u must be 0 else fault;
    RSW ignored; leaf at L0 with r=x=0 fault (i<0).
    4M mega (leaf at L1): PPN0[9:0]==0 required (virt+phys 4M align) else fault.
    R/W/X (Table 41): 000 pointer; 001 R; 011 RW; 100 X-only; 101 RX; 111 RWX; R=0,W=1 reserved.
  Perms (12.3.2 steps 6/8/9): fetch: X (R NOT required); load: R, MXR=1 -> R|X;
    store/AMO: W (MXR no effect; AMO needs R&W, fault class=store, never load fault).
    U: fetch S->U=0 (SUM never allows S fetch on U=1); load/store S->U=0|SUM;
    U-mode -> U=1; M -> no walk.
  A/D: Svade implemented = FAULT scheme per spec 12.3 (this spec version): A=0 -> page
    fault; store/AMO with D=0 -> page fault. NO hardware A/D PTE update.
    Firmware must set A|D bits in PTEs. (Documented.)
  Faults: cause 12 fetch / 13 load+LR / 15 store+SC+AMO page fault; mtval=VA.
  TLB (per side I and D):
    L1: 16e = 4 sets x 4 ways; set VA[5:4]; tag VA[31:6] + sz + asid[8:0]/g.
    L2: 256e = 32 sets x 8 ways; set VA[12:8]; tag VA[31:13] + sz + asid/g.
    Entry: v,g,sz,asid[8:0],tagva,pa[31:12] (4M: only upper 10b PA[31:22] used;
    PA[21:12] taken from VA),r,w,x,u,a,d; LRU 4b stamp (global 4b counter).
    4K pa[31:0] = {pa12, va[11:0]}; 4M pa[31:0] = {pa12[19:10], va[21:0]}.
    Walk done -> fill L1+L2 of the REQUESTING side. Fill way: INVALID way
    first (lowest index), else min stamp (4-bit global fill counter; hits do
    NOT refresh -> FIFO within a set; invalid-first prevents stale-stamp
    wrap evictions -- t04 chk42). I-side walk stalls IF, D-side stalls MA;
    the two walkers share WB port m2 via a core grant handshake
    (GRANT_I/WANT_O/BUSY_O, D-priority): an ungranted walk-start latches
    (pend) in the mmu -> at most one STB on m2 any cycle.
    satp write: no auto-flush (SFENCE.VMA does it). Classification keys on
    rs FIELDS (spec 12.1.7.5), not values: sf_all (rs1=rs2=x0),
    sf_asid (rs1=x0, rs2!=x0 FIELD; G entries survive even if rs2 holds 0),
    sf_page (rs1!=x0 FIELD). SFENCE.VMA is S+ privilege (U-mode attempt:
    illegal, NO flush) and on commit self-redirects fetch to pc+4 (flush
    drains FIFO/in-flight like fence.i, no I$ INVAL): the ordering
    guarantee covers implicit refs of SUBSEQUENT instructions, so
    pre-sfence fetches must refetch (F6, F17).
  Page sizes: 4K / 4M only (Sv32 LEVELS=2; no 1G in Sv32).
  Walk latency (1cyc slave), IMPLEMENTED (tb M1-M13): hit = 1 cyc; 4M miss = 3;
  4K miss = 5. DONE presented the cycle the last PTE ACK lands (S_R/S_RESP
  bubbles folded; result computed combinationally from DAT_I in W1W/W2W).
  1-AHEAD: REQ_I may be asserted during the previous request's DONE cycle
  (S_RESP accept) -> sustained 1 translation/cyc on hits; core IF relies on
  this for IPC 1. Walks are 1-outstanding. SF_I never coincident with REQ_I;
  mid-walk SF held pending, applied after the walk's fill.
  WB B4 read-only master, 1 outstanding.

I-cache: 4 KiB, direct-mapped, 1-word lines (1024 x 32b), tag PA[31:12].
  Miss: stall IF, refill from memory (m0). fence.i & mret/sret: invalidate all.
  Throughput (tb I11): hit pulses DONE, stays IDLE -> sustained 1 hit/cyc.
D-cache: 4 KiB, direct-mapped, 1-word lines, write-through, no-allocate on miss.
  CACHEABLE WINDOW ONLY (sec.2): DRAM PA[31:22]==0x200 + ROM PA[31:22]==0.
  Device PAs (CLINT/PLIC/UART) are strict pass-through: never hit, never fill
  (reads have side effects: PLIC claim/pop, UART R0; a cached claim would skip
  the atomic IP-clear -> interrupt storm; found in tb_soc p2).
  Load: hit->data 1 cyc (MA); miss->m1 read (stall MA, flush pipeline).
  Store: hit->write-through m1 (stall MA); miss->direct m1 write.
  Throughput (tb I11/D11): HIT path pulses DONE and stays in IDLE -> sustained
  1 hit/cyc (back-to-back REQ on consecutive cycles, no WB). Miss/store = 3 cyc
  and 1/cyc-capped by the 1-outstanding WB slave (inherent, not a design bubble).
  No 1-ahead S_RESP state needed (unlike mmu, whose DONE sat in S_RESP).
  fence: pipeline flush (1 cycle). No coherence issue (single core, ROM code only
  modified via DRAM when I-cache disabled? -> code in ROM or invalid I-cache line:
  firmware places self-mod code in DRAM, uses fence.i; I-cache lines for DRAM
  invalidated by fence.i. OK.)

## 8. Caches note: tag compare, single-cycle; no ECC (F4K has no ECC on BRAM reads?
ECP5 BRAM: no ECC. OK.)

## 9. SoC top (sim) ports: clk, rst, uart_tx/uart_rx (16550), rom hex load, RAM init hex,
JTAG: none in sim. Board top: +sdram pads (wb_sdram_ctrl), led (T6), key (R7), clk P6 25 MHz,
jtag header (J27/J31/J32/J30) for openocd bit loading only (no in-design JTAG).

## 10. Test plan (tests/, C-first, compiled by clang)

LANGUAGE POLICY (session 26): test code is written in **C** (freestanding;
C++ acceptable) -- NOT new hand-written .S programs. Assembly only where
required: crt0/bootloader (fw_entry.S), trap shims (fw_traps.S), inline-asm
accessors C cannot express (CSR ops, sfence.vma, ecall/ebreak, raw/illegal
encodings, self-modifying blobs). Existing .S benches tb_core.S/tb_c8.S are
RTL-block harnesses (kept); t01/t02 (generated/hand asm ISA+trap encodings,
where the encoding IS the test) stay as legacy coverage. New batteries
(t07+) are C via the fw framework (tests/lib.h chk protocol).

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
- [x] 16550 + tb
- [x] clint + tb
- [x] plic + tb
- [x] wb_ic + tb (with WB slave models)
- [x] mmu + tb (walker + TLB, with WB ram): 71 checks M1-M12 all pass
- [x] l1_i + tb (23 checks I1-I10) and l1_d + tb (20 checks D1-D10)
- [x] mmu.v fully pipelined: hit 1 / 4M 3 / 4K 5 + 1-ahead (sustained 1/cyc);
      tb +M13 +invalid-first fill +m2 GRANT handshake (71 checks total)
- [x] l1_i & l1_d fully pipelined: verified sustained 1.0 hit/cyc (DONE pulses
      in IDLE; no change needed); tb +I11 (28), +D11 (24)
- [ ] core + tb (plan: notes/core.md)
  - [x] C1 fetch+decode+R/I ALU (+LUI/AUIPC), straight-line, Bare
  - [x] C2 M ext (MUL/DIV/REM x4 + H/HSU/HU)
  - [x] C3 branches+jumps+younger flush
  - [x] C4 loads/stores via l1_d (+l1_i in IF), misaligned trap
  - [x] C5 CSRs + sync exceptions (ecall/ebreak/illegal, mret, deleg)
  - [x] C6 interrupts (mip/mie, M>S, timer/sw/external, nested M->S)
  - [x] C7 MMU on (Sv32 I+D, perm faults, A/D, sfence, 4K/4M)
  - [x] C8 Bare: dense hazard/forwarding regression (36 checks, tb/tb_c8) +
        d=1 load-use bug fix + IPC measured (1/cyc hits; 0.667 steady loop)
  - [x] C8 Sv32 regression: t07_sv32_haz.c (C-first policy, session 26) --
        C8 hazard classes as C volatile dataflow under live Sv32 + alias
        triangle + I-side ROM-alias calls + asid1 rerun; 45 chk GREEN,
        trap-free (286,299 cyc)
- [x] soc top (sim) + tb (hex firmware): core+wb_ic+{ROM,DRAM,16550,CLINT,
      PLIC} wired (rtl/soc.v, wb_rom/wb_ram); tb_soc 16 checks GREEN (boot,
      UART rx irq via PLIC->meip, TX loopback echo, msip, mtime monotonic,
      mcause/mepc latched) — session 18
- [ ] firmware: tests/*.S + build (clang) — framework tests/fw_tb.S + harness
      tb/tb_fw.v + Makefile (fw image rules + regress targets):
      A: t01_isa.S GENERATED (tools/isa_gen.py) 378 checks GREEN;
      B: t02_traps.S 33 checks GREEN (T9 = CSR-forward d=1/d=2 regression);
      C: t03_intr.c 61 checks GREEN; D: t04_mmu.c 53 checks GREEN (Sv32 on
      SoC: MPRV/perm/A-D/U/SUM/MXR/4M-alias/sfence-classes/asid/satp);
      E: t05_soc.c 94 checks GREEN (interconnect/SoC battery: ROM D-reads, DRAM
stress+random-walk, null slave, region decode edges, 16550 reg scan,
loopback+FIFO+trigger-level irq, CLINT regs/mtime-RO/overlap, PLIC gateway
cycle live on mip, fence.i coherence + mret-inval);
+ fence.i/Zifencei RTL landed (core.v: FENCE/FENCE.I decode, self-redirect,
INVAL-held-until-idle; req_mmu gated by inval_p — t05 caught the held-INVAL
translation-overwrite {pc,data} mispair bug);
F: dhrystone DONE (session 24): tests/dry*.c Dhrystone 2.2a port + fw_entry.S
bootloader (.data copy + .bss zero) in tb_fw; RUNS=5 -> 2480 cyc/run,
0.229 DMIPS/MHz, 5.7 DMIPS @25 MHz (tools/dmips.py off +DUMP); exposed +
fixed 2 real bugs (rf_we_w !stall WB-drop, negative-verified by C8-T25;
missing bootloader = production gap, fixed in link.lds/fw_entry.S);
tb_fw XGUARD now always-on (fail-fast on X rf-write / fetch-ctrl X);
G(C-first): t07_sv32_haz.c 45 checks GREEN (session 26) — C8-Sv32 on-core:
dense RAW/forwarding classes as C volatile dataflow with cold-TLB walks parked
under every hazard window + alias triangle + I-side ROM-alias calls + asid1
rerun (see sec.12 C8)
- [x] full suite green (make regress: mmu l1_i l1_d wb_ic 16550 clint plic
      core(54) c8(36) t01-t07 soc(16) — session 26; t04 WDT->800k, t05 500k,
      t06 1M, t07 500k: bootloader .bss-zero adds ~215k cyc on t04, ~119k t06)
- [ ] yosys synth + nextpnr + ecppack (bitstream) + fmax report
- [ ] board lpf + openocd + boot stub (BRAM)
- [ ] docs (README + every wishbone peripheral)
