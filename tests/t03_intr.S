# t03_intr.S -- interrupts through the real PLIC/CLINT (design.md sec.10 C)
#
# Delivery path under test: CLINT level -> PLIC gateway (level-latched IP)
# -> per-ctx enable/priority/threshold -> EIP -> core meip/seip -> handler
# (claim / clear source / complete). Real PLIC + CLINT in the SoC; the TB
# only provides the UART txd->rxd loopback (run with +LOOP).
#
# Cause is constant per context (ALL sources route via PLIC):
#   M: 0x8000_000B (machine external)    S: 0x8000_0009 (supervisor ext.)
# Source identity = claimed ID: 1=UART-RX 2=S-sw(sstip) 3=M-sw(msip)
#   6=S-timer(stimecmp) 7=M-timer(mtimecmp) 10=S-ext 11=M-ext (CLINT mtip
# fans out to BOTH 10 and 11).
#
# PLIC enable words: bit N = source N (plic.v). M sources {1,3,7,11} =
# bits {1,3,7,11} = 0x88A; S sources {2,6,10} = bits {2,6,10} = 0x444.
# Prio: p1=1 p2=2 p3=5 p6=1 p7=1 p10=1 p11=8.
#
# DRAM word map (x1 = base 0x8000_0000):
#   [16] mcause_last  [17] mepc_last  [18] mtval_last  [19] m_count
#   [20] m_hist_n;    M hist entry n: [22+3n] = id, [23+3n] = epc
#   [100] s_count  [101] s_hist_n;  S hist entry n: [103+3n] = id,
#          [104+3n] = epc
#   [96][97] saved x2/x3 (handlers preserve them -> main checks are atomic)
#   [98] UART RX byte   [99] handler mode: 0=normal 1=no-clear(msip)
#                        3=mret-into(msip armed at the mret of h_mtime)
#
# mie = 0xA00 (core mip: meip=bit11, seip=bit9), mideleg = 0x200 (SEIP -> S),
# sie = 0x200 (SEIE). PLIC ctx0=M (CL0 0x0C200004), ctx1=S (CL1 0x0C201004).
#
# Wait-loop epc property: the waited-on fire happens while m_count < target,
# so the loop's bne is always taken (no clean not-taken retire); epc =
# interrupted-PC+4 = loop-head+4 (or +8). chkloop checks exactly that.
#
# Checklist (58): P1 mtimecmp fire/clear/rearm, P2 msip, P3 tie->low-ID,
# P4 prio desc, P5 threshold (+claim-ignores-thr), P6 per-ctx enable gate,
# P7 complete re-arm (level held), P8 UART RX irq (src1), P9 S phase
# (SIE gate, sstip, stimecmp, S-ext, M-from-S while in S, IP latched while
# ctx-disabled), P10 mret-into-interrupt.
    .section .text
    .globl _fw_test
    .macro chk ID
    beq   x2, x3, 2f
    li    x2, \ID
    j     fw_fail
2:
    .endm
    # chkloop ID A: x2 (epc) must be A+4 or A+8 (see header). Clobbers x4.
    # (No 'g' direction marker: the LLVM integrated assembler rejects it.)
    .macro chkloop ID A
    la    x4, \A
    addi  x4, x4, 4
    beq   x2, x4, 9f
    addi  x4, x4, 4
    beq   x2, x4, 9f
    li    x2, \ID
    j     fw_fail
9:
    .endm
    # rngchk ID lo hi: lo <= x2 < hi. Clobbers x4.
    .macro rngchk ID lo hi
    la    x4, \lo
    sub   x4, x2, x4
    bltz  x4, 9f
    la    x4, \hi
    sub   x4, x2, x4
    bgez  x4, 9f
    j     8f
9:
    li    x2, \ID
    j     fw_fail
8:
    .endm
_fw_test:
    # ---- zero init (wb_ram is X at reset) ----
    li    x5, 0
    sw    x5, 64(x1)                # [16..20]
    sw    x5, 68(x1)
    sw    x5, 72(x1)
    sw    x5, 76(x1)
    sw    x5, 80(x1)
    sw    x5, 384(x1)               # [96..99]
    sw    x5, 388(x1)
    sw    x5, 392(x1)
    sw    x5, 396(x1)
    sw    x5, 400(x1)               # [100..101]
    sw    x5, 404(x1)
    # ---- vectors ----
    la    x15, m_hnd
    csrw  mtvec, x15
    la    x15, s_hnd
    csrw  stvec, x15
    # ---- PLIC: priorities (word N @ 0x0C00_0004*N) ----
    li    x15, 0x0C000004
    li    x2, 1
    sw    x2, 0(x15)                # prio(1)=1  @0x0C00_0004
    addi  x15, x15, 4
    li    x2, 2
    sw    x2, 0(x15)                # prio(2)=2  @0x0C00_0008
    addi  x15, x15, 4
    li    x2, 5
    sw    x2, 0(x15)                # prio(3)=5  @0x0C00_000C
    addi  x15, x15, 12              # @0x0C00_0018
    li    x2, 1
    sw    x2, 0(x15)                # prio(6)=1  @0x0C00_0018
    addi  x15, x15, 4
    sw    x2, 0(x15)                # prio(7)=1  @0x0C00_001C
    addi  x15, x15, 12              # @0x0C00_0028
    sw    x2, 0(x15)                # prio(10)=1 @0x0C00_0028
    addi  x15, x15, 4
    li    x2, 8
    sw    x2, 0(x15)                # prio(11)=8 @0x0C00_002C
    # ---- PLIC: enables (bit N = src N) + thresholds (0 = all pass) ----
    li    x15, 0x0C002000
    li    x2, 0x88A                 # EN0 (ctx0=M): src 1,3,7,11
    sw    x2, 0(x15)
    addi  x15, x15, 0x80
    li    x2, 0x444                 # EN1 (ctx1=S): src 2,6,10
    sw    x2, 0(x15)
    li    x15, 0x0C200000
    li    x2, 0
    sw    x2, 0(x15)                # THR0
    li    x15, 0x0C201000
    sw    x2, 0(x15)                # THR1
    # ---- UART: IER = 1 (DR irq; bit1 THRE never set) ----
    li    x15, 0x10000001
    li    x2, 1
    sb    x2, 0(x15)
    # ---- CSR: enables, then MIE=1 (intake from here on) ----
    li    x15, 0xA00
    csrw  mie, x15                  # MEIP(b11)|SEIP(b9)
    li    x15, 0x200
    csrw  mideleg, x15              # SEIP -> S
    csrw  sie, x15                  # SEIE
    csrrsi x0, 0x300, 8             # mstatus.MIE = 1

    # ================================================== P1: M timer (src7)
    li    x15, 0x0200BFF8
    lw    x2, 0(x15)                # T0 = mtime
    addi  x2, x2, 100
    li    x15, 0x02004004
    li    x3, 0
    sw    x3, 0(x15)                # word 0x1001=0: mtimecmp_hi=0,
                                    #   stimecmp_lo=0 (S stays clear)
    addi  x15, x15, -4
    sw    x2, 0(x15)                # mtimecmp = T0+100
    li    x2, 1
w_p1a:
    lw    x3, 76(x1)
    bne   x3, x2, w_p1a
w_p1a_end:
    lw    x2, 76(x1)
    li    x3, 1
    chk   1                         # m_count == 1
    lw    x2, 64(x1)
    li    x3, 0x8000000B
    chk   2                         # mcause == machine external
    lw    x2, 68(x1)
    chkloop 3 w_p1a                 # epc inside wait loop
    lw    x2, 72(x1)
    li    x3, 0
    chk   4                         # mtval == 0
    # handler cleared mtimecmp -> no refire
    li    x2, 200
p1c:
    addi  x2, x2, -1
    bnez  x2, p1c
    lw    x2, 76(x1)
    li    x3, 1
    chk   5
    # rearm: mtimecmp = mtime+100
    li    x15, 0x0200BFF8
    lw    x2, 0(x15)
    addi  x2, x2, 100
    li    x15, 0x02004000
    sw    x2, 0(x15)
    li    x2, 2
w_p1b:
    lw    x3, 76(x1)
    bne   x3, x2, w_p1b
w_p1b_end:
    lw    x2, 76(x1)
    li    x3, 2
    chk   6
    lw    x2, 68(x1)
    chkloop 7 w_p1b

    # ============================================ P2: M software (src3 msip)
    li    x15, 0x02000000
    li    x2, 1
    sw    x2, 0(x15)                # msip = 1
    li    x2, 3
w_p2:
    lw    x3, 76(x1)
    bne   x3, x2, w_p2
w_p2_end:
    lw    x2, 76(x1)
    li    x3, 3
    chk   8
    lw    x2, 64(x1)
    li    x3, 0x8000000B
    chk   9
    addi  x15, x1, 112              # M hist id[2]
    lw    x2, 0(x15)
    li    x3, 3
    chk   10
    li    x2, 200
p2s:
    addi  x2, x2, -1
    bnez  x2, p2s
    lw    x2, 76(x1)
    li    x3, 3
    chk   11                        # no refire (msip cleared)

    # ============================== P3: tie -> lower ID first (3 before 11)
    csrrci x0, 0x300, 8             # MIE = 0: arm both sources atomically
    li    x15, 0x0C00000C
    li    x2, 5
    sw    x2, 0(x15)                # prio(3)=5
    li    x15, 0x0C00002C
    li    x2, 5
    sw    x2, 0(x15)                # prio(11)=5 (tie)
    li    x15, 0x02000000
    li    x2, 1
    sw    x2, 0(x15)                # msip = 1
    li    x15, 0x02007000
    sw    x2, 0(x15)                # mtip = 1
    csrrsi x0, 0x300, 8             # MIE = 1: both pending -> tie -> low ID
    li    x2, 5
w_p3:
    lw    x3, 76(x1)
    bne   x3, x2, w_p3
w_p3_end:
    lw    x2, 76(x1)
    li    x3, 5
    chk   12
    addi  x15, x1, 124              # M hist id[3]
    lw    x2, 0(x15)
    li    x3, 3
    chk   13                        # tie -> low ID first
    addi  x15, x1, 136              # M hist id[4]
    lw    x2, 0(x15)
    li    x3, 11
    chk   14
    li    x2, 200
p3s:
    addi  x2, x2, -1
    bnez  x2, p3s
    lw    x2, 76(x1)
    li    x3, 5
    chk   15

    # ============================================ P4: priority desc (11,3)
    csrrci x0, 0x300, 8             # MIE = 0: arm both sources atomically
    li    x15, 0x0C00000C
    li    x2, 5
    sw    x2, 0(x15)                # prio(3)=5
    li    x15, 0x0C00002C
    li    x2, 8
    sw    x2, 0(x15)                # prio(11)=8
    li    x15, 0x02000000
    li    x2, 1
    sw    x2, 0(x15)
    li    x15, 0x02007000
    sw    x2, 0(x15)
    csrrsi x0, 0x300, 8             # MIE = 1: both pending -> prio desc (11,3)
    li    x2, 7
w_p4:
    lw    x3, 76(x1)
    bne   x3, x2, w_p4
w_p4_end:
    lw    x2, 76(x1)
    li    x3, 7
    chk   16
    addi  x15, x1, 148              # M hist id[5]
    lw    x2, 0(x15)
    li    x3, 11
    chk   17                        # higher prio first
    addi  x15, x1, 160              # M hist id[6]
    lw    x2, 0(x15)
    li    x3, 3
    chk   18
    li    x2, 200
p4s:
    addi  x2, x2, -1
    bnez  x2, p4s
    lw    x2, 76(x1)
    li    x3, 7
    chk   19

    # ===================== P5: threshold: EIP needs prio>thr; claim ignores
    li    x15, 0x0C200000
    li    x2, 5
    sw    x2, 0(x15)                # THR0 = 5
    li    x15, 0x02000000
    li    x2, 1
    sw    x2, 0(x15)                # msip = 1  (prio 5 <= 5: not notified)
    li    x15, 0x02007000
    sw    x2, 0(x15)                # mtip = 1  (prio 8 > 5: notified)
    li    x2, 8
w_p5:
    lw    x3, 76(x1)
    bne   x3, x2, w_p5
w_p5_end:
    lw    x2, 76(x1)
    li    x3, 8
    chk   20                        # only src11 notified
    addi  x15, x1, 172              # M hist id[7]
    lw    x2, 0(x15)
    li    x3, 11
    chk   21
    li    x2, 200
p5s:
    addi  x2, x2, -1
    bnez  x2, p5s
    lw    x2, 76(x1)
    li    x3, 8
    chk   22                        # src3 pending but not notified
    # main-code claim: returns 3 (claim ignores threshold), clears IP3
    li    x15, 0x0C200004
    lw    x2, 0(x15)
    li    x3, 3
    chk   23
    li    x15, 0x02000000
    li    x3, 0
    sw    x3, 0(x15)                # msip = 0 (level low BEFORE complete)
    li    x2, 3
    li    x15, 0x0C200004
    sw    x2, 0(x15)                # complete(3): level low -> no re-pend
    li    x15, 0x0C200000
    li    x2, 0
    sw    x2, 0(x15)                # THR0 = 0
    li    x2, 200
p5e:
    addi  x2, x2, -1
    bnez  x2, p5e
    lw    x2, 76(x1)
    li    x3, 8
    chk   24

    # ============================ P6: per-ctx enable gate (src11 ctx0)
    csrrci x0, 0x300, 8             # MIE = 0: latch + disable atomically
    li    x15, 0x02007000
    li    x2, 1
    sw    x2, 0(x15)                # mtip = 1 (src11 level -> IP latches)
    li    x15, 0x0C002000
    li    x2, 0x8A                  # EN0 without bit11: src 1,3,7
    sw    x2, 0(x15)
    csrrsi x0, 0x300, 8             # MIE = 1: src11 disabled -> no fire
    li    x2, 200
p6a:
    addi  x2, x2, -1
    bnez  x2, p6a
    lw    x2, 76(x1)
    li    x3, 8
    chk   25                        # disabled: no fire
    li    x15, 0x0C002000
    li    x2, 0x88A
    sw    x2, 0(x15)                # re-enable src11 -> fires
    li    x2, 9
w_p6:
    lw    x3, 76(x1)
    bne   x3, x2, w_p6
w_p6_end:
    lw    x2, 76(x1)
    li    x3, 9
    chk   26
    addi  x15, x1, 184              # M hist id[8]
    lw    x2, 0(x15)
    li    x3, 11
    chk   27
    li    x2, 200
p6s:
    addi  x2, x2, -1
    bnez  x2, p6s
    lw    x2, 76(x1)
    li    x3, 9
    chk   28

    # ====================== P7: complete re-arms gateway (level still high)
    addi  x15, x1, 396
    li    x2, 1
    sw    x2, 0(x15)                # mode = 1 (entry 1: no clear of msip)
    li    x15, 0x02000000
    li    x2, 1
    sw    x2, 0(x15)                # msip = 1
    li    x2, 11
w_p7:
    lw    x3, 76(x1)
    bne   x3, x2, w_p7
w_p7_end:
    lw    x2, 76(x1)
    li    x3, 11
    chk   29                        # two entries (re-arm then consumed)
    addi  x15, x1, 196              # M hist id[9]
    lw    x2, 0(x15)
    li    x3, 3
    chk   30
    addi  x15, x1, 208              # M hist id[10]
    lw    x2, 0(x15)
    li    x3, 3
    chk   31
    li    x2, 200
p7s:
    addi  x2, x2, -1
    bnez  x2, p7s
    lw    x2, 76(x1)
    li    x3, 11
    chk   32

    # ======================================= P8: UART RX irq (src1, loopback)
    li    x15, 0x10000000
    li    x2, 0x2A
    sb    x2, 0(x15)                # TX 0x2A -> TB loops txd->rxd -> RX
    li    x2, 12
w_p8:
    lw    x3, 76(x1)
    bne   x3, x2, w_p8
w_p8_end:
    lw    x2, 76(x1)
    li    x3, 12
    chk   33
    lw    x2, 64(x1)
    li    x3, 0x8000000B
    chk   34
    addi  x15, x1, 220              # M hist id[11]
    lw    x2, 0(x15)
    li    x3, 1
    chk   35
    addi  x15, x1, 392              # [98] RX byte
    lw    x2, 0(x15)
    li    x3, 0x2A
    chk   36
    li    x2, 200
p8s:
    addi  x2, x2, -1
    bnez  x2, p8s
    lw    x2, 76(x1)
    li    x3, 12
    chk   37                        # irq deasserts when FIFO empty

    # ================================================= P9: S phase (nested)
    # disable src11 in ctx0 for the S window (its IP latches anyway: the
    # gateway is per-source; it is claimed after the window -> proof)
    li    x15, 0x0C002000
    li    x2, 0x8A                  # EN0: src 1,3,7 (no 11)
    sw    x2, 0(x15)
    # arm S timer: stimecmp = T2+60 via the 0x4004 overlap word
    li    x15, 0x0200BFF8
    lw    x2, 0(x15)                # T2
    addi  x2, x2, 60
    li    x15, 0x02004004
    sw    x2, 0(x15)                # word 0x1001: stimecmp_lo=T2+60
                                    #   (mtimecmp_hi too: M timer clear)
    li    x15, 0x02004008
    li    x2, 0
    sw    x2, 0(x15)                # stimecmp_hi=0 -> S-timer = T2+60
    li    x15, 0x02000004
    li    x2, 1
    sw    x2, 0(x15)                # sstip = 1 (src2)
    li    x2, 2
    csrc  sstatus, x2               # SIE = 0 at S entry (SIE gate test)
    li    x2, 0x800
    csrs  mstatus, x2               # MPP = S
    la    x2, s_code
    csrw  mepc, x2
    mret
s_code:
    # (S mode, SIE=0): src2 pending but SIE-gated
    li    x2, 100
s_busy:
    addi  x2, x2, -1
    bnez  x2, s_busy
s_ie1:
    li    x2, 2
    csrs  sstatus, x2               # SIE = 1 -> S1: sstip (src2)
    addi  x2, x2, 0
    li    x15, 0x02007000
    li    x3, 1
    sw    x3, 0(x15)                # mtip = 1 -> src10(S)+src11(M, ctx0-dis)
    li    x15, 0x02000000
    sw    x3, 0(x15)                # msip = 1 -> M-from-S (src3)
    li    x2, 40
s_w2:
    addi  x2, x2, -1
    bnez  x2, s_w2
    li    x2, 0x1800
    csrs  mstatus, x2               # MPP = M
    la    x2, m_afterS
    csrw  mepc, x2
    mret
s_code_end:
m_afterS:
    li    x15, 0x0C002000
    li    x2, 0x88A
    sw    x2, 0(x15)                # restore EN0: src11 IP latched during
                                    #   the window -> one M entry, id 11
    li    x2, 14
w_p9:
    lw    x3, 76(x1)
    bne   x3, x2, w_p9
w_p9_end:
    lw    x2, 400(x1)
    li    x3, 3
    chk   38                        # s_count == 3
    addi  x15, x1, 412              # S hist id[0]
    lw    x2, 0(x15)
    li    x3, 2
    chk   39
    addi  x15, x1, 424              # S hist id[1]
    lw    x2, 0(x15)
    li    x3, 6
    chk   40                        # tie prio 1 -> low ID: 6 before 10
    addi  x15, x1, 436              # S hist id[2]
    lw    x2, 0(x15)
    li    x3, 10
    chk   41
    addi  x15, x1, 416              # S hist epc[0]
    lw    x2, 0(x15)
    rngchk 42 s_ie1 s_code_end      # S1 only after SIE=1 (SIE gate)
    addi  x15, x1, 428              # S hist epc[1]
    lw    x2, 0(x15)
    rngchk 43 s_code s_code_end
    addi  x15, x1, 440              # S hist epc[2]
    lw    x2, 0(x15)
    rngchk 44 s_code s_code_end
    lw    x2, 76(x1)
    li    x3, 14
    chk   45                        # m_count: +M-from-S(src3) +src11
    addi  x15, x1, 232              # M hist id[12]
    lw    x2, 0(x15)
    li    x3, 3
    chk   46                        # M-from-S: taken while in S
    lw    x2, 236(x1)               # M hist epc[12]
    rngchk 47 s_code s_code_end
    addi  x15, x1, 244              # M hist id[13]
    lw    x2, 0(x15)
    li    x3, 11
    chk   48                        # IP latched while ctx0-disabled
    li    x2, 200
p9s:
    addi  x2, x2, -1
    bnez  x2, p9s
    lw    x2, 400(x1)
    li    x3, 3
    chk   49                        # S sources consumed: stable
    lw    x2, 76(x1)
    li    x3, 14
    chk   50

    # ====================== P10: mret-into-interrupt (msip at the mret)
    addi  x15, x1, 396
    li    x2, 3
    sw    x2, 0(x15)                # mode = 3
    li    x15, 0x0200BFF8
    lw    x2, 0(x15)                # T3
    addi  x2, x2, 100
    li    x15, 0x02004004
    li    x3, 0
    sw    x3, 0(x15)                # word 0x1001 = 0 (S stays clear)
    addi  x15, x15, -4
    sw    x2, 0(x15)                # mtimecmp = T3+100
    li    x2, 16
w_p10:
    lw    x3, 76(x1)
    bne   x3, x2, w_p10
w_p10_end:
    lw    x2, 76(x1)
    li    x3, 16
    chk   51
    addi  x15, x1, 256              # M hist id[14]
    lw    x2, 0(x15)
    li    x3, 7
    chk   52
    addi  x15, x1, 268              # M hist id[15]
    lw    x2, 0(x15)
    li    x3, 3
    chk   53                        # taken right after the handler mret
    lw    x2, 260(x1)               # M hist epc[14]
    chkloop 54 w_p10
    lw    x2, 272(x1)               # M hist epc[15]
    chkloop 55 w_p10
    li    x2, 200
p10s:
    addi  x2, x2, -1
    bnez  x2, p10s
    lw    x2, 76(x1)
    li    x3, 16
    chk   56

    # ---- final: nothing pending anywhere ----
    li    x15, 0x0C200004
    lw    x2, 0(x15)
    li    x3, 0
    chk   57                        # claim ctx0 empty
    li    x15, 0x0C201004
    lw    x2, 0(x15)
    chk   58                        # claim ctx1 empty
    li    x2, 0
    j     fw_done

# ============================================================ handlers
# m_hnd: log, claim(CL0), dispatch by id, complete, mret.
# Preserves x1, x2, x3 (main's check values are atomic wrt this handler).
m_hnd:
    sw    x2, 384(x1)
    sw    x3, 388(x1)
    csrr  x10, mcause
    csrr  x11, mepc
    csrr  x12, mtval
    sw    x10, 64(x1)
    sw    x11, 68(x1)
    sw    x12, 72(x1)
    lw    x13, 76(x1)
    addi  x13, x13, 1
    sw    x13, 76(x1)               # m_count
    lw    x13, 80(x1)               # m_hist_n
    slli  x14, x13, 1
    add   x14, x14, x13             # 3n
    slli  x14, x14, 2
    add   x14, x14, x1
    addi  x14, x14, 84              # x1 + 4*(21+3n)
    li    x15, 0x0C200004
    lw    x16, 0(x15)               # claim -> ID
    sw    x16, 4(x14)               # id
    sw    x11, 8(x14)               # epc (entry value; core already set
                                     #   mepc = interrupted-PC+4 for int)
    addi  x13, x13, 1
    sw    x13, 80(x1)               # m_hist_n++
    beqz  x16, h_mret               # defensive: nothing claimed
    li    x15, 1
    beq   x16, x15, h_uart
    li    x15, 3
    beq   x16, x15, h_msip
    li    x15, 7
    beq   x16, x15, h_mtime
    li    x15, 11
    beq   x16, x15, h_mext
    j     h_mret                    # unknown id: just return
h_uart:
    li    x12, 0x10000000
    lw    x12, 0(x12)               # R0: pops one byte
    andi  x12, x12, 0xFF
    addi  x15, x1, 392
    sw    x12, 0(x15)               # [98]
    li    x12, 1
    li    x15, 0x0C200004
    sw    x12, 0(x15)               # complete(1)
    j     h_mret
h_msip:
    addi  x15, x1, 396
    lw    x15, 0(x15)               # mode
    beqz  x15, h_msip_cl
    li    x12, 1
    beq   x15, x12, h_msip_nc
    j     h_msip_cl                 # mode 3 consumed in h_mtime
h_msip_cl:
    li    x12, 0x02000000
    li    x13, 0
    sw    x13, 0(x12)               # msip = 0
    li    x12, 3
    li    x15, 0x0C200004
    sw    x12, 0(x15)               # complete(3)
    j     h_mret
h_msip_nc:
    addi  x15, x1, 396
    li    x13, 0
    sw    x13, 0(x15)               # mode -> 0 (next entry normal)
    li    x12, 3
    li    x15, 0x0C200004
    sw    x12, 0(x15)               # complete(3); msip LEFT -> re-arm
    j     h_mret
h_mtime:
    addi  x15, x1, 396
    lw    x15, 0(x15)               # mode
    beqz  x15, h_mtime_cl
    li    x12, 3
    beq   x15, x12, h_mtime_mri
    j     h_mtime_cl
h_mtime_cl:
    li    x12, 0x02004000
    li    x13, -1
    sw    x13, 0(x12)               # mtimecmp_lo = -1 (clear)
    li    x12, 7
    li    x15, 0x0C200004
    sw    x12, 0(x15)               # complete(7)
    j     h_mret
h_mtime_mri:
    li    x12, 0x02004000
    li    x13, -1
    sw    x13, 0(x12)               # clear mtimecmp
    li    x12, 7
    li    x15, 0x0C200004
    sw    x12, 0(x15)               # complete(7)
    addi  x15, x1, 396
    li    x13, 0
    sw    x13, 0(x15)               # mode -> 0
    li    x15, 0x02000000
    li    x13, 1
    sw    x13, 0(x15)               # msip = 1 (pending at the mret)
    csrrsi x0, 0x300, 8             # MIE = 1
    j     h_mret                    # -> mret: taken at next clean retire
h_mext:
    li    x12, 0x02007000
    li    x13, 0
    sw    x13, 0(x12)               # mtip = 0
    li    x12, 11
    li    x15, 0x0C200004
    sw    x12, 0(x15)               # complete(11)
    j     h_mret
h_mret:
    lw    x2, 384(x1)
    lw    x3, 388(x1)
    mret

# s_hnd: log, claim(CL1), dispatch by id (2/6/10), complete, sret.
s_hnd:
    sw    x2, 384(x1)
    sw    x3, 388(x1)
    csrr  x10, scause
    csrr  x11, sepc
    csrr  x12, stval
    lw    x13, 400(x1)
    addi  x13, x13, 1
    sw    x13, 400(x1)              # s_count
    lw    x13, 404(x1)              # s_hist_n
    slli  x14, x13, 1
    add   x14, x14, x13
    slli  x14, x14, 2
    add   x14, x14, x1
    addi  x14, x14, 408             # x1 + 4*(102+3n)
    li    x15, 0x0C201004
    lw    x16, 0(x15)               # claim ctx1 -> ID
    sw    x16, 4(x14)
    sw    x11, 8(x14)               # epc (entry value; core already set
                                     #   sepc = interrupted-PC+4 for int)
    addi  x13, x13, 1
    sw    x13, 404(x1)
    beqz  x16, s_mret
    li    x12, 2
    beq   x16, x12, h_ssw
    li    x12, 6
    beq   x16, x12, h_stmr
    li    x12, 10
    beq   x16, x12, h_sext
    j     s_mret
h_ssw:
    li    x12, 0x02000004
    li    x13, 0
    sw    x13, 0(x12)               # sstip = 0
    li    x12, 2
    li    x15, 0x0C201004
    sw    x12, 0(x15)               # complete(2)
    j     s_mret
h_stmr:
    li    x12, 0x02004008
    li    x13, -1
    sw    x13, 0(x12)               # stimecmp_hi = -1 (clear)
    li    x12, 6
    li    x15, 0x0C201004
    sw    x12, 0(x15)               # complete(6)
    j     s_mret
h_sext:
    li    x12, 0x02007000
    li    x13, 0
    sw    x13, 0(x12)               # mtip = 0
    li    x12, 10
    li    x15, 0x0C201004
    sw    x12, 0(x15)               # complete(10)
    j     s_mret
s_mret:
    lw    x2, 384(x1)
    lw    x3, 388(x1)
    sret
