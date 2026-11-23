# t02_traps.S -- synchronous traps, delegation, xret (design.md sec.10 B)
# Framework: tests/fw_tb.S. x1 = DRAM base (persistent; handlers rely on it).
#
# Trap log (DRAM words 16..23):
#   [16] mcause  [17] mepc  [18] mtval  [19] m_count
#   [20] scause  [21] sepc  [22] stval  [23] s_count
# m_hnd: log, mepc+=4 (skip), force MPP=M, mret.
# s_hnd: log, sepc+=4, sret.
#
# Cause codes (priv Table 16/37): 2 illegal, 3 ebreak, 4 ld-misaligned,
# 6 st-misaligned, 9 ecall-S, 11 ecall-M. mtval: ebreak=pc, misaligned=VA,
# illegal/ecall=0 (core.md, matches tb_core C5).
    .section .text
    .globl _fw_test
    .macro chk ID
    beq   x2, x3, 2f
    li    x2, \ID
    j     fw_fail
2:
    .endm
_fw_test:
    # ---- zero log; install vectors; data area = word 30+ ----
    li    x5, 0
    sw    x5, 64(x1)
    sw    x5, 68(x1)
    sw    x5, 72(x1)
    sw    x5, 76(x1)
    sw    x5, 80(x1)
    sw    x5, 84(x1)
    sw    x5, 88(x1)
    sw    x5, 92(x1)
    la    x15, m_hnd
    csrw  mtvec, x15
    la    x15, s_hnd
    csrw  stvec, x15
    addi  x14, x1, 120             # x14 = misaligned-access base (persistent)

    # ---- T1: ebreak (M) ----
t1:
    ebreak
    lw    x2, 76(x1)
    li    x3, 1
    chk 1                          # m_count == 1
    lw    x2, 64(x1)
    li    x3, 3
    chk 2                          # mcause == 3
    lw    x2, 68(x1)
    la    x3, t1
    chk 3                          # mepc == &t1 (ebreak PC)
    lw    x2, 72(x1)
    la    x3, t1
    chk 4                          # mtval == ebreak PC (VA)

    # ---- T2: ecall from M ----
t2:
    ecall
    lw    x2, 76(x1)
    li    x3, 2
    chk 5                          # m_count == 2
    lw    x2, 64(x1)
    li    x3, 11
    chk 6                          # mcause == 11 (M ecall)
    lw    x2, 68(x1)
    la    x3, t2
    chk 7
    lw    x2, 72(x1)
    li    x3, 0
    chk 8                          # mtval == 0

    # ---- T3: illegal instruction (.word 0) ----
t3:
    .word 0
    lw    x2, 76(x1)
    li    x3, 3
    chk 9
    lw    x2, 64(x1)
    li    x3, 2
    chk 10                         # mcause == 2
    lw    x2, 68(x1)
    la    x3, t3
    chk 11
    lw    x2, 72(x1)
    li    x3, 0
    chk 12                         # mtval == 0

    # ---- T4: write to read-only CSR (misa) -> illegal ----
    li    x9, -1
t4:
    csrw  misa, x9
    lw    x2, 76(x1)
    li    x3, 4
    chk 13
    lw    x2, 64(x1)
    li    x3, 2
    chk 14                         # mcause == 2
    lw    x2, 68(x1)
    la    x3, t4
    chk 15

    # ---- T5: misaligned load ----
t5:
    lw    x6, 2(x14)
    lw    x2, 76(x1)
    li    x3, 5
    chk 16
    lw    x2, 64(x1)
    li    x3, 4
    chk 17                         # mcause == 4 (load address misaligned)
    lw    x2, 68(x1)
    la    x3, t5
    chk 18
    lw    x2, 72(x1)
    addi  x3, x14, 2
    chk 19                         # mtval == faulting VA

    # ---- T6: misaligned store ----
t6:
    sw    x0, 1(x14)
    lw    x2, 76(x1)
    li    x3, 6
    chk 20
    lw    x2, 64(x1)
    li    x3, 6
    chk 21                         # mcause == 6 (store address misaligned)
    lw    x2, 68(x1)
    la    x3, t6
    chk 22
    lw    x2, 72(x1)
    addi  x3, x14, 1
    chk 23                         # mtval == faulting VA

    # ---- T7: delegation to S: ebreak->S handler; then S ecall->M ----
    li    x9, 8
    csrw  medeleg, x9              # delegate cause 3 (breakpoint) to S
    csrc  mstatus, x9              # (x9=8 clears MIE harmlessly)
    li    x9, 0x1000
    csrc  mstatus, x9              # MPP[12] <- 0
    li    x9, 0x800
    csrs  mstatus, x9              # MPP[11] <- 1  => MPP = S (01)
    la    x9, scode
    csrw  mepc, x9
    mret                           # -> S at scode
scode:
t7:
    ebreak                         # delegated -> s_hnd (cause 3)
    lw    x2, 92(x1)
    li    x3, 1
    chk 24                         # s_count == 1
    lw    x2, 80(x1)
    li    x3, 3
    chk 25                         # scause == 3
    lw    x2, 84(x1)
    la    x3, t7
    chk 26                         # sepc == &t7
    lw    x2, 88(x1)
    la    x3, t7
    chk 27                         # stval == &t7
t8:
    ecall                          # not delegated -> m_hnd (cause 9)
afterS:
    lw    x2, 64(x1)
    li    x3, 9
    chk 28                         # mcause == 9 (S ecall)
    lw    x2, 68(x1)
    la    x3, t8
    chk 29                         # mepc == &t8
    lw    x2, 76(x1)
    li    x3, 7
    chk 30                         # m_count == 7
    li    x9, 0
    csrw  medeleg, x9              # restore: no delegation

    # ---- T9: CSR producer consumed at d=1/d=2 (CSR-forward regression;
    #      pre-fix RTL forwarded alu_res/exma_res = 0 for CSR rd) ----
    li    x9, 0x5A5A1234
    csrw  mscratch, x9
    csrr  x10, mscratch
    sw    x10, 64(x1)              # d=1 store-data consumer
    addi  x12, x10, 1              # d=2 ALU consumer
    sw    x12, 68(x1)
    csrr  x11, mscratch
    addi  x13, x0, 0               # spacer: x11 now d=2 for next store
    sw    x11, 72(x1)
    lw    x2, 64(x1)
    li    x3, 0x5A5A1234
    chk 31                         # d=1 store data == CSR value
    lw    x2, 68(x1)
    li    x3, 0x5A5A1235
    chk 32                         # d=2 ALU consumer
    lw    x2, 72(x1)
    li    x3, 0x5A5A1234
    chk 33                         # d=2 store data == CSR value

    # ---- done ----
    li    x2, 0
    j     fw_done

m_hnd:
    csrr  x19, mcause
    csrr  x20, mepc
    csrr  x21, mtval
    sw    x19, 64(x1)
    sw    x20, 68(x1)
    sw    x21, 72(x1)
    lw    x13, 76(x1)
    addi  x13, x13, 1
    sw    x13, 76(x1)
    addi  x20, x20, 4              # skip trapping instruction
    csrw  mepc, x20
    li    x15, 0x1800
    csrs  mstatus, x15             # MPP <- M (return to M after mret)
    mret

s_hnd:
    csrr  x19, scause
    csrr  x20, sepc
    csrr  x21, stval
    sw    x19, 80(x1)
    sw    x20, 84(x1)
    sw    x21, 88(x1)
    lw    x13, 92(x1)
    addi  x13, x13, 1
    sw    x13, 92(x1)
    addi  x20, x20, 4
    csrw  sepc, x20
    sret
