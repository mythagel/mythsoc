# tb_c8.S -- C8: dense hazard/forwarding stress + full RV32IM regression
#
# Each block is a controlled RAW hazard pattern; the result is stored to a
# DRAM scoreboard slot. The C8 bench (tb/tb_c8.v) checks the scoreboard +
# register file against the tools/emu32.py golden (architectural truth), so
# any forwarding/hazard bug in the RTL surfaces as a mismatch.
# Runs M-mode Bare (satp=0 reset) -> VA==PA, DRAM base 0x8000_0000.
    .section .text
    .globl _start
_start:
    lui   x1, 0x80000            # x1 = 0x8000_0000 DRAM base = scoreboard base

    # ---- T1: ALU d=1 RAW (li -> add) ----
    li    x2, 10
    add   x3, x2, x2             # x3 = 20
    sw    x3, 0(x1)              # slot0 = 20

    # ---- T2: ALU d=2 RAW (spacer) ----
    li    x2, 6
    nop
    add   x3, x2, x2             # x3 = 12
    sw    x3, 4(x1)              # slot1 = 12

    # ---- T3: ALU d=3 RAW (2 spacers) ----
    li    x2, 5
    nop
    nop
    add   x3, x2, x2             # x3 = 10
    sw    x3, 8(x1)              # slot2 = 10

    # ---- T4: ALU d=4 RAW (3 spacers) ----
    li    x2, 4
    nop
    nop
    nop
    add   x3, x2, x2             # x3 = 8
    sw    x3, 12(x1)             # slot3 = 8

    # ---- T5: load-use d=1 (lw) ----
    li    x4, 0x12345678
    sw    x4, 16(x1)             # data @ slot4
    lw    x5, 16(x1)             # x5 = 0x12345678
    add   x6, x5, x5             # d=1 load-use
    sw    x6, 20(x1)             # slot5 = 2*0x12345678 (wraps)

    # ---- T6: load-use d=1 (lb, sign-ext) ----
    li    x4, 0x80
    sb    x4, 24(x1)             # byte = 0x80
    lb    x5, 24(x1)             # x5 = 0xFFFFFF80
    addi  x6, x5, 1              # d=1: 0xFFFFFF81
    sw    x6, 28(x1)             # slot7

    # ---- T7: load-use d=1 (lh, sign-ext) ----
    li    x4, 0xC0FF
    sh    x4, 32(x1)             # half = 0xC0FF
    lh    x5, 32(x1)             # x5 = 0xFFFFC0FF
    addi  x6, x5, 1              # d=1
    sw    x6, 36(x1)             # slot9

    # ---- T8: load-use d=1 (lbu) ----
    li    x4, 0xAB
    sb    x4, 40(x1)
    lbu   x5, 40(x1)             # x5 = 0xAB
    addi  x6, x5, 1              # d=1: 0xAC
    sw    x6, 44(x1)             # slot11

    # ---- T9: load-use d=1 (lhu) ----
    li    x4, 0xC0FF
    sh    x4, 48(x1)
    lhu   x5, 48(x1)             # x5 = 0xC0FF
    addi  x6, x5, 1              # d=1: 0xC100
    sw    x6, 52(x1)             # slot13

    # ---- T10: store->load b2b (same addr) ----
    li    x4, 0xAABBCCDD
    sw    x4, 56(x1)
    lw    x5, 56(x1)             # x5 = 0xAABBCCDD
    sw    x5, 60(x1)             # slot15

    # ---- T11: store->store b2b (same addr, overwrite) ----
    li    x4, 0x11223344
    sw    x4, 64(x1)
    li    x5, 0x55667788
    sw    x5, 64(x1)             # overwrite
    lw    x6, 64(x1)             # x6 = 0x55667788
    sw    x6, 68(x1)             # slot17

    # ---- T12: load->branch (taken on loaded value) ----
    li    x4, 3
    sw    x4, 72(x1)
    lw    x5, 72(x1)             # x5 = 3
    bne   x5, x0, t12f           # taken
    li    x6, 0xDEAD             # must NOT execute
t12f:
    sw    x5, 76(x1)             # slot19 = 3

    # ---- T13: load->jump (jalr to a loaded ROM label, d=1) ----
    la    x4, t13sub            # x4 = address of t13sub (ROM code)
    sw    x4, 80(x1)           # store the address to DRAM (slot20)
    lw    x5, 80(x1)           # load it back: x5 = t13sub
    jalr  x0, 0(x5)           # jump to t13sub (d=1 use of loaded value)
t13sub:
    sw    x5, 84(x1)           # slot21 = t13sub address (reached)

    # ---- T14: MUL->MUL b2b ----
    li    x2, 6
    li    x3, 7
    mul   x4, x2, x3             # x4 = 42
    mul   x5, x4, x3             # d=1: x5 = 294
    sw    x5, 88(x1)             # slot22

    # ---- T15: DIV->use (d=1) ----
    li    x2, 100
    li    x3, 7
    div   x4, x2, x3             # x4 = 14
    addi  x5, x4, 1             # d=1: x5 = 15
    sw    x5, 92(x1)             # slot23

    # ---- T16: DIV->DIV b2b ----
    li    x2, 100
    li    x3, 7
    div   x4, x2, x3             # x4 = 14
    div   x5, x4, x3             # d=1: x5 = 14/7 = 2 (x4 from d=1)
    sw    x5, 96(x1)             # slot24 = 2

    # ---- T17: JAL link value (call/return; link read back at return) ----
    jal   x2, t17sub            # x2 = return address (PC+4), PC -> t17sub
    addi  x3, x2, 0             # return point: x3 = link
    j     t17end                # skip sub body
t17sub:
    jalr  x0, 0(x2)             # return to x2 (the return point)
t17end:
    sw    x3, 100(x1)            # slot25 = link value (x2 = return addr)

    # ---- T18: JALR link value (call/return) ----
    la    x4, t18sub            # x4 = address of t18sub
    jalr  x2, 0(x4)             # x2 = return address (PC+4), PC -> t18sub
    addi  x3, x2, 0             # return point: x3 = link
    j     t18end                # skip sub body
t18sub:
    jalr  x0, 0(x2)             # return to x2 (the return point)
t18end:
    sw    x3, 104(x1)            # slot26 = link value (x2 = return addr)

    # ---- T19: CSR read d=1 use ----
    li    x4, 0xC0FFEE
    csrw  mscratch, x4
    csrr  x2, mscratch          # x2 = 0xC0FFEE
    addi  x3, x2, 1             # d=1: x3 = 0xC0FFEF
    sw    x3, 108(x1)           # slot27

    # ---- T20: tight loop (acc/counter RAW + taken bwd branch) ----
    li    x2, 0
    li    x3, 5
t20loop:
    add   x2, x2, x3            # acc += cnt
    addi  x3, x3, -1           # cnt--
    bne   x3, x0, t20loop
    sw    x2, 112(x1)           # slot28 = 5+4+3+2+1 = 15

    # ---- T21: back-to-back same-reg overwrite ----
    li    x2, 1
    addi  x2, x2, 1             # x2 = 2
    addi  x2, x2, 1             # x2 = 3
    addi  x2, x2, 1             # x2 = 4
    sw    x2, 116(x1)           # slot29

    # ---- T22: varied ALU ops d=1 (sub/sll/slt) ----
    li    x2, 100
    sub   x3, x2, x2            # x3 = 0 (d=1)
    slli  x4, x2, 3            # x4 = 800 (d=1)
    slt   x5, x3, x2            # x5 = 1 (0<100)
    sw    x5, 120(x1)           # slot30

    # ---- T23: REM/REMU b2b use ----
    li    x2, 10
    li    x3, 4
    li    x4, 3
    rem   x5, x2, x3             # x5 = 2 (10 % 4)
    remu  x6, x5, x4             # d=1: x6 = 2 % 3 = 2 (x5 from d=1)
    sw    x6, 124(x1)           # slot31

    # ---- T24: CSR-in-MA vs younger same-rd producer (t03 forwarding hole) ----
    # csrr parks x2 in MA when the consumer is in ID (d=2); the lui between
    # redefines x2 at d=1. The younger d=1 must win; if the CSR mux outranks
    # it, mv gets 0xC0FFEE (the session-20 t03 P9 bug).
    li    x4, 0xC0FFEE
    csrw  mscratch, x4
    csrr  x2, mscratch
    lui   x2, 0x80000
    mv    x3, x2                # 0x8000_0000 (not 0xC0FFEE)
    addi  x3, x3, 7
    sw    x3, 128(x1)           # slot32 = 0x80000007

1:  j     1b                    # halt (self-loop)
