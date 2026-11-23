# tb_soc_fw.S -- SoC firmware (runs in ROM @0x0, M-mode Bare).
#
# Boot checks (unchanged): DRAM store/load, mul, div, loop sum (words 0..4).
# Then end-to-end peripheral interrupts (M-mode, PLIC ctx0):
#   setup : mtvec=m_hnd; PLIC prio/en (src1=UART RX, src3=CLINT msip);
#           DRAM[9]/[10] zero-init; UART IER=1 (last step); mie|=MEIE; MIE=1
#   p1    : TB drives 0x2A onto rxd -> 16550 RX -> irq -> PLIC src1 -> meip
#           -> handler: claim, pop byte -> DRAM[6], complete
#   p2    : firmware TX 0x5A; TB has looped txd->rxd -> same path -> DRAM[8]
#   mtime : latch mtime lo around a delay loop -> DRAM[7] < DRAM[11]
#   msip  : CLINT msip=1 -> PLIC src3 -> handler clears, DRAM[10]=1
#   last  : sentinel DRAM[5]=0xDEADBEEF (TB polls this)
#
# DRAM word map: 0-4 data, 5 sentinel, 6 rx1 byte, 7 t0, 8 echo byte,
#   9 irq count (=3 at end: uart-rx, echo, msip), 10 msip done (=1), 11 t1,
#   12 mcause (0x8000_000B), 13 mepc (main PCs, ROM range)
#
# Registers: x1 = DRAM base, persistent (handler relies on it). x2-x4 and
# x10-x15 are temps; main never preserves a temp across a wait point.
    .section .text
    .globl _start
_start:
    lui   x1, 0x80000             # x1 = 0x8000_0000 DRAM base
    # --- boot: store/load round trip ---
    li    x2, 0xAABBCCDD
    sw    x2, 0(x1)               # DRAM[0] = 0xAABBCCDD
    lw    x3, 0(x1)
    add   x3, x3, x3
    sw    x3, 4(x1)               # DRAM[1]
    # --- M-ext: mul ---
    li    x2, 7
    li    x3, 6
    mul   x4, x2, x3
    sw    x4, 8(x1)               # DRAM[2] = 42
    # --- M-ext: div ---
    li    x2, 100
    li    x3, 7
    div   x4, x2, x3
    sw    x4, 12(x1)              # DRAM[3] = 14
    # --- taken back-ward loop: sum 1..100 = 5050 ---
    li    x2, 101
    li    x3, 0
    li    x4, 1
1:  add   x3, x3, x4
    addi  x4, x4, 1
    blt   x4, x2, 1b
    sw    x3, 16(x1)              # DRAM[4] = 5050
    # --- interrupt setup (Ier write is the LAST step) ---
    la    x2, m_hnd
    csrw  mtvec, x2
    li    x2, 1
    li    x3, 0x0C000004
    sw    x2, 0(x3)               # PLIC prio src1 = 1
    addi  x4, x3, 8
    sw    x2, 0(x4)               # PLIC prio src3 = 1
    li    x3, 0x0C002000
    li    x2, 0x0A                # en ctx0: src1 + src3
    sw    x2, 0(x3)
    li    x2, 0                   # zero-init words the handler increments
    sw    x2, 36(x1)              # DRAM[9]  = uart irq count
    sw    x2, 40(x1)              # DRAM[10] = msip done
    li    x3, 0x10000000
    li    x2, 1
    sb    x2, 1(x3)               # UART IER = 1 (DR irq)
    li    x2, 0x800
    csrs  mie, x2                 # MEIE
    csrrsi x0, 0x300, 8           # mstatus.MIE <- 1
w1:
    lw    x2, 36(x1)              # wait: phase-1 byte consumed
    li    x3, 1
    blt   x2, x3, w1
    # settle: let the TB enable loopback before we TX
    li    x2, 200
l1:
    addi  x2, x2, -1
    bnez  x2, l1
    # --- phase 2: TX 0x5A (TB loops txd->rxd) ---
    li    x2, 0x5A
    li    x3, 0x10000000
    sb    x2, 0(x3)
w2:
    lw    x2, 36(x1)              # wait: echo consumed
    li    x3, 2
    blt   x2, x3, w2
    # --- mtime monotonic ---
    li    x3, 0x0200BFF8
    lw    x2, 0(x3)
    sw    x2, 28(x1)              # DRAM[7] = t0
    li    x4, 300
d1:
    addi  x4, x4, -1
    bnez  x4, d1
    lw    x2, 0(x3)
    sw    x2, 44(x1)              # DRAM[11] = t1
    # --- CLINT msip -> PLIC src3 ---
    li    x3, 0x02000000
    li    x2, 1
    sw    x2, 0(x3)
w3:
    lw    x2, 40(x1)              # wait: msip handler done
    li    x3, 1
    blt   x2, x3, w3
2:  li    x2, 0xDEADBEEF
    sw    x2, 20(x1)              # sentinel (LAST)
3:  j     3b
m_hnd:
    li    x14, 0x0C200004         # PLIC claim/complete ctx0
    csrr  x12, mcause
    sw    x12, 48(x1)             # DRAM[12] = mcause
    csrr  x13, mepc
    sw    x13, 52(x1)             # DRAM[13] = mepc
    lw    x10, 0(x14)             # claim
    beqz  x10, hret
    lw    x11, 36(x1)
    addi  x11, x11, 1
    sw    x11, 36(x1)             # DRAM[9] = uart irq count
    li    x15, 1
    beq   x10, x15, h_uart
    li    x15, 3
    beq   x10, x15, h_msip
    j     hret
h_uart:
    li    x12, 0x10000000
    lw    x12, 0(x12)             # UART R0 word (byte in [7:0])
    andi  x12, x12, 0xFF
    li    x15, 1
    bne   x11, x15, h_u1
    sw    x12, 24(x1)             # DRAM[6] = first byte
    j     h_u2
h_u1:
    sw    x12, 32(x1)             # DRAM[8] = echo byte
h_u2:
    li    x12, 1
    sw    x12, 0(x14)             # complete(1)
    j     hret
h_msip:
    li    x12, 0x02000000
    sw    x0, 0(x12)              # CLINT msip = 0
    li    x12, 1
    sw    x12, 40(x1)             # DRAM[10] = msip done
    li    x12, 3
    sw    x12, 0(x14)             # complete(3)
hret:
    mret
