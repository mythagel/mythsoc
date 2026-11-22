# tb_soc_fw.S -- minimal SoC firmware (runs in ROM @0x0, M-mode Bare).
# Exercises ROM fetch, DRAM store/load, M-ext (mul/div), a taken back-ward
# loop, then writes a sentinel to DRAM word 5 (last) so the TB knows it ran.
    .section .text
    .globl _start
_start:
    lui   x1, 0x80000            # x1 = 0x8000_0000 DRAM base
    # --- store/load round trip ---
    li    x2, 0xAABBCCDD
    sw    x2, 0(x1)             # DRAM[0] = 0xAABBCCDD
    lw    x3, 0(x1)             # x3 = 0xAABBCCDD
    add   x3, x3, x3            # x3 = 2*0xAABBCCDD (wrap)
    sw    x3, 4(x1)             # DRAM[1]
    # --- M-ext: mul ---
    li    x4, 7
    li    x5, 6
    mul   x6, x4, x5            # x6 = 42
    sw    x6, 8(x1)             # DRAM[2] = 42
    # --- M-ext: div ---
    li    x4, 100
    li    x5, 7
    div   x6, x4, x5            # x6 = 14
    sw    x6, 12(x1)            # DRAM[3] = 14
    # --- taken back-ward loop: sum 1..100 = 5050 ---
    li    x4, 101               # stop (n+1)
    li    x5, 0                 # acc
    li    x6, 1                 # i
1:  add   x5, x5, x6
    addi  x6, x6, 1
    blt   x6, x4, 1b
    sw    x5, 16(x1)            # DRAM[4] = 5050
    # --- sentinel (written LAST so the TB can detect completion) ---
    li    x5, 0xDEADBEEF
    sw    x5, 20(x1)            # DRAM[5] = sentinel
2:  j     2b
