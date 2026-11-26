# fw_traps.S -- trap entry shims for the C firmware framework
#
# m_shim / s_shim are the mtvec / stvec entry points. On entry they save
# sp + all caller-saved regs (16 words) into a FIXED area _sav_m / _sav_s
# (NOT on the interrupted context's stack: that holds its live C frame and
# must not be touched), move sp to the private handler stack (_stk_m /
# _stk_s, sp = one past the end), call the C handler, restore,
# mret/sret. The interrupted context's register file is fully restored, so
# being interrupted mid-sequence is safe.
#
# CRITICAL: the address scratch for the save/restore is t2 (reg 5), NOT
# t3/s1 (reg 8). Using s1 for `la s1,_sav` would clobber the original t3
# BEFORE it is saved (s1 == t3 == reg 8), so t3 would be restored as
# &_sav_m -- silently corrupting the C code's s1 across the interrupt
# (t03 P1 rearm read the S-stack instead of mtime). t2 is saved too, so
# it is safe to reuse as the address.
#
# Arrays (defined by the test .c; 1024 words = 4 KiB, 16 words = 64 B):
#   _stk_m[1024] / _stk_s[1024] : handler stacks (grow down from the top)
#   _sav_m[16]    / _sav_s[16]  : fixed save areas (sp slot at [0])
# Nested M-in-S: the save areas are per-context, so a nested entry saves
# to _sav_m while s_shim's save sits untouched in _sav_s.
    .section .text
    .globl m_shim
    .globl s_shim

m_shim:
    la    t2, _sav_m
    sw    sp,    0(t2)
    sw    ra,    4(t2)
    sw    t0,    8(t2)
    sw    t1,   12(t2)
    sw    t2,   16(t2)
    sw    a0,   20(t2)
    sw    a1,   24(t2)
    sw    a2,   28(t2)
    sw    a3,   32(t2)
    sw    a4,   36(t2)
    sw    a5,   40(t2)
    sw    t3,   44(t2)
    sw    t4,   48(t2)
    sw    t5,   52(t2)
    sw    t6,   56(t2)
    sw    s0,   60(t2)
    la    t2, _stk_m
    addi  sp, t2, 1024
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    jal   ra, m_hnd_c
    la    t2, _sav_m
    lw    s0,   60(t2)
    lw    t6,   56(t2)
    lw    t5,   52(t2)
    lw    t4,   48(t2)
    lw    t3,   44(t2)
    lw    a5,   40(t2)
    lw    a4,   36(t2)
    lw    a3,   32(t2)
    lw    a2,   28(t2)
    lw    a1,   24(t2)
    lw    a0,   20(t2)
    lw    t2,   16(t2)
    lw    t1,   12(t2)
    lw    t0,    8(t2)
    lw    ra,    4(t2)
    lw    sp,    0(t2)
    mret

s_shim:
    la    t2, _sav_s
    sw    sp,    0(t2)
    sw    ra,    4(t2)
    sw    t0,    8(t2)
    sw    t1,   12(t2)
    sw    t2,   16(t2)
    sw    a0,   20(t2)
    sw    a1,   24(t2)
    sw    a2,   28(t2)
    sw    a3,   32(t2)
    sw    a4,   36(t2)
    sw    a5,   40(t2)
    sw    t3,   44(t2)
    sw    t4,   48(t2)
    sw    t5,   52(t2)
    sw    t6,   56(t2)
    sw    s0,   60(t2)
    la    t2, _stk_s
    addi  sp, t2, 1024
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    jal   ra, s_hnd_c
    la    t2, _sav_s
    lw    s0,   60(t2)
    lw    t6,   56(t2)
    lw    t5,   52(t2)
    lw    t4,   48(t2)
    lw    t3,   44(t2)
    lw    a5,   40(t2)
    lw    a4,   36(t2)
    lw    a3,   32(t2)
    lw    a2,   28(t2)
    lw    a1,   24(t2)
    lw    a0,   20(t2)
    lw    t2,   16(t2)
    lw    t1,   12(t2)
    lw    t0,    8(t2)
    lw    ra,    4(t2)
    lw    sp,    0(t2)
    sret
