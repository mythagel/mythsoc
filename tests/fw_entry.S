# fw_entry.S -- C firmware entry (design.md sec.10, C framework)
#
# _start: bootloader contract per scripts/link.lds -- copy the .data ROM
# image to its DRAM VMA, zero .bss (dram[] scoreboard + stacks; wb_ram
# powers up X, so this is MANDATORY for defined C semantics), then sp ->
# top of _stk_main (4 x 1024 words) and call main(). main never returns
# (fw_done spins). Trap entry: fw_traps.S (mtvec/stvec).
#
# NOTE: runs before any stack exists (no push/pop). Word loops only:
# all VMA/LMA are 4-aligned by the linker script (ALIGN(4) on section
# ends; rv32 objects are at most 8-aligned but nothing spans here).
    .section .text
    .globl _start
    .type   _start, @function
_start:
    la    t0, __data_start
    la    t1, __data_end
    la    t2, __data_load
.Lcp:
    bgeu  t0, t1, .Lzs
    lw    t3, 0(t2)
    sw    t3, 0(t0)
    addi  t0, t0, 4
    addi  t2, t2, 4
    j     .Lcp
.Lzs:
    la    t0, __bss_start
    la    t1, __bss_end
.Lz:
    bgeu  t0, t1, .Lsp
    sw    x0, 0(t0)
    addi  t0, t0, 4
    j     .Lz
.Lsp:
    la    sp, _stk_main
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    jal   ra, main
1:
    j     1b
