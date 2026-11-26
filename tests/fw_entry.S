# fw_entry.S -- C firmware entry (design.md sec.10, C framework)
#
# _start: sp -> top of _stk_main (2048 words), then call main() (C). main
# never returns (fw_done spins). Trap entry: fw_traps.S (mtvec/stvec).
    .section .text
    .globl _start
    .type   _start, @function
_start:
    la    sp, _stk_main
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    addi  sp, sp, 1024
    jal   ra, main
1:
    j     1b
