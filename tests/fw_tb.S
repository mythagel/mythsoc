# fw_tb.S -- shared framework for the firmware test suite (design.md sec.10)
#
# Link order: fw_tb.o FIRST, then the test object (Makefile, fw_asm flow).
# Layout: _start (this file) -> _fw_test (the test, in the test object).
#
# Protocol (with tb/tb_fw.v):
#   x1 = DRAM base 0x8000_0000 (set below, NEVER clobbered; handlers must
#         save/restore it if they clobber regs).
#   Test data lives in DRAM words >= 16 (0..15 reserved for the result
#   block + scratch).
#   Done:   j fw_done   with x2 = 0        (pass)
#   Fail:   j fw_fail   with x2 = N        (check N failed -> result 0x8000_000N)
#   fw_fail/fw_done: DRAM[0] = result; DRAM[1] = 0x5A5A0000 (marker, LAST),
#   then spin. The TB polls DRAM[1] (bit 23) and checks DRAM[0].
    .section .text
    .globl _start
    .globl fw_done
    .globl fw_fail
_start:
    lui   x1, 0x80000             # x1 = 0x8000_0000
    j     _fw_test
fw_done:
    sw    x2, 0(x1)
    li    x3, 0x5A5A0000
    sw    x3, 4(x1)
2:  j     2b
fw_fail:
    li    x3, 0x80000000
    or    x2, x2, x3
    j     fw_done
