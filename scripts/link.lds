# link.lds -- C firmware link script (scripts/fw_build.sh, dry_build.sh)
#
# .text (code + rodata) @ 0x0 (ROM; _start first, from the entry object).
# Bootloader contract (tests/fw_entry.S runs it before main):
#   .bss   (NOLOAD @0x8000_0000): dram[] FIRST (KEEP), then all
#          uninit .bss/.sbss/COMMON -> zeroed by the entry loop.
#   .data  (PROGBITS, VMA after .bss, LMA = ROM right after .text)
#          -> ROM image copied by the entry loop.
# This matters because wb_ram boots X (rtl/wb_ram.v, board: random):
# C semantics require zeroed .bss + materialized .data or every global
# read is X (t06 forensics 2026-11-28: X -> operand -> X flush -> X mio
# -> unrecoverable fetch freeze). Assembly tests (scripts/fw_asm.sh)
# do NOT link the entry loop: they self-zero what they read (t05 rule).
ENTRY(_start)
SECTIONS {
  . = 0x00000000;
  .text : {
    *(.text*)
    *(.rodata*)
  }
  . = 0x80000000;
  .bss (NOLOAD) : {
    __bss_start = .;
    KEEP(*(.data.dram))
    *(.bss*)
    *(.sbss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end = .;
  }
  __data_start = .;
  .data : AT (ADDR(.text) + SIZEOF(.text)) {
    *(.data*)
    *(.sdata*)
  }
  __data_end = .;
  __data_load = LOADADDR(.data);
}
