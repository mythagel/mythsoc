# link.lds -- C firmware link script (scripts/fw_build.sh)
#
# .text (code + rodata) @ 0x0 (ROM; _start first, from the entry object).
# .data: dram[] @ exactly 0x80000000 (scoreboard; tb dram_dump word i =
# DRAM byte 0x80000000 + 4i), then the test's stacks/arrays in .bss.
ENTRY(_start)
SECTIONS {
  . = 0x00000000;
  .text : {
    *(.text*)
    *(.rodata*)
  }
  . = 0x80000000;
  .data : {
    KEEP(*(.data.dram))
    *(.data*)
    *(.bss*)
    *(COMMON)
  }
}
