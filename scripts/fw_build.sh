#!/bin/sh
# fw_build.sh -- build one C firmware test -> ROM hex (C framework,
# design.md sec.10). Links tests/fw_entry.S (_start) + tests/fw_traps.S
# (m_shim/s_shim) + <test.c>. Layout: .text @0 (ROM), dram[] @0x80000000
# + stacks in DRAM (scripts/link.lds).
#   usage: scripts/fw_build.sh <test.c> <hex_out>
set -e
cd "$(dirname "$0")/.."
TEST=$1; HEX=$2
BASE=${HEX%.hex}
OBJ=$BASE.o; ELF=$BASE.elf; BIN=$BASE.bin
B=build/fw_c; mkdir -p build "$B"
CFLAGS="-target riscv32-unknown-elf -march=rv32im -mabi=ilp32 -O2 -Itests \
        -Wall -Wextra -Werror -ffreestanding -nostdlib"
clang $CFLAGS -c tests/fw_entry.S  -o "$B/e.o"
clang $CFLAGS -c tests/fw_traps.S  -o "$B/r.o"
clang $CFLAGS -c "$TEST"            -o "$OBJ"
clang --target=riscv32-unknown-elf -march=rv32im -mabi=ilp32 -nostdlib \
      -T scripts/link.lds -e _start -o "$ELF" "$B/e.o" "$OBJ" "$B/r.o"
# sanity: _start @0, dram @0x80000000, bootloader symbols present
S=$(llvm-nm "$ELF" | awk '$3 == "_start" {print $1}')
D=$(llvm-nm "$ELF" | awk '$3 == "dram"   {print $1}')
[ "$S" = "00000000" ]   || { echo "_start @ 0x$S != 0"; exit 1; }
[ "$D" = "80000000" ] || { echo "dram @ 0x$D != 0x80000000"; exit 1; }
for s in __data_start __data_end __data_load __bss_start __bss_end; do
    llvm-nm "$ELF" | grep -q " $s\$" || { echo "MISSING linker symbol $s"; exit 1; }
done
# ROM image = .text + .data load image (LMA-adjacent; entry copies it)
llvm-objcopy --only-section=.text --only-section=.data -O binary "$ELF" "$BIN"
SZ=$(wc -c < "$BIN")
[ "$SZ" -le 16384 ] || { echo "ROM image $SZ bytes > 16 KiB"; exit 1; }
python3 -c "
import sys
d = open(sys.argv[1], 'rb').read()
d = d[:len(d)//4*4]
sys.stdout.write(''.join(format(int.from_bytes(d[i:i+4], 'little'), '08x') + '\n'
                        for i in range(0, len(d), 4)))
" "$BIN" > "$HEX"
# verify epc-range check constants (SPANs in the test) vs the linked layout
python3 tools/fw_ranges.py "$TEST" "$ELF" || { echo "RANGE-FAIL: fix SPANs in $TEST"; exit 1; }
echo "built $HEX ($(wc -l < "$HEX") words, code $SZ bytes)"
