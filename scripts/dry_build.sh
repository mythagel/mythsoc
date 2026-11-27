#!/bin/sh
# dry_build.sh -- build the Dhrystone firmware benchmark -> ROM hex
# (design.md sec.10 F, t06). Two-TU compile model per the canonical
# dhry_1.c/dhry_2.c split + freestanding libc (tests/dry_lib.c).
#   usage: scripts/dry_build.sh [hex_out]   RUNS=n env -> -DDRY_RUNS=n
set -e
cd "$(dirname "$0")/.."
HEX=${1:-tb/t_fw_t06.hex}
RUNS=${RUNS:-100}
B=build/fw_c; mkdir -p build "$B"
# -fno-builtin: strcpy/strcmp/memcpy MUST be real calls (measurement
# fidelity); clang may not expand or remove them.
CFLAGS="-target riscv32-unknown-elf -march=rv32im -mabi=ilp32 -O2 \
        -ffreestanding -fno-builtin -Itests -Wall -Wextra -Werror \
        -DDRY_RUNS=$RUNS"
clang $CFLAGS -c tests/fw_entry.S  -o "$B/e.o"
clang $CFLAGS -c tests/fw_traps.S  -o "$B/r.o"
clang $CFLAGS -c tests/dry1.c      -o "$B/dry1.o"
clang $CFLAGS -c tests/dry2.c      -o "$B/dry2.o"
clang $CFLAGS -c tests/dry_lib.c   -o "$B/dryl.o"
clang --target=riscv32-unknown-elf -march=rv32im -mabi=ilp32 -nostdlib \
      -T scripts/link.lds -e _start -o "$B/dry.elf" \
      "$B/e.o" "$B/dry1.o" "$B/dry2.o" "$B/dryl.o" "$B/r.o"
# sanity: entry + dram placement, bootloader symbols, and the measured
# loop keeps REAL calls
S=$(llvm-nm "$B/dry.elf" | awk '$3 == "_start" {print $1}')
D=$(llvm-nm "$B/dry.elf" | awk '$3 == "dram"   {print $1}')
[ "$S" = "00000000" ]   || { echo "_start @ 0x$S != 0"; exit 1; }
[ "$D" = "80000000" ] || { echo "dram @ 0x$D != 0x80000000"; exit 1; }
for s in __data_start __data_end __data_load __bss_start __bss_end; do
    llvm-nm "$B/dry.elf" | grep -q " $s\$" || { echo "MISSING linker symbol $s"; exit 1; }
done
for f in Proc_1 Proc_2 Proc_3 Proc_4 Proc_5 Proc_6 Proc_7 Proc_8 \
         Func_1 Func_2 Func_3 strcpy strcmp; do
    llvm-objdump -d "$B/dry.elf" | grep -q "<$f>:" || \
        { echo "MISSING symbol $f (linker dead-code elim?)"; exit 1; }
done
# call-fidelity: main + Proc_1..8/Func_2 must CALL (jal), not inline
for f in Proc_5 Proc_4 Func_2 Proc_8 Proc_1 Proc_2 Proc_7 Proc_6 Func_1 strcpy strcmp; do
    llvm-objdump -d "$B/dry.elf" | grep -q "jal.*<$f>" || \
        { echo "NO CALL SITE for $f (inlined? measurement invalid)"; exit 1; }
done
# ROM image = .text + .data load image (LMA-adjacent; entry copies it)
llvm-objcopy --only-section=.text --only-section=.data -O binary "$B/dry.elf" "$B/dry.bin"
SZ=$(wc -c < "$B/dry.bin")
[ "$SZ" -le 16384 ] || { echo "ROM image $SZ bytes > 16 KiB"; exit 1; }
python3 -c "
import sys
d = open(sys.argv[1], 'rb').read()
d = d[:len(d)//4*4]
sys.stdout.write(''.join(format(int.from_bytes(d[i:i+4], 'little'), '08x') + '\n'
                        for i in range(0, len(d), 4)))
" "$B/dry.bin" > "$HEX"
python3 tools/fw_ranges.py tests/dry1.c "$B/dry.elf" || { echo "RANGE-FAIL"; exit 1; }
echo "built $HEX: $SZ bytes text, RUNS=$RUNS"
