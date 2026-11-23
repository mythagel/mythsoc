#!/bin/sh
# soc_asm.sh -- build SoC firmware with clang/llvm -> ROM hex (1 word/line,
# little-endian words as $readmemh expects).
#   usage: scripts/soc_asm.sh <src.S> <hex_out>
set -e
SRC=$1; HEX=$2
BASE=${HEX%.hex}
OBJ=$BASE.o; ELF=$BASE.elf; BIN=$BASE.bin
mkdir -p "$(dirname "$HEX")"
clang -c -target riscv32-unknown-elf -march=rv32im -mabi=ilp32 -o "$OBJ" "$SRC"
# link: clang -c leaves R_RISCV_BRANCH relocs zero-filled; ld.lld resolves them
clang --target=riscv32-unknown-elf -march=rv32im -mabi=ilp32 -nostdlib \
      -nostartfiles -Wl,-Ttext=0x0 -Wl,--entry=_start -o "$ELF" "$OBJ"
llvm-objcopy -O binary "$ELF" "$BIN"
python3 -c "
import sys
d = open(sys.argv[1], 'rb').read()
d = d[:len(d)//4*4]
sys.stdout.write(''.join(format(int.from_bytes(d[i:i+4], 'little'), '08x') + '\n'
                       for i in range(0, len(d), 4)))
" "$BIN" > "$HEX"
echo "built $HEX ($(wc -l < "$HEX") words)"
