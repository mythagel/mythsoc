#!/bin/sh
# fw_asm.sh -- build one firmware test (fw_tb.S + <test.S>) -> ROM hex.
# fw_tb.o must link FIRST (layout: _start @0, then the test's .text).
#   usage: scripts/fw_asm.sh <test.S> <hex_out>
set -e
TEST=$1; HEX=$2
BASE=${HEX%.hex}
OBJ=$BASE.o; ELF=$BASE.elf; BIN=$BASE.bin
FTOB=build/fw_tb.o
mkdir -p "$(dirname "$HEX")" build
clang -c -target riscv32-unknown-elf -march=rv32im -mabi=ilp32 -o "$FTOB" tests/fw_tb.S
clang -c -target riscv32-unknown-elf -march=rv32im -mabi=ilp32 -o "$OBJ" "$TEST"
clang --target=riscv32-unknown-elf -march=rv32im -mabi=ilp32 -nostdlib \
      -nostartfiles -Wl,-Ttext=0x0 -Wl,--entry=_start -o "$ELF" "$FTOB" "$OBJ"
llvm-objcopy -O binary "$ELF" "$BIN"
python3 -c "
import sys
d = open(sys.argv[1], 'rb').read()
d = d[:len(d)//4*4]
sys.stdout.write(''.join(format(int.from_bytes(d[i:i+4], 'little'), '08x') + '\n'
                       for i in range(0, len(d), 4)))
" "$BIN" > "$HEX"
echo "built $HEX ($(wc -l < "$HEX") words)"
