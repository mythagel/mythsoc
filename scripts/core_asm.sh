#!/bin/sh
# core_asm.sh -- assemble a tb/test program with clang/llvm and generate
# goldens with tools/emu32.py.
#   usage: scripts/core_asm.sh <src.S> <hex_out> <exp_vh_out>
set -e
SRC=$1; HEX=$2; EXP=$3
OBJ=${HEX%.hex}.o; BIN=${HEX%.hex}.bin; ELF=${HEX%.hex}.elf
mkdir -p "$(dirname "$OBJ")"
clang -c -target riscv32-unknown-elf -march=rv32im -mabi=ilp32 -o "$OBJ" "$SRC"
# link: clang -c leaves R_RISCV_BRANCH relocs zero-filled; ld.lld resolves them
clang --target=riscv32-unknown-elf -march=rv32im -mabi=ilp32 -nostdlib \
      -nostartfiles -Wl,-Ttext=0x0 -Wl,--entry=_start -o "$ELF" "$OBJ"
llvm-objcopy -O binary "$ELF" "$BIN"
python3 tools/emu32.py "$BIN" --hex "$HEX" --exp "$EXP"
