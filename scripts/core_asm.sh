#!/bin/sh
# core_asm.sh -- assemble a tb/test program with clang/llvm and generate
# goldens with tools/emu32.py.
#   usage: scripts/core_asm.sh <src.S> <hex_out> <exp_vh_out>
set -e
SRC=$1; HEX=$2; EXP=$3
OBJ=${HEX%.hex}.o; BIN=${HEX%.hex}.bin
mkdir -p "$(dirname "$OBJ")"
clang -c -target riscv32-unknown-elf -march=rv32im -mabi=ilp32 -Wno-linker-warnings -o "$OBJ" "$SRC"
llvm-objcopy -O binary "$OBJ" "$BIN"
python3 tools/emu32.py "$BIN" --hex "$HEX" --exp "$EXP"
