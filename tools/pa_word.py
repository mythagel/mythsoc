#!/usr/bin/env python3
# pa_word.py PA [BASE] -> s1ram.mem word index for PA (BASE default 0x80000000)
# usage: pa_word.py 0x80001000 [0x80000000]
import sys

pa = int(sys.argv[1], 0)
base = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x80000000
off = pa - base
assert off % 4 == 0, f"PA {pa:#x} not word-aligned vs base {base:#x}"
w = off // 4
print(f"PA={pa:#x} base={base:#x} off={off:#x} word={w} (0x{w:x})")
