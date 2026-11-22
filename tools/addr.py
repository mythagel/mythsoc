#!/usr/bin/env python3
# tools/addr.py -- address decode / register index calculator (mythrv32).
# Modes:
#   region  BASE SIZE [NAME]   -> in-range decode field, word index field
#   regmap  BASE OFF [OFF...]  -> word index (off>>2) and byte lane for each offset
# All args hex (0x...) or decimal.
import sys

def to_i(s):
    return int(s, 0)

def region(base, size, name):
    assert size & (size - 1) == 0 and size > 0
    log2 = size.bit_length() - 1
    assert base % size == 0
    width = 32 - log2
    fixed = base >> log2
    print(f"{name}: base={base:#x} size={size:#x} (2^{log2})")
    print(f"  in_range = (ADR_I[31:{log2}] == {width}'h{fixed:0{(width + 3) // 4}x})   // {width} bits")
    print(f"  word idx = ADR_I[{log2 - 1}:2]   ({log2 - 2} bits)")
    print(f"  byte ofs = ADR_I[{log2 - 1}:0]")

def regmap(base, offs):
    print(f"base={base:#x}")
    for off in offs:
        print(f"  off={off:#06x}  word_idx={off >> 2:#06x}  byte_lane={off & 3}  "
              f"hi={off >> 32 if off >> 32 else None}")

def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    mode = sys.argv[1]
    if mode == "region":
        region(to_i(sys.argv[2]), to_i(sys.argv[3]), sys.argv[4] if len(sys.argv) > 4 else "")
    elif mode == "regmap":
        regmap(to_i(sys.argv[2]), [to_i(x) for x in sys.argv[3:]])
    else:
        sys.exit(__doc__)

if __name__ == "__main__":
    main()
