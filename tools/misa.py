#!/usr/bin/env python3
"""tools/misa.py -- compute RV32 misa reset value for a given extension set.

misa[31:0] = MXL[31:30] WARL | ISA bits [29:0].
Spec: unpriv Ch.2 / priv 3.1.1 (misa): MXL=1 for RV32;
  I = bit 8 (base integer), M = bit 12.

Usage: python3 tools/misa.py I M        # extensions to include
"""
import sys

NAMES = {
    "A": 0, "B": 1, "C": 2, "D": 3, "E": 4, "F": 5, "G": 6,
    "H": 7, "I": 8, "J": 9, "K": 10, "L": 11, "M": 12,
    "N": 13, "O": 14, "P": 15, "Q": 16, "R": 17, "S": 18,
    "T": 19, "U": 20, "V": 21, "W": 22, "X": 23,
}


def main():
    exts = sys.argv[1:]
    val = 1 << 30  # MXL = 1 -> XLEN = 32 (bits[31:30] = 01)
    for e in exts:
        if e not in NAMES:
            raise SystemExit(f"unknown extension: {e}")
        val |= 1 << NAMES[e]
    print(f"misa({','.join(exts) or 'none'}) = {val:#010x} "
          f"({val:032b})")


if __name__ == "__main__":
    main()
