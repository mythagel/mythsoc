#!/usr/bin/env python3
"""fw_ranges.py -- verify epc-range check constants against the linked ELF.

A firmware test checks `in_range(epc, (uint32_t)FN, SPAN)`. The range
[FN, FN+SPAN) must fit inside FN: it must not reach the start of any other
function symbol (else an epc in the NEIGHBOURING function passes the check
-- a false green, or a false fail depending on layout).

usage: fw_ranges.py <test.c> <elf>
Symbols: llvm-nm sorted by address. Exits nonzero on any violation.
"""
import re
import subprocess
import sys

def main():
    src, elf = sys.argv[1], sys.argv[2]
    syms = []
    for line in subprocess.check_output(["llvm-nm", "-n", elf], text=True).splitlines():
        p = line.split()
        if len(p) == 3 and p[2] not in ("dram",):
            syms.append((int(p[0], 16), p[2]))
    syms.sort()
    starts = [a for a, _ in syms]

    pat = re.compile(r"in_range\([^,]+,\s*\(uint32_t\)(\w+),\s*(0x[0-9a-fA-F]+|\d+)\)")
    fails = 0
    for fn, span in pat.findall(open(src).read()):
        span = int(span, 0)
        hits = [a for a, n in syms if n == fn]
        if not hits:
            print(f"MISSING symbol {fn} in {elf}")
            fails += 1
            continue
        a = hits[0]
        nxt = [s for s in starts if s > a]
        end = nxt[0] if nxt else a + 4096
        ok = a + span <= end
        print(f"{'ok  ' if ok else 'BAD '} {fn} @0x{a:x} span 0x{span:x} "
              f"-> 0x{a + span:x} (fn ends 0x{end:x}, next {nxt and hex(nxt[0]) or '-'})")
        if not ok:
            fails += 1
    sys.exit(1 if fails else 0)

if __name__ == "__main__":
    main()
