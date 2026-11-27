#!/usr/bin/env python3
"""dmips.py -- Dhrystone result calculator (design.md sec.10 F).

Usage:
  tools/dmips.py --log <vvp.log> [--clock MHz]   # extract runs/cycles from
                                                 # the +DUMP scoreboard
  tools/dmips.py <runs> <cycles> [--clock MHz]

1 DMIPS = 1757 Dhrystones/s (VAX 11/780 reference).
DMIPS/MHz = (runs/cycles) * 1e6 / 1757  (mtime = 1 tick/cycle).
"""
import re
import sys


def from_log(path):
    """parse tb_fw +DUMP: dram[42]=runs, dram[40]=loop cycles (lo),
    dram[41]=cycles hi, dram[44]=setup cycles."""
    g = {}
    for line in open(path):
        m = re.search(r"dram\[(\d+)\]\s*=\s*([0-9a-fA-Fx]+)", line)
        if m:
            g[int(m.group(1))] = m.group(2)

    def w(i):
        v = g.get(i, None)
        if v is None or "x" in v.lower():
            sys.exit(f"log has no usable dram[{i}] (got {v!r})")
        return int(v, 16)

    cycles = w(40) | (w(41) << 32)
    return w(42), cycles, w(44)


def main():
    a = sys.argv[1:]
    clock = 25.0
    if "--clock" in a:
        i = a.index("--clock")
        clock = float(a[i + 1])
        del a[i:i + 2]
    if a and a[0] == "--log":
        runs, cycles, setup = from_log(a[1])
        src = a[1]
    else:
        if len(a) != 2:
            print(__doc__)
            sys.exit(1)
        runs, cycles, setup = int(a[0]), int(a[1]), None
        src = "args"

    cpr = cycles / runs
    dmips_mhz = (runs / cycles) * 1e6 / 1757.0
    print(f"source:      {src}")
    print(f"runs:        {runs}")
    print(f"loop cycles: {cycles}" + (f"  (setup {setup})" if setup is not None else ""))
    print(f"cycles/run:  {cpr:.1f}")
    print(f"DMIPS/MHz:   {dmips_mhz:.3f}")
    print(f"DMIPS @{clock:g} MHz: {dmips_mhz * clock:.1f}")


if __name__ == "__main__":
    main()
