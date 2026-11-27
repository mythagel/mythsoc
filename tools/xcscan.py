#!/usr/bin/env python3
"""Parse XC ring dumps from tb_fw (+XCATCH) log: report, per field, the
oldest cycle where it shows X, and print a condensed timeline.

Usage: python3 tools/xcscan.py <log>  (reads lines starting 'XC ')
"""
import re
import sys

def main(path):
    rows = []
    hdr = None
    for l in open(path):
        if l.startswith("XC ") or l.startswith("XCATCH"):
            if l.startswith("XCATCH"):
                print(l.strip())
            rows.append(l.strip())
    if not rows:
        print("no XC lines")
        return
    # derive field names from the first XC line's template: tokens "k=v"
    first = [r for r in rows if r.startswith("XC ")][0]
    toks = first.split()
    keys = []
    for t in toks[2:]:
        if "=" in t:
            keys.append(t.split("=")[0])
    vals = {}
    for r in [x for x in rows if x.startswith("XC ")]:
        p = r.split()
        cyc = int(p[1])
        d = {}
        for t in p[2:]:
            k, v = t.split("=")
            d[k] = v
        vals[cyc] = d
    cycles = sorted(vals)
    print(f"{len(cycles)} rows, cycles {cycles[0]}..{cycles[-1]}")
    # per-field first-X
    for k in keys:
        for c in cycles:
            v = vals[c].get(k, "")
            if "x" in v.lower():
                print(f"first X in {k:6s} at cyc {c}  (val {v})")
                break
    # timeline: print only fields that are X anywhere, plus priv/fpc
    bad = [k for k in keys if any("x" in vals[c].get(k, "").lower() for c in cycles)]
    print("fields ever-X:", bad)
    for c in cycles:
        mark = " ".join(f"{k}={vals[c][k]}" for k in bad)
        print(f"  {c}: fpc={vals[c]['fpc']} priv={vals[c]['priv']} {mark}")

if __name__ == "__main__":
    main(sys.argv[1])
