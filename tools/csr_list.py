#!/usr/bin/env python3
"""tools/csr_list.py -- parse the Chapter 19 "RISC-V Privileged Instruction Set
Listings" CSR table out of /tmp/specs_txt/priv.txt (pdftotext output).

The listing is a sequence of aligned runs: a block of hex CSR addresses, a
block of access tokens (MRO/MRW/SRW/...), then a block of CSR names, then
description text until the next address block. We zip addresses<->access<->name.

Usage:
  python3 tools/csr_list.py                 # full listing
  python3 tools/csr_list.py mstatus mtvec   # only these names
"""
import re
import sys

TXT = "/tmp/specs_txt/priv.txt"

ADDR = re.compile(r"^0x[0-9A-Fa-f]+$")
ACCESS = re.compile(r"^[A-Z]{1,2}(RO|RW)$")  # MRO MRW SRO SRW UMRO UMRU ...
NAME = re.compile(r"^[a-z][a-z0-9]*$")


def runs(lines):
    """Yield (kind, items) runs: kind in {'addr','access','name','other'}."""
    kind, cur = None, []
    for raw in lines:
        s = raw.strip()
        if not s:
            continue
        if ADDR.match(s):
            k = "addr"
        elif ACCESS.match(s):
            k = "access"
        elif NAME.match(s):
            k = "name"
        else:
            k = "other"
        if kind is None:
            kind, cur = k, [s]
        elif k == kind:
            cur.append(s)
        else:
            yield kind, cur
            kind, cur = k, [s]
    if kind is not None:
        yield kind, cur


def parse(lines):
    """Return list of (addr, access, name)."""
    out = []
    it = iter(runs(lines))
    # Walk: expect addr run, access run, name run, other run; repeat.
    pending_addr, pending_acc, pending_name = [], [], []
    for kind, items in it:
        if kind == "addr":
            # flush previous triple if all three present
            if pending_name:
                flush(pending_addr, pending_acc, pending_name, out)
                pending_addr, pending_acc, pending_name = [], [], []
            pending_addr = items
        elif kind == "access":
            if pending_name:
                flush(pending_addr, pending_acc, pending_name, out)
                pending_addr, pending_acc, pending_name = [], [], []
            pending_acc = items
        elif kind == "name":
            pending_name = items
        # 'other' (descriptions / section headers) is ignored for alignment
    if pending_name:
        flush(pending_addr, pending_acc, pending_name, out)
    return out


def flush(addr, acc, name, out):
    n = min(len(addr), len(acc), len(name))
    for i in range(n):
        out.append((addr[i], acc[i], name[i]))


def main():
    want = set(sys.argv[1:])
    with open(TXT) as f:
        lines = f.readlines()
    rows = parse(lines)
    # de-dup, keep first occurrence
    seen = set()
    for a, c, n in rows:
        if n in seen:
            continue
        seen.add(n)
        if want and n not in want:
            continue
        print(f"{a}  {c:<5} {n}")


if __name__ == "__main__":
    main()
