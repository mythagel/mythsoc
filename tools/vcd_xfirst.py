#!/usr/bin/env python3
"""vcd_xfirst.py -- find the FIRST time chosen signals take x/z in a VCD.

Usage: tools/vcd_xfirst.py <vcd> sig1 [sig2 ...] [--window N] [--ctx]
                           [--after T]   # ignore X at/below t=T (reset window)

For each listed signal (matched as full name at the deepest scope, e.g.
'mio' finds tb_fw.dut.core0.mio), report the first timestamp where its
value contains x or z, plus (with --ctx) the last K value-changes of ALL
signals before that time (scope-annotated) to trace the contamination
source. Rule-2 tool: never eyeball VCDs.
"""
import re
import sys


def main():
    a = sys.argv[1:]
    window = 60
    ctx = False
    after = -1
    if "--after" in a:
        i = a.index("--after")
        after = int(a[i + 1])
        del a[i:i + 2]
    if "--window" in a:
        i = a.index("--window")
        window = int(a[i + 1])
        del a[i:i + 2]
    if "--ctx" in a:
        a.remove("--ctx")
        ctx = True
    vcd = a[0]
    want = a[1:]
    ids = {}            # id -> full hierarchical name
    scopes = []         # id stack while parsing $scope
    cur = 0             # current timestamp (integer time-unit steps)
    hist = []           # ring of (time, name, value)
    hits = {}           # name -> first time

    def watch(idcode, val, name_hint=None):
        nonlocal hist
        nm = ids.get(idcode)
        if nm is None:
            return
        hist.append((cur, nm, val))
        if len(hist) > 4000:
            hist.pop(0)
        if any(w == nm.split(".")[-1] or w in nm for w in want):
            if cur > after and ("x" in val or "z" in val):
                if nm not in hits:
                    hits[nm] = cur

    t_re = re.compile(r"#(\d+)")
    tok = None
    with open(vcd) as f:
        intok = False
        for line in f:
            s = line.strip()
            if s.startswith("$end"):
                if intok:
                    toks = (s[5:] if False else None)
                intok = False
                continue
            if s.startswith("$scope"):
                parts = s.split()
                scopes.append(parts[2])
                continue
            if s.startswith("$upscope"):
                scopes.pop()
                continue
            if s.startswith("$var"):
                parts = s.split()
                vid, name = parts[3], parts[4]
                ids[vid] = ".".join(scopes + [name])
                continue
            if s.startswith("$dumpvars") or s.startswith("$dumpport"):
                intok = True
                continue
            m = t_re.match(s)
            if m:
                cur = int(m.group(1))
                continue
            if not s:
                continue
            # value change lines: scalar "<val><id>" ; vector "v<val> <id>"
            if s[0] in "01xzXZ":
                watch(s[1:], s[0])
            elif s[0] in "rRkKhpPnNfF":
                pass
            elif s[0] == "b" or s[0] == "B":
                parts = s[1:].split()
                if len(parts) == 2:
                    watch(parts[1], parts[0])
            elif s[0] == "#" or s[0] == "/":
                continue
            else:
                parts = s.split()
                if len(parts) == 2 and re.fullmatch(r"[01xXzZ]+", parts[0]):
                    watch(parts[1], parts[0])
    if not hits:
        print("no x/z found on:", " ".join(want))
        return
    for nm, t in sorted(hits.items(), key=lambda kv: kv[1]):
        print(f"FIRST-X {nm} @ t={t}")
    if ctx:
        t0 = min(hits.values())
        print(f"\n-- last {window} changes before t={t0} --")
        pre = [h for h in hist if h[0] <= t0][-window:]
        for tt, nm, v in pre:
            print(f"t={tt:<12} {nm:<55} = {v}")


if __name__ == "__main__":
    main()
