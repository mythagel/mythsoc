#!/usr/bin/env python3
"""vcd_last.py -- dump the last K time samples of a VCD (per-signal changes),
plus a cumulative value table of all signals at the final sample.

usage: vcd_last.py <vcd> <keep-samples> [substr ...]
  changes: only signals whose FULL NAME contains one of the substrs (if any).
  table:    same filter.
"""
import sys

def parse(path):
    with open(path) as f:
        data = f.read()
    # body starts at the first line that is a timestamp (#NNN); VCD ids may
    # themselves be '#' so partitioning on the first '#' char is wrong
    lines = data.splitlines()
    start = 0
    for idx, ln in enumerate(lines):
        if ln.startswith("#"):
            start = idx
            break
    scopes = []
    varmap = {}   # id -> full dotted path
    for line in lines[:start]:
        if line.startswith("$scope"):
            parts = line.split()
            if len(parts) >= 3:
                scopes.append(parts[2].strip('$'))
        elif line.startswith("$upscope"):
            scopes.pop()
        elif line.startswith("$var"):
            parts = line.split()
            if len(parts) < 5:
                continue
            p, name = parts[3], parts[4]
            idl = len(p) // 4
            varmap[p[-idl:]] = ".".join(scopes + [name])
    state = {}
    window = []   # (time, {id: val} changes within that sample)
    cur = {}
    t = 0
    for line in lines[start:]:
        if line.startswith("#"):
            if cur:
                window.append((t, cur))
            t = int(line[1:])
            cur = {}
        else:
            # value line: "b<value> <id>" or single-char "<v><id>" (e.g. "1v(")
            if line[0] in "bB":
                parts = line.split()
                if len(parts) < 2:
                    continue
                val, vid = parts[0], parts[1]
            else:
                val, vid = line[0], line[1:]
            cur[vid] = val
            state[vid] = val
    if cur:
        window.append((t, cur))
    return varmap, state, window

def main():
    path = sys.argv[1]
    keep = int(sys.argv[2])
    subs = sys.argv[3:]
    varmap, state, window = parse(path)

    def keepid(vid):
        return any(s in varmap.get(vid, "") for s in subs) if subs else True

    print(f"samples={len(window)} last_time={window[-1][0]} ({window[-1][0]//20000} cyc)")
    for (t, ch) in window[-keep:]:
        shown = {i: v for i, v in ch.items() if keepid(i) and varmap.get(i)}
        if shown:
            print(f"#{t//20000:6d} " +
                  " ".join(f"{varmap[i]}={v}" for i, v in sorted(shown.items())))
    print("\n=== values at freeze ===")
    for i, v in sorted(state.items(), key=lambda kv: varmap.get(kv[0], kv[0])):
        if keepid(i) and varmap.get(i):
            print(f"{varmap[i]:70s} {v}")

if __name__ == "__main__":
    main()
