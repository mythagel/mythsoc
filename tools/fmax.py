#!/usr/bin/env python3
"""fmax -- extract achieved Fmax from nextpnr-ecp5 output and apply gates.

design.md sec.1/10F: performance gate MIN 25 MHz (board crystal -> HARD),
TARGET 60, IDEAL 80. The board runs clk_i = 25 MHz; the gate reports how
much headroom P&R actually closed.

reads the nextpnr text log ("Max frequency for clock '...': 65.34 MHz",
optionally "(PASS at 60.00 MHz)") and/or the --report JSON
{"clocks": {"<name>": {"fmax": <MHz>}}}.

usage: python3 tools/fmax.py <pnr.log> [report.json] [--target FMHZ]
exit: 0 if every clock Fmax >= 25.0; 1 otherwise (or if no clock found).
prints one verdict line per clock: MIN/TARGET/IDEAL states + slack vs target.
"""
import json
import re
import sys

HARD_MIN, TARGET, IDEAL = 25.0, 60.0, 80.0

log = open(sys.argv[1]).read() if len(sys.argv) > 1 else ""
clocks = {}  # name -> MHz

# text: Info: Max frequency for clock 'clk_i$gbuf': 65.11 MHz
for m in re.finditer(r"Max frequency for clock '([^']+)':\s*([0-9.]+)\s*MHz", log):
    clocks[m.group(1)] = float(m.group(2))
# text: Info: Max frequency: 65.11 MHz at ... (older phrasing)
for m in re.finditer(r"Max frequency:\s*([0-9.]+)\s*MHz", log):
    clocks.setdefault("(design)", float(m.group(1)))

for a in sys.argv[2:]:
    if a.endswith(".json"):
        rep = json.load(open(a))
        for name, c in (rep.get("clocks") or {}).items():
            if isinstance(c, dict) and "fmax" in c:
                clocks[name] = float(c["fmax"])

if not clocks:
    print("FMAX: NO-CLOCK-FOUND (nextpnr timing report missing?)")
    sys.exit(1)

rc = 0
for name, f in sorted(clocks.items()):
    states = []
    states.append("MIN-OK"    if f >= HARD_MIN else "MIN-FAIL")
    states.append("TGT-OK"    if f >= TARGET  else "tgt-no")
    states.append("IDEAL-OK"  if f >= IDEAL   else "ideal-no")
    verdict = " ".join(states)
    if f < HARD_MIN:
        rc = 1
        verdict += "  <<< UNDER 25 MHz: board clock will NOT close"
    print(f"FMAX {name}: {f:.2f} MHz  [{verdict}]")

print("FMAX: " + ("PASS (min 25 MHz closed)" if rc == 0 else "FAIL"))
sys.exit(rc)
