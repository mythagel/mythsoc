#!/usr/bin/env python3
"""lpf_check -- verify lpf/mythrv32-5a-75b-v82.lpf against the board sources.

Rule-2 tool: pin sites are transcribed into the LPF by hand; this checks
every transcription against the hardware doc tables (and the design.md
memory facts), plus structural sanity (duplicate sites, ports without a
LOCATE, LOCATEs for nonexistent ports).

sources:
  ~/dev/project/chubby75/5a-75b/hardware_V8.2.md   (SDRAM table, clk/key/led)
  notes/design.md                                   (sec.9 pin roles)

checks (exit 1 on any FAIL):
  L1  every net->site pair in the V8.2 SDRAM table appears identically
      in the lpf (names mapped: DQn->sd_dq_io[n], BAn->sd_ba_o[n],
      An->sd_a_o[n], WE#/CAS#/RAS#->sd_*_n_o, CLK->sd_clk_o;
      DQM/CS#/CKE must NOT appear -- hard-wired on v8.2)
  L2  clk P6 / led T6 / key R7 (design sec.9 + hardware doc text)
  L3  no duplicate SITE strings across all LOCATE lines
  L4  every port (scalar or bus element) declared in rtl/top.v has exactly
      one LOCATE, and every LOCATE names a real top.v port

usage: python3 tools/lpf_check.py [lpf] [top.v]
"""
import re
import sys
from pathlib import Path

LPF = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("lpf/mythrv32-5a-75b-v82.lpf")
TOP = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("rtl/top.v")
HW = Path.home() / "dev/project/chubby75/5a-75b/hardware_V8.2.md"

fails = 0
def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    if not ok:
        fails += 1

# ---- parse lpf -------------------------------------------------------------
lpf_txt = LPF.read_text()
locate = {}   # port -> site (first occurrence)
sites = {}    # site -> [ports]
for m in re.finditer(r'LOCATE\s+COMP\s+"([^"]+)"\s+SITE\s+"([^"]+)"', lpf_txt):
    port, site = m.groups()
    if port in locate:
        check(locate[port] == site, f"LPF {port}: second LOCATE {site} == {locate[port]}")
    locate[port] = site
    sites.setdefault(site, []).append(port)

# ---- L1: SDRAM table from hardware_V8.2.md ----------------------------------
hw = HW.read_text()
def site_of(signal):
    # SDRAM/flash tables: signal is the FIRST cell of the row; anchor to the
    # line ("| DQ1 | A2 |" would otherwise match the site cell of an
    # earlier row when searching for "A2"). "A10/AP": optional /alias.
    m = re.search(r"^\|\s*" + re.escape(signal) + r"(?:/[A-Z]+)?\s*\|\s*([^|]+?)\s*\|",
                  hw, re.MULTILINE)
    return m.group(1) if m else None

name_map = {}
for i in range(32):
    name_map[f"DQ{i}"] = f"sd_dq_io[{i}]"
for i in range(2):
    name_map[f"BA{i}"] = f"sd_ba_o[{i}]"
for i in range(11):
    name_map[f"A{i}"] = f"sd_a_o[{i}]"
name_map.update({"WE#": "sd_we_n_o", "CAS#": "sd_cas_n_o",
                 "RAS#": "sd_ras_n_o", "CLK": "sd_clk_o"})
bad = 0
for sig, port in name_map.items():
    s = site_of(sig)
    if s is None:
        check(False, f"hardware doc: table row for {sig} not found")
        bad += 1
        continue
    if locate.get(port) != s:
        check(False, f"{sig} -> {port}: lpf {locate.get(port)!r} vs doc {s!r}")
        bad += 1
check(bad == 0, f"all 47 SDRAM nets match the V8.2 table (dq32+ba2+a11+3ctrl+clk)")

# CS#/CKE/DQM wired on-board: must have no FPGA pad / no LOCATE
for sig in ["DQM0", "DQM1", "DQM2", "DQM3", "CS#", "CKE"]:
    s = site_of(sig)
    check(s == "*na*", f"hardware doc: {sig} hard-wired (site {s!r})")

# ---- L2: clk / led / key ----------------------------------------------------
check(re.search(r"FPGA \(U33\) pin P6", hw) and locate.get("clk_i") == "P6",
      "clk_i P6 (25 MHz from phy1)")
check("LED (DATA_LED-) at T6" in hw and locate.get("led_o") == "T6",
      "led_o T6 (DATA_LED-, active low)")
check(re.search(r"button \(R7, KEY\+\)", hw) and locate.get("key_i") == "R7",
      "key_i R7 (button, pressed=low)")
check(locate.get("uart_txd") == "C4",
      "uart_txd C4 (design.md sec.1: USB-serial adapter)")

# ---- L3: duplicate sites ----------------------------------------------------
dups = {s: p for s, p in sites.items() if len(p) > 1}
check(not dups, f"no site used twice ({dups or 'clean'})")

# ---- L4: LOCATE set == top.v port set ---------------------------------------
top = TOP.read_text()
# strip line comments, then parse ANSI port declarations
top_nc = re.sub(r"//[^\n]*", "", top)
declared = set()
for m in re.finditer(
        r"\b(input|output|inout)\s+(?:reg\s+|wire\s+)?"
        r"(?:\[(\d+):(\d+)\]\s*)?([A-Za-z_]\w*)", top_nc):
    _, hi, lo, name = m.groups()
    if hi is not None:                      # [1:0] sd_x -> sd_x[0..1]
        for i in range(int(lo), int(hi) + 1):
            declared.add(f"{name}[{i}]")
    else:
        declared.add(name)
located = set(locate)
check(located <= declared, f"LOCATEs name real ports (extra: {sorted(located-declared)})")
check(declared <= located, f"every top port LOCATEd (missing: {sorted(declared-located)})")

print("LPF-CHECK: " + (f"FAIL ({fails})" if fails else "ALL GREEN"))
sys.exit(1 if fails else 0)
