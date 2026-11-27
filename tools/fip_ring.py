#!/usr/bin/env python3
"""fip_ring.py -- annotate tb_fw +FIP fetch ring dumps.

usage: tools/fip_ring.py <log> [start_label]
Parses "F %d fpc=... b=%b ..." lines; decodes the 14-bit flag vector
{req_mmu,req_l1,enq,deq,bypass,syn,fresh_mmu,fresh_l1,flush,pav,lio,
 inval_p,d_mio,d_lio} and prints a compact table.
"""
import sys, re

FIELDS = ["req_mmu","req_l1","enq","deq","bypass","syn","fresh_mmu",
          "fresh_l1","flush","pav","lio","inval_p","d_mio","d_lio"]

def main():
    log = open(sys.argv[1]).read().splitlines()
    start = sys.argv[2] if len(sys.argv) > 2 else "F 0 "
    rows = []
    collecting = False
    for ln in log:
        if "FIP trigger" in ln:
            collecting = True
            print(ln)
            continue
        m = re.match(r"F (\d+) fpc=(\w+) rpc=(\w+) ip=(\w+) is=(\w+) v=(\w{4}) "
                     r"xp=(\w+) xis=(\w+) b=([01]{14}) l1st=(\d) fqc=(\w+) "
                     r"pmm=(\w+) ppv=(\w+) pl1=(\w+) pa=(\w+) ld=(\w+) fw=(\w+) "
                     r"ia=(\w+) ar=(\w+) ra1=(\w+)", ln)
        if collecting and m:
            idx, fpc, rpc, ip, is_, v, xp, xis, b, l1st, fqc, pmm, ppv, pl1, \
                pa, ld, fw, ia, ar, ra1 = m.groups()
            fl = {name: int(bit) for name, bit in zip(FIELDS, b)}
            hot = " ".join(k for k in FIELDS if fl[k])
            iv = int(fqc, 8)
            print(f"F{idx:>2} fpc={fpc} ip={ip}/{is_} v[i,e,w,iw]={v} xp={xp}/{xis} "
                  f"l1st={l1st} ifidv={iv>>3} fq={iv&7} pmm={pmm} ppv={ppv} "
                  f"pl1={pl1} pa={pa} ld={ld} ar={ar} | {hot}")
    return 0

sys.exit(main())
