#!/usr/bin/env python3
# mmu_addr -- Sv32 PTE address math + reference walk model (mythrv32 mmu)
#
# SoT: specs/riscv-privileged.pdf sec.12.3 (Sv32), 12.1.11 (satp).
#   LEVELS=2, PAGESIZE=4096, PTESIZE=4
#   VA (Fig 65): VPN[1]=VA[31:22] (10b), VPN[0]=VA[21:12] (10b), pgoff VA[11:0]
#   PA (Fig 66): PPN[1]=PA[33:22] (12b), PPN[0]=PA[21:12] (10b), pgoff PA[11:0]
#   PTE bits (Fig 67): V[0] R[1] W[2] X[3] U[4] G[5] A[6] D[7] RSW[9:8]
#                      PPN0[19:10] (10b) PPN1[31:20] (12b)
#   satp(RV32): MODE[31] ASID[30:22](9) PPN[21:0]; MODE 0=Bare 1=Sv32 (Table 40)
#
# Walk (spec 12.3.2):
#   L1: a = satp.ppn*4096 + VA[31:22]*4      (root idx = VA[31:22])
#   L0: a = L1.pte.ppn*4096 + VA[21:12]*4
#   validity: v=0 | r=0&w=1 | non-leaf(d|a|u set) -> fault
#   leaf at L1: pte.ppn[9:0]==0 else fault (4M align)
#   4K: PA[31:12] = {PPN1[9:0], PPN0[9:0]}; 4M: PA[31:0] = PPN1[9:0]<<22 | VA[21:0]
#   U/SUM/MXR (MXR affects LOADS: R|X), R/W/X, A/D (Svade fault mode:
#   A=0 fault; store/AMO D=0 fault)
#   faults: cause 12 fetch / 13 load+LR / 15 store+SC+AMO; mtval=VA

CAUSE = {"fetch": 12, "load": 13, "store": 15, "amo": 15}


def pte_addr1(satp_ppn, va):
    # level 1: base = satp.ppn*4096; index VA[31:22]
    return ((satp_ppn << 12) + (((va >> 22) & 0x3FF) << 2)) & 0xFFFFFFFF


def pte_addr0(ppn, va):
    # level 0: base = L1.pte.ppn*4096; index VA[21:12]
    return ((ppn << 12) + (((va >> 12) & 0x3FF) << 2)) & 0xFFFFFFFF


# PTE word encode/decode -------------------------------------------------
def pte(v=0, r=0, w=0, x=0, u=0, g=0, a=0, d=0, ppn=0):
    assert ppn < (1 << 22)
    return (ppn << 10) | (d << 7) | (a << 6) | (g << 5) | (u << 4) | (x << 3) | (w << 2) | (r << 1) | v


def bits(pte_w):
    v, r, w, x, u, g, a, d = ((pte_w >> i) & 1 for i in (0, 1, 2, 3, 4, 5, 6, 7))
    ppn = (pte_w >> 10) & 0x3FFFFF
    return v, r, w, x, u, g, a, d, ppn


# reference walk ----------------------------------------------------------
def walk(satp, va, mem, mode, acc, mxr=0, ssum=0):
    """satp: satp register word (MODE[31] ASID[30:22] PPN[21:0]).
    mem: callable addr32 -> 32b word (0 if absent).
    acc: 'fetch' | 'load' | 'store' | 'amo'; mode 'M'|'S'|'U'.
    returns (ok, pa32, cause, faulted)
    """
    cause = CAUSE[acc]
    if mode == "M":                      # M-mode: no translation
        return True, va & 0xFFFFFFFF, 0, False
    if not (satp >> 31) & 1:             # Bare
        return True, va & 0xFFFFFFFF, 0, False
    ppn = satp & 0x3FFFF
    # level 1
    pte1 = mem(pte_addr1(ppn, va))
    v, r, w, x, u, g, a, d, p1 = bits(pte1)
    if not v or (r == 0 and w == 1):
        return False, 0, cause, True
    if r | x:                            # 4M megapage leaf
        if p1 & 0x3FF:                  # PPN0 != 0: misaligned superpage
            return False, 0, cause, True
        pa = (((p1 >> 10) & 0x3FF) << 22) | (va & 0x3FFFFF)
        return _leaf(pte1, acc, mode, mxr, ssum, cause, pa)
    if d or a or u:                     # non-leaf reserved
        return False, 0, cause, True
    # level 0
    pte0 = mem(pte_addr0(p1, va))
    v, r, w, x, u, g, a, d, p0 = bits(pte0)
    if not v or (r == 0 and w == 1) or not (r | x):
        return False, 0, cause, True
    pa = (p0 << 12) | (va & 0xFFF)
    return _leaf(pte0, acc, mode, mxr, ssum, cause, pa)


def _leaf(pte_w, acc, mode, mxr, ssum, cause, pa):
    v, r, w, x, u, g, a, d, _ = bits(pte_w)
    # step 6: U/SUM
    if mode == "S":
        if u and (acc == "fetch" or not ssum):
            return False, 0, cause, True
    elif mode == "U" and not u:
        return False, 0, cause, True
    # step 8: perms
    if acc == "fetch" and not x:
        return False, 0, cause, True
    if acc == "load" and not (r or (mxr and x)):
        return False, 0, cause, True
    if acc in ("store", "amo") and not w:
        return False, 0, cause, True
    if acc == "amo" and not r:
        return False, 0, cause, True
    # step 9: A/D (Svade fault mode)
    if not a:
        return False, 0, cause, True
    if acc in ("store", "amo") and not d:
        return False, 0, cause, True
    return True, pa & 0xFFFFFFFF, 0, False


RAM = []  # (name, addr, data) -- TB RAM loads


def scenarios():
    # TB constants (tb/tb_mmu.v). SATP word = MODE[31]|ASID[30:22]|PPN[21:0].
    _a = pte_addr1(0x8, 0x40_0000)
    A = [
        # map A: 4K. root ppn=0x4; l0 @0x10000 (ppn=0x10)
        ("A_root0 @%08X root[0]->l0 ppn=0x10" % pte_addr1(0x4, 0x1000),
         pte_addr1(0x4, 0x1000), pte(v=1, ppn=0x10)),
        ("A_l0[1] @%08X VA0x1000->PA0x1000 RWX A=D=1" % pte_addr0(0x10, 0x1000),
         pte_addr0(0x10, 0x1000), pte(v=1, r=1, w=1, x=1, a=1, d=1, ppn=0x1)),
        ("A_l0[2] @%08X VA0x2000->PA0x2000 RWX A=D=1" % pte_addr0(0x10, 0x2000),
         pte_addr0(0x10, 0x2000), pte(v=1, r=1, w=1, x=1, a=1, d=1, ppn=0x2)),
        # map B: 4M mega. root ppn=0x8; leaf at region idx 1 (VA 0x40_0000)
        ("B_root1 @%08X 4M RWX ppn1=8 (PA 0x02000000 base)" % pte_addr1(0x8, 0x40_0000),
         pte_addr1(0x8, 0x40_0000), pte(v=1, r=1, w=1, x=1, a=1, d=1, ppn=8 << 10)),
        ("B_4M_misalign @%08X 4M misaligned ppn0=5 (variant)" % pte_addr1(0x8, 0x40_0000),
         pte_addr1(0x8, 0x40_0000), pte(v=1, r=1, w=1, x=1, a=1, d=1, ppn=(8 << 10) | 5)),
        ("B_4M_R0W1 @%08X 4M R=0W=1 reserved (variant)" % pte_addr1(0x8, 0x40_0000),
         pte_addr1(0x8, 0x40_0000), pte(v=1, w=1, a=1, d=1, ppn=8 << 10)),
        # region of VA 0x7FFFFFFF (TB M5 "new set miss"): root idx + PTE addr from tool
        ("B_root_hi @%08X 4M RWX ppn1=8 (VA 0x7FFFFFFF region)"
         % pte_addr1(0x8, 0x7FFFFFFF),
         pte_addr1(0x8, 0x7FFFFFFF), pte(v=1, r=1, w=1, x=1, a=1, d=1, ppn=8 << 10)),
        # map C: perms. root ppn=0xC; l0 @0x20000 (ppn=0x20)
        ("C_root0 @%08X root[0]->l0 ppn=0x20" % pte_addr1(0xC, 0x1000),
         pte_addr1(0xC, 0x1000), pte(v=1, ppn=0x20)),
        ("C_l0[1] @%08X VA0x1000 R-only A=D=1" % pte_addr0(0x20, 0x1000),
         pte_addr0(0x20, 0x1000), pte(v=1, r=1, a=1, d=1, ppn=0x1)),
        ("C_l0[2] @%08X VA0x2000 X-only A=D=1" % pte_addr0(0x20, 0x2000),
         pte_addr0(0x20, 0x2000), pte(v=1, x=1, a=1, d=1, ppn=0x1)),
        ("C_l0[3] @%08X VA0x3000 RW A=1 D=0" % pte_addr0(0x20, 0x3000),
         pte_addr0(0x20, 0x3000), pte(v=1, r=1, w=1, a=1, ppn=0x1)),
        ("C_l0[4] @%08X VA0x4000 R=0W=1 reserved" % pte_addr0(0x20, 0x4000),
         pte_addr0(0x20, 0x4000), pte(v=1, w=1, a=1, d=1, ppn=0x1)),
        ("C_l0[5] @%08X VA0x5000 U=1 RWX A=D=1" % pte_addr0(0x20, 0x5000),
         pte_addr0(0x20, 0x5000), pte(v=1, r=1, w=1, x=1, u=1, a=1, d=1, ppn=0x1)),
        ("C_l0[6] @%08X VA0x6000 A=0" % pte_addr0(0x20, 0x6000),
         pte_addr0(0x20, 0x6000), pte(v=1, r=1, x=1, ppn=0x1)),
        ("C_l0[7] @%08X VA0x7000 G=1 RWX A=D=1" % pte_addr0(0x20, 0x7000),
         pte_addr0(0x20, 0x7000), pte(v=1, r=1, w=1, x=1, g=1, a=1, d=1, ppn=0x1)),
        ("C_l0[8] @%08X VA0x8000 RWX A=D=1" % pte_addr0(0x20, 0x8000),
         pte_addr0(0x20, 0x8000), pte(v=1, r=1, w=1, x=1, a=1, d=1, ppn=0x1)),
    ]
    RAM.extend(A)
    _base = {}
    for name, a, d in RAM:
        if a not in _base:
            _base[a] = (name, d)
    print("\n# TB RAM model loads (base map):")
    for a in sorted(_base):
        name, d = _base[a]
        print("  ram_set(32'h%08X, 32'h%08X);  // %s" % (a, d, name))
    print("\n# Variant PTE words (re-load at 0x%08X in later phases):" % _a)
    for name, a, d in RAM:
        if a in _base and (name, d) != _base[a]:
            print("  %s = 32'h%08X" % (name.split('@')[0].strip().replace(' ', '_'), d))
    print("\n# SATP words:")
    for name, ppn, asid in [("map A root ppn=0x4 asid=0", 0x4, 0),
                           ("map A root ppn=0x4 asid=1", 0x4, 1),
                           ("map B root ppn=0x8 asid=0", 0x8, 0),
                           ("map C root ppn=0xC asid=0", 0xC, 0),
                           ("map C root ppn=0xC asid=1", 0xC, 1)]:
        print("  satp(%s) = 32'h%08X" % (name, (1 << 31) | (asid << 22) | ppn))

    base_mem = {}
    for _, a, d in RAM:
        if a not in base_mem:      # first occurrence per address is the base map
            base_mem[a] = d
    misalign4m = dict(base_mem); misalign4m[_a] = pte(v=1, r=1, w=1, x=1, a=1, d=1, ppn=(8 << 10) | 5)
    r0w1_4m = dict(base_mem); r0w1_4m[_a] = pte(v=1, w=1, a=1, d=1, ppn=8 << 10)
    checks = [
        ("4K  VA 0x1004 S fetch", 0x4, 0x1004, "S", "fetch"),
        ("4K  VA 0x1004 S load", 0x4, 0x1004, "S", "load"),
        ("4K  VA 0x1004 S store", 0x4, 0x1004, "S", "store"),
        ("4K  VA 0x2004 S fetch", 0x4, 0x2004, "S", "fetch"),
        ("4M  VA 0x40_0004 S fetch", 0x8, 0x40_0004, "S", "fetch"),
        ("4M  VA 0x40_4000 S load", 0x8, 0x40_4000, "S", "load"),
        ("4M  VA 0x7F_FFFF S fetch", 0x8, 0x7F_FFFF, "S", "fetch"),
        ("4M  VA 0x7FFFFFFF S fetch", 0x8, 0x7FFFFFFF, "S", "fetch"),
        ("4M  VA 0x40_0004 S store", 0x8, 0x40_0004, "S", "store"),
        ("4M  VA 0x40_0004 S fetch (ppn0=5)", 0x8, 0x40_0004, "S", "fetch", 0, 0, misalign4m),
        ("4M  VA 0x40_0004 S load  (R0W1)", 0x8, 0x40_0004, "S", "load", 0, 0, r0w1_4m),
        ("C   VA 0x1004 S load  R-only", 0xC, 0x1004, "S", "load"),
        ("C   VA 0x1004 S fetch R-only", 0xC, 0x1004, "S", "fetch"),
        ("C   VA 0x1004 S store R-only", 0xC, 0x1004, "S", "store"),
        ("C   VA 0x2004 S fetch X-only", 0xC, 0x2004, "S", "fetch"),
        ("C   VA 0x2004 S load  X-only", 0xC, 0x2004, "S", "load"),
        ("C   VA 0x2004 S load  X-only MXR=1", 0xC, 0x2004, "S", "load", 1),
        ("C   VA 0x2004 S store X-only", 0xC, 0x2004, "S", "store"),
        ("C   VA 0x3004 S load  RW D=0", 0xC, 0x3004, "S", "load"),
        ("C   VA 0x3004 S store RW D=0", 0xC, 0x3004, "S", "store"),
        ("C   VA 0x4004 S load  R0W1", 0xC, 0x4004, "S", "load"),
        ("C   VA 0x5004 S load  U=1", 0xC, 0x5004, "S", "load"),
        ("C   VA 0x5004 S load  U=1 SUM=1", 0xC, 0x5004, "S", "load", 0, 1),
        ("C   VA 0x5004 S store U=1 SUM=1", 0xC, 0x5004, "S", "store", 0, 1),
        ("C   VA 0x5004 S fetch U=1 (SUM n/a)", 0xC, 0x5004, "S", "fetch", 0, 1),
        ("C   VA 0x5004 U load  U=1", 0xC, 0x5004, "U", "load"),
        ("C   VA 0x5004 U fetch U=1", 0xC, 0x5004, "U", "fetch"),
        ("C   VA 0x8004 U load  U=0", 0xC, 0x8004, "U", "load"),
        ("C   VA 0x6004 S load  A=0", 0xC, 0x6004, "S", "load"),
        ("C   VA 0x7004 S load  G page", 0xC, 0x7004, "S", "load"),
    ]
    print("\n# Expected translations (walk model):")
    for c in checks:
        name, ppn, va, mode, acc = c[:5]
        mxr = c[5] if len(c) > 5 else 0
        ssum = c[6] if len(c) > 6 else 0
        mem = c[7] if len(c) > 7 else base_mem
        ok, pa, cause, fa = walk((1 << 31) | ppn, va, mem.get, mode, acc, mxr, ssum)
        tag = "OK  pa=%08X" % pa if ok else "FAULT cause=%d" % cause
        print("  %-34s -> %s" % (name, tag))


if __name__ == "__main__":
    # ---- sanity: address math ----
    assert pte_addr1(0x4, 0x0000_0000) == 0x0000_4000      # root idx 0
    assert pte_addr1(0x4, 0x0040_0000) == 0x0000_4004      # root idx 1 (4M region 1)
    assert pte_addr1(0x4, 0x0080_0000) == 0x0000_4008      # root idx 2
    assert pte_addr0(0x10, 0x0000_0000) == 0x0001_0000     # l0 idx 0
    assert pte_addr0(0x10, 0x0000_1000) == 0x0001_0004     # l0 idx 1
    assert pte_addr0(0x10, 0x0000_2000) == 0x0001_0008     # l0 idx 2
    print("addr math OK")
    # ---- PTE encode/decode round trip ----
    p = pte(v=1, r=1, x=1, a=1, d=1, ppn=0x123)
    assert bits(p) == (1, 1, 0, 1, 0, 0, 1, 1, 0x123)  # v r w x u g a d ppn
    print("pte encode OK:", hex(p))
    # ---- walk model self-test ----
    mem = {}

    def m(a):
        return mem.get(a, 0)

    # 4K: root[0] -> l0@0x10000 -> VA 0x1004 page ppn=0x1
    mem[pte_addr1(0x4, 0x1000)] = pte(v=1, ppn=0x10)
    mem[pte_addr0(0x10, 0x1000)] = pte(v=1, r=1, w=1, x=1, a=1, d=1, ppn=0x1)
    ok, pa, c, fa = walk((1 << 31) | 0x4, 0x1004, m, "S", "fetch")
    assert (ok, pa, fa) == (True, 0x1004, False), (ok, hex(pa), fa)
    ok, pa, c, fa = walk((1 << 31) | 0x4, 0x1004, m, "S", "store")
    assert (ok, pa, fa) == (True, 0x1004, False)
    # A=0 -> fault
    mem[pte_addr0(0x10, 0x1000)] = pte(v=1, r=1, x=1, ppn=0x1)
    ok, pa, c, fa = walk((1 << 31) | 0x4, 0x1004, m, "S", "load")
    assert (ok, c, fa) == (False, 13, True)
    # store D=0 (A=1) -> fault 15
    mem[pte_addr0(0x10, 0x1000)] = pte(v=1, r=1, w=1, x=1, a=1, ppn=0x1)
    ok, pa, c, fa = walk((1 << 31) | 0x4, 0x1004, m, "S", "store")
    assert (ok, c, fa) == (False, 15, True)
    # U-mode: U=0 page faults, U=1 page ok; S fetch on U=1 faults
    mem[pte_addr0(0x10, 0x1000)] = pte(v=1, r=1, x=1, a=1, d=1, ppn=0x1)
    assert not walk((1 << 31) | 0x4, 0x1004, m, "U", "load")[0]
    mem[pte_addr0(0x10, 0x1000)] = pte(v=1, r=1, x=1, a=1, d=1, u=1, ppn=0x1)
    assert walk((1 << 31) | 0x4, 0x1004, m, "U", "load")[0]
    assert not walk((1 << 31) | 0x4, 0x1004, m, "S", "fetch")[0]
    # 4M: root ppn=0x8, region 1 (VA 0x40_0000), ppn1=8 -> PA = 8<<22 | VA[21:0]
    mem[pte_addr1(0x8, 0x40_0000)] = pte(v=1, r=1, x=1, a=1, d=1, ppn=8 << 10)
    ok, pa, c, fa = walk((1 << 31) | 0x8, 0x40_0004, m, "S", "fetch")
    assert (ok, pa, fa) == (True, 0x02000004, False), (ok, hex(pa))
    ok, pa, c, fa = walk((1 << 31) | 0x8, 0x40_4000, m, "S", "fetch")
    assert (ok, pa, c, fa) == (True, 0x02004000, 0, False), (ok, hex(pa))
    ok, pa, c, fa = walk((1 << 31) | 0x8, 0x7F_FFFF, m, "S", "fetch")
    assert (ok, pa, c, fa) == (True, 0x023FFFFF, 0, False), (ok, hex(pa))
    # misaligned 4M (ppn0 != 0) -> fault
    mem[pte_addr1(0x8, 0x40_0000)] = pte(v=1, r=1, x=1, a=1, d=1, ppn=(8 << 10) | 5)
    assert not walk((1 << 31) | 0x8, 0x40_0000, m, "S", "fetch")[0]
    # r=0 w=1 reserved -> fault
    mem[pte_addr1(0x8, 0x40_0000)] = pte(v=1, w=1, a=1, d=1, ppn=8 << 10)
    assert not walk((1 << 31) | 0x8, 0x40_0000, m, "S", "store")[0]
    # M-mode identity even with satp set; Bare identity
    ok, pa, c, fa = walk((1 << 31) | 0x4, 0xDEAD_0004, m, "M", "fetch")
    assert (ok, pa, fa) == (True, 0xDEAD_0004, False)
    ok, pa, c, fa = walk(0, 0xDEAD_0004, m, "S", "fetch")
    assert (ok, pa, fa) == (True, 0xDEAD_0004, False)
    print("walk model self-test OK")
    scenarios()
