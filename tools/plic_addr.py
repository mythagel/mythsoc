#!/usr/bin/env python3
# tools/plic_addr.py -- verify plic WB address decode (mythrv32)
#
# PLIC base 0x0C00_0000, region 4 MiB (wb_ic decode ADR[31:22]==10'h030).
# NUM_SOURCES=16, NUM_CONTEXTS=2 (ctx0=M, ctx1=S).
#
# Maps (offset from base, spec riscv-plic v1.0.0 ch.3):
#   pr   : 0x0004..0x003C    source N priority (word N; word0=src0 reserved;
#                            words 17..1023: src 17..1023 not implemented)
#   pen  : 0x001000          pending word 0 (bit N = src N) [RO]
#   en   : 0x2000+ctx*0x80   enable word 0 for ctx (bit N = src N)
#   thr  : 0x200000+ctx*0x1000
#   cl   : 0x200004+ctx*0x1000  (read=claim, write=complete)
#   else : reserved (read 0, write ignored)
#
# Decode under test (mirror of rtl/plic.v), on word offset o (ADR[1:0]==0,
# ADR[k:0]==BASE[k:0]+o[k:0] for k>=2 since BASE is 4 MiB aligned):
#   h12 = ADR[21:12] = (BASE>>12)&0xFFF + o[21:12]   (no carry, o < 4 MiB)
#   d_pr  = h12 == PB                 (PB = (BASE>>12)&0xFFF; page 0)
#   d_pen = h12 == PB+1 && o[11:2]==0   (page 1, pending word 0 only)
#   d_en  = h12 == PB+2
#           && (o[12:7] < N_CTX) && (o[10:2]==0)
#           -- en block = 0x2000 + N_CTX*0x80 (N_CTX<=8 fits page 2);
#              ctx = o[12:7], word idx = o[10:2] (32 words/ctx)
#   d_thr = h12 in PB+0x200..PB+0x200+N_CTX-1 && o[11:2]==0
#   d_cl  = h12 in PB+0x200..PB+0x200+N_CTX-1 && o[11:2]==1
#   (offset 0x200000>>12 = 0x200 pages)
#
# Reserved-but-decoded: pr page words 0 and 17..255 fire d_pr (read 0 /
# write ignored in RTL). All other reserved offsets fire no decode.
#
# Usage: python3 tools/plic_addr.py
#   brute-forces all 2^20 word-aligned byte offsets of the region.
import sys

BASE = 0x0C00_0000
REGION = 4 * 1024 * 1024      # 4 MiB
N_SRC = 16
N_CTX = 2
PB = (BASE >> 12) & 0xFFF
TPAGE = PB + (0x200000 >> 12)


def classify(o):
    """Expected class + params for word offset o.
    ('pr'|'pr_resv'|'pen'|'en'|'thr'|'cl'|'resv', id, word)"""
    if o < 0x1000:                                   # priority block, page 0
        word = o >> 2
        if 1 <= word <= N_SRC:
            return ('pr', word, word)
        return ('pr_resv', word, word)
    if o < 0x2000:                                  # pending block, page 1
        word = (o - 0x1000) >> 2
        if word == 0:
            return ('pen', 0, 0)
        return ('resv', 0, word)
    if o < 0x200000:                                # enables, pages 2..0x1FF
        ctx = (o - 0x2000) >> 7
        word = (o - 0x2000 - ctx * 0x80) >> 2
        if ctx < N_CTX and word == 0:
            return ('en', ctx, 0)
        return ('resv', ctx, word)
    ctx = (o - 0x200000) >> 12
    off = (o - 0x200000) & 0xFFF
    if ctx < N_CTX:
        if off == 0:
            return ('thr', ctx, 0)
        if off == 4:
            return ('cl', ctx, 0)
    return ('resv', ctx, off)


def decode(o):
    """Decode wires (mirror of rtl/plic.v) for word offset o.
    -> (d_pr, d_pen, d_en, d_thr, d_cl, pr_src, en_ctx, tc_ctx)"""
    h12 = ((BASE + o) >> 12) & 0xFFF
    d_pr  = (h12 == PB)
    d_pen = (h12 == PB + 1) and ((o >> 2) & 0x3FF) == 0
    d_en  = (h12 == PB + 2) and ((o >> 7) & 0x3F) < N_CTX \
            and ((o >> 2) & 0x1F) == 0
    d_thr = (TPAGE <= h12 <= TPAGE + N_CTX - 1) and ((o >> 2) & 0x3FF) == 0
    d_cl  = (TPAGE <= h12 <= TPAGE + N_CTX - 1) and ((o >> 2) & 0x3FF) == 1
    pr_src = (o >> 2) & 0x3FF if d_pr else -1
    en_ctx = (o >> 7) & 0x3F if d_en else -1
    tc_ctx = h12 - TPAGE if (d_thr or d_cl) else -1
    return d_pr, d_pen, d_en, d_thr, d_cl, pr_src, en_ctx, tc_ctx


def expected_decode(cls, idc):
    """Which decode should fire for class cls (idc = ctx/word as classified)."""
    if cls == 'pr':
        return ('pr', idc, 0)
    if cls == 'pr_resv':
        return ('pr', idc, 0)
    if cls == 'pen':
        return ('pen', 0, 0)
    if cls == 'en':
        return ('en', idc, 0)
    if cls == 'thr':
        return ('thr', idc, 0)
    if cls == 'cl':
        return ('cl', idc, 0)
    return ('resv', -1, 0)


def main():
    assert (0x200000 >> 12) == 0x200, "0x200000 page math"
    assert 0x200000 + 0x1000 == 0x201000, "ctx1 thr = ctx0 + 0x1000"
    assert 0x2000 + 0x80 == 0x2080, "ctx1 en = ctx0 + 0x80"

    errors = 0
    for o in range(0, REGION, 4):
        cls, idc, word = classify(o)
        d_pr, d_pen, d_en, d_thr, d_cl, pr_src, en_ctx, tc_ctx = decode(o)
        nf = sum(1 for x in (d_pr, d_pen, d_en, d_thr, d_cl) if x)
        if nf > 1:
            print("FAIL off=0x%06X: %d decodes overlap" % (o, nf))
            errors += 1
            continue
        got = None
        if d_pr:
            got = ('pr', pr_src, 0)
        elif d_pen:
            got = ('pen', 0, 0)
        elif d_en:
            got = ('en', en_ctx, 0)
        elif d_thr:
            got = ('thr', tc_ctx, 0)
        elif d_cl:
            got = ('cl', tc_ctx, 0)
        else:
            got = ('resv', -1, 0)
        exp = expected_decode(cls, idc)
        if got != exp:
            print("FAIL off=0x%06X class=%s: decode=%r expected=%r"
                  % (o, cls, got, exp))
            errors += 1
        if errors > 20:
            print("too many errors, aborting")
            return 1
    if errors:
        print("plic_addr: %d errors" % errors)
        return 1

    addrs = {
        "P(N)=BASE+4*N": BASE + 4,
        "PEND": BASE + 0x1000,
        "EN0": BASE + 0x2000,
        "EN1": BASE + 0x2080,
        "THR0": BASE + 0x200000,
        "CL0": BASE + 0x200004,
        "THR1": BASE + 0x201000,
        "CL1": BASE + 0x201004,
        "RSV_P0": BASE + 0x0,
        "RSV_PW17": BASE + 4 * 17,
        "RSV_PENDW1": BASE + 0x1004,
        "RSV_ENW1": BASE + 0x2004,
        "RSV_THR8": BASE + 0x200008,
        "RSV_THRCTX2": BASE + 0x202000,
        "RSV_CLCTX3": BASE + 0x203004,
    }
    print("plic_addr: OK (1M-word region scan, decode exclusive+correct)")
    for k, v in addrs.items():
        assert 0 <= (v - BASE) < REGION and v % 4 == 0
        print("  %-12s = 0x%08X" % (k, v))
    print("  decode: pr=page %X, pen=page %X w0, en=pages %X..%X"
          " ctx=o[12:7] w0=o[10:2]==0" % (PB, PB + 1, PB + 2, PB + 0x1FF))
    print("          thr/cl=pages %X..%X o[11:2]=0/1 (ctx=h12-%X)"
          % (TPAGE, TPAGE + N_CTX - 1, TPAGE))
    return 0


if __name__ == "__main__":
    sys.exit(main())
