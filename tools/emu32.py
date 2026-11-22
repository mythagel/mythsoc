#!/usr/bin/env python3
"""emu32 -- RV32IM reference emulator for mythrv32 test goldens.

Consumes a flat little-endian instruction image (llvm-objcopy -O binary),
runs from a fixed load address, and emits:
  --hex  F : $readmemh word image (8 hex digits/word)
  --exp  F : verilog `define C1_NCOMMIT n + exp[i] assignments
             (final register file contents)

Instruction subset implemented grows with the core increments:
  C1: ADD SUB SLL SLT SLTU XOR SRL SRA OR AND
      ADDI SLTI SLTIU XORI ORI ANDI SLLI SRLI SRAI LUI AUIPC NOP
Unknown/illegal opcodes stop execution with a report (never silently).
"""
import sys, struct

MASK = 0xFFFFFFFF

def s(x):
    x &= MASK
    return x - (1 << 32) if x & 0x80000000 else x


def sx(v, bits):
    return v - (1 << bits) if v & (1 << (bits - 1)) else v


def imm_s(insn):
    v = (((insn >> 31) & 1) << 12) | (((insn >> 25) & 0x7F) << 5) | \
        ((insn >> 7) & 0x1F)
    return v - (1 << 13) if v & 0x1000 else v

def run(words, start_pc, verbose=False):
    rf = [0] * 32
    memw = {}                      # word index -> word (DRAM model, C4)
    pc = start_pc
    n = 0
    while pc in range(0, 4 * len(words), 4):
        insn = words[pc // 4]
        op = insn & 0x7F
        rd = (insn >> 7) & 31
        f3 = (insn >> 12) & 7
        rs1 = (insn >> 15) & 31
        rs2 = (insn >> 20) & 31
        imm_i = (insn >> 20) & 0xFFF
        if imm_i & 0x800: imm_i -= 0x1000
        imm_u = insn & 0xFFFFF000
        f7 = (insn >> 25) & 0x7F
        a, b = rf[rs1], rf[rs2]
        w = None
        if op == 0x37:                      # LUI
            w = imm_u
        elif op == 0x17:                    # AUIPC
            w = (pc + imm_u) & MASK
        elif op == 0x13:                    # OP-IMM
            if f3 == 0:   w = (a + imm_i) & MASK          # ADDI
            elif f3 == 1: w = (a << (imm_i & 31)) & MASK  # SLLI
            elif f3 == 2: w = int(s(a) < s(imm_i))        # SLTI
            elif f3 == 3: w = int(a < (imm_i & MASK))     # SLTIU
            elif f3 == 4: w = a ^ imm_i                   # XORI
            elif f3 == 5:
                sh = imm_i & 31
                w = (a >> sh) if ((insn >> 30) & 1) == 0 else ((s(a) >> sh) & MASK)
            elif f3 == 6: w = a | imm_i                   # ORI
            elif f3 == 7: w = a & imm_i                   # ANDI
        elif op == 0x33 and f7 == 1:              # RV32M (Table 15/11)
            if f3 == 0:   w = (a * b) & MASK
            elif f3 == 1: w = (s(a) * s(b) >> 32) & MASK
            elif f3 == 2: w = (s(a) * b >> 32) & MASK
            elif f3 == 3: w = ((a * b) >> 32) & MASK
            else:
                signed = (f3 in (4, 6))
                if b == 0:
                    q, r = MASK, a
                elif signed and a == 0x80000000 and b == 0xFFFFFFFF:
                    q, r = 0x80000000, 0
                else:
                    sa, sb = (s(a), s(b)) if signed else (a, b)
                    q = abs(sa) // abs(sb)
                    if sa < 0 and sb > 0 or sa > 0 and sb < 0:
                        q = -q
                    r = abs(sa) % abs(sb)
                    if sa < 0:
                        r = -r
                    q, r = q & MASK, r & MASK
                w = r if (f3 in (6, 7)) else q
        elif op == 0x33:                    # OP
            sh = b & 31
            if f3 == 0:
                w = ((a - b) if (f7 >> 5) & 1 else (a + b)) & MASK
            elif f3 == 1: w = (a << sh) & MASK            # SLL
            elif f3 == 2: w = int(s(a) < s(b))            # SLT
            elif f3 == 3: w = int(a < b)                  # SLTU
            elif f3 == 4: w = a ^ b                       # XOR
            elif f3 == 5:
                sh = b & 31
                w = (a >> sh) if ((f7 >> 5) & 1) == 0 else ((s(a) >> sh) & MASK)
            elif f3 == 6: w = a | b                       # OR
            elif f3 == 7: w = a & b                       # AND
        elif op == 0x03:                            # LOAD (C4)
            eff = (a + imm_i) & MASK
            word = memw.get(eff >> 2, 0)
            off = eff & 3
            if f3 == 0:
                w = sx((word >> (8 * off)) & 0xFF, 8)
            elif f3 == 1:
                w = sx((word >> (8 * off)) & 0xFFFF, 16)
            elif f3 == 2:
                w = word
            elif f3 == 4:
                w = (word >> (8 * off)) & 0xFF
            else:
                w = (word >> (8 * off)) & 0xFFFF
        elif op == 0x23:                            # STORE (C4)
            eff = (a + imm_s(insn)) & MASK
            off = eff & 3
            sel = {0: 1 << off, 1: (0b1100 if off == 2 else 0b0011), 2: 0b1111}[f3]
            wd = {0: (b & 0xFF) * 0x01010101,
                  1: ((b & 0xFFFF) << 16) | (b & 0xFFFF),
                  2: b & MASK}[f3]
            widx = eff >> 2
            word = memw.get(widx, 0)
            mask = sum(0xFF << (8 * i) for i in range(4) if (sel >> i) & 1)
            memw[widx] = ((word & ~mask) | (wd & mask)) & MASK
        elif op in (0x63, 0x67, 0x6F):              # BRANCH/JALR/JAL (C3)
            pass
        else:
            print("emu32: stop at pc=%#x: unsupported insn %#010x" % (pc, insn),
                  file=sys.stderr)
            break
        nxt = (pc + 4) & MASK
        if op == 0x63:                              # BRANCH
            off = imm_b(insn)
            t = {0: a == b, 1: a != b, 4: s(a) < s(b), 5: s(a) >= s(b),
                 6: a < b, 7: a >= b}[f3]
            if t:
                nxt = (pc + off) & MASK
        elif op == 0x6F:                            # JAL
            rf[rd] = (pc + 4) & MASK if rd else 0
            nxt = (pc + imm_j(insn)) & MASK
        elif op == 0x67:                            # JALR
            rf[rd] = (pc + 4) & MASK if rd else 0
            nxt = ((a + imm_i) & ~1) & MASK
        if op not in (0x63, 0x67, 0x6F, 0x23):
            rf[rd] = 0 if rd == 0 else (w & MASK)
        n += 1
        if verbose:
            print("%#010x: %#010x  x%d=%#010x" % (pc, insn, rd, rf[rd]),
                  file=sys.stderr)
        if nxt == pc:
            break                                   # halt self-loop
        pc = nxt
    return rf, n, pc, memw


def imm_b(insn):
    v = (((insn >> 31) & 1) << 12) | (((insn >> 7) & 1) << 11) | \
        (((insn >> 25) & 0x3F) << 5) | (((insn >> 8) & 0xF) << 1)
    return v - (1 << 13) if v & 0x1000 else v


def imm_j(insn):
    v = (((insn >> 31) & 1) << 20) | (((insn >> 12) & 0xFF) << 12) | \
        (((insn >> 20) & 1) << 11) | (((insn >> 21) & 0x3FF) << 1)
    return v - (1 << 21) if v & 0x100000 else v

def main():
    args = sys.argv[1:]
    bin_f = args[0]
    hex_f = exp_f = None
    start = 0
    trace = "--trace" in args
    for i, a in enumerate(args):
        if a == "--hex":  hex_f = args[i + 1]
        if a == "--exp":  exp_f = args[i + 1]
        if a == "--start": start = int(args[i + 1], 0)
    data = open(bin_f, "rb").read()
    data += b"\0" * ((4 - len(data) % 4) % 4)
    words = list(struct.unpack("<%dI" % (len(data) // 4), data))
    rf, n, endpc, memw = run(words, start, verbose=trace)
    print("emu32: %d instrs, end pc=%#x" % (n, endpc))
    nz = [(i, v) for i, v in enumerate(rf) if i and v]
    print("emu32: rf non-zero: " + " ".join("x%d=%#010x" % t for t in nz))
    if hex_f:
        with open(hex_f, "w") as f:
            for w in words:
                f.write("%08x\n" % w)
    if exp_f:
        with open(exp_f, "w") as f:
            f.write("// generated by tools/emu32.py from %s -- do not edit\n" % bin_f)
            f.write("`define C1_NCOMMIT %d\n" % n)
            for i in range(32):
                f.write("exp[%d] = 32'h%08x;\n" % (i, rf[i]))
            for k in sorted(memw):
                if 0 <= k - 0x20000000 < 16:      # TB m1 RAM page window
                    f.write("mexp[%d] = 32'h%08x;\n" % (k - 0x20000000, memw[k]))

if __name__ == "__main__":
    main()
