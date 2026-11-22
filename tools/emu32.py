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
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from mmu_addr import walk

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

def imm_b(insn):
    v = (((insn >> 31) & 1) << 12) | (((insn >> 7) & 1) << 11) | \
        (((insn >> 25) & 0x3F) << 5) | (((insn >> 8) & 0xF) << 1)
    return v - (1 << 13) if v & 0x1000 else v


def imm_j(insn):
    v = (((insn >> 31) & 1) << 20) | (((insn >> 12) & 0xFF) << 12) | \
        (((insn >> 20) & 1) << 11) | (((insn >> 21) & 0x3FF) << 1)
    return v - (1 << 21) if v & 0x100000 else v


def run(words, start_pc, verbose=False, arm_m=-1, arm_s=-1, pt=None):
    # pt: dict word-index -> PTE word (tb/tb_core_pt.hex); Sv32 ref walk
    rf = [0] * 32
    memw = {}                      # word index -> word (DRAM model, C4)
    P = {'priv': 3, 'mstatus': 0, 'medeleg': 0, 'mideleg': 0, 'mie': 0,
         'mtvec': 0, 'mscratch': 0, 'mepc': 0, 'mcause': 0, 'mtval': 0,
         'stvec': 0, 'sscratch': 0, 'sepc': 0, 'scause': 0, 'stval': 0,
         'satp': 0, 'stimecmp': 0xFFFFFFFF, 'ssip': 0}
    MMSK, SMSK = 0x000e1baa, 0x000c0322
    DELEGS, DELEGW, MIEM = 0x0000b7ff, 0x00000222, 0x00000aaa
    pc = start_pc
    n = 0

    def csr_read(a):
        m = P['mstatus']
        return {0x300: m & MMSK, 0x301: 0x40001100, 0x302: P['medeleg'],
                0x303: P['mideleg'], 0x304: P['mie'] & MIEM, 0x305: P['mtvec'],
                0x340: P['mscratch'], 0x341: P['mepc'], 0x342: P['mcause'],
                0x343: P['mtval'],
                0x344: (0x800 if False else 0) | (0x200 if False else 0) | (2 * P['ssip']),
                0x100: m & SMSK, 0x104: P['mie'] & P['mideleg'],
                0x105: P['stvec'], 0x140: P['sscratch'], 0x141: P['sepc'],
                0x142: P['scause'], 0x143: P['stval'],
                0x144: (P['ssip'] * 2) & P['mideleg'],
                0x14D: P['stimecmp'], 0x15D: 0, 0x180: P['satp']}.get(a, None)

    def csr_write(a, v):
        if a in (0x301, 0xF11, 0xF12, 0xF13, 0xF14, 0xF15) or (a >> 10) == 3:
            return                              # RO
        if a == 0x300:
            nv = v & MMSK
            if (nv >> 11) & 3 == 2: nv = (nv & ~0x1800)   # MPP 10 -> U
            P['mstatus'] = (P['mstatus'] & ~MMSK) | nv
        elif a == 0x302: P['medeleg'] = v & DELEGS
        elif a == 0x303: P['mideleg'] = v & DELEGW
        elif a == 0x304: P['mie'] = v & MIEM
        elif a == 0x305: P['mtvec'] = v & ~1
        elif a == 0x340: P['mscratch'] = v
        elif a == 0x341: P['mepc'] = v & ~1
        elif a == 0x342: P['mcause'] = v
        elif a == 0x343: P['mtval'] = v
        elif a == 0x344: P['ssip'] = (v >> 1) & 1
        elif a == 0x100:
            P['mstatus'] = (P['mstatus'] & ~SMSK) | (v & SMSK)
        elif a == 0x104: P['mie'] = (P['mie'] & ~P['mideleg']) | (v & P['mideleg'])
        elif a == 0x105: P['stvec'] = v & ~1
        elif a == 0x140: P['sscratch'] = v
        elif a == 0x141: P['sepc'] = v & ~1
        elif a == 0x142: P['scause'] = v
        elif a == 0x143: P['stval'] = v
        elif a == 0x144: P['ssip'] = (P['ssip'] & ~(P['mideleg'] >> 1 & 1)) | ((v >> 1) & P['mideleg'] >> 1 & 1)
        elif a in (0x14D, 0x15D): P['stimecmp'] = v if a == 0x14D else P['stimecmp']
        elif a == 0x180: P['satp'] = v

    def trap(cause, tval):
        to_s = P['priv'] != 3 and ((P['medeleg'] >> cause) & 1)
        if to_s:
            P['sepc'], P['scause'], P['stval'] = pc & ~1, cause, tval
            m = P['mstatus']
            P['mstatus'] = (m & ~(1 << 8)) | ((P['priv'] & 1) << 8)
            P['mstatus'] = (P['mstatus'] & ~((1 << 5) | (1 << 1))) \
                           | (((m >> 1) & 1) << 5)
            P['priv'] = 1
            return P['stvec'] & ~3
        else:
            P['mepc'], P['mcause'], P['mtval'] = pc & ~1, cause, tval
            P['mstatus'] = (P['mstatus'] & ~0x1800) | (P['priv'] << 11)
            if (P['mstatus'] >> 3) & 1: P['mstatus'] |= 1 << 7
            else: P['mstatus'] &= ~(1 << 7)
            P['mstatus'] &= ~(1 << 3)
            P['priv'] = 3
            return P['mtvec'] & ~3

    def illegal():
        return trap(2, 0)

    PTM = pt if pt else {}
    def xlate(va, acc):
        # effective mode for translation (MPRV affects data, not fetch)
        if acc == 'fetch':
            md = {3: 'M', 1: 'S', 0: 'U'}[P['priv']]
        else:
            eff = P['priv']
            if (P['mstatus'] >> 17) & 1:
                eff = (P['mstatus'] >> 11) & 3
            md = {3: 'M', 1: 'S', 0: 'U'}.get(eff, 'U')
        if md == 'M' or not ((P['satp'] >> 31) & 1):
            return True, va & MASK
        okp, pa, cz, _ = walk(P['satp'], va, lambda ba: PTM.get(ba >> 2, 0),
                              md, acc,
                              mxr=(P['mstatus'] >> 19) & 1,
                              ssum=(P['mstatus'] >> 18) & 1)
        return okp, (pa if okp else cz)

    pins = {'m': 0, 's': 0, 'm_to': 0, 's_to': 0}
    arm = {'m': arm_m, 's': arm_s}
    LENT = 60

    def mip_now():
        return 0x800 * pins['m'] | 0x200 * pins['s'] | 2 * P['ssip']

    def int_deliver(which, epc):
        if which == 'm':
            P['mepc'] = epc & ~1; P['mcause'] = 0x8000000B; P['mtval'] = 0
            m = P['mstatus']
            P['mstatus'] = (m & ~0x1800) | (P['priv'] << 11)
            P['mstatus'] = (P['mstatus'] | (1 << 7)) if ((m >> 3) & 1) \
                else (P['mstatus'] & ~(1 << 7))
            P['mstatus'] &= ~(1 << 3)
            P['priv'] = 3
            return P['mtvec'] & ~3
        else:
            P['sepc'] = epc & ~1; P['scause'] = 0x80000009; P['stval'] = 0
            m = P['mstatus']
            P['mstatus'] = (m & ~(1 << 8)) | ((P['priv'] & 1) << 8)
            P['mstatus'] = (P['mstatus'] & ~((1 << 5) | (1 << 1))) | (((m >> 1) & 1) << 5)
            P['priv'] = 1
            return P['stvec'] & ~3

    guard = 0
    while True:
        if not (P['satp'] >> 31) and not (0 <= pc < 4 * len(words)):
            break
        guard += 1
        if guard > 10000:
            print('emu32: RUNAWAY pc=%#x priv=%d mstatus=%#x' % (pc, P['priv'], P['mstatus']), file=sys.stderr)
            break
        # boundary before retiring this pc: consume pin timers, snapshot
        # interrupt candidacy (pins+enables as of NOW = RTL retire cycle start)
        for k in ('m', 's'):
            if pins[k]:
                pins[k + '_to'] -= 1
                if pins[k + '_to'] <= 0:
                    pins[k] = 0
        mp = P['mie'] & mip_now() & ~P['mideleg']
        sp = P['mie'] & mip_now() & P['mideleg']
        g_m = P['priv'] != 3 or ((P['mstatus'] >> 3) & 1)
        g_s = P['priv'] == 0 or (P['priv'] == 1 and ((P['mstatus'] >> 1) & 1))
        icand = 'm' if (mp and g_m) else ('s' if (sp and g_s) else None)
        fok, fpa = xlate(pc, 'fetch')
        if not fok:
            n += 1
            if verbose:
                print("%#010x: FETCH-FAULT %d" % (pc, fpa), file=sys.stderr)
            pc = trap(12, pc)
            continue
        insn = words[fpa // 4] if 0 <= fpa // 4 < len(words) else 0x0000_000B
        if verbose:
            print("%#010x: %#010x prv=%d satp=%#x ms=%#x" % (pc, insn, P['priv'],
                  P['satp'], P['mstatus']), file=sys.stderr)
        if pc == arm['m'] and not pins['m']:
            pins['m'] = 1; pins['m_to'] = LENT
        if pc == arm['s'] and not pins['s']:
            pins['s'] = 1; pins['s_to'] = LENT
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

        nxt, memevt = pc + 4, None
        if op not in (0x37, 0x17, 0x13, 0x33, 0x03, 0x23, 0x63, 0x67, 0x6F, 0x73):
            pc = illegal(); n += 1; continue
        if op == 0x33 and not (f7 in (0, 0x20, 1) and (f7 != 0x20 or f3 in (0, 5))):
            pc = illegal(); n += 1; continue
        if op == 0x13 and not (
                (f3 in (0, 2, 3, 4, 6, 7)) or
                (f3 == 1 and (insn >> 25) == 0) or
                (f3 == 5 and (insn >> 31) == 0 and ((insn >> 25) & 0x1F) == 0)):
            pc = illegal(); n += 1; continue
        if op == 0x03 and f3 in (3, 6, 7):
            pc = illegal(); n += 1; continue
        if op == 0x23 and f3 not in (0, 1, 2):
            pc = illegal(); n += 1; continue
        if op == 0x63 and f3 in (2, 3):
            pc = illegal(); n += 1; continue
        if op == 0x67 and f3 != 0:
            pc = illegal(); n += 1; continue

        if op == 0x73 and f3 == 0:                    # SYSTEM
            if f7 == 0 and rs2 == 0:                  # ECALL
                pc = trap({3: 11, 1: 9, 0: 8}[P['priv']], 0); n += 1; continue
            if f7 == 0 and rs2 == 1:                  # EBREAK
                pc = trap(3, pc); n += 1; continue
            if f7 == 2 and rs2 == 5:                  # WFI = nop
                n += 1; pc = (pc + 4) & MASK; continue
            elif f7 == 9:                             # SFENCE.VMA = nop (C5)
                n += 1; pc = (pc + 4) & MASK; continue
            elif f7 == 0x18 and rs2 == 2:             # MRET
                if P['priv'] != 3: pc = illegal(); n += 1; continue
                m = P['mstatus']
                P['priv'] = (m >> 11) & 3
                P['mstatus'] = (m & ~(1 << 3)) | (((m >> 7) & 1) << 3)
                P['mstatus'] |= 1 << 7
                P['mstatus'] &= ~0x1800
                P['mstatus'] &= ~(1 << 17)
                pc = P['mepc'] & ~1; n += 1; continue
            elif f7 == 8 and rs2 == 2:                # SRET
                if P['priv'] < 1: pc = illegal(); n += 1; continue
                if P['priv'] == 1 and (P['mstatus'] >> 8) & 1 == 0: pass
                m = P['mstatus']
                P['priv'] = (m >> 8) & 1
                P['mstatus'] = (m & ~(1 << 1)) | (((m >> 5) & 1) << 1)
                P['mstatus'] |= 1 << 5
                P['mstatus'] &= ~(1 << 8)
                pc = P['sepc'] & ~1; n += 1; continue
            else:
                pc = illegal(); n += 1; continue
        elif op == 0x73:                              # CSR (f3 != 0)
            addr = (insn >> 20) & 0xFFF
            need = (addr >> 8) & 3
            pvok = need == 0 or (need == 1 and P['priv'] >= 1) or (need >= 2 and P['priv'] == 3)
            old = csr_read(addr)
            if old is None or not pvok:
                pc = illegal(); n += 1; continue
            src = (insn >> 15) & 31 if (f3 & 4) else a
            ro = (addr >> 10) == 3 or addr in (0x301, 0xF11, 0xF12,
                                               0xF13, 0xF14, 0xF15)
            wr = (f3 in (1, 5)) or (src != 0)         # RW always; RS/RC iff src
            if ro and wr:
                pc = illegal(); n += 1; continue      # write to RO = illegal
            w = old
            if wr:
                nv = src if f3 in (1, 5) else (old | src) if f3 in (2, 6) else (old & ~src)
                csr_write(addr, nv & MASK)
            rf[rd] = 0 if rd == 0 else (w & MASK)
            n += 1
            if icand:
                pc = int_deliver(icand, pc + 4); continue
            pc = (pc + 4) & MASK; continue

        w = None
        if op == 0x37: w = imm_u
        elif op == 0x17: w = (pc + imm_u) & MASK
        elif op == 0x13:
            sh = imm_i & 31
            if f3 == 0: w = (a + imm_i) & MASK
            elif f3 == 1: w = (a << sh) & MASK
            elif f3 == 2: w = int(s(a) < imm_i)
            elif f3 == 3: w = int(a < (imm_i & MASK))
            elif f3 == 4: w = (a ^ imm_i) & MASK
            elif f3 == 5: w = ((a >> sh) & MASK) if ((insn >> 30) & 1) == 0 else ((s(a) >> sh) & MASK)
            elif f3 == 6: w = (a | imm_i) & MASK
            else: w = (a & imm_i) & MASK
        elif op == 0x33 and f7 == 1:
            if f3 == 0: w = (a * b) & MASK
            elif f3 == 1: w = (s(a) * s(b) >> 32) & MASK
            elif f3 == 2: w = (s(a) * b >> 32) & MASK
            elif f3 == 3: w = ((a * b) >> 32) & MASK
            else:
                signed = (f3 in (4, 6))
                if b == 0: q, r = MASK, a
                elif signed and a == 0x80000000 and b == 0xFFFFFFFF: q, r = 0x80000000, 0
                else:
                    sa, sb = (s(a), s(b)) if signed else (a, b)
                    q = abs(sa) // abs(sb)
                    if (sa < 0) != (sb < 0): q = -q
                    r = abs(sa) % abs(sb)
                    if sa < 0: r = -r
                    q, r = q & MASK, r & MASK
                w = r if (f3 in (6, 7)) else q
        elif op == 0x33:
            sh = b & 31
            if f3 == 0: w = ((a - b) if (f7 >> 5) & 1 else (a + b)) & MASK
            elif f3 == 1: w = (a << sh) & MASK
            elif f3 == 2: w = int(s(a) < s(b))
            elif f3 == 3: w = int(a < b)
            elif f3 == 4: w = (a ^ b) & MASK
            elif f3 == 5: w = ((a >> sh) & MASK) if ((f7 >> 5) & 1) == 0 else ((s(a) >> sh) & MASK)
            elif f3 == 6: w = (a | b) & MASK
            else: w = (a & b) & MASK
        elif op == 0x03:                              # LOAD
            eff = (a + imm_i) & MASK
            if (f3 == 2 and eff & 3) or (f3 == 1 and eff & 1):
                pc = trap(4, eff); n += 1; continue
            dok, dpa = xlate(eff, 'load')
            if not dok:
                pc = trap(13, eff); n += 1; continue
            word = memw.get(dpa >> 2, 0)
            off = eff & 3
            if f3 == 0: w = sx((word >> (8 * off)) & 0xFF, 8)
            elif f3 == 1: w = sx((word >> (8 * off)) & 0xFFFF, 16)
            elif f3 == 2: w = word
            elif f3 == 4: w = (word >> (8 * off)) & 0xFF
            else: w = (word >> (8 * off)) & 0xFFFF
        elif op == 0x23:                              # STORE
            eff = (a + imm_s(insn)) & MASK
            off = eff & 3
            if (f3 == 2 and eff & 3) or (f3 == 1 and eff & 1):
                pc = trap(6, eff); n += 1; continue
            dok, dpa = xlate(eff, 'store')
            if not dok:
                pc = trap(15, eff); n += 1; continue
            sel = {0: 1 << off, 1: (0b1100 if off == 2 else 0b0011), 2: 0b1111}[f3]
            wd = {0: (b & 0xFF) * 0x01010101,
                  1: ((b & 0xFFFF) << 16) | (b & 0xFFFF),
                  2: b & MASK}[f3]
            widx = dpa >> 2
            word = memw.get(widx, 0)
            mask = sum(0xFF << (8 * i) for i in range(4) if (sel >> i) & 1)
            memw[widx] = ((word & ~mask) | (wd & mask)) & MASK
        elif op == 0x63:                              # BRANCH
            off = imm_b(insn)
            t = {0: a == b, 1: a != b, 4: s(a) < s(b), 5: s(a) >= s(b),
                 6: a < b, 7: a >= b}[f3]
            if t: nxt = pc + off
        elif op == 0x6F:                              # JAL
            rf[rd] = (pc + 4) & MASK if rd else 0
            nxt = pc + imm_j(insn)
        elif op == 0x67:                              # JALR
            rf[rd] = (pc + 4) & MASK if rd else 0
            nxt = (a + imm_i) & ~1

        if op not in (0x63, 0x67, 0x6F, 0x23):
            rf[rd] = 0 if rd == 0 else (w & MASK)
        n += 1
        if icand and (nxt & MASK) == ((pc + 4) & MASK):
            pc = int_deliver(icand, pc + 4)
            continue
        if verbose:
            print("%#010x: %#010x  x%d=%#010x prv=%d" % (pc, insn, rd, rf[rd], P['priv']),
                  file=sys.stderr)
        nxt &= MASK
        if nxt == pc:
            break
        pc = nxt
    return rf, n, pc, memw


def main():
    args = sys.argv[1:]
    bin_f = args[0]
    hex_f = exp_f = None
    start = 0
    trace = "--trace" in args
    arm_m = arm_s = -1
    for i, a in enumerate(args):
        if a == "--arm-m": arm_m = int(args[i + 1], 0)
        if a == "--arm-s": arm_s = int(args[i + 1], 0)
        if a == "--hex":  hex_f = args[i + 1]
        if a == "--exp":  exp_f = args[i + 1]
        if a == "--start": start = int(args[i + 1], 0)
    pt_f = None
    for i, a in enumerate(args):
        if a == "--pt": pt_f = args[i + 1]
    PT = {}
    if pt_f:
        idx = 0
        for ln in open(pt_f):
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            if ln.startswith('@'): idx = int(ln[1:], 16)
            else: PT[idx] = int(ln, 16); idx += 1
    data = open(bin_f, "rb").read()
    data += b"\0" * ((4 - len(data) % 4) % 4)
    words = list(struct.unpack("<%dI" % (len(data) // 4), data))
    rf, n, endpc, memw = run(words, start, verbose=trace, arm_m=arm_m, arm_s=arm_s, pt=PT)
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
                if 0 <= k - 0x20000000 < 160:      # TB m1 RAM page window
                    f.write("mexp[%d] = 32'h%08x;\n" % (k - 0x20000000, memw[k]))

if __name__ == "__main__":
    main()
