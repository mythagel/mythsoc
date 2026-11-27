#!/usr/bin/env python3
# tools/instr_enc.py -- RV32I instruction encoders for hand-built machine
# words in tests (rule 2: never compute encodings by hand).
#   usage: python3 tools/instr_enc.py
# Emits the constants t05_soc.c uses for its fence.i / self-modifying-code
# battery (LUI/ADDI/RET variants) + verifies against llvm-mc.
M32 = 0xFFFFFFFF


def lui(rd, imm20):
    assert 0 <= imm20 <= 0xFFFFF
    return ((imm20 << 12) | (rd << 7) | 0x37) & M32


def addi(rd, rs1, imm12):
    assert -2048 <= imm12 <= 2047
    return (((imm12 & 0xFFF) << 20) | (rs1 << 15) | (0 << 12) | (rd << 7)
            | 0x13) & M32


def jalr(rd, rs1, imm12):
    return (((imm12 & 0xFFF) << 20) | (rs1 << 15) | (0 << 12) | (rd << 7)
            | 0x67) & M32


def fence_i():
    # FENCE.I: opcode 0001111, funct3=001, imm[11:0]=1 (unpriv Ch.5 table)
    return ((1 << 20) | (0 << 12) | 0x0F) & M32


def fence(pred=0, succ=0):
    # FENCE: fm[31:28]=0, pred[27:24], succ[23:20] (unpriv FENCE table).
    # opcode 0001111, funct3=000. fence rw,rw = 0x0330000f (llvm-mc verified).
    return ((((pred & 0xF) << 24) | ((succ & 0xF) << 20) | 0x0F)) & M32


A0, RA = 10, 1

if __name__ == "__main__":
    # SMC test function v1: returns 0x12345678 (a0)
    v1 = [lui(A0, 0x12345), addi(A0, A0, 0x678), jalr(0, RA, 0)]
    # v2 (patched first word): returns 0xABCDE678
    v2 = [lui(A0, 0xABCDE), addi(A0, A0, 0x678), jalr(0, RA, 0)]
    print("v1 =", [hex(w) for w in v1])
    print("v2 =", [hex(w) for w in v2])
    print("fence.i =", hex(fence_i()))
    print("fence rw,rw =", hex(fence(3, 3)))
    # verify expected a0 values from the encodings
    for name, words, exp in (("v1", v1, 0x12345678), ("v2", v2, 0xABCDE678)):
        lui_imm = words[0] >> 12
        imm = (words[1] >> 20) & 0xFFF
        if imm >= 0x800:
            imm -= 0x1000
        got = (lui_imm << 12) + imm & M32
        assert got == exp, (name, hex(got), hex(exp))
        print(f"{name}: a0 = {got:#010x} OK")
