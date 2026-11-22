# tb_core.S -- C1 group: straight-line RV32I ALU (mythrv32)
# Every op (Table 13), all rd (x1..x31), x0 immovable, forwarding chains.
# Goldens computed by tools/emu32.py -- nothing here is hand-checked.
# No branches/loads/stores (C3/C4). Ends with nops for pipeline drain.
    .section .text
    .globl _start
_start:
    lui   x1, 0xFFFFF           # 0xFFFFF000
    addi  x1, x1, -2048         # 0xFFFFF800
    lui   x2, 0x1               # 0x00001000
    addi  x2, x2, 2047          # 0x00001FFF  (imm boundary)
    lui   x3, 0x80000           # 0x80000000  (sign bit)
    addi  x4, x3, -2048         # wrap -> positive
    add   x5, x1, x2            # R-type ADD w/ carry-out
    sub   x6, x2, x1            # R-type SUB
    and   x7, x1, x2
    or    x8, x1, x2
    xor   x9, x1, x2
    slt   x10, x1, x2           # neg < pos -> 1
    slt   x11, x2, x1           # -> 0
    sltu  x12, x1, x2           # big unsigned vs small -> 0
    sltu  x13, x2, x1           # -> 1
    slti  x14, x1, -1           # signed imm cmp
    slti  x15, x2, 1
    sltiu x16, x2, -1           # imm sext -> unsigned max -> 1
    sltiu x17, x3, 1            # -> 0
    xori  x18, x3, -1           # NOT
    ori   x19, x1, 0x10
    andi  x20, x1, 0x7F8
    slli  x21, x2, 31           # shamt max
    srli  x22, x3, 31
    srai  x23, x3, 4            # sign fill
    srai  x24, x4, 31           # positive -> 0
    srli  x25, x4, 12
    sll   x26, x2, x1           # shamt = x1 & 31 (rs2 mask)
    srl   x27, x1, x2           # shamt = 31
    sra   x28, x1, x4           # shamt = 0
    lui   x29, 0x12345
    auipc x30, 0x100            # pc-relative
    addi  x0, x1, 5             # x0 must stay 0
    add   x31, x0, x0           # 0 via x0 sources
    # forwarding: EX/MA->EX and MA/WB->EX, back-to-back
    addi  x20, x29, 7
    xor   x21, x20, x30
    and   x22, x21, x22
    or    x23, x22, x23
    slt   x24, x23, x24
    add   x25, x24, x20
    # ---- C2: M extension (unpriv 12.1/12.2, Table 11 div semantics) ----
    lui   x1, 0xF0F0F           # operands
    ori   x1, x1, 0x0F0         # x1 = 0xF0F0F0F0 (neg)
    lui   x2, 0xABC             #
    ori   x2, x2, 0x5EF         # x2 = 0x00ABC5EF (pos)
    lui   x15, 0xFFFFF
    addi  x15, x15, -16         # x15 = 0xFFFFEFF0 (neg small mag)
    mul    x3, x1, x2           # MUL low32
    mulh   x4, x1, x2           # MULH  s x s high
    mulhsu x5, x1, x2           # MULHSU s x u high
    mulhu  x6, x1, x2           # MULHU  u x u high
    div    x7, x1, x15          # DIV  s/s
    divu   x8, x1, x2           # DIVU (u view of neg a)
    rem    x9, x1, x15          # REM  sign = dividend
    remu   x10, x1, x2          # REMU
    addi   x16, x0, 0           # zero divisor operand
    div    x11, x1, x16         # div-by-zero -> all ones
    rem    x12, x1, x16         # rem-by-zero -> dividend
    divu   x13, x2, x16         # unsigned zero -> all ones
    remu   x14, x2, x16         # -> dividend
    lui   x17, 0x80000          # x17 = 0x8000_0000 (INT_MIN)
    addi  x19, x0, -1           # x19 = -1
    div    x18, x17, x19        # overflow -> quotient = dividend
    rem    x20, x17, x19        # overflow -> remainder = 0
    divu   x21, x17, x2         # 0x80000000 edge unsigned
    remu   x22, x17, x2
    divu   x23, x2, x17         # quotient 0 remainder = dividend
    remu   x24, x2, x17
    mul    x25, x17, x19        # MIN * -1 low32
    mulhu  x26, x17, x19        # MIN * 0xFFFFFFFF unsigned high
    addi  x0, x0, 0             # nop
    addi  x0, x0, 0
