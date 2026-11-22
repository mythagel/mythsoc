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
    div    x26, x17, x19        # overflow -> quotient = dividend
    rem    x27, x17, x19        # overflow -> remainder = 0
    divu   x21, x17, x2         # 0x80000000 edge unsigned
    remu   x22, x17, x2
    divu   x23, x2, x17         # quotient 0 remainder = dividend
    remu   x24, x2, x17
    mul    x25, x17, x19        # MIN * -1 low32
    mulhu  x28, x17, x19        # MIN * 0xFFFFFFFF unsigned high
    # ---- C3: control flow (branch taken/not-taken, jal/jalr, loop) ----
    addi  x13, x0, 0            # acc
    addi  x14, x0, 1            # i
    addi  x15, x0, 11           # n
1:  add   x13, x13, x14         # sum 1..10 backward loop
    addi  x14, x14, 1
    blt   x14, x15, 1b
    addi  x16, x0, 0
    beq   x16, x0, 2f           # taken forward
    addi  x16, x0, 99           # must NOT execute
2:  bne   x16, x0, 3f           # not taken
    addi  x17, x0, 42           # executes (x17 overwritten: fine)
3:  jal   x1, 5f                # call subroutine
4:  addi  x19, x0, 7            # after return
    j     6f
5:  add   x18, x13, x13         # body: 2*acc
    jalr  x0, 0(x1)             # return
6:  bltu  x17, x16, 7f          # 42<0 false: not taken
    bgeu  x17, x16, 8f          # 42>=0 true: taken
7:  addi  x21, x0, 111          # must NOT execute
8:  blt   x16, x17, 85f         # 0<42 taken
    addi  x21, x0, 222          # must NOT execute
85: # ---- C4: load/store via l1_d (widths, lanes, store->load, b2b use) ----
    lui   x22, 0x80000          # DRAM base 0x8000_0000 (Bare = identity)
    lui   x24, 0x11223
    addi  x24, x24, 0x344       # data 0x11223344
    sw    x24, 0(x22)
    sw    x24, 16(x22)          # second line
    lw    x23, 0(x22)           # round-trip
    add   x23, x23, x23         # immediately dependent: load forward
    lb    x24, 3(x22)           # byte3 0x11 -> sign-extended
    lbu   x25, 0(x22)           # byte0 0x44
    lh    x28, 2(x22)           # half1 0x1122 -> sign-extended (pos)
    lhu   x29, 2(x22)
    addi  x30, x0, 0xAA
    sb    x30, 1(x22)           # word0 -> 0x1122AA44
    addi  x30, x0, 0x2A5
    sh    x30, 2(x22)           # half1 -> 0x02A5: word0 -> 0x02A5AA44
    lw    x31, 0(x22)           # merged result after sw+sb+sh
    lb    x29, 1(x22)           # 0xAA byte sign-extended (post-store)
9:  jal   x20, 11f
10: addi  x21, x20, 0           # x21 = link (validates jalr path ran)
    j     30f
11: jalr  x0, 0(x20)            # return to 10
30: # ---- C5: CSR + synchronous exceptions ----
    lui   x22, 0x80000          # (keep DRAM base live; handler relies on it)
    lui   x29, 0x7FF
    ori   x29, x29, 0x55A
    csrw  mscratch, x29
    csrr  x28, mscratch         # must equal x29 pattern
    csrr  x27, misa             # 0x40001100 (RO)
    la    x24, m_hnd            # install handler BEFORE trapping tests
    csrw  mtvec, x24
    addi  x23, x0, 0            # M/S record slot counter
    li    x25, -1
    csrw  misa, x25             # write to RO -> illegal trap (rec0: cause2)
    ebreak                      # rec1: cause3 tval=pc
    .word 0                     # illegal encoding: rec2 cause2
    lw    x0, 2(x22)            # misaligned load:  rec3 cause4 tval=+2
    sw    x0, 1(x22)            # misaligned store:  rec4 cause6 tval=+1
    ecall                       # rec5: cause11 (M ecall)
    # --- S-mode phase: delegate ebreak(3) only, drop to S ---
    la    x24, s_hnd
    csrw  stvec, x24
    li    x25, 0x8
    csrw  medeleg, x25
    li    x25, 0x800
    csrs  mstatus, x25          # MPP <- S
    la    x25, scode
    csrw  mepc, x25
    mret                        # -> S at scode, priv=S
scode:
    ebreak                      # delegated -> S handler (rec 0x23), sret->S
    ecall                       # not delegated -> M handler (cause 9):
                                # m_hnd sets MPP=M, mret resumes here in M
    csrr  x26, sstatus          # back in M: view of sstatus - no trap
    # ---- C6: M external interrupt (meip pin armed at irq_arm_m, by TB) ----
irq_arm_m:
    li    x25, 0x800
    csrw  mie, x25              # MEIE
    li    x25, 0x8
    csrs  mstatus, x25          # MIE -> pending meip taken at a clean retire
    addi  x30, x30, 1           # busy retires while pin held
    addi  x30, x30, 1
    addi  x30, x30, 1
    addi  x30, x30, 1
    addi  x30, x30, 1
    addi  x30, x30, 1
    addi  x30, x30, 1
    addi  x30, x30, 1
    addi  x30, x30, 1
    addi  x30, x30, 1
    csrw  mie, x0               # belt: no refire after handler consumed
    li    x25, 0x8
    csrc  mstatus, x25          # MIE off
    # ---- C6b: S-delegated external interrupt (seip armed at irq_arm_s) ----
    li    x25, 0x200
    csrw  mideleg, x25          # SEIP delegated to S
    li    x25, 0x200
    csrs  mie, x25              # SEIE
    li    x25, 0x800
    csrs  mstatus, x25          # MPP <- S
    la    x25, irq_arm_s
    csrw  mepc, x25
    mret                        # -> S
irq_arm_s:
    li    x25, 0x2
    csrs  sstatus, x25          # SIE on (S-mode) -> seip fires
    addi  x31, x0, 1            # S busy retires
    addi  x31, x31, 1
    addi  x31, x31, 1
    addi  x31, x31, 1
    addi  x31, x31, 1
    addi  x31, x31, 1
    addi  x31, x31, 1
    addi  x31, x31, 1
    ecall                       # SEIE/SIE cleared in s_hnd; exit to M:
                                # medeleg[9]=0 now -> m_hnd cause9 -> M
    # ---- C7: Sv32 translation on (page map = tools/ptmap.py) ----
    li    x25, 0xB008
    csrw  medeleg, x25              # delegate 3,12,13,15 to S
    lui   x25, 0x80000
    ori   x25, x25, 0x3FF           # SATP_A0: Sv32, asid0, root PPN 0x3FF
    csrw  satp, x25
    sfence.vma
    li    x25, 0x800
    csrs  mstatus, x25              # MPP=S
    la    x25, c7s
    csrw  mepc, x25
    mret                            # -> S, first translated fetch
c7s:
    lui   x24, 0x10000              # 4K-alias VA base
    lui   x25, 0xC0FF0
    ori   x25, x25, 0x0E5           # 0xC0FF00E5
    sw    x25, 0x100(x24)           # 4K walk -> PA 0x80001100
    lui   x30, 0x80001
    lw    x28, 0x100(x30)           # identity read-back of aliased PA
    lui   x27, 0x20000              # RX-only 4M page
    sw    x25, 0x40(x27)            # -> scause 15 (s_hnd record+skip)
    lw    x29, 0x40(x27)            # load OK: PA 0x80000040
    lui   x26, 0x30000              # U page
    lw    x30, 0x40(x26)            # -> scause 13 (S, no SUM)
    li    x25, 0x40000
    csrs  sstatus, x25              # SUM=1
    lw    x30, 0x40(x26)            # OK now
    csrc  sstatus, x25              # SUM off
    lui   x25, 0x50000              # A=0 page
    lw    x25, 0x40(x25)            # -> 13
    lui   x25, 0x60000              # D=0 page
    lw    x26, 0x40(x25)            # OK
    sw    x26, 0x40(x25)            # -> 15
    lui   x26, 0x40000              # unmapped VA
    la    x25, c7rec
    csrw  sscratch, x25             # cause-12 recovery pc for s_hnd
    jalr  x0, 0(x26)                # instruction fetch fault -> 12
c7rec:
    lui   x25, 0x80400
    ori   x25, x25, 0x3FD           # SATP_A1: asid1, root 0x3FD000
    csrw  satp, x25
    sfence.vma
    lui   x24, 0x10000
    lui   x25, 0xBEEF0              # 0xBEEF0000
    sw    x25, 0x100(x24)           # asid1 alias -> PA 0x80001100
    lui   x27, 0x80000
    lw    x28, 0x100(x30)           # 0xC0FF00E5 (asid0 page, PA 0x80001100)
    lui   x31, 0x80002
    lw    x29, 0x100(x31)           # 0xBEEF0000 (asid1 page, PA 0x80002100)
    ecall                           # cause9 not delegated -> m_hnd -> M
    # ---- MPRV: M data access with effective mode S ----
    li    x25, 0x1800
    csrc  mstatus, x25              # clear MPP
    li    x25, 0x800
    csrs  mstatus, x25              # MPP=S
    lui   x25, 0x2
    csrs  mstatus, x25              # MPRV=1
    lui   x26, 0x30000
    lw    x27, 0x40(x26)            # eff S vs U page, SUM=0 -> 13 (M path)
    li    x25, 0x20000
    csrc  mstatus, x25              # MPRV off
    csrw  satp, x0                  # back to Bare
    sfence.vma
    csrr  x17, misa                 # final re-reads for named checks
    csrr  x16, mscratch
    j     12f
    .align 4
m_hnd:                          # M handler: rec=[cause,epc,tval]; skip; mret
    csrr  x19, mcause
    csrr  x20, mepc
    csrr  x21, mtval
    slli  x25, x23, 4
    addi  x26, x22, 0x100
    add   x25, x26, x25
    sw    x19, 0(x25)
    sw    x20, 4(x25)
    sw    x21, 8(x25)
    addi  x23, x23, 1
    addi  x20, x20, 4
    csrw  mepc, x20
    srli  x26, x19, 31
    beq   x26, x0, 22f          # interrupt: consume (mie=0)
    csrw  mie, x0
22: li    x26, 9
    bne   x19, x26, 20f
    li    x26, 0x1800           # ecall-from-S: return to M (MPP=M)
    csrs  mstatus, x26
20: mret
    .align 4
s_hnd:                          # S handler: record cause|0x20; skip (or
    csrr  x19, scause           # recover via sscratch for fetch-fault 12)
    csrr  x20, sepc
    csrr  x21, stval
    li    x26, 12
    beq   x19, x26, 30f
    addi  x20, x20, 4
    j     31f
30: csrr  x20, sscratch         # jump-to-unmapped recovery pc
31: ori   x19, x19, 0x20
    slli  x25, x23, 4
    addi  x26, x22, 0x100
    add   x25, x26, x25
    sw    x19, 0(x25)
    sw    x20, 4(x25)
    sw    x21, 8(x25)
    addi  x23, x23, 1
    csrw  sepc, x20
    li    x26, 0x200
    csrc  sie, x26              # consume: clear SEIE (mie&mideleg view)
    li    x26, 0x2
    csrc  sstatus, x26          # SIE off (refire guard)
    sret
12: j     12b                   # halt (self-loop)
