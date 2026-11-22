// core -- mythrv32 RV32IM 5-stage in-order core (rtl/core.v)
//
// Increment C1 (notes/core.md): IF/ID/EX/MA/WB with the full fetch path
// (mmu_i -> l1_i), straight-line RV32I ALU subset:
//   ADD SUB SLL SLT SLTU XOR SRL SRA OR AND
//   ADDI SLTI SLTIU XORI ORI ANDI SLLI SRLI SRAI  LUI AUIPC
// Includes regfile (32x32, 2 read @ID + 2 write MA/WB, MA wins) and
// EX/MA + MA/WB forwarding into ID operand muxes. Branches/M-ext/loads/
// stores/CSR/traps/interrupts arrive in C2..C7; the control nets (stall,
// flush, redirect, rf MA-port) exist now and are tied inactive.
//
// MMU configured: satp=0 (Bare), MODE=I (M-mode) -> identity, no fault,
// DONE 1 cycle after REQ.
//
// Fetch protocol (mmu 1-ahead, l1_i 1-ahead on hits; design.md sec.7):
//   mio     mmu request in flight, response not yet consumed
//   pav     mmu DONE seen, PA_O held awaiting l1_i (mmu reqs blocked)
//   lio     l1_i request in flight or DONE not yet consumed (DATA_O held)
//   sustained 1 instr/cycle on all-hit; +1 bubble per mmu/l1 stall cycle.
//
// Reset (priv 3.4 + notes/core.md C5 reset list): M-mode, PC=RESET_PC,
// mstatus.MIE=0, satp=0, all pipeline regs cleared (x0=0, regs=0).
`default_nettype none

module core (
    input clk, rst,
    // interrupt pending (C6; unused now)
    input meip, seip,
    // WB B4 master m0: instruction fetch (read-only, from l1_i)
    output       m0_stb,
    output [3:0] m0_sel,
    output [31:0] m0_adr,
    output       m0_we,
    input        m0_ack,
    input  [31:0] m0_dat,
    // WB B4 master m1: data (read/write, from l1_d; idle in C1)
    output       m1_stb,
    output [3:0] m1_sel,
    output [31:0] m1_adr,
    output       m1_we,
    output [31:0] m1_wdat,
    input        m1_ack,
    input  [31:0] m1_dat,
    // WB B4 master m2: MMU page-table walk (read-only; idle in C1)
    output       m2_stb,
    output [3:0] m2_sel,
    output [31:0] m2_adr,
    output       m2_we,
    input        m2_ack,
    input  [31:0] m2_dat
);

    localparam [31:0] RESET_PC = 32'h0000_0000;

    // ------------------------------------------------------------ ALU ops
    localparam [4:0] ALU_ADD=5'd0, ALU_SUB=5'd1, ALU_XOR=5'd2, ALU_OR=5'd3,
                     ALU_AND=5'd4, ALU_SLL=5'd5, ALU_SRL=5'd6, ALU_SRA=5'd7,
                     ALU_SLT=5'd8, ALU_SLTU=5'd9, ALU_PASS=5'd10,
                     ALU_MUL=5'd11, ALU_MULH=5'd12, ALU_MULHSU=5'd13,
                     ALU_MULHU=5'd14, ALU_DIV=5'd15, ALU_DIVU=5'd16,
                     ALU_REM=5'd17, ALU_REMU=5'd18;
    // operand sources
    localparam [1:0] SRC_RS1=2'd0, SRC_PC=2'd1, SRC_IMM=2'd2;

    // ------------------------------------------------------------ hazard ctl
    // stall: freeze ID/EX/MA/WB + IF/ID (C8: load-use). flush: kill IF/ID and
    // in-flight fetch (C3 branches/traps). redirect: load fpc (C3).
    wire        div_stall;                 // iterative divide (C2)
    wire        stall = div_stall;
    // C3: control transfers resolve in MA (no prediction). redirect loads
    // fpc; flush kills IF/ID + ID/EX + in-flight fetch (discard flags).
    wire        exma_jbr, exma_taken;
    wire [31:0] exma_target;
    wire        redirect = exma_v && exma_jbr && exma_taken;
    wire        flush = redirect;
    wire [31:0] redirect_pc = exma_target;

    // ------------------------------------------------------------ fetch (IF)
    reg  [31:0] fpc;          // next VA to fetch
    reg         mio;          // mmu req in flight (response not consumed)
    reg         pav;          // PA_O held awaiting l1_i
    reg         lio;          // l1_i req in flight / DONE not consumed
    reg         d_mio;        // discard next mmu DONE (post-redirect residue)
    reg         d_lio;        // discard next l1 DONE (post-redirect residue)
    reg  [31:0] pc_mmu;       // VA of the mmu request awaiting DONE
    reg  [31:0] pc_pav;       // VA matching the held PA_O
    reg  [31:0] pc_l1;        // VA of the l1_i request in flight

    // fetch response FIFO (4 x {pc,data}): decouples l1_i DONE pulses from
    // IF/ID stalls (divider etc). Issue rule keeps count_next + worst-case
    // in-flight arrivals <= depth: sustained 1/cyc on hits (net count 0-1).
    reg  [31:0] fq_data [0:3];
    reg  [31:0] fq_pc   [0:3];
    reg  [2:0]  fq_w, fq_r;
    reg  [2:0]  fq_c;

    wire        mmu_done, mmu_fault;
    wire [31:0] mmu_pa;
    wire [3:0]  mmu_cause;
    wire        l1_done;
    wire [31:0] l1_data;

    wire fresh_mmu = mmu_done && !(d_mio || flush);
    wire fresh_l1  = l1_done  && !(d_lio || flush);
    wire deq        = (fq_c != 3'd0) && !stall;
    wire bypass     = fresh_l1 && (fq_c == 3'd0) && !stall;   // straight to IF/ID
    wire enq        = fresh_l1 && !bypass;
    wire consume    = deq || bypass;
    wire [2:0] fq_c_next = fq_c + (enq ? 3'd1 : 3'd0) - (deq ? 3'd1 : 3'd0);
    wire l1_free  = !lio || (l1_done && !stall); // 1-AHEAD needs slot + no stall
    wire space    = (fq_c_next + (lio && !l1_done ? 3'd1 : 3'd0)) <= 3'd3;
    wire req_l1   = !flush && (pav || (mio && fresh_mmu)) && l1_free && space;
    wire req_mmu  = !flush && !pav && (!mio || (fresh_mmu && l1_free && space));

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fpc    <= RESET_PC;
            mio    <= 1'b0;
            pav    <= 1'b0;
            lio    <= 1'b0;
            d_mio  <= 1'b0;
            d_lio  <= 1'b0;
            fq_w   <= 3'd0;
            fq_r   <= 3'd0;
            fq_c   <= 3'd0;
            pc_mmu <= 32'h0;
            pc_pav <= 32'h0;
            pc_l1  <= 32'h0;
        end else begin
            mio <= flush ? 1'b0 : (req_mmu ? 1'b1 : (mmu_done ? 1'b0 : mio));
            pav <= flush ? 1'b0 : (req_l1 ? 1'b0 : (pav || fresh_mmu));
            lio <= req_l1 ? 1'b1 : (l1_done ? 1'b0 : lio);
            d_mio <= (d_mio || (flush && mio)) && !mmu_done;
            d_lio <= (d_lio || (flush && lio)) && !l1_done;
            if (enq) begin
                fq_data[fq_w[1:0]] <= l1_data;
                fq_pc  [fq_w[1:0]] <= pc_l1;
                fq_w <= flush ? 3'd1 : fq_w + 3'd1;
            end
            if (deq && !flush) fq_r <= fq_r + 3'd1;
            fq_c <= flush ? 3'd0 : fq_c_next;
            if (req_mmu) pc_mmu <= fpc;
            if (req_l1)  pc_l1  <= pav ? pc_pav : pc_mmu;
            if (mmu_done && !req_l1) pc_pav <= pc_mmu;
            if (redirect)      fpc <= redirect_pc;
            else if (req_mmu)  fpc <= fpc + 32'h4;
        end
    end

    // ------------------------------------------------------------ sub-blocks
    // I-side MMU: fetch (ACC=0), M-mode, satp=0 -> identity, no fault.
    wire       mmi_stb, mmi_we;
    wire [3:0] mmi_sel;
    wire [31:0] mmi_adr;
    mmu mmu_i (
        .clk(clk), .rst(rst),
        .REQ_I(req_mmu), .VA_I(fpc),
        .ACC_I(2'b00), .MODE_I(2'b00),
        .SUM_I(1'b0), .MXR_I(1'b0), .SATP_I(32'h0),
        .DONE_O(mmu_done), .PA_O(mmu_pa),
        .FAULT_O(mmu_fault), .CAUSE_O(mmu_cause),
        .SF_I(2'b00), .SFASID_I(9'h0),
        .STB_O(mmi_stb), .SEL_O(mmi_sel), .ADR_O(mmi_adr), .WE_O(mmi_we),
        .ACK_I(m2_ack), .DAT_I(m2_dat)
    );

    // D-side MMU: idle until C4 (REQ held low).
    wire       mmd_done, mmd_fault;
    wire [31:0] mmd_pa;
    wire [3:0]  mmd_cause;
    wire       mmd_stb, mmd_we;
    wire [3:0] mmd_sel;
    wire [31:0] mmd_adr;
    mmu mmu_d (
        .clk(clk), .rst(rst),
        .REQ_I(1'b0), .VA_I(32'h0),
        .ACC_I(2'b01), .MODE_I(2'b00),
        .SUM_I(1'b0), .MXR_I(1'b0), .SATP_I(32'h0),
        .DONE_O(mmd_done), .PA_O(mmd_pa),
        .FAULT_O(mmd_fault), .CAUSE_O(mmd_cause),
        .SF_I(2'b00), .SFASID_I(9'h0),
        .STB_O(mmd_stb), .SEL_O(mmd_sel), .ADR_O(mmd_adr), .WE_O(mmd_we),
        .ACK_I(m2_ack), .DAT_I(m2_dat)
    );

    // m2 arbiter: both MMU walkers are WB-RO; D-side priority, grant follows
    // mmd_stb. In C1 mmd_stb is always 0 (pure passthrough to mmu_i).
    assign m2_stb = mmd_stb ? 1'b1 : mmi_stb;
    assign m2_sel = 4'b1111;
    assign m2_we  = 1'b0;
    assign m2_adr = mmd_stb ? mmd_adr : mmi_adr;

    // I-cache: 1-AHEAD requests on consecutive cycles are legal on hits.
    l1_i l1i (
        .clk(clk), .rst(rst),
        .REQ_I(req_l1), .PA_I(mmu_pa), .INVAL_I(1'b0),
        .DONE_O(l1_done), .DATA_O(l1_data),
        .STB_O(m0_stb), .SEL_O(m0_sel), .ADR_O(m0_adr), .WE_O(m0_we),
        .ACK_I(m0_ack), .DAT_I(m0_dat)
    );

    // D-cache: idle until C4.
    wire       mld_done;
    wire [31:0] mld_data;
    l1_d l1d (
        .clk(clk), .rst(rst),
        .REQ_I(1'b0), .PA_I(32'h0), .WEN_I(1'b0), .WDAT_I(32'h0),
        .DONE_O(mld_done), .DATA_O(mld_data),
        .STB_O(m1_stb), .SEL_O(m1_sel), .ADR_O(m1_adr), .WE_O(m1_we),
        .WDAT_O(m1_wdat), .ACK_I(m1_ack), .DAT_I(m1_dat)
    );

    // ------------------------------------------------------------ IF/ID
    reg         ifid_v;
    reg  [31:0] ifid_pc, ifid_instr;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ifid_v <= 1'b0; ifid_pc <= 32'h0; ifid_instr <= 32'h0;
        end else if (flush) begin
            ifid_v <= 1'b0;
        end else if (!stall) begin
            ifid_v     <= consume;
            ifid_pc    <= deq ? fq_pc[fq_r[1:0]] : pc_l1;
            ifid_instr <= deq ? fq_data[fq_r[1:0]] : l1_data;
        end
    end

    // ------------------------------------------------------------ stage regs
    reg         idex_v, idex_wr;
    reg  [4:0]  idex_rd;
    reg  [4:0]  idex_alu;
    reg         idex_jbr, idex_jump, idex_ind;
    reg  [2:0]  idex_bsel3;
    reg  [31:0] idex_pc, idex_a, idex_b, idex_off, idex_c;
    reg         exma_v, exma_wr;
    reg  [4:0]  exma_rd;
    reg  [31:0] exma_res, exma_pc;
    reg         exma_jbr_q, exma_taken_q;
    reg  [31:0] exma_target_q;
    reg         mawb_v, mawb_wr;
    reg  [4:0]  mawb_rd;
    reg  [31:0] mawb_res, mawb_pc;

    assign exma_jbr   = exma_jbr_q;
    assign exma_taken = exma_taken_q;
    assign exma_target = exma_target_q;

    // ------------------------------------------------------------ ID decode
    wire [6:0] op   = ifid_instr[6:0];
    wire [4:0] rd   = ifid_instr[11:7];
    wire [2:0] f3   = ifid_instr[14:12];
    wire [4:0] rs1  = ifid_instr[19:15];
    wire [4:0] rs2  = ifid_instr[24:20];
    wire [6:0] f7   = ifid_instr[31:25];

    wire [31:0] imm_i = {{20{ifid_instr[31]}}, ifid_instr[31:20]};
    wire [31:0] imm_u = {ifid_instr[31:12], 12'h0};
    wire [31:0] imm_b = {{19{ifid_instr[31]}}, ifid_instr[31], ifid_instr[7],
                         ifid_instr[30:25], ifid_instr[11:8], 1'b0};
    wire [31:0] imm_j = {{11{ifid_instr[31]}}, ifid_instr[31], ifid_instr[19:12],
                         ifid_instr[20], ifid_instr[30:21], 1'b0};

    wire is_op    = (op == 7'h33);
    wire is_opimm = (op == 7'h13);
    wire is_lui   = (op == 7'h37);
    wire is_auipc = (op == 7'h17);
    wire is_br    = (op == 7'h63);
    wire is_jal   = (op == 7'h6F);
    wire is_jalr  = (op == 7'h67);
    wire legal    = is_op | is_opimm | is_lui | is_auipc | is_br | is_jal | is_jalr; // C5 traps illegal

    reg        c_wr;
    reg  [1:0] c_asrc;
    reg  [1:0] c_bsel;  // 0=rs2(fwd), 1=imm, 2=const 4
    reg  [4:0] c_alu;
    reg  [4:0] d_rs1, d_rs2;
    reg  [31:0] d_imm;
    reg        c_jbr, c_jump, c_ind;
    reg  [2:0] c_bsel3;

    always @(*) begin
        c_wr = 1'b0;  c_asrc = SRC_RS1; c_bsel = 2'd0; c_alu = ALU_PASS;
        d_rs1  = 5'h0;  d_rs2  = 5'h0;    d_imm = imm_i;
        c_jbr = 1'b0;   c_jump = 1'b0;    c_ind = 1'b0; c_bsel3 = 3'h0;
        if (is_br) begin
            c_jbr = 1'b1; c_asrc = SRC_RS1; c_bsel = 2'd0; d_imm = imm_b;
            c_bsel3 = f3;
        end else if (is_jal) begin
            c_wr = 1'b1; c_jbr = 1'b1; c_jump = 1'b1;
            c_asrc = SRC_PC; c_bsel = 2'd2; c_alu = ALU_ADD; d_imm = imm_j;
        end else if (is_jalr) begin
            c_wr = 1'b1; c_jbr = 1'b1; c_jump = 1'b1; c_ind = 1'b1;
            c_asrc = SRC_PC; c_bsel = 2'd2; c_alu = ALU_ADD; d_imm = imm_i;
        end else if (is_op && (f7 == 7'h01)) begin   // M extension (f7=0x01)
            c_wr = 1'b1; c_asrc = SRC_RS1; c_bsel = 2'd0;
            d_rs1 = rs1; d_rs2 = rs2;
            case (f3)
                3'b000: c_alu = ALU_MUL;    3'b001: c_alu = ALU_MULH;
                3'b010: c_alu = ALU_MULHSU; 3'b011: c_alu = ALU_MULHU;
                3'b100: c_alu = ALU_DIV;    3'b101: c_alu = ALU_DIVU;
                3'b110: c_alu = ALU_REM;    3'b111: c_alu = ALU_REMU;
            endcase
        end else if (is_op) begin
            c_wr = 1'b1; c_asrc = SRC_RS1; c_bsel = 2'd0;
            d_rs1 = rs1; d_rs2 = rs2;
            case (f3)
                3'b000: c_alu = f7[5] ? ALU_SUB : ALU_ADD;
                3'b001: c_alu = ALU_SLL;
                3'b010: c_alu = ALU_SLT;
                3'b011: c_alu = ALU_SLTU;
                3'b100: c_alu = ALU_XOR;
                3'b101: c_alu = f7[5] ? ALU_SRA : ALU_SRL;
                3'b110: c_alu = ALU_OR;
                3'b111: c_alu = ALU_AND;
            endcase
        end else if (is_opimm) begin
            c_wr = 1'b1; c_asrc = SRC_RS1; c_bsel = 2'd1; d_rs1 = rs1;
            case (f3)
                3'b000: c_alu = ALU_ADD;
                3'b001: c_alu = ALU_SLL;
                3'b010: c_alu = ALU_SLT;
                3'b011: c_alu = ALU_SLTU;
                3'b100: c_alu = ALU_XOR;
                3'b101: c_alu = ifid_instr[30] ? ALU_SRA : ALU_SRL;
                3'b110: c_alu = ALU_OR;
                3'b111: c_alu = ALU_AND;
            endcase
        end else if (is_lui) begin
            c_wr = 1'b1; c_asrc = SRC_IMM; c_alu = ALU_PASS; d_imm = imm_u;
        end else if (is_auipc) begin
            c_wr = 1'b1; c_asrc = SRC_PC; c_bsel = 2'd1; c_alu = ALU_ADD;
            d_imm = imm_u;
        end
    end

    // ------------------------------------------------------------ regfile
    reg [31:0] rf [0:31];
    wire [31:0] rf_r1 = (rs1 == 5'h0) ? 32'h0 : rf[rs1];
    wire [31:0] rf_r2 = (rs2 == 5'h0) ? 32'h0 : rf[rs2];

    // rf write ports: MA (load, C4; wins) + WB (ALU results)
    wire        rf_we_a = 1'b0;            // MA port: no memory ops in C2
    wire [4:0]  rf_waddr_a = 5'h0;
    wire [31:0] rf_wdata_a = 32'h0;
    wire        rf_we_w = mawb_wr && (mawb_rd != 5'h0) && !stall;

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1) rf[i] <= 32'h0;
        end else if (rf_we_a && (rf_waddr_a != 5'h0)) begin
            rf[rf_waddr_a] <= rf_wdata_a;   // MA wins on same-target conflict
            if (rf_we_w && (rf_waddr_a != mawb_rd)) rf[mawb_rd] <= mawb_res;
        end else if (rf_we_w) begin
            rf[mawb_rd] <= mawb_res;
        end
    end

    // ------------------------------------------------------------ ID/EX
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            idex_v <= 1'b0; idex_wr <= 1'b0; idex_rd <= 5'h0; idex_alu <= 5'h0;
            idex_jbr <= 1'b0; idex_jump <= 1'b0; idex_ind <= 1'b0; idex_bsel3 <= 3'h0;
            idex_pc <= 32'h0; idex_a <= 32'h0; idex_b <= 32'h0;
            idex_off <= 32'h0; idex_c <= 32'h0;
        end else if (flush) begin
            idex_v <= 1'b0; idex_wr <= 1'b0;
        end else if (!stall) begin
            idex_v    <= ifid_v && legal;
            idex_wr   <= ifid_v && c_wr;
            idex_rd   <= rd;
            idex_alu  <= c_alu;
            idex_jbr  <= ifid_v && c_jbr;
            idex_jump <= c_jump;
            idex_ind  <= c_ind;
            idex_bsel3<= c_bsel3;
            idex_pc   <= ifid_pc;
            idex_off  <= d_imm;
            idex_c    <= fwd_a;
            idex_a    <= (c_asrc == SRC_RS1) ? fwd_a :
                         (c_asrc == SRC_PC)  ? ifid_pc : d_imm;
            idex_b    <= (c_bsel == 2'd0) ? fwd_b :
                        (c_bsel == 2'd1) ? d_imm : 32'h4;
        end
    end


    // ------------------------------- ID-side forwarding (operands at latch)
    // Consumer latches operands on its ID->EX edge; producer is then either
    // in idex (d=1: combinational alu_res -- safe for mul/div because the
    // latch only ever happens on a !stall edge, i.e. the divider's D_DONE
    // cycle for div), exma (d=2), mawb (d=3), or retired to rf (d>=4).
    // Youngest matching writer wins; x0 never forwards.
    wire [31:0] fwd_a =
        (idex_v && idex_wr && (idex_rd == rs1) && (rs1 != 5'h0)) ? alu_res :
        (exma_wr && (exma_rd != 5'h0) && (exma_rd == rs1))       ? exma_res :
        (mawb_wr && (mawb_rd != 5'h0) && (mawb_rd == rs1))       ? mawb_res : rf_r1;
    wire [31:0] fwd_b =
        (idex_v && idex_wr && (idex_rd == rs2) && (rs2 != 5'h0)) ? alu_res :
        (exma_wr && (exma_rd != 5'h0) && (exma_rd == rs2))       ? exma_res :
        (mawb_wr && (mawb_rd != 5'h0) && (mawb_rd == rs2))       ? mawb_res : rf_r2;

    // ------------------------------------------------------------ EX (ALU)
    // NB (iverilog -g2005): $signed()>>>sh inside a ?: chain mis-evaluates
    // (probe: /tmp/opencode/sra_probe.v) -> SRA computed on its own wire.
    // Same rule applied to the signed 64-bit products (C2): dedicated wires,
    // ternary only selects.
    wire [4:0]  sh = idex_b[4:0];
    wire [31:0] sra_res = ($signed(idex_a) >>> sh);
    wire signed [63:0] mul_ss = $signed(idex_a) * $signed(idex_b);
    wire signed [63:0] mul_su = $signed(idex_a) * $signed({1'b0, idex_b});
    wire        [63:0] mul_uu = idex_a * idex_b;

    // ---------------------------------------------------- C2: divider (EX)
    // 34-cycle restoring divide (1 load + 32 iter + 1 done) stalling EX.
    // Special cases (1 cycle, no FSM): div-0 -> quo=all1/rem=dividend;
    // signed overflow MIN/-1 -> quo=dividend/rem=0 (unpriv Table 11).
    wire is_div = idex_v && ((idex_alu == ALU_DIV)  || (idex_alu == ALU_DIVU) ||
                             (idex_alu == ALU_REM)  || (idex_alu == ALU_REMU));
    wire is_rem = (idex_alu == ALU_REM) || (idex_alu == ALU_REMU);
    wire sdiv   = (idex_alu == ALU_DIV) || (idex_alu == ALU_REM);
    wire div_z  = (idex_b == 32'h0);
    wire div_ov = sdiv && (idex_a == 32'h8000_0000) && (idex_b == 32'hFFFF_FFFF);
    wire div_sp = is_div && (div_z || div_ov);
    wire [31:0] div_aa = (sdiv && idex_a[31]) ? (~idex_a + 32'h1) : idex_a;
    wire [31:0] div_bb = (sdiv && idex_b[31]) ? (~idex_b + 32'h1) : idex_b;

    localparam D_IDLE = 2'd0, D_RUN = 2'd1, D_DONE = 2'd2;
    reg [1:0]  ds;
    reg [4:0]  dc;
    reg [32:0] drem;
    reg [31:0] dquo, ddiv, dnd;
    reg        dqs, drs;

    wire [31:0] qmag = dqs ? (~dquo + 32'h1) : dquo;
    wire [31:0] rmag = drs ? (~drem[31:0] + 32'h1) : drem[31:0];
    wire [31:0] div_q = div_sp ? (div_ov ? idex_a : 32'hFFFF_FFFF) : qmag;
    wire [31:0] div_r = div_sp ? (div_ov ? 32'h0 : idex_a) : rmag;
    wire [31:0] div_res = is_rem ? div_r : div_q;

    wire [32:0] trial = {drem[31:0], dnd[31]};
    assign div_stall = is_div && (ds != D_DONE) && !div_sp;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ds <= D_IDLE; dc <= 5'h0;
            drem <= 33'h0; dquo <= 32'h0; ddiv <= 32'h0; dnd <= 32'h0;
            dqs <= 1'b0; drs <= 1'b0;
        end else begin
            if (flush && is_div) ds <= D_IDLE;   // killed div: abort FSM
            else case (ds)
            D_IDLE: if (is_div && !div_sp) begin
                ds   <= D_RUN;
                dc   <= 5'h0;
                drem <= 33'h0;
                dquo <= 32'h0;
                ddiv <= div_bb;
                dnd  <= div_aa;
                dqs  <= sdiv && (idex_a[31] ^ idex_b[31]);
                drs  <= sdiv && idex_a[31];
            end
            D_RUN: begin
                if (trial >= ddiv) begin
                    drem <= trial - ddiv;
                    dquo <= {dquo[30:0], 1'b1};
                end else begin
                    drem <= trial;
                    dquo <= {dquo[30:0], 1'b0};
                end
                dnd <= dnd << 1;
                if (dc == 5'd31) ds <= D_DONE;
                else             dc <= dc + 5'h1;
            end
            D_DONE: ds <= D_IDLE;   // EX/MA captured div_res this cycle
            default: ds <= D_IDLE;
            endcase
        end
    end

    wire [31:0] alu_res;
    assign alu_res =
        (idex_alu == ALU_ADD)     ? (idex_a + idex_b) :
        (idex_alu == ALU_SUB)     ? (idex_a - idex_b) :
        (idex_alu == ALU_XOR)     ? (idex_a ^  idex_b) :
        (idex_alu == ALU_OR)      ? (idex_a |  idex_b) :
        (idex_alu == ALU_AND)     ? (idex_a &  idex_b) :
        (idex_alu == ALU_SLL)     ? (idex_a << sh) :
        (idex_alu == ALU_SRL)     ? (idex_a >> sh) :
        (idex_alu == ALU_SRA)     ? sra_res :
        (idex_alu == ALU_SLT)     ? {31'h0, $signed(idex_a) < $signed(idex_b)} :
        (idex_alu == ALU_SLTU)    ? {31'h0, idex_a < idex_b} :
        (idex_alu == ALU_MUL)     ? mul_ss[31:0] :
        (idex_alu == ALU_MULH)    ? mul_ss[63:32] :
        (idex_alu == ALU_MULHSU)  ? mul_su[63:32] :
        (idex_alu == ALU_MULHU)   ? mul_uu[63:32] :
        (idex_alu == ALU_PASS)    ? idex_a :
        is_div                    ? div_res : 32'h0;

    // ------------------------------------------------ EX: branch/jump (C3)
    // taken/target computed in EX, committed (redirect+flush) in MA.
    wire cond_eq  = (idex_a == idex_b);
    wire cond_lt  = ($signed(idex_a) < $signed(idex_b));
    wire cond_ltu = (idex_a < idex_b);
    wire ex_taken = idex_jump ? 1'b1 :
                    (idex_bsel3 == 3'b000) ?  cond_eq  :
                    (idex_bsel3 == 3'b001) ? ~cond_eq  :
                    (idex_bsel3 == 3'b100) ?  cond_lt  :
                    (idex_bsel3 == 3'b101) ? ~cond_lt  :
                    (idex_bsel3 == 3'b110) ?  cond_ltu :
                    (idex_bsel3 == 3'b111) ? ~cond_ltu : 1'b0;
    wire [31:0] ex_target = idex_ind ? ((idex_c + idex_off) & ~32'h1)
                                     : (idex_pc + idex_off);

    // ------------------------------------------------------------ EX/MA
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            exma_v <= 1'b0; exma_wr <= 1'b0; exma_rd <= 5'h0; exma_res <= 32'h0; exma_pc <= 32'h0;
            exma_jbr_q <= 1'b0; exma_taken_q <= 1'b0; exma_target_q <= 32'h0;
        end else if (flush) begin
            exma_v <= 1'b0; exma_wr <= 1'b0; exma_jbr_q <= 1'b0;
        end else if (!stall) begin
            exma_v    <= idex_v;
            exma_wr   <= idex_wr;
            exma_rd   <= idex_rd;
            exma_res  <= alu_res;
            exma_pc   <= idex_pc;
            exma_jbr_q   <= idex_jbr;
            exma_taken_q <= ex_taken;
            exma_target_q <= ex_target;
        end
    end

    // ------------------------------------------------------------ MA/WB
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mawb_v <= 1'b0; mawb_wr <= 1'b0; mawb_rd <= 5'h0; mawb_res <= 32'h0; mawb_pc <= 32'h0;
        end else if (!stall) begin
            mawb_v    <= exma_v;
            mawb_wr   <= exma_wr;
            mawb_rd   <= exma_rd;
            mawb_res  <= exma_res;
            mawb_pc   <= exma_pc;
        end
    end

    // ------------------------------------------------------------ commit tap
    wire commit = mawb_v && !stall;   // retirement pulse (TB trace/terminate)

endmodule
`default_nettype wire
