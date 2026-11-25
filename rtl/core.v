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

    // ------------------------------------------------ C5: priv & CSR state
    // priv arch encoding: 2'b00 U, 2'b01 S, 2'b11 M (10 reserved/unsupported)
    localparam [1:0] PU=2'b00, PS=2'b01, PM=2'b11;
    reg  [1:0]  priv;
    wire [1:0]  mmu_mode = (priv == PM) ? 2'd0 : (priv == PS) ? 2'd1 : 2'd2;
    // mstatus bits: SIE1 MIE3 SPIE5 MPIE7 SPP8 SPV9 MPP12:11 MPRV17 SUM18 MXR19
    reg  [31:0] mstatus;
    reg  [31:0] medeleg, mideleg, mie;
    reg  [31:0] mtvec, mepc, mcause, mtval;
    reg  [31:0] mscratch_r;
    reg  [31:0] stvec, sscratch, sepc, scause, stval, satp_r;
    reg  [31:0] stimecmp_r;
    reg         ssip_r;
    // mip: MEIP11/SEIP9 = pins (PLIC), SSIP1 = software-writable, others C6
    wire [31:0] mip_r = (meip ? 32'h800 : 32'h0) | (seip ? 32'h200 : 32'h0)
                      | (ssip_r ? 32'h2 : 32'h0);
    localparam [31:0] MISA_VAL = 32'h40001100;   // tools/misa.py: RV32IM
    localparam [31:0] MSTATUS_WMASK = 32'h000e1baa;
    localparam [31:0] SSTATUS_MASK  = 32'h000c0322;
    localparam [31:0] MEDELEG_MASK  = 32'h0000b7ff;
    localparam [31:0] MIDELEG_MASK  = 32'h00000222;
    localparam [31:0] MIE_MASK      = 32'h00000aaa;
    localparam [31:0] MIP_WMASK     = 32'h00000002;

    // ------------------------------------------------------------ hazard ctl
    // stall: freeze ID/EX/MA/WB + IF/ID (C8: load-use). flush: kill IF/ID and
    // in-flight fetch (C3 branches/traps). redirect: load fpc (C3).
    wire        div_stall;                 // iterative divide (C2)
    // C4: EX holds a mem op until mmu_d PA is in hand; MA holds until the
    // l1_d DONE (load data / store ack). Younger stages freeze with it.
    wire        pa_wait, ma_wait;
    wire        stall = div_stall | pa_wait | ma_wait;
    // C8: d=1 load-use — consumer (at ID) depends on a load sitting in EX.
    // Hold the consumer in ID and bubble idex so it re-latches operands when
    // the load is in MA with mld_done (lf_a path); ma_wait then covers the
    // load-in-MA portion. stall_if is the consumer-side stall only (the
    // producer load keeps advancing, so no deadlock).
    wire        lu_stall;
    wire        stall_if = stall | lu_stall;
    // C3: control transfers resolve in MA (no prediction). redirect loads
    // fpc; flush kills IF/ID + ID/EX + in-flight fetch (discard flags).
    wire        exma_jbr, exma_taken, exma_v2;
    wire [31:0] exma_target;
    // C5: MA trap / xret also redirect (and flush younger + fetch residue)
    wire        ma_trap, ma_xret, ma_mret, int_fire, int_to_s;
    wire [31:0] trap_pc, xret_pc, trap_cause, trap_tval;
    wire        trap_to_s;
    wire        branch_committed = exma_v && exma_jbr && exma_taken;
    wire        redirect = branch_committed || ma_trap || ma_xret || int_fire;
    wire        flush = redirect;
    wire [31:0] trap_pc_i = int_to_s ? {stvec[31:2], 2'b00} : {mtvec[31:2], 2'b00};
    wire [31:0] redirect_pc = ma_trap ? trap_pc
                              : ma_xret ? xret_pc
                              : branch_committed ? exma_target
                              : trap_pc_i;

    // ------------------------------------------------------------ fetch (IF)
    reg  [31:0] fpc;          // next VA to fetch
    reg         mio;          // mmu req in flight (response not consumed)
    reg         pav;          // PA_O held awaiting l1_i
    reg         pav_f;        // C7: held translation FAULTED
    reg         lio;          // l1_i req in flight / DONE not consumed
    reg         d_mio;        // discard next mmu DONE (post-redirect residue)
    reg         d_lio;        // discard next l1 DONE (post-redirect residue)
    reg  [31:0] pc_mmu;       // VA of the mmu request awaiting DONE
    reg  [31:0] pc_pav;       // VA matching the held PA_O
    reg  [31:0] pc_l1;        // VA of the l1_i request in flight

    // fetch response FIFO (4 x {pc,data}): decouples l1_i DONE pulses from
    // IF/ID stalls (divider etc). Issue rule keeps count_next + worst-case
    // in-flight arrivals <= depth: sustained 1/cyc on hits (net count 0-1).
    localparam [31:0] FMARK = 32'h0000_000B;   // synthetic fetch-fault insn
    reg  [31:0] fq_data [0:3];
    reg  [31:0] fq_pc   [0:3];
    reg         fq_pf   [0:3];   // C7: entry is a fetch page-fault marker
    reg  [2:0]  fq_w, fq_r;
    reg  [2:0]  fq_c;

    wire        mmu_done, mmu_fault;
    wire [31:0] mmu_pa;
    wire [3:0]  mmu_cause;
    wire        l1_done;
    wire [31:0] l1_data;

    wire fresh_mmu = mmu_done && !(d_mio || flush);
    wire fresh_l1  = l1_done  && !(d_lio || flush);
    wire [31:0] pc_syn = pav ? pc_pav : pc_mmu;   // VA of the synthesized marker
    // C7: a FAULTED translation becomes a synthetic marker instruction (the
    // page has no fetchable line); fq_pf carries the fault through the FIFO.
    wire t_pa_v   = pav || (mio && fresh_mmu);
    wire t_fault  = pav ? pav_f : mmu_fault;
    wire deq        = (fq_c != 3'd0) && !stall_if;
    wire syn      = !flush && !stall && t_pa_v && t_fault && !fresh_l1 &&
                    ((fq_c - (deq ? 3'd1 : 3'd0)) <= 3'd3);
    wire done_ev  = fresh_l1 | syn;
    wire bypass     = done_ev && (fq_c == 3'd0) && !stall_if;   // straight to IF/ID
    wire enq        = done_ev && !bypass;
    wire consume    = deq || bypass;
    wire [2:0] fq_c_next = fq_c + (enq ? 3'd1 : 3'd0) - (deq ? 3'd1 : 3'd0);
    wire l1_free  = !lio || (l1_done && !stall); // 1-AHEAD needs slot + no stall
    wire space    = (fq_c_next + (lio && !l1_done ? 3'd1 : 3'd0)) <= 3'd3;
    wire req_l1   = !flush && !syn && t_pa_v && l1_free && space;
    wire req_mmu  = !flush && !sf_now && !sf_h && !pav && (!mio || (fresh_mmu && l1_free && space));

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fpc    <= RESET_PC;
            mio    <= 1'b0;
            pav    <= 1'b0;
            pav_f  <= 1'b0;
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
            // C7 pav next: fresh consumed DIRECTLY (bypassing pav) iff no
            // held PA and (cache|syn) took it this cycle; else it is held.
            // pav: held translation. Cleared when taken by cache/syn, unless a
            // fresh response replaces the held one this same cycle.
            pav <= flush ? 1'b0 : ((req_l1 || syn) ? (fresh_mmu && pav)
                                                   : (pav || fresh_mmu));
            if (fresh_mmu && !((req_l1 || syn) && !pav)) pav_f <= mmu_fault;
            lio <= req_l1 ? 1'b1 : (l1_done ? 1'b0 : lio);
            d_mio <= (d_mio || (flush && mio)) && !mmu_done;
            d_lio <= (d_lio || (flush && lio)) && !l1_done;
            if (enq) begin
                fq_data[fq_w[1:0]] <= (syn && !fresh_l1) ? FMARK : l1_data;
                fq_pc  [fq_w[1:0]] <= (syn && !fresh_l1) ? pc_syn : pc_l1;
                fq_pf  [fq_w[1:0]] <= syn && !fresh_l1;
                fq_w <= fq_w + 3'd1;
            end
            if (deq) fq_r <= fq_r + 3'd1;
            if (flush) begin fq_w <= 3'd0; fq_r <= 3'd0; end
            fq_c <= flush ? 3'd0 : fq_c_next;
            if (req_mmu) pc_mmu <= fpc;
            if (req_l1)  pc_l1  <= pav ? pc_pav : pc_mmu;
            if (fresh_mmu && !((req_l1 || syn) && !pav)) pc_pav <= pc_mmu;
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
        .REQ_I(req_mmu), .VA_I(sf_now ? exma_res : fpc),
        .ACC_I(2'b00), .MODE_I(mmu_mode),
        .SUM_I(mstatus[18]), .MXR_I(mstatus[19]), .SATP_I(satp_r),
        .DONE_O(mmu_done), .PA_O(mmu_pa),
        .FAULT_O(mmu_fault), .CAUSE_O(mmu_cause),
        .SF_I(sf_now ? sf_typ : 2'b00), .SFASID_I(exma_d[8:0]),
        .STB_O(mmi_stb), .SEL_O(mmi_sel), .ADR_O(mmi_adr), .WE_O(mmi_we),
        .ACK_I(m2_ack), .DAT_I(m2_dat)
    );

    // C4 nets (defined in the MA section; declared early for instances)
    wire        req_md, req_ld;
    wire [31:0] ex_addr, load_rdata, store_wd, md_pa;
    wire [3:0]  store_sel;

    // D-side MMU: load/store translation (M-mode Bare = identity).
    wire       mmd_done, mmd_fault;
    wire [31:0] mmd_pa;
    wire [3:0]  mmd_cause;
    wire       mmd_stb, mmd_we;
    wire [3:0] mmd_sel;
    wire [31:0] mmd_adr;
    mmu mmu_d (
        .clk(clk), .rst(rst),
        .REQ_I(req_md), .VA_I(sf_now ? exma_res : ex_addr),
        .ACC_I(idex_st ? 2'b10 : 2'b01), .MODE_I(mode_d),
        .SUM_I(mstatus[18]), .MXR_I(mstatus[19]), .SATP_I(satp_r),
        .DONE_O(mmd_done), .PA_O(mmd_pa),
        .FAULT_O(mmd_fault), .CAUSE_O(mmd_cause),
        .SF_I(sf_now ? sf_typ : 2'b00), .SFASID_I(exma_d[8:0]),
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
        .REQ_I(req_ld), .SEL_I(store_sel), .PA_I(exma_pa),
        .WEN_I(exma_st), .WDAT_I(store_wd),
        .DONE_O(mld_done), .DATA_O(mld_data),
        .STB_O(m1_stb), .SEL_O(m1_sel), .ADR_O(m1_adr), .WE_O(m1_we),
        .WDAT_O(m1_wdat), .ACK_I(m1_ack), .DAT_I(m1_dat)
    );

    // ------------------------------------------------------------ IF/ID
    reg         ifid_v;
    reg         ifid_pf;      // C7: fetch page-fault marker
    reg  [31:0] ifid_pc, ifid_instr;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ifid_v <= 1'b0; ifid_pf <= 1'b0; ifid_pc <= 32'h0; ifid_instr <= 32'h0;
        end else if (flush) begin
            ifid_v <= 1'b0;
        end else if (!stall_if) begin
            ifid_v     <= consume;
            ifid_pf    <= deq ? fq_pf[fq_r[1:0]] : (syn && !fresh_l1);
            ifid_pc    <= deq ? fq_pc[fq_r[1:0]] : (syn && !fresh_l1 ? pc_syn : pc_l1);
            ifid_instr <= deq ? fq_data[fq_r[1:0]] : (syn && !fresh_l1 ? FMARK : l1_data);
        end
    end

    // ------------------------------------------------------------ stage regs
    reg         idex_v, idex_wr;
    reg  [4:0]  idex_rd;
    reg  [4:0]  idex_alu;
    reg         idex_jbr, idex_jump, idex_ind;
    reg  [2:0]  idex_bsel3;
    reg         idex_mem, idex_st;
    reg  [2:0]  idex_wd3;
    reg  [31:0] idex_d;
    reg         idex_csr, idex_sys, idex_legal;
    reg         idex_pf;
    reg  [31:0] idex_instr, idex_addr;
    reg  [31:0] idex_pc, idex_a, idex_b, idex_off, idex_c;
    reg         exma_v, exma_wr;
    reg  [4:0]  exma_rd;
    reg  [31:0] exma_res, exma_pc;
    reg         exma_jbr_q, exma_taken_q;
    reg  [31:0] exma_target_q;
    reg         exma_mem_q, exma_st_q;
    reg  [2:0]  exma_wd3_q;
    reg  [31:0] exma_pa_q, exma_d_q;
    reg         exma_csr_q, exma_sys_q, exma_legal_q;
    reg         exma_pf_q, exma_df_q;
    reg  [3:0]  exma_dfc_q;
    reg  [31:0] exma_instr_q, exma_addr_q;
    reg         mawb_v, mawb_wr;
    reg  [4:0]  mawb_rd;
    reg  [31:0] mawb_res, mawb_pc;

    wire        exma_mem, exma_st;
    wire [2:0]  exma_wd3;
    wire [31:0] exma_pa, exma_d;
    assign exma_jbr   = exma_jbr_q;
    assign exma_taken = exma_taken_q;
    assign exma_target = exma_target_q;
    wire        exma_csr, exma_sys, exma_legal, exma_pf, exma_df;
    wire [3:0]  exma_dfc;
    wire [31:0] exma_instr, exma_addr;
    assign exma_pf  = exma_pf_q;
    assign exma_df  = exma_df_q;
    assign exma_dfc = exma_dfc_q;
    assign exma_csr   = exma_csr_q;
    assign exma_sys   = exma_sys_q;
    assign exma_legal = exma_legal_q;
    assign exma_instr = exma_instr_q;
    assign exma_addr  = exma_addr_q;
    assign exma_mem   = exma_mem_q;
    assign exma_st    = exma_st_q;
    assign exma_wd3   = exma_wd3_q;
    assign exma_pa    = exma_pa_q;
    assign exma_d     = exma_d_q;

    // ------------------------------------------------------------ ID decode
    wire [6:0] op   = ifid_instr[6:0];
    wire [4:0] rd   = ifid_instr[11:7];
    wire [2:0] f3   = ifid_instr[14:12];
    wire [4:0] rs1  = ifid_instr[19:15];
    wire [4:0] rs2  = ifid_instr[24:20];
    wire [6:0] f7   = ifid_instr[31:25];

    wire [31:0] imm_i = {{20{ifid_instr[31]}}, ifid_instr[31:20]};
    wire [31:0] imm_u = {ifid_instr[31:12], 12'h0};
    wire [31:0] imm_s = {{20{ifid_instr[31]}}, ifid_instr[31:25], ifid_instr[11:7]};
    wire [31:0] imm_b = {{19{ifid_instr[31]}}, ifid_instr[31], ifid_instr[7],
                         ifid_instr[30:25], ifid_instr[11:8], 1'b0};
    wire [31:0] imm_j = {{11{ifid_instr[31]}}, ifid_instr[31], ifid_instr[19:12],
                         ifid_instr[20], ifid_instr[30:21], 1'b0};

    wire is_op    = (op == 7'h33);
    wire is_opimm = (op == 7'h13);
    wire is_lui   = (op == 7'h37);
    wire is_auipc = (op == 7'h17);
    wire is_load  = (op == 7'h03);
    wire is_sys   = (op == 7'h73);
    wire is_store = (op == 7'h23);
    wire is_br    = (op == 7'h63);
    wire is_jal   = (op == 7'h6F);
    wire is_jalr  = (op == 7'h67);
    // C5 legality table: bad sub-encodings (funct7/funct3/rs2 combos) trap.
    wire alu7_ok  = (f7 == 7'h00) || (f7 == 7'h20) || (f7 == 7'h01);
    wire op7_ok   = (f7 == 7'h00) || ((f7 == 7'h20) && ((f3 == 3'b000) || (f3 == 3'b101)))
                  || (f7 == 7'h01);
    wire opimm_ok = !(((f3 == 3'b001) && (f7 != 7'h00)) ||
                      ((f3 == 3'b101) && (f7 != 7'h00) && (f7 != 7'h20)));
    wire ld_ok    = (f3 != 3'b011) && (f3 != 3'b110) && (f3 != 3'b111);
    wire st_ok    = (f3 == 3'b000) || (f3 == 3'b001) || (f3 == 3'b010);
    wire br_ok    = (f3 != 3'b010) && (f3 != 3'b011);
    wire jr_ok    = (f3 == 3'b000);
    wire sys0     = (f3 == 3'b000);
    wire sys_ecall  = sys0 && (f7 == 7'h00) && (rs2 == 5'd0);
    wire sys_ebreak = sys0 && (f7 == 7'h00) && (rs2 == 5'd1);
    wire sys_wfi    = sys0 && (f7 == 7'h02) && (rs2 == 5'd5);
    wire sys_sfence = sys0 && (f7 == 7'h09);
    wire sys_sret   = sys0 && (f7 == 7'h08) && (rs2 == 5'd2);
    wire sys_mret   = sys0 && (f7 == 7'h18) && (rs2 == 5'd2);
    wire sys_csr    = (f3 == 3'b001) || (f3 == 3'b010) || (f3 == 3'b011)
                    || (f3 == 3'b101) || (f3 == 3'b110) || (f3 == 3'b111);
    wire sys_ok     = sys_csr || sys_ecall || sys_ebreak || sys_wfi
                    || sys_sfence || sys_sret || sys_mret;
    wire legal      = (is_op & op7_ok) | (is_opimm & opimm_ok) | is_lui | is_auipc
                    | (is_br & br_ok) | is_jal | (is_jalr & jr_ok)
                    | (is_load & ld_ok) | (is_store & st_ok) | (is_sys & sys_ok);

    reg        c_wr;
    reg  [1:0] c_asrc;
    reg  [1:0] c_bsel;  // 0=rs2(fwd), 1=imm, 2=const 4
    reg  [4:0] c_alu;
    reg  [4:0] d_rs1, d_rs2;
    reg  [31:0] d_imm;
    reg        c_jbr, c_jump, c_ind;
    reg  [2:0] c_bsel3;
    reg        c_mem, c_st;
    reg  [2:0] c_wd3;
    reg        c_csr, c_sys;

    always @(*) begin
        c_wr = 1'b0;  c_asrc = SRC_RS1; c_bsel = 2'd0; c_alu = ALU_PASS;
        d_rs1  = 5'h0;  d_rs2  = 5'h0;    d_imm = imm_i;
        c_jbr = 1'b0;   c_jump = 1'b0;    c_ind = 1'b0; c_bsel3 = 3'h0;
        c_mem = 1'b0;   c_st = 1'b0;      c_wd3 = 3'h0;
        c_csr = 1'b0;   c_sys = 1'b0;
        if (is_sys) c_sys = sys0;
        if (is_sys && sys_csr) begin
            c_wr = 1'b1; c_csr = 1'b1;
            c_asrc = f3[2] ? SRC_IMM : SRC_RS1;
            c_bsel = 2'd1; c_alu = ALU_PASS;
            d_imm  = {27'h0, rs1};      // zimm (rs1 field); reg form: imm unused
        end else if (is_load) begin
            c_wr = 1'b1; c_mem = 1'b1; c_asrc = SRC_RS1; c_bsel = 2'd1;
            d_imm = imm_i; c_wd3 = f3; d_rs1 = rs1;
        end else if (is_store) begin
            c_mem = 1'b1; c_st = 1'b1; c_asrc = SRC_RS1; c_bsel = 2'd1;
            d_imm = imm_s; c_wd3 = f3; d_rs1 = rs1; d_rs2 = rs2;
        end else if (is_br) begin
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
            idex_mem <= 1'b0; idex_st <= 1'b0; idex_wd3 <= 3'h0; idex_d <= 32'h0;
            idex_csr <= 1'b0; idex_sys <= 1'b0; idex_legal <= 1'b0;
            idex_pf  <= 1'b0;
            idex_instr <= 32'h0; idex_addr <= 32'h0;
            idex_pc <= 32'h0; idex_a <= 32'h0; idex_b <= 32'h0;
            idex_off <= 32'h0; idex_c <= 32'h0;
        end else if (flush) begin
            idex_v <= 1'b0; idex_wr <= 1'b0;
        end else if (!stall) begin
            // ALL fields forced inactive on a bubble (ifid_v=0): decode of
            // absent/garbage words must never leak X into control or data.
            // C8 lu_stall: bubble (consumer held in ID, load moved to MA).
            idex_v    <= lu_stall ? 1'b0 : ifid_v;
            idex_legal <= ifid_v ? legal : 1'b1;
            idex_csr  <= ifid_v && c_csr;
            idex_pf   <= ifid_v && ifid_pf;
            idex_sys  <= ifid_v && c_sys;
            idex_instr <= ifid_v ? ifid_instr : 32'h0000_0013;  // nop
            idex_addr <= ifid_v ? (fwd_a + d_imm) : 32'h0;
            idex_wr   <= ifid_v && c_wr;
            idex_rd   <= ifid_v ? rd : 5'h0;
            idex_alu  <= c_alu;
            idex_jbr  <= ifid_v && c_jbr;
            idex_jump <= ifid_v && c_jump;
            idex_ind  <= ifid_v && c_ind;
            idex_bsel3<= ifid_v ? c_bsel3 : 3'h0;
            idex_mem  <= ifid_v && c_mem;
            idex_st   <= ifid_v && c_st;
            idex_wd3  <= ifid_v ? c_wd3 : 3'h0;
            idex_d    <= ifid_v ? fwd_b : 32'h0;
            idex_pc   <= ifid_pc;
            idex_off  <= ifid_v ? d_imm : 32'h0;
            idex_c    <= ifid_v ? fwd_a : 32'h0;
            idex_a    <= !ifid_v ? 32'h0 :
                         (c_asrc == SRC_RS1) ? fwd_a :
                         (c_asrc == SRC_PC)  ? ifid_pc : d_imm;
            idex_b    <= !ifid_v ? 32'h0 :
                         (c_bsel == 2'd0) ? fwd_b :
                         (c_bsel == 2'd1) ? d_imm : 32'h4;
        end
    end


    // ------------------------------- ID-side forwarding (operands at latch)
    // Consumer latches operands on its ID->EX edge; producer is then either
    // in idex (d=1: combinational alu_res -- safe for mul/div because the
    // latch only ever happens on a !stall edge, i.e. the divider's D_DONE
    // cycle for div), exma (d=2), mawb (d=3), or retired to rf (d>=4).
    // Youngest matching writer wins; x0 never forwards.
    // load-data forward: exma load on its l1_d DONE cycle (ma_wait keeps
    // younger stages parked until exactly this cycle).
    // C8: d=1 load-use — consumer (ID: rs1/rs2) reads the rd of a load in
    // EX (idex). Hold consumer in ID (stall_if) + bubble idex; the load
    // advances to MA and ma_wait+lf_a then deliver load_rdata on re-latch.
    // CSR-use (t02 debug): a CSR-op in EX (idex) with rd!=0 has the SAME
    // d=1/d=2 hole — its result rides csr_rdata in MA -> mawb_res, never
    // alu_res/exma_res. Hold the consumer one cycle and forward csr_rdata
    // from exma (d=2), exactly like loads.
    wire lu_ld_ex = ((idex_v && idex_mem && !idex_st && (idex_rd != 5'h0)) ||
                     (idex_v && idex_csr && idex_wr && (idex_rd != 5'h0))) &&
                   ((rs1 == idex_rd && rs1 != 5'h0) ||
                    (rs2 == idex_rd && rs2 != 5'h0));
    assign lu_stall = lu_ld_ex;
    wire lf_a = (exma_v && exma_mem && !exma_st && mld_done &&
                 (exma_rd != 5'h0) && (exma_rd == rs1));
    wire lf_b = (exma_v && exma_mem && !exma_st && mld_done &&
                 (exma_rd != 5'h0) && (exma_rd == rs2));
    wire cs_a = (exma_v && exma_csr && exma_wr && (exma_rd != 5'h0) &&
                 (exma_rd == rs1));
    wire cs_b = (exma_v && exma_csr && exma_wr && (exma_rd != 5'h0) &&
                 (exma_rd == rs2));
    wire [31:0] fwd_a =
        lf_a ? load_rdata :
        cs_a ? csr_rdata :
        (idex_v && idex_wr && (idex_rd == rs1) && (rs1 != 5'h0)) ? alu_res :
        (exma_wr && (exma_rd != 5'h0) && (exma_rd == rs1))       ? exma_res :
        (mawb_wr && (mawb_rd != 5'h0) && (mawb_rd == rs1))       ? mawb_res : rf_r1;
    wire [31:0] fwd_b =
        lf_b ? load_rdata :
        cs_b ? csr_rdata :
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

    // ---------------------------------------------- C4: D-side translation
    // mmu_d REQ issues while the mem op is in EX (PA ready by MA entry;
    // pa_wait extends EX if not). md_pv holds a PA whose cache slot waits.
    wire [31:0] ex_addr_c = idex_a + idex_b;   // rs1 + offset (both forwarded)
    assign ex_addr = ex_addr_c;

    reg  md_mio, md_pv, d_md;
    reg  md_pv_f;
    reg  [31:0] md_pa_r;
    reg  [3:0]  md_pv_c;
    assign md_pa = md_pa_r;
    wire md_fresh_fault = fresh_md ? mmd_fault  : md_pv_f;
    wire md_fresh_cause = fresh_md ? mmd_cause : md_pv_c;

    // NB: NO !flush term here. flush->redirect->int_fire->(!stall)->pa_wait
    // ->pa_now_c->fresh_md->(!flush) is a combinational loop (odd inverter
    // count: oscillates -> vvp zero-delay delta storm, the t03 hang; verilator
    // UNOPTFLAT on soc.core0.stall). A flush's dead op is discarded by the
    // flush-clear of md_pv/d_md/exma_mem_q, not by fresh_md.
    wire fresh_md = mmd_done && !d_md;
    wire pa_now_c = md_pv || fresh_md;
    assign pa_wait = idex_v && idex_mem && !pa_now_c;
    assign req_md  = idex_v && idex_mem && !md_mio && !md_pv && !d_md
                      && !sf_now && !sf_h;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            md_mio <= 1'b0; md_pv <= 1'b0; d_md <= 1'b0; md_pa_r <= 32'h0;
            md_pv_f <= 1'b0; md_pv_c <= 4'h0;
        end else begin
            md_mio <= flush ? 1'b0 : (req_md ? 1'b1 : (mmd_done ? 1'b0 : md_mio));
            md_pv_f <= flush ? 1'b0 : (fresh_md ? stall
                                   : (md_pv_f && !stall ? 1'b1 : 1'b0));
            if (fresh_md) md_pv_c <= mmd_cause;
            md_pv  <= flush   ? 1'b0 :
                      fresh_md ? stall     // held only if op can't advance now
                      : (md_pv && !stall) ? 1'b0   // consumed by advance
                      : md_pv;
            d_md   <= (d_md || (flush && md_mio)) && !mmd_done;
            if (fresh_md) md_pa_r <= mmd_pa;
        end
    end

    // ------------------------------------------------------ C4: MA memory
    reg  ld_mio;
    wire ma_trap_w;                                  // declared in trap engine
    assign req_ld = exma_v && exma_mem && !ld_mio && !ma_trap_w;
    // faulted access (df) or sync trap: never touch the cache
    wire mem_wait_c = exma_v && exma_mem && !ma_trap_w && !mld_done;
    assign ma_wait = mem_wait_c;

    always @(posedge clk or posedge rst)
        if (rst) ld_mio <= 1'b0;
        else     ld_mio <= req_ld ? 1'b1 : (mld_done ? 1'b0 : ld_mio);

    // load extract (unpriv 2.6: width by funct3, lane by addr[1:0])
    wire [7:0]  ld_b = exma_pa[1] ? (exma_pa[0] ? mld_data[31:24] : mld_data[23:16])
                                  : (exma_pa[0] ? mld_data[15:8]  : mld_data[7:0]);
    wire [15:0] ld_h = exma_pa[1] ? mld_data[31:16] : mld_data[15:0];
    wire [31:0] load_rdata_c =
        (exma_wd3 == 3'b000) ? {{24{ld_b[7]}},  ld_b} :   // LB
        (exma_wd3 == 3'b001) ? {{16{ld_h[15]}}, ld_h} :   // LH
        (exma_wd3 == 3'b100) ? {24'h0, ld_b} :            // LBU
        (exma_wd3 == 3'b101) ? {16'h0, ld_h} :            // LHU
                               mld_data;                  // LW
    assign load_rdata = load_rdata_c;

    // store lane merge (l1_d passes SEL through to the WB write)
    wire [31:0] store_wd_c =
        (exma_wd3 == 3'b000) ? {4{exma_d[7:0]}} :
        (exma_wd3 == 3'b001) ? {2{exma_d[15:0]}} : exma_d;
    wire [3:0] store_sel_c =
        (exma_wd3 == 3'b000) ? (4'b0001 << exma_pa[1:0]) :
        (exma_wd3 == 3'b001) ? (exma_pa[1] ? 4'b1100 : 4'b0011) : 4'b1111;
    assign store_wd  = store_wd_c;
    assign store_sel = store_sel_c;

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
            exma_mem_q <= 1'b0; exma_st_q <= 1'b0; exma_wd3_q <= 3'h0;
            exma_pa_q <= 32'h0; exma_d_q <= 32'h0;
            exma_csr_q <= 1'b0; exma_sys_q <= 1'b0; exma_legal_q <= 1'b0;
            exma_pf_q <= 1'b0; exma_df_q <= 1'b0; exma_dfc_q <= 4'h0;
            exma_instr_q <= 32'h0; exma_addr_q <= 32'h0;
        end else if (flush) begin
            exma_v <= 1'b0; exma_wr <= 1'b0; exma_jbr_q <= 1'b0;
            exma_mem_q <= 1'b0;
        end else if (!stall) begin
            exma_v    <= idex_v;
            exma_wr   <= idex_wr;
            exma_rd   <= idex_rd;
            exma_res  <= alu_res;
            exma_pc   <= idex_pc;
            exma_jbr_q   <= idex_jbr;
            exma_taken_q <= ex_taken;
            exma_target_q <= ex_target;
            exma_mem_q   <= idex_mem;
            exma_st_q    <= idex_st;
            exma_wd3_q   <= idex_wd3;
            exma_pa_q    <= mmd_done ? mmd_pa : md_pa;
            exma_d_q     <= idex_d;
            exma_csr_q   <= idex_csr;
            exma_sys_q   <= idex_sys;
            exma_legal_q <= idex_legal;
            exma_instr_q <= idex_instr;
            exma_addr_q  <= idex_addr;
            exma_pf_q    <= idex_pf;
            exma_df_q    <= idex_v ? (fresh_md ? mmd_fault  : md_pv_f) : 1'b0;
            exma_dfc_q   <= idex_v ? (fresh_md ? mmd_cause : md_pv_c) : 4'h0;
        end
    end

    // ------------------------------------------------------------ MA/WB
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mawb_v <= 1'b0; mawb_wr <= 1'b0; mawb_rd <= 5'h0; mawb_res <= 32'h0; mawb_pc <= 32'h0;
        end else if (!stall) begin
            mawb_v    <= exma_v;
            mawb_wr   <= exma_wr && !ma_trap_w;   // sync trap kills writeback
            mawb_rd   <= exma_rd;
            mawb_res  <= exma_csr ? csr_rdata
                       : (exma_mem && !exma_st) ? load_rdata : exma_res;
            mawb_pc   <= exma_pc;
        end
    end

    // ============================================ C5: MA trap + CSR engine
    wire [2:0]  x_f3  = exma_instr[14:12];
    wire [6:0]  x_f7  = exma_instr[31:25];
    wire [4:0]  x_rs2 = exma_instr[24:20];
    wire [11:0] x_csr = exma_instr[31:20];
    wire [31:0] x_zimm = {27'h0, exma_instr[19:15]};
    wire [31:0] x_src  = x_f3[2] ? x_zimm : exma_res;  // rs1 val rode ALU PASS      // wdata: zimm | rs1
    wire        is_csr_op = exma_v && exma_csr;
    wire        csr_wr_attempt = is_csr_op &&
                    ((x_f3[1:0] == 2'b01) || (x_src != 32'h0)); // RW always; RS/RC if src!=0

    // CSR register file (read mux)
    reg [31:0] csr_rdata;
    reg        csr_known, csr_ro;
    always @(*) begin
        csr_rdata = 32'h0; csr_known = 1'b0; csr_ro = 1'b0;
        case (x_csr)
            12'h300: begin csr_rdata = mstatus & MSTATUS_WMASK; csr_known = 1; end
            12'h301: begin csr_rdata = MISA_VAL; csr_known = 1; csr_ro = 1; end
            12'h302: begin csr_rdata = medeleg; csr_known = 1; end
            12'h303: begin csr_rdata = mideleg; csr_known = 1; end
            12'h304: begin csr_rdata = mie & MIE_MASK; csr_known = 1; end
            12'h305: begin csr_rdata = mtvec; csr_known = 1; end
            12'h340: begin csr_rdata = mscratch_r; csr_known = 1; end
            12'h341: begin csr_rdata = mepc; csr_known = 1; end
            12'h342: begin csr_rdata = mcause; csr_known = 1; end
            12'h343: begin csr_rdata = mtval; csr_known = 1; end
            12'h344: begin csr_rdata = mip_r; csr_known = 1; end
            12'h100: begin csr_rdata = mstatus & SSTATUS_MASK; csr_known = 1; end
            12'h104: begin csr_rdata = mie & mideleg; csr_known = 1; end
            12'h105: begin csr_rdata = stvec; csr_known = 1; end
            12'h140: begin csr_rdata = sscratch; csr_known = 1; end
            12'h141: begin csr_rdata = sepc; csr_known = 1; end
            12'h142: begin csr_rdata = scause; csr_known = 1; end
            12'h143: begin csr_rdata = stval; csr_known = 1; end
            12'h144: begin csr_rdata = mip_r & mideleg; csr_known = 1; end
            12'h14D: begin csr_rdata = stimecmp_r; csr_known = 1; end
            12'h15D: begin csr_rdata = 32'h0; csr_known = 1; end   // stimecmph (32-bit only)
            12'h180: begin csr_rdata = satp_r; csr_known = 1; end
            12'hF11, 12'hF12, 12'hF13, 12'hF14, 12'hF15:
                     begin csr_rdata = 32'h0; csr_known = 1; csr_ro = 1; end
            default: ;
        endcase
        if (x_csr[11:10] == 2'b11) csr_ro = 1'b1;    // hardware RO range
    end

    wire [1:0] csr_need = x_csr[9:8];                // 0=U 1=S (2,3)=M
    wire priv_ok = (csr_need == 2'd0) ? 1'b1 :
                   (csr_need == 2'd1) ? (priv != PU) : (priv == PM);
    wire csr_illegal = is_csr_op && (!csr_known || !priv_ok ||
                                     (csr_wr_attempt && csr_ro));

    // SYSTEM ops at MA
    wire sys_ecall_  = exma_v && exma_sys && (x_f7 == 7'h00) && (x_rs2 == 5'd0);
    wire sys_ebreak_ = exma_v && exma_sys && (x_f7 == 7'h00) && (x_rs2 == 5'd1);
    wire sys_mret_   = exma_v && exma_sys && (x_f7 == 7'h18) && (x_rs2 == 5'd2);
    wire sys_sret_   = exma_v && exma_sys && (x_f7 == 7'h08) && (x_rs2 == 5'd2);
    wire xret_priv_bad = (sys_mret_ && (priv != PM)) || (sys_sret_ && (priv == PU));
    assign ma_xret = (sys_mret_ && (priv == PM)) || (sys_sret_ && (priv != PU));
    assign ma_mret = sys_mret_ && (priv == PM);

    // memory misalignment (spec Vol I 2.6: raises address-misaligned fault)
    wire mem_ld_op = exma_v && exma_mem && !exma_st;
    wire mem_st_op = exma_v && exma_mem &&  exma_st;
    wire mis_ld = mem_ld_op && (((x_f3 == 3'b010) && (exma_addr[1:0] != 2'b00)) ||
                                ((x_f3 == 3'b001) &&  exma_addr[0]));
    wire mis_st = mem_st_op && (((x_f3 == 3'b010) && (exma_addr[1:0] != 2'b00)) ||
                                ((x_f3 == 3'b001) &&  exma_addr[0]));

    wire trap_pf   = exma_v && exma_pf;               // C7 fetch page fault
    wire trap_df   = exma_v && exma_df && !mis_ld && !mis_st;  // C7 data pf
    wire trap_illegal = exma_v && (!exma_legal || csr_illegal || xret_priv_bad);
    wire trap_ecall   = sys_ecall_;
    wire trap_ebreak  = sys_ebreak_;
    wire trap_misld   = mis_ld;
    wire trap_misst   = mis_st;
    assign ma_trap_w = trap_pf | trap_illegal | trap_misld | trap_misst
                     | trap_ebreak | trap_ecall | trap_df;
    assign ma_trap   = ma_trap_w;

    // cause/tval priority (all mutually exclusive by instruction class)
    reg [3:0] t_cause;
    reg [31:0] t_tval;
    always @(*) begin
        t_cause = 4'h2; t_tval = 32'h0;
        if      (trap_pf)    begin t_cause = 4'hc; t_tval = exma_pc;   end
        else if (trap_misld) begin t_cause = 4'h4; t_tval = exma_addr; end
        else if (trap_misst) begin t_cause = 4'h6; t_tval = exma_addr; end
        else if (trap_df)    begin t_cause = exma_dfc; t_tval = exma_addr; end
        else if (trap_ebreak) begin t_cause = 4'h3; t_tval = exma_pc;  end
        else if (trap_ecall) begin
            t_cause = (priv == PM) ? 4'hB : (priv == PS) ? 4'h9 : 4'h8;
            t_tval  = 32'h0;
        end
        else if (trap_illegal) begin t_cause = 4'h2; t_tval = 32'h0; end
    end
    assign trap_cause = {28'h0, t_cause};
    assign trap_tval  = t_tval;
    assign trap_to_s  = (priv != PM) && ((medeleg >> t_cause) & 32'h1);
    assign trap_pc    = trap_to_s ? {stvec[31:2], 2'b00} : {mtvec[31:2], 2'b00};
    assign xret_pc    = ma_mret ? {mepc[31:1], 1'b0} : {sepc[31:1], 1'b0};

    // ---------------------------------------- C6: interrupt evaluation
    // Sources wired: MEIP(11)/SEIP(9) pins (PLIC), SSIP(1) CSR bit.
    // Taken at MA commit boundary after a clean sequential retire; the
    // interrupted-stream instruction retires normally, epc = pc+4.
    wire [31:0] mip_p = mip_r;
    wire        m_gate = (priv != PM) || mstatus[3];     // MIE
    wire        s_gate = (priv == PU) ||
                         ((priv == PS) && mstatus[1]);   // SIE (S-mode only)
    wire [31:0] m_pend = mie & mip_p & ~mideleg;
    wire [31:0] s_pend = mie & mip_p &  mideleg;
    wire int_m = (m_pend != 32'h0) && m_gate && (priv != PM || mstatus[3]);
    wire int_s = (s_pend != 32'h0) && s_gate;
    // fire only on a clean sequential retire (no trap/xret/branch that cycle)
    assign int_fire  = exma_v && !stall && !ma_trap && !ma_xret &&
                       !branch_committed && (int_m || int_s);
    assign int_to_s  = !int_m && int_s;

    // ---------------------------------------- C7: sfence.vma + MPRV
    wire x_sfence   = exma_v && exma_sys && (x_f7 == 7'h09);
    wire sf_now = x_sfence && !stall;          // commit-cycle pulse
    wire [1:0] sf_typ = (exma_res == 32'h0) ? ((exma_d == 32'h0) ? 2'b01 : 2'b10)
                                            : 2'b11;
    reg  sf_h;                                   // 1-cyc mmu REQ hold
    always @(posedge clk or posedge rst)
        if (rst) sf_h <= 1'b0; else sf_h <= sf_now;

    // effective D-side mode: MPRV ? MPP : priv
    wire [1:0] mpp_mode = (mstatus[12:11] == 2'b11) ? 2'd0 :
                          (mstatus[12:11] == 2'b01) ? 2'd1 : 2'd2;
    wire [1:0] mode_d   = mstatus[17] ? mpp_mode : mmu_mode;

    // ---------------------------------------------- C5: architectural state
    // RS/RC merge: write value = src | old (RS), old & ~src (RC), src (RW)
    wire [31:0] csr_wv = (x_f3[1:0] == 2'b01) ? x_src :
                         (x_f3[1:0] == 2'b10) ? (csr_rdata | x_src)
                                              : (csr_rdata & ~x_src);
    wire [31:0] mstatus_new = csr_wv & MSTATUS_WMASK;
    wire [31:0] mstatus_wv  = (mstatus_new[12:11] == 2'b10)
                            ? ((mstatus_new & ~32'h1800) | 32'h0000)  // 10 -> U
                            : mstatus_new;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            priv <= PM; mstatus <= 32'h0; medeleg <= 32'h0; mideleg <= 32'h0;
            mie <= 32'h0; mtvec <= 32'h0; mscratch_r <= 32'h0; mepc <= 32'h0;
            mcause <= 32'h0; mtval <= 32'h0; stvec <= 32'h0; sscratch <= 32'h0;
            sepc <= 32'h0; scause <= 32'h0; stval <= 32'h0; satp_r <= 32'h0;
            stimecmp_r <= 32'hFFFF_FFFF; ssip_r <= 1'b0;
        end else begin
            // --- CSR write commit (MA, non-trapping) ---
            if (csr_wr_attempt && !csr_illegal) begin
                case (x_csr)
                    12'h300: mstatus <= (mstatus & ~MSTATUS_WMASK) | mstatus_wv;
                    12'h302: medeleg <= csr_wv & MEDELEG_MASK;
                    12'h303: mideleg <= csr_wv & MIDELEG_MASK;
                    12'h304: mie     <= csr_wv & MIE_MASK;
                    12'h305: mtvec   <= {csr_wv[31:1], 1'b0};   // MODE=0 direct only
                    12'h340: mscratch_r <= csr_wv;
                    12'h341: mepc    <= {csr_wv[31:1], 1'b0};
                    12'h342: mcause  <= csr_wv;
                    12'h343: mtval   <= csr_wv;
                    12'h344: ssip_r  <= csr_wv[1];              // mip: SSIP writable
                    12'h100: mstatus <= (mstatus & ~SSTATUS_MASK)
                                     | (csr_wv & SSTATUS_MASK);
                    12'h104: mie     <= (mie & ~mideleg) | (csr_wv & mideleg);
                    12'h105: stvec   <= {csr_wv[31:1], 1'b0};
                    12'h140: sscratch <= csr_wv;
                    12'h141: sepc    <= {csr_wv[31:1], 1'b0};
                    12'h142: scause  <= csr_wv;
                    12'h143: stval   <= csr_wv;
                    12'h14D: stimecmp_r <= csr_wv;
                    12'h180: satp_r  <= csr_wv;
                    default: ;
                endcase
            end
            // --- trap entry (synchronous) or interrupt delivery (C6) ---
            if (ma_trap || int_fire) begin
                if (trap_to_s || int_to_s) begin
                    sepc   <= int_fire ? (exma_pc + 32'h4)
                                       : {exma_pc[31:1], 1'b0};
                    scause <= int_fire ? 32'h8000_0009 : trap_cause;
                    stval  <= int_fire ? 32'h0 : trap_tval;
                    mstatus[8] <= priv[0];            // SPP
                    mstatus[5] <= mstatus[1];         // SPIE <- SIE
                    mstatus[1] <= 1'b0;               // SIE  <- 0
                    priv     <= PS;
                end else begin
                    mepc   <= int_fire ? (exma_pc + 32'h4)
                                       : {exma_pc[31:1], 1'b0};
                    mcause <= int_fire ? 32'h8000_000B : trap_cause;
                    mtval  <= int_fire ? 32'h0 : trap_tval;
                    mstatus[12:11] <= priv;           // MPP (arch encoding)
                    mstatus[7] <= mstatus[3];         // MPIE <- MIE
                    mstatus[3] <= 1'b0;               // MIE  <- 0
                    priv     <= PM;
                end
            end
            // --- xret ---
            if (ma_xret) begin
                if (ma_mret) begin
                    priv           <= mstatus[12:11];
                    mstatus[3]     <= mstatus[7];     // MIE  <- MPIE
                    mstatus[7]     <= 1'b1;           // MPIE <- 1
                    mstatus[12:11] <= PU;             // MPP  <- U (lowest)
                    mstatus[17]    <= 1'b0;           // MPRV cleared on mret
                end else begin
                    priv          <= {1'b0, mstatus[8]};  // SPP: 0=U 1=S
                    mstatus[1]    <= mstatus[5];      // SIE  <- SPIE
                    mstatus[5]    <= 1'b1;            // SPIE <- 1
                    mstatus[8]    <= 1'b0;            // SPP  <- U
                end
            end
        end
    end

    // ------------------------------------------------------------ commit tap
    wire commit = mawb_v && !stall;   // retirement pulse (TB trace/terminate)

endmodule
`default_nettype wire
