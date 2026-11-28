// wb_ic -- pipelined Wishbone B4 interconnect, 3 masters x 5 slaves (+null)
// (mythrv32)
//
// Masters (m0=I-cache, m1=D-cache, m2=MMU walk):
//   - STB_k: 1-cycle pulse = one transfer; {WE_k,SEL_k,ADR_k,WDAT_k} valid
//     on that cycle. Must NOT assert while STALL_k. May assert in the same
//     cycle it latches ACK_k.
//   - STALL_k: 1 when 4 transfers queued (FIFO full).
//   - ACK_k: 1-cycle pulse per accepted transfer, strictly in issue order.
//   - RDAT_k: read data, valid on the ACK_k cycle.
//   - Max 1 outstanding transfer per master (per-master in-order by
//     construction).
//
// Slaves (s0=ROM s1=DRAM s2=UART s3=CLINT s4=PLIC):
//   - STB_s: 1-cycle pulse = one transfer, never to a busy slave, max 1
//     outstanding per slave. WE_s/SEL_s/ADR_s/WDAT_s valid on that cycle
//     (presented combinationally at grant).
//   - Slave ACK_s one or more cycles later; RDAT_s valid on it (reads).
//   - No ERR/RTY/STALL/CYC on this interface (simple 1-cycle slaves;
//     multi-cycle / held-STB slaves live behind an adapter).
//
// Address decode (mutually exclusive; verified with tools/addr.py):
//   s0 ROM  : ADR[31:15] == 17'h00000  (0x0000_0000, 32 KiB)
//   s1 DRAM : ADR[31:22] == 10'h200    (0x8000_0000, 4 MiB)
//   s2 UART : ADR[31:12] == 20'h10000  (0x1000_0000, 4 KiB)
//   s3 CLINT: ADR[31:16] == 16'h0200   (0x0200_0000, 64 KiB)
//   s4 PLIC : ADR[31:22] == 10'h030    (0x0C00_0000, 4 MiB)
//   s5 null : otherwise (read -> 0, write dropped, 1-cycle latency)
//
// Timing (per notes/design.md sec.6):
//   - fixed-priority grant, one per cycle, m0 > m1 > m2; up to 3
//     outstanding masters (6 slave destinations incl. null).
//   - Grant at cycle c presents STB+data to the slave combinationally;
//     1-cycle slave => master ACK at c+1 (1 cycle total).
//   - null slave: grant at c => master ACK at c+1 (same as 1-cycle slaves).
//   - FIFO: push on master STB; entry popped one cycle after grant (when
//     the slave latches); bypass: empty FIFO + STB grants the pushed word
//     directly (push lands next cycle, popped the cycle after).
`default_nettype none

module wb_ic (
    input  clk, rst,
    // ---- masters (m0=I-cache, m1=D-cache, m2=MMU walk) ----
    input         m0_STB, m0_WE,
    input  [3:0]  m0_SEL,
    input  [31:0] m0_ADR, m0_WDAT,
    output        m0_STALL, m0_ACK,
    output [31:0] m0_RDAT,
    input         m1_STB, m1_WE,
    input  [3:0]  m1_SEL,
    input  [31:0] m1_ADR, m1_WDAT,
    output        m1_STALL, m1_ACK,
    output [31:0] m1_RDAT,
    input         m2_STB, m2_WE,
    input  [3:0]  m2_SEL,
    input  [31:0] m2_ADR, m2_WDAT,
    output        m2_STALL, m2_ACK,
    output [31:0] m2_RDAT,
    // ---- slaves (s0=ROM s1=DRAM s2=UART s3=CLINT s4=PLIC) ----
    output        s0_STB, s0_WE,
    output [3:0]  s0_SEL,
    output [31:0] s0_ADR, s0_WDAT,
    input         s0_ACK,
    input  [31:0] s0_RDAT,
    output        s1_STB, s1_WE,
    output [3:0]  s1_SEL,
    output [31:0] s1_ADR, s1_WDAT,
    input         s1_ACK,
    input  [31:0] s1_RDAT,
    output        s2_STB, s2_WE,
    output [3:0]  s2_SEL,
    output [31:0] s2_ADR, s2_WDAT,
    input         s2_ACK,
    input  [31:0] s2_RDAT,
    output        s3_STB, s3_WE,
    output [3:0]  s3_SEL,
    output [31:0] s3_ADR, s3_WDAT,
    input         s3_ACK,
    input  [31:0] s3_RDAT,
    output        s4_STB, s4_WE,
    output [3:0]  s4_SEL,
    output [31:0] s4_ADR, s4_WDAT,
    input         s4_ACK,
    input  [31:0] s4_RDAT
);

    // ------------------------------------------------------------- decode
    function [2:0] dec;
        input [31:0] a;
        begin
            if      (a[31:15] == 17'h00000) dec = 3'd0;  // ROM
            else if (a[31:22] == 10'h200)   dec = 3'd1;  // DRAM
            else if (a[31:12] == 20'h10000) dec = 3'd2;  // UART
            else if (a[31:16] == 16'h0200)  dec = 3'd3;  // CLINT
            else if (a[31:22] == 10'h030)   dec = 3'd4;  // PLIC
            else                              dec = 3'd5; // null
        end
    endfunction

    // ------------------------------------------------------------- queues
    // entry layout: {we, sel[3:0], adr[31:0], wdat[31:0]}  (69b)
    wire [68:0] din0 = {m0_WE, m0_SEL, m0_ADR, m0_WDAT};
    wire [68:0] din1 = {m1_WE, m1_SEL, m1_ADR, m1_WDAT};
    wire [68:0] din2 = {m2_WE, m2_SEL, m2_ADR, m2_WDAT};

    wire [68:0] h0, h1, h2;
    wire [2:0]  fc0, fc1, fc2;

    // pop: grant with nonempty FIFO pops at grant+1 (input high at grant);
    // bypass grant (empty FIFO) pushes next cycle, popped the cycle after.
    wire pop0, pop1, pop2;

    wb_ic_mfifo u_f0 (.clk(clk), .rst(rst), .push(m0_STB), .pop(pop0),
                     .din(din0), .head(h0), .cnt(fc0), .full(m0_STALL));
    wb_ic_mfifo u_f1 (.clk(clk), .rst(rst), .push(m1_STB), .pop(pop1),
                     .din(din1), .head(h1), .cnt(fc1), .full(m1_STALL));
    wb_ic_mfifo u_f2 (.clk(clk), .rst(rst), .push(m2_STB), .pop(pop2),
                     .din(din2), .head(h2), .cnt(fc2), .full(m2_STALL));

    // ------------------------------------------------------------- state
    reg         busy0, busy1, busy2;   // master has 1 outstanding
    reg [4:0]   s_busy;                 // per real slave 0..4
    reg         nbusy;                  // null slave outstanding
    reg [1:0]   sm0, sm1, sm2, sm3, sm4, sm5;  // slave 0..5 (5=null) -> master idx
                                             // (scalar: every access is constant-
                                             // indexed; an array here would make
                                             // this async-reset process a memory
                                             // writer -> yosys register-list warn)
    reg         byp1_0, byp1_1, byp1_2; // bypass grant, 1-cycle delayed

    wire [5:0] busyv = {nbusy, s_busy};

    // head: FIFO head, or the pushed word when empty (bypass grant)
    wire [68:0] he0 = (fc0 == 3'd0 && m0_STB) ? din0 : h0;
    wire [68:0] he1 = (fc1 == 3'd0 && m1_STB) ? din1 : h1;
    wire [68:0] he2 = (fc2 == 3'd0 && m2_STB) ? din2 : h2;

    wire [2:0] d0 = dec(he0[63:32]);
    wire [2:0] d1 = dec(he1[63:32]);
    wire [2:0] d2 = dec(he2[63:32]);

    // fixed-priority grant, one per cycle
    wire g0r = !busy0 && (m0_STB || fc0 != 3'd0) && ~busyv[d0];
    wire g1r = !busy1 && (m1_STB || fc1 != 3'd0) && ~busyv[d1];
    wire g2r = !busy2 && (m2_STB || fc2 != 3'd0) && ~busyv[d2];
    wire g0  =  g0r;
    wire g1  = !g0 && g1r;
    wire g2  = !g0 && !g1 && g2r;

    wire        grant = g0 | g1 | g2;
    wire [2:0]  gslv  = g0 ? d0 : (g1 ? d1 : d2);
    wire [1:0]  gmst  = g0 ? 2'd0 : (g1 ? 2'd1 : 2'd2);
    wire [68:0] gdin  = g0 ? he0 : (g1 ? he1 : he2);
    wire        bp0   = g0  && fc0 == 3'd0;
    wire        bp1   = g1  && fc1 == 3'd0;
    wire        bp2   = g2  && fc2 == 3'd0;

    // pop inputs (see queues comment)
    assign pop0 = (g0  && fc0 != 3'd0) || byp1_0;
    assign pop1 = (g1  && fc1 != 3'd0) || byp1_1;
    assign pop2 = (g2  && fc2 != 3'd0) || byp1_2;

    // ------------------------------------------------------------- slaves
    // STB/data presented combinationally at grant
    assign s0_STB = grant && (gslv == 3'd0);
    assign s1_STB = grant && (gslv == 3'd1);
    assign s2_STB = grant && (gslv == 3'd2);
    assign s3_STB = grant && (gslv == 3'd3);
    assign s4_STB = grant && (gslv == 3'd4);
    assign s0_WE  = s0_STB && gdin[68];
    assign s1_WE  = s1_STB && gdin[68];
    assign s2_WE  = s2_STB && gdin[68];
    assign s3_WE  = s3_STB && gdin[68];
    assign s4_WE  = s4_STB && gdin[68];
    assign s0_SEL = gdin[67:64];
    assign s1_SEL = gdin[67:64];
    assign s2_SEL = gdin[67:64];
    assign s3_SEL = gdin[67:64];
    assign s4_SEL = gdin[67:64];
    assign s0_ADR  = gdin[63:32];
    assign s1_ADR  = gdin[63:32];
    assign s2_ADR  = gdin[63:32];
    assign s3_ADR  = gdin[63:32];
    assign s4_ADR  = gdin[63:32];
    assign s0_WDAT = gdin[31:0];
    assign s1_WDAT = gdin[31:0];
    assign s2_WDAT = gdin[31:0];
    assign s3_WDAT = gdin[31:0];
    assign s4_WDAT = gdin[31:0];

    // ------------------------------------------------------------- return
    wire [4:0]   s_ack  = {s4_ACK, s3_ACK, s2_ACK, s1_ACK, s0_ACK};
    wire [31:0]  rs0 = s0_RDAT, rs1 = s1_RDAT, rs2 = s2_RDAT,
                   rs3 = s3_RDAT, rs4 = s4_RDAT;
    wire [1:0]  mr0 = sm0, mr1 = sm1, mr2 = sm2,
                 mr3 = sm3, mr4 = sm4, mr5 = sm5;
    // owning-master return this cycle (<=1 per master: 1 outstanding each)
    wire r0 = s_ack[0] && s_busy[0];
    wire r1 = s_ack[1] && s_busy[1];
    wire r2 = s_ack[2] && s_busy[2];
    wire r3 = s_ack[3] && s_busy[3];
    wire r4 = s_ack[4] && s_busy[4];
    wire rn = nbusy;

    assign m0_ACK = (r0 && mr0 == 2'd0) || (r1 && mr1 == 2'd0) ||
                    (r2 && mr2 == 2'd0) || (r3 && mr3 == 2'd0) ||
                    (r4 && mr4 == 2'd0) || (rn && mr5 == 2'd0);
    assign m1_ACK = (r0 && mr0 == 2'd1) || (r1 && mr1 == 2'd1) ||
                    (r2 && mr2 == 2'd1) || (r3 && mr3 == 2'd1) ||
                    (r4 && mr4 == 2'd1) || (rn && mr5 == 2'd1);
    assign m2_ACK = (r0 && mr0 == 2'd2) || (r1 && mr1 == 2'd2) ||
                    (r2 && mr2 == 2'd2) || (r3 && mr3 == 2'd2) ||
                    (r4 && mr4 == 2'd2) || (rn && mr5 == 2'd2);

    // data mux: select the returning source for this master
    wire [31:0] m0_data =
        (r0 && mr0 == 2'd0) ? rs0 :
        (r1 && mr1 == 2'd0) ? rs1 :
        (r2 && mr2 == 2'd0) ? rs2 :
        (r3 && mr3 == 2'd0) ? rs3 :
        (r4 && mr4 == 2'd0) ? rs4 :
        (rn && mr5 == 2'd0) ? 32'h0 : 32'h0;
    wire [31:0] m1_data =
        (r0 && mr0 == 2'd1) ? rs0 :
        (r1 && mr1 == 2'd1) ? rs1 :
        (r2 && mr2 == 2'd1) ? rs2 :
        (r3 && mr3 == 2'd1) ? rs3 :
        (r4 && mr4 == 2'd1) ? rs4 :
        (rn && mr5 == 2'd1) ? 32'h0 : 32'h0;
    wire [31:0] m2_data =
        (r0 && mr0 == 2'd2) ? rs0 :
        (r1 && mr1 == 2'd2) ? rs1 :
        (r2 && mr2 == 2'd2) ? rs2 :
        (r3 && mr3 == 2'd2) ? rs3 :
        (r4 && mr4 == 2'd2) ? rs4 :
        (rn && mr5 == 2'd2) ? 32'h0 : 32'h0;

    assign m0_RDAT = m0_data;
    assign m1_RDAT = m1_data;
    assign m2_RDAT = m2_data;

    // ------------------------------------------------------------- update
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy0 <= 1'b0; busy1 <= 1'b0; busy2 <= 1'b0;
            s_busy <= 5'b00000;
            nbusy  <= 1'b0;
            sm0 <= 2'd0; sm1 <= 2'd0; sm2 <= 2'd0; sm3 <= 2'd0;
            sm4 <= 2'd0; sm5 <= 2'd0;
            byp1_0 <= 1'b0; byp1_1 <= 1'b0; byp1_2 <= 1'b0;
        end else begin
            // ---- returns: clear busy (one cycle after slave ACK)
            if (r0) begin
                if      (mr0 == 2'd0) busy0 <= 1'b0;
                else if (mr0 == 2'd1) busy1 <= 1'b0;
                else                  busy2 <= 1'b0;
                s_busy[0] <= 1'b0;
            end
            if (r1) begin
                if      (mr1 == 2'd0) busy0 <= 1'b0;
                else if (mr1 == 2'd1) busy1 <= 1'b0;
                else                  busy2 <= 1'b0;
                s_busy[1] <= 1'b0;
            end
            if (r2) begin
                if      (mr2 == 2'd0) busy0 <= 1'b0;
                else if (mr2 == 2'd1) busy1 <= 1'b0;
                else                  busy2 <= 1'b0;
                s_busy[2] <= 1'b0;
            end
            if (r3) begin
                if      (mr3 == 2'd0) busy0 <= 1'b0;
                else if (mr3 == 2'd1) busy1 <= 1'b0;
                else                  busy2 <= 1'b0;
                s_busy[3] <= 1'b0;
            end
            if (r4) begin
                if      (mr4 == 2'd0) busy0 <= 1'b0;
                else if (mr4 == 2'd1) busy1 <= 1'b0;
                else                  busy2 <= 1'b0;
                s_busy[4] <= 1'b0;
            end
            if (rn) begin
                if      (mr5 == 2'd0) busy0 <= 1'b0;
                else if (mr5 == 2'd1) busy1 <= 1'b0;
                else                  busy2 <= 1'b0;
                nbusy <= 1'b0;
            end

            // ---- grants: set busy / ownership (one cycle after grant)
            byp1_0 <= g0 && bp0;
            byp1_1 <= g1 && bp1;
            byp1_2 <= g2 && bp2;
            if (grant) begin
                case (gslv)
                3'd0: begin s_busy[0] <= 1'b1; sm0 <= gmst; end
                3'd1: begin s_busy[1] <= 1'b1; sm1 <= gmst; end
                3'd2: begin s_busy[2] <= 1'b1; sm2 <= gmst; end
                3'd3: begin s_busy[3] <= 1'b1; sm3 <= gmst; end
                3'd4: begin s_busy[4] <= 1'b1; sm4 <= gmst; end
                3'd5: begin nbusy <= 1'b1;           sm5 <= gmst; end
                endcase
                if      (gmst == 2'd0) busy0 <= 1'b1;
                else if (gmst == 2'd1) busy1 <= 1'b1;
                else                    busy2 <= 1'b1;
            end
        end
    end

endmodule
`default_nettype wire
