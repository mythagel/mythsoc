// tb_wb_ic -- directed + concurrent testbench for wb_ic (mythrv32)
//
// Slave models: s0,s1,s2 = 1-cycle RAM, s3 = 3-cycle RAM,
// s4 = NOACK (dead) until reset, then 1-cycle RAM.
// Protocol per notes/design.md sec.6: master STB 1cyc/transfer, no STB while
// STALL, ACK 1cyc pulse in issue order, may STB same cycle as ACK latch.
//
// iverilog -g2005: all locals at module scope (no in-block declarations).
`timescale 1ns/1ps

// ---------------------------------------------------------------- tb_wb_slv
// WB B4 slave model: 1024-word RAM (ADR[11:2]); ACK one or more cycles
// after STB (sec.6): STB@P -> ACK@[P+max(1,DELAY-1), P+max(1,DELAY-1)+1).
// NOACK=1: latches first STB, never ACKs. Write merges per SEL.
module tb_wb_slv #(parameter DELAY=1) (
    input NOACK,
    input clk, rst,
    input        STB_I, WE_I,
    input  [3:0] SEL_I,
    input  [31:0] ADR_I, DAT_I,
    output reg [31:0] DAT_O,
    output reg        ACK_O,
    output reg [31:0] last_adr,
    output reg [31:0] last_dat,
    output reg [3:0]  last_sel,
    output reg        last_we,
    output reg [15:0] stb_cnt
);
    reg [31:0] mem [0:1023];
    reg [31:0] wt;
    reg [11:0] adr_l;
    reg        we_l;
    reg        pend;
    reg [2:0]  wait_n;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ACK_O <= 1'b0; DAT_O <= 32'h0;
            pend <= 1'b0; wait_n <= 3'd0;
            stb_cnt <= 16'h0;
            last_we <= 1'b0; last_adr <= 32'h0;
            last_sel <= 4'h0; last_dat <= 32'h0;
            we_l <= 1'b0; adr_l <= 12'h0;
        end else begin
            ACK_O <= 1'b0;
            if (STB_I && !pend) begin
                stb_cnt <= stb_cnt + 16'h1;
                last_we <= WE_I; last_adr <= ADR_I;
                last_sel <= SEL_I; last_dat <= DAT_I;
                pend <= 1'b1;
                we_l <= WE_I; adr_l <= ADR_I[11:2];
                if (NOACK == 0) begin
                    // ACK one or more cycles after STB (design.md sec.6):
                    // DELAY=N -> ACK at STB+N-1 (N=1: next cycle).
                    wait_n <= (DELAY == 3'd1) ? 3'd1 : DELAY - 3'd1;
                    pend <= 1'b1;
                    if (WE_I) begin
                        wt = mem[ADR_I[11:2]];
                        if (SEL_I[0]) wt = {wt[31:8], DAT_I[7:0]};
                        if (SEL_I[1]) wt = {wt[31:16], DAT_I[15:8], wt[7:0]};
                        if (SEL_I[2]) wt = {wt[31:24], DAT_I[23:16], wt[15:0]};
                        if (SEL_I[3]) wt = {DAT_I[31:24], wt[23:0]};
                        mem[ADR_I[11:2]] <= wt;
                    end
                end
            end else if (pend && NOACK == 0) begin
                if (wait_n == 3'd1) begin
                    wait_n <= 3'd0;
                    ACK_O <= 1'b1;
                    pend <= 1'b0;
                    if (!we_l) DAT_O <= mem[adr_l];
                end else begin
                    wait_n <= wait_n - 3'd1;
                end
            end
        end
    end

endmodule

// ----------------------------------------------------------------- module
module tb_wb_ic;

    reg clk = 1, rst = 0;
    always #10 clk = ~clk;
    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    integer errors = 0, passes = 0;
    // NOTE: label must be <= 32 chars. iverilog %-formats a [255:0] string
    // port as 32 chars: longer literals are truncated (high end) on print.
    task err(input [255:0] s);
        begin
            errors = errors + 1;
            $display("FAIL: %0s", s);
        end
    endtask
    task ok(input [255:0] s);
        begin
            passes = passes + 1;
            $display("PASS: %0s", s);
        end
    endtask

    // ---- master side (TB drives)
    reg m0_STB = 0, m0_WE = 0;
    reg [3:0] m0_SEL = 0;
    reg [31:0] m0_ADR = 0, m0_WDAT = 0;
    reg m1_STB = 0, m1_WE = 0;
    reg [3:0] m1_SEL = 0;
    reg [31:0] m1_ADR = 0, m1_WDAT = 0;
    reg m2_STB = 0, m2_WE = 0;
    reg [3:0] m2_SEL = 0;
    reg [31:0] m2_ADR = 0, m2_WDAT = 0;

    wire m0_STALL, m0_ACK, m1_STALL, m1_ACK, m2_STALL, m2_ACK;
    wire [31:0] m0_RDAT, m1_RDAT, m2_RDAT;

    // ---- slave side
    wire s0_STB, s0_WE, s1_STB, s1_WE, s2_STB, s2_WE, s3_STB, s3_WE, s4_STB, s4_WE;
    wire [3:0] s0_SEL, s1_SEL, s2_SEL, s3_SEL, s4_SEL;
    wire [31:0] s0_ADR, s0_WDAT, s1_ADR, s1_WDAT, s2_ADR, s2_WDAT,
                s3_ADR, s3_WDAT, s4_ADR, s4_WDAT;
    wire s0_ACK, s1_ACK, s2_ACK, s3_ACK, s4_ACK;
    wire [31:0] s0_RDAT, s1_RDAT, s2_RDAT, s3_RDAT, s4_RDAT;
    wire [15:0] s0_cnt, s1_cnt, s2_cnt, s3_cnt, s4_cnt;
    wire s0_lwe, s1_lwe, s2_lwe, s3_lwe, s4_lwe;
    wire [3:0] s0_lsel, s1_lsel, s2_lsel, s3_lsel, s4_lsel;
    wire [31:0] s0_ladr, s0_ldat, s1_ladr, s1_ldat, s2_ladr, s2_ldat,
                s3_ladr, s3_ldat, s4_ladr, s4_ldat;

    wb_ic dut (
        .clk(clk), .rst(rst),
        .m0_STB(m0_STB), .m0_WE(m0_WE), .m0_SEL(m0_SEL),
        .m0_ADR(m0_ADR), .m0_WDAT(m0_WDAT),
        .m0_STALL(m0_STALL), .m0_ACK(m0_ACK), .m0_RDAT(m0_RDAT),
        .m1_STB(m1_STB), .m1_WE(m1_WE), .m1_SEL(m1_SEL),
        .m1_ADR(m1_ADR), .m1_WDAT(m1_WDAT),
        .m1_STALL(m1_STALL), .m1_ACK(m1_ACK), .m1_RDAT(m1_RDAT),
        .m2_STB(m2_STB), .m2_WE(m2_WE), .m2_SEL(m2_SEL),
        .m2_ADR(m2_ADR), .m2_WDAT(m2_WDAT),
        .m2_STALL(m2_STALL), .m2_ACK(m2_ACK), .m2_RDAT(m2_RDAT),
        .s0_STB(s0_STB), .s0_WE(s0_WE), .s0_SEL(s0_SEL),
        .s0_ADR(s0_ADR), .s0_WDAT(s0_WDAT),
        .s0_ACK(s0_ACK), .s0_RDAT(s0_RDAT),
        .s1_STB(s1_STB), .s1_WE(s1_WE), .s1_SEL(s1_SEL),
        .s1_ADR(s1_ADR), .s1_WDAT(s1_WDAT),
        .s1_ACK(s1_ACK), .s1_RDAT(s1_RDAT),
        .s2_STB(s2_STB), .s2_WE(s2_WE), .s2_SEL(s2_SEL),
        .s2_ADR(s2_ADR), .s2_WDAT(s2_WDAT),
        .s2_ACK(s2_ACK), .s2_RDAT(s2_RDAT),
        .s3_STB(s3_STB), .s3_WE(s3_WE), .s3_SEL(s3_SEL),
        .s3_ADR(s3_ADR), .s3_WDAT(s3_WDAT),
        .s3_ACK(s3_ACK), .s3_RDAT(s3_RDAT),
        .s4_STB(s4_STB), .s4_WE(s4_WE), .s4_SEL(s4_SEL),
        .s4_ADR(s4_ADR), .s4_WDAT(s4_WDAT),
        .s4_ACK(s4_ACK), .s4_RDAT(s4_RDAT)
    );

    tb_wb_slv #(.DELAY(1))  u_s0 (.clk(clk), .rst(rst), .NOACK(1'b0),
        .STB_I(s0_STB), .WE_I(s0_WE), .SEL_I(s0_SEL),
        .ADR_I(s0_ADR), .DAT_I(s0_WDAT), .DAT_O(s0_RDAT), .ACK_O(s0_ACK),
        .last_adr(s0_ladr), .last_dat(s0_ldat), .last_sel(s0_lsel),
        .last_we(s0_lwe), .stb_cnt(s0_cnt));
    tb_wb_slv #(.DELAY(1))  u_s1 (.clk(clk), .rst(rst), .NOACK(1'b0),
        .STB_I(s1_STB), .WE_I(s1_WE), .SEL_I(s1_SEL),
        .ADR_I(s1_ADR), .DAT_I(s1_WDAT), .DAT_O(s1_RDAT), .ACK_O(s1_ACK),
        .last_adr(s1_ladr), .last_dat(s1_ldat), .last_sel(s1_lsel),
        .last_we(s1_lwe), .stb_cnt(s1_cnt));
    tb_wb_slv #(.DELAY(1))  u_s2 (.clk(clk), .rst(rst), .NOACK(1'b0),
        .STB_I(s2_STB), .WE_I(s2_WE), .SEL_I(s2_SEL),
        .ADR_I(s2_ADR), .DAT_I(s2_WDAT), .DAT_O(s2_RDAT), .ACK_O(s2_ACK),
        .last_adr(s2_ladr), .last_dat(s2_ldat), .last_sel(s2_lsel),
        .last_we(s2_lwe), .stb_cnt(s2_cnt));
    tb_wb_slv #(.DELAY(3))  u_s3 (.clk(clk), .rst(rst), .NOACK(1'b0),
        .STB_I(s3_STB), .WE_I(s3_WE), .SEL_I(s3_SEL),
        .ADR_I(s3_ADR), .DAT_I(s3_WDAT), .DAT_O(s3_RDAT), .ACK_O(s3_ACK),
        .last_adr(s3_ladr), .last_dat(s3_ldat), .last_sel(s3_lsel),
        .last_we(s3_lwe), .stb_cnt(s3_cnt));
    reg dead4 = 1;
    tb_wb_slv #(.DELAY(1)) u_s4 (.clk(clk), .rst(rst), .NOACK(dead4),
        .STB_I(s4_STB), .WE_I(s4_WE), .SEL_I(s4_SEL),
        .ADR_I(s4_ADR), .DAT_I(s4_WDAT), .DAT_O(s4_RDAT), .ACK_O(s4_ACK),
        .last_adr(s4_ladr), .last_dat(s4_ldat), .last_sel(s4_lsel),
        .last_we(s4_lwe), .stb_cnt(s4_cnt));

    // ---- ACK capture (master latches every cycle) ----
    reg [31:0] cap0 [0:63], cap1 [0:63], cap2 [0:63];
    integer cap0_w, cap0_r, cap1_w, cap1_r, cap2_w, cap2_r;
    always @(posedge clk) #1 begin
        if (m0_ACK) begin cap0[cap0_w % 64] = m0_RDAT; cap0_w = cap0_w + 1; end
        if (m1_ACK) begin cap1[cap1_w % 64] = m1_RDAT; cap1_w = cap1_w + 1; end
        if (m2_ACK) begin cap2[cap2_w % 64] = m2_RDAT; cap2_w = cap2_w + 1; end
    end

    // ---- FIFO unit-test instance
    reg ut_push = 0, ut_pop = 0;
    reg [68:0] ut_din;
    wire [68:0] ut_head;
    wire [2:0] ut_cnt;
    wire ut_full;
    wb_ic_mfifo u_ut (.clk(clk), .rst(rst), .push(ut_push), .pop(ut_pop),
                     .din(ut_din), .head(ut_head), .cnt(ut_cnt), .full(ut_full));

    // ---- helpers ----------------------------------------------------------
    reg [31:0] d;
    integer p0, p1, p2, a0, a1, a2;

    function [31:0] seedA;   input [10:0] i;
        begin seedA = 32'hC0DE_0000 + {i, 2'b00}; end
    endfunction
    function [31:0] seedS0; input [10:0] i;
        begin seedS0 = 32'hBEEF_0000 + {i, 2'b00}; end
    endfunction

    task m0_push(input [31:0] a, input [31:0] dd, input [3:0] s, input [1:0] w);
        begin
            while (m0_STALL) @(posedge clk);
            @(negedge clk);
            m0_STB = 1; m0_ADR = a; m0_WDAT = dd; m0_SEL = s; m0_WE = w;
            @(posedge clk) #1;
            m0_STB = 0; m0_WE = 0;
            p0 = p0 + 1;
        end
    endtask
    task m1_push(input [31:0] a, input [31:0] dd, input [3:0] s, input [1:0] w);
        begin
            while (m1_STALL) @(posedge clk);
            @(negedge clk);
            m1_STB = 1; m1_ADR = a; m1_WDAT = dd; m1_SEL = s; m1_WE = w;
            @(posedge clk) #1;
            m1_STB = 0; m1_WE = 0;
            p1 = p1 + 1;
        end
    endtask
    task m2_push(input [31:0] a, input [31:0] dd, input [3:0] s, input [1:0] w);
        begin
            while (m2_STALL) @(posedge clk);
            @(negedge clk);
            m2_STB = 1; m2_ADR = a; m2_WDAT = dd; m2_SEL = s; m2_WE = w;
            @(posedge clk) #1;
            m2_STB = 0; m2_WE = 0;
            p2 = p2 + 1;
        end
    endtask

    // wait for the master's next captured ACK; dd = latched read data
    task m0_wait_ack(output [31:0] dd);
        begin
            wait (cap0_r < cap0_w);
            dd = cap0[cap0_r % 64];
            cap0_r = cap0_r + 1;
            @(posedge clk) #1;
            a0 = a0 + 1;
        end
    endtask
    task m1_wait_ack(output [31:0] dd);
        begin
            wait (cap1_r < cap1_w);
            dd = cap1[cap1_r % 64];
            cap1_r = cap1_r + 1;
            @(posedge clk) #1;
            a1 = a1 + 1;
        end
    endtask
    task m2_wait_ack(output [31:0] dd);
        begin
            wait (cap2_r < cap2_w);
            dd = cap2[cap2_r % 64];
            cap2_r = cap2_r + 1;
            @(posedge clk) #1;
            a2 = a2 + 1;
        end
    endtask

    // seed words [b, b+n) of a slave via m1: word i = lo + step*(i+1)
    task seed_region(input [31:0] base, input [10:0] b, input [10:0] nn,
                    input [31:0] lo, input [31:0] step);
        begin
            for (i = 0; i < nn; i = i + 1) begin
                m1_push(base + {b + i, 2'b00}, lo + step * i, 4'b1111, 1'b1);
                m1_wait_ack(d);
            end
        end
    endtask

    // ---- stress tracking (module scope)
    reg [31:0] lb [0:27];   // s1 words 300..327 last full write
    reg [31:0] ls [0:63];   // s3 words 0..63 last write
    reg [31:0] l2 [0:63];   // s2 words 0..63 last write
    reg [31:0] le [0:63];   // s4 words 400..463 last write
    reg [31:0] lw_b [0:31]; // s1 words 200..231 last byte-pattern write
    reg [31:0] lcg0 = 32'h1234_5678, lcg1 = 32'h1234_5678, lcg2 = 32'h1234_5678;

    // ---- flattened locals (iverilog -g2005: no in-block declarations)
    integer i, si, k, it, v, op, w, ln, bk, bad;
    integer t0, t1, dt0, dt1;
    integer c0, c1, c2, c3;
    reg [31:0] dv, dv0, dv1, dv2;
    integer it0, i0, v0, op0, it1, i1, v1, op1, w1, ln1, bk1, it2, i2, v2, op2;
    integer i10a, i10c;

    initial begin
        p0 = 0; p1 = 0; p2 = 0; a0 = 0; a1 = 0; a2 = 0;
        cap0_w = 0; cap0_r = 0; cap1_w = 0; cap1_r = 0; cap2_w = 0; cap2_r = 0;
        i = 0; si = 0; k = 0; it = 0; v = 0; op = 0; w = 0; ln = 0; bk = 0; bad = 0;
        t0 = 0; t1 = 0; dt0 = 0; dt1 = 0; c0 = 0; c1 = 0; c2 = 0; c3 = 0;
        dv = 0; dv0 = 0; dv1 = 0; dv2 = 0;
        it0 = 0; i0 = 0; v0 = 0; op0 = 0; it1 = 0; i1 = 0; v1 = 0; op1 = 0;
        w1 = 0; ln1 = 0; bk1 = 0; it2 = 0; i2 = 0; v2 = 0; op2 = 0;
        i10a = 0; i10c = 0;
        // reset
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (5) @(negedge clk);

        // ---- T1 reset state
        if (m0_STALL || m1_STALL || m2_STALL) err("T1: stall after reset");
        else                                  ok("T1: stall low after reset");
        if (s0_cnt !== 0 || s1_cnt !== 0 || s2_cnt !== 0 || s3_cnt !== 0 || s4_cnt !== 0)
            err("T1: slave stb after reset");
        else
            ok("T1: no slave stb after reset");
        repeat (5) @(posedge clk);
        if (m0_ACK || m1_ACK || m2_ACK) err("T1: phantom ack");

        // ---- T2 latency + data (push->ack: 1cyc slave = 3, 3cyc = 4;
        //      see tools/wb_ic_latency.py: STB@P, slave ACK@P+max(1,N-1),
        //      m_ACK combinational same cycle, capture that posedge,
        //      wait_ack +1 posedge, t0 sampled one posedge before STB)
        m1_push(32'h8000_0000, 32'h1122_3344, 4'b1111, 1'b1);
        m1_wait_ack(d);
        t0 = cyc;
        m0_push(32'h8000_0000, 32'h0, 4'b1111, 1'b0);
        m0_wait_ack(d);
        t1 = cyc;
        if (d !== 32'h1122_3344) err("T2: s1 m0 rdata");
        if (t1 - t0 !== 3) err("T2: 1cyc slave push->ack != 3");
        else               ok("T2: 1cyc slave push->ack = 3cyc");
        t0 = cyc;
        m0_push(32'h0200_0000, 32'h0, 4'b1111, 1'b0);
        m0_wait_ack(d);
        t1 = cyc;
        if (t1 - t0 !== 4) err("T2: 3cyc slave push->ack != 4");
        else               ok("T2: 3cyc slave push->ack = 4cyc");

        // ---- T3 cross-master round trip (m1 wr, m0/m2 rd)
        m1_push(32'h8000_0014, 32'hDEAD_BEEF, 4'b1111, 1'b1);
        m1_wait_ack(d);
        m0_push(32'h8000_0014, 32'h0, 4'b1111, 1'b0);
        m0_wait_ack(d);
        if (d !== 32'hDEAD_BEEF) err("T3: m0 rd after m1 wr");
        else                     ok("T3: m0 rd after m1 wr");
        m2_push(32'h8000_0014, 32'h0, 4'b1111, 1'b0);
        m2_wait_ack(d);
        if (d !== 32'hDEAD_BEEF) err("T3: m2 rd after m1 wr");
        else                     ok("T3: m2 rd after m1 wr");

        // ---- T4 per-slave decode routing (s0..s3): delta STB counts
        for (si = 0; si < 4; si = si + 1) begin
            c0 = s0_cnt; c1 = s1_cnt; c2 = s2_cnt; c3 = s3_cnt;
            if (si == 0) m1_push(32'h0000_0028, 32'hA500_0000, 4'b1111, 1'b1);
            else if (si == 1) m1_push(32'h8000_0028, 32'hA500_0001, 4'b1111, 1'b1);
            else if (si == 2) m1_push(32'h1000_0028, 32'hA500_0002, 4'b1111, 1'b1);
            else             m1_push(32'h0200_0028, 32'hA500_0003, 4'b1111, 1'b1);
            m1_wait_ack(d);
            if (si == 0) begin
                if (s0_cnt - c0 !== 16'h1) err("T4: s0 stb delta");
                if (s1_cnt - c1 !== 0 || s2_cnt - c2 !== 0 || s3_cnt - c3 !== 0)
                    err("T4: xtalk stb");
                if (s0_lwe !== 1'b1) err("T4: s0 last_we");
                if (s0_ladr !== 32'h0000_0028) err("T4: s0 last_adr");
                if (s0_ldat !== 32'hA500_0000) err("T4: s0 last_dat");
                if (s0_lsel !== 4'b1111) err("T4: s0 last_sel");
            end else if (si == 1) begin
                if (s1_cnt - c1 !== 16'h1) err("T4: s1 stb delta");
                if (s2_cnt - c2 !== 0 || s3_cnt - c3 !== 0) err("T4: xtalk stb");
                if (s1_lwe !== 1'b1) err("T4: s1 last_we");
                if (s1_ladr !== 32'h8000_0028) err("T4: s1 last_adr");
                if (s1_ldat !== 32'hA500_0001) err("T4: s1 last_dat");
            end else if (si == 2) begin
                if (s2_cnt - c2 !== 16'h1) err("T4: s2 stb delta");
                if (s3_cnt - c3 !== 0) err("T4: xtalk stb");
                if (s2_lwe !== 1'b1) err("T4: s2 last_we");
                if (s2_ladr !== 32'h1000_0028) err("T4: s2 last_adr");
                if (s2_ldat !== 32'hA500_0002) err("T4: s2 last_dat");
            end else begin
                if (s3_cnt - c3 !== 16'h1) err("T4: s3 stb delta");
                if (s3_lwe !== 1'b1) err("T4: s3 last_we");
                if (s3_ladr !== 32'h0200_0028) err("T4: s3 last_adr");
                if (s3_ldat !== 32'hA500_0003) err("T4: s3 last_dat");
            end
            // read back
            if (si == 0) m1_push(32'h0000_0028, 32'h0, 4'b1111, 1'b0);
            else if (si == 1) m1_push(32'h8000_0028, 32'h0, 4'b1111, 1'b0);
            else if (si == 2) m1_push(32'h1000_0028, 32'h0, 4'b1111, 1'b0);
            else             m1_push(32'h0200_0028, 32'h0, 4'b1111, 1'b0);
            m1_wait_ack(d);
            if (si == 0 && d !== 32'hA500_0000) err("T4: s0 rdata");
            if (si == 1 && d !== 32'hA500_0001) err("T4: s1 rdata");
            if (si == 2 && d !== 32'hA500_0002) err("T4: s2 rdata");
            if (si == 3 && d !== 32'hA500_0003) err("T4: s3 rdata");
        end
        ok("T4: decode routing s0..s3");

        // ---- T5 null slave: read -> 0, write dropped, both ACKed
        m0_push(32'h1234_5678, 32'h0, 4'b1111, 1'b0);
        m0_wait_ack(d);
        if (d !== 32'h0) err("T5: null rd != 0");
        else             ok("T5: null rd -> 0");
        c0 = s0_cnt; c1 = s1_cnt; c2 = s2_cnt; c3 = s3_cnt;
        m1_push(32'h1234_5678, 32'hCAFE_BABE, 4'b1111, 1'b1);
        m1_wait_ack(d);
        if (s0_cnt - c0 !== 0 || s1_cnt - c1 !== 0 ||
            s2_cnt - c2 !== 0 || s3_cnt - c3 !== 0)
            err("T5: null hit a real slave");
        else
            ok("T5: null write dropped, ACKed");

        // ---- T6 byte/SEL merge through IC (s1 word 100)
        m1_push(32'h8000_0190, 32'h0000_0000, 4'b1111, 1'b1); // seed 0
        m1_wait_ack(d);
        m1_push(32'h8000_0190, 32'h7788_9900, 4'b0101, 1'b1); // lanes 0,2
        m1_wait_ack(d);
        m1_push(32'h8000_0190, 32'hAABB_CCDD, 4'b1000, 1'b1); // lane 3
        m1_wait_ack(d);
        m0_push(32'h8000_0190, 32'h0, 4'b1111, 1'b0);
        m0_wait_ack(d);
        if (d !== 32'hAA88_0000) err("T6: byte merge rdata");
        else                     ok("T6: byte merge {AA,00,88,00}");

$display("INFO: stage T7");
        // ---- T7 burst pipelining: 8 back-to-back reads drained 1/cyc
        for (i = 0; i < 8; i = i + 1)
            m1_push(32'h8000_0258 + i * 32'h4, 32'h4A50_0000 + i * 32'h4, 4'b1111, 1'b1);
        for (i = 0; i < 8; i = i + 1) m1_wait_ack(d);
        t0 = cyc;
        for (i = 0; i < 8; i = i + 1)
            m0_push(32'h8000_0258 + i * 32'h4, 32'h0, 4'b1111, 1'b0);
        t1 = cyc;
        for (i = 0; i < 8; i = i + 1) begin
            m0_wait_ack(d);
            if (d !== 32'h4A50_0000 + i * 32'h4) err("T7: burst rdata/order");
            t1 = cyc;
        end
        if (t1 - t0 !== 24) err("T7: burst total != 24cyc");
        else               ok("T7: burst total 24cyc (8 reads)");
        // m1: 8 writes + m0 readback
        for (i = 0; i < 8; i = i + 1)
            m1_push(32'h8000_0320 + i * 32'h4, 32'h600D_0000 + i * 32'h4, 4'b1111, 1'b1);
        for (i = 0; i < 8; i = i + 1) m1_wait_ack(d);
        for (i = 0; i < 8; i = i + 1) begin
            m0_push(32'h8000_0320 + i * 32'h4, 32'h0, 4'b1111, 1'b0);
            m0_wait_ack(d);
            if (d !== 32'h600D_0000 + i * 32'h4) err("T7: burst wr rdata");
        end
        ok("T7: burst writes+readback");

$display("INFO: stage T8");
        // ---- T8 no-ack slave: m0 stuck, STALL after 4 queued, m1/m2 free
        for (i = 0; i < 5; i = i + 1)
            m0_push(32'h0C00_0010 + i * 32'h4, 32'h0, 4'b1111, 1'b0);
        // 1 granted (s4 never ACKs), 4 queued -> STALL now
        if (m0_STALL !== 1'b1) err("T8: m0 STALL not set at 4 queued");
        else                   ok("T8: m0 STALL at 4 queued");
        // m1/m2 still get through
        m1_push(32'h8000_0000, 32'h00FF_00FF, 4'b1111, 1'b1);
        m1_wait_ack(d);
        m2_push(32'h1000_0028, 32'h0, 4'b1111, 1'b0);
        m2_wait_ack(d);
        if (d !== 32'hA500_0002) err("T8: m2 rd rdata");
        if (!m0_STALL) err("T8: m0 STALL dropped");
        else           ok("T8: m1/m2 free while m0 stalled");
        repeat (10) @(posedge clk);
        if (m0_ACK) err("T8: m0 acked w/ no-ack slave");

$display("INFO: stage T9");
        // ---- T9 reset mid-flight (granted to 3cyc slave): clean, revive
        m1_push(32'h0200_0008, 32'h0, 4'b1111, 1'b0);  // s3 read: in flight
        @(posedge clk) #1;                             // ~grant edge (s_busy set)
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (5) @(negedge clk);
        repeat (15) @(posedge clk);
        if (m0_ACK || m1_ACK || m2_ACK) err("T9: phantom ack after reset");
        if (m0_STALL || m1_STALL || m2_STALL) err("T9: stall after reset");
        else
            ok("T9: clean post reset mid-flight");
        // s3/s4 models also reset -> normal again
        dead4 = 0;   // s4 model back to 1-cycle RAM
        m1_push(32'h0200_0008, 32'h3344_5566, 4'b1111, 1'b1);
        m1_wait_ack(d);
        m0_push(32'h0200_0008, 32'h0, 4'b1111, 1'b0);
        m0_wait_ack(d);
        if (d !== 32'h3344_5566) err("T9: s3 rdata after revive");
        else                     ok("T9: s3 revived, round trip");
        m1_push(32'h0C00_0028, 32'hC1DE_A001, 4'b1111, 1'b1);
        m1_wait_ack(d);
        m0_push(32'h0C00_0028, 32'h0, 4'b1111, 1'b0);
        m0_wait_ack(d);
        if (d !== 32'hC1DE_A001) err("T9: s4 rdata after revive");
        else                     ok("T9: s4 revived, round trip");

$display("INFO: stage T10");
        // ---- T10 parallelism: m1 on 3cyc slave, m0/m2 streaming 10 each
        seed_region(32'h0000_0000, 11'h80, 11'd10, 32'h9900_0000, 32'h4); // s0 128..137
        seed_region(32'h8000_0000, 11'h80, 11'd10, 32'h9900_0000, 32'h4); // s1 128..137
        t0 = cyc;
        fork
            begin  // m0: 10 reads s0
                for (i10a = 0; i10a < 10; i10a = i10a + 1)
                    m0_push(32'h0000_0200 + i10a * 32'h4, 32'h0, 4'b1111, 1'b0);
                for (i10a = 0; i10a < 10; i10a = i10a + 1) begin
                    m0_wait_ack(dv0);
                    if (dv0 !== 32'h9900_0000 + i10a * 32'h4) err("T10: m0 rdata");
                end
            end
            begin  // m1: 1 read s3 (3cyc)
                m1_push(32'h0200_0004, 32'h0, 4'b1111, 1'b0);
                m1_wait_ack(dv1);
            end
            begin  // m2: 10 reads s1
                for (i10c = 0; i10c < 10; i10c = i10c + 1)
                    m2_push(32'h8000_0200 + i10c * 32'h4, 32'h0, 4'b1111, 1'b0);
                for (i10c = 0; i10c < 10; i10c = i10c + 1) begin
                    m2_wait_ack(dv2);
                    if (dv2 !== 32'h9900_0000 + i10c * 32'h4) err("T10: m2 rdata");
                end
            end
        join
        t1 = cyc;
        if (t1 - t0 > 60) err("T10: not parallel (serialized)");
        else              ok("T10: 3cyc slave overlapped 10+10");

        // ---- T11 seed stress regions
        seed_region(32'h8000_0000, 11'h0, 11'd64, 32'hC0DE_0000, 32'h4); // s1 A
        seed_region(32'h0000_0000, 11'h0, 11'd64, 32'hBEEF_0000, 32'h4); // s0 A
        seed_region(32'h8000_0000, 11'hC8, 11'd128, 32'h0000_0000, 32'h0); // s1 B
        seed_region(32'h0200_0000, 11'h0, 11'd64, 32'h0000_0000, 32'h0);  // s3 A
        seed_region(32'h1000_0000, 11'h0, 11'd64, 32'h0000_0000, 32'h0);  // s2 A
        seed_region(32'h0C00_0000, 11'h190, 11'd64, 32'h0000_0000, 32'h0); // s4 400..463
        // clear stress trackers
        for (k = 0; k < 28; k = k + 1) lb[k] = 32'h0;
        for (k = 0; k < 64; k = k + 1) begin
            ls[k] = 32'h0; l2[k] = 32'h0; le[k] = 32'h0;
        end
        for (k = 0; k < 32; k = k + 1) lw_b[k] = 32'h0;

$display("INFO: stage T12");
        // ---- T12 concurrent stress: 3 engines, 150 ops each
        dt0 = cyc;
        fork
            begin  // e0: m0 reads s1-A / s0-A / null
                for (it0 = 0; it0 < 150; it0 = it0 + 1) begin
                    v0 = lcg0;
                    lcg0 = lcg0 * 32'h4F5A_0C1 + 32'h8031_AB17;
                    op0 = v0[1:0];
                    while (m0_STALL) @(posedge clk);
                    if (op0 == 0) begin
                        i0 = v0[25:16] & 6'h3F;
                        m0_push(32'h8000_0000 + {i0, 2'b00}, 32'h0, 4'b1111, 1'b0);
                        m0_wait_ack(dv0);
                        if (dv0 !== seedA(i0[10:0])) err("T12: e0 s1-A rdata");
                    end else if (op0 == 1) begin
                        i0 = v0[25:16] & 6'h3F;
                        m0_push(32'h0000_0000 + {i0, 2'b00}, 32'h0, 4'b1111, 1'b0);
                        m0_wait_ack(dv0);
                        if (dv0 !== seedS0(i0[10:0])) err("T12: e0 s0-A rdata");
                    end else begin
                        i0 = v0[25:16] & 6'h3F;
                        m0_push(32'h1234_5678 + {i0, 2'b00}, 32'h0, 4'b1111, 1'b0);
                        m0_wait_ack(dv0);
                        if (dv0 !== 32'h0) err("T12: e0 null rdata");
                    end
                end
            end
            begin  // e1: m1 wr s1 full/byte, s3, s2
                bk1 = 0;
                for (it1 = 0; it1 < 150; it1 = it1 + 1) begin
                    v1 = lcg1;
                    lcg1 = lcg1 * 32'h4F5A_0C1 + 32'h8031_AB17;
                    op1 = v1[2:0];
                    while (m1_STALL) @(posedge clk);
                    if (op1 <= 2) begin
                        i1 = v1[25:16] % 28;
                        w1 = 300 + i1;
                        m1_push(32'h8000_0000 + {w1, 2'b00},
                                32'hDEAD_0000 + i1 * 32'h4, 4'b1111, 1'b1);
                        lb[i1] = 32'hDEAD_0000 + i1 * 32'h4;
                        m1_wait_ack(dv1);
                    end else if (op1 == 3) begin
                        w1 = 200 + bk1 % 32;
                        ln1 = (it1 * 3) % 4;
                        bk1 = bk1 + 1;
                        m1_push(32'h8000_0000 + {w1, 2'b00},
                                v1[7:0] << {ln1, 2'b00}, 4'b1111, 1'b1);
                        lw_b[w1 - 200] = v1[7:0] << {ln1, 2'b00};
                        m1_wait_ack(dv1);
                    end else if (op1 <= 5) begin
                        i1 = v1[25:16] & 5'h3F;
                        m1_push(32'h0200_0000 + {i1, 2'b00},
                                32'hF00D_0000 + i1 * 32'h4, 4'b1111, 1'b1);
                        ls[i1] = 32'hF00D_0000 + i1 * 32'h4;
                        m1_wait_ack(dv1);
                    end else begin
                        i1 = v1[25:16] & 5'h3F;
                        m1_push(32'h1000_0000 + {i1, 2'b00},
                                32'h7777_0000 + i1 * 32'h4, 4'b1111, 1'b1);
                        l2[i1] = 32'h7777_0000 + i1 * 32'h4;
                        m1_wait_ack(dv1);
                    end
                end
            end
            begin  // e2: m2 wr+rd s4, rd s1-A, rd s0-A, null
                for (it2 = 0; it2 < 150; it2 = it2 + 1) begin
                    v2 = lcg2;
                    lcg2 = lcg2 * 32'h4F5A_0C1 + 32'h8031_AB17;
                    op2 = v2[2:0];
                    while (m2_STALL) @(posedge clk);
                    if (op2 <= 2) begin
                        i2 = v2[25:16] % 64;
                        m2_push(32'h0C00_0000 + (400 + i2) * 32'h4,
                                32'hABCD_0000 + i2 * 32'h4, 4'b1111, 1'b1);
                        m2_wait_ack(dv2);
                        m2_push(32'h0C00_0000 + (400 + i2) * 32'h4,
                                32'h0, 4'b1111, 1'b0);
                        m2_wait_ack(dv2);
                        if (dv2 !== 32'hABCD_0000 + i2 * 32'h4) err("T12: e2 s4 wb");
                        le[i2] = 32'hABCD_0000 + i2 * 32'h4;
                    end else if (op2 <= 4) begin
                        i2 = v2[25:16] & 5'h3F;
                        m2_push(32'h8000_0000 + {i2, 2'b00}, 32'h0, 4'b1111, 1'b0);
                        m2_wait_ack(dv2);
                        if (dv2 !== seedA(i2[10:0])) err("T12: e2 s1-A rdata");
                    end else if (op2 == 5) begin
                        i2 = v2[25:16] & 5'h3F;
                        m2_push(32'h0000_0000 + {i2, 2'b00}, 32'h0, 4'b1111, 1'b0);
                        m2_wait_ack(dv2);
                        if (dv2 !== seedS0(i2[10:0])) err("T12: e2 s0-A rdata");
                    end else begin
                        i2 = v2[25:16] & 5'h3F;
                        m2_push(32'h1234_5678 + {i2, 2'b00}, 32'h0, 4'b1111, 1'b0);
                        m2_wait_ack(dv2);
                        if (dv2 !== 32'h0) err("T12: e2 null rdata");
                    end
                end
            end
        join
        dt1 = cyc;
        $display("INFO: T12 stress took %0d cycles", dt1 - dt0);
        if (dt1 - dt0 < 500) err("T12: stress suspiciously fast");
        else if (dt1 - dt0 > 1500) err("T12: stress too slow");
        else ok("T12: 450 concurrent ops in range");

$display("INFO: stage T13");
        // ---- T13 post-stress verification (hierarchical mem + readbacks)
        bad = 0;
        for (i = 0; i < 32; i = i + 1)
            if (u_s1.mem[200 + i] !== lw_b[i])
                bad = bad + 1;
        if (bad !== 0) err("T13: s1 byte region final words");
        bad = 0;
        for (i = 0; i < 28; i = i + 1)
            if (u_s1.mem[300 + i] !== lb[i]) bad = bad + 1;
        if (u_s1.mem[5] !== seedA(11'd5)) bad = bad + 1;
        if (bad !== 0) err("T13: s1 full-word region");
        else          ok("T13: s1 regions exact");
        bad = 0;
        for (i = 0; i < 64; i = i + 1) begin
            if (u_s3.mem[i] !== ls[i]) bad = bad + 1;
            if (u_s2.mem[i] !== l2[i]) bad = bad + 1;
            if (u_s4.mem[400 + i] !== le[i]) bad = bad + 1;
        end
        if (bad !== 0) err("T13: s2/s3/s4 regions");
        else            ok("T13: s2/s3/s4 regions exact");
        // readback spot checks
        m0_push(32'h8000_0000, 32'h0, 4'b1111, 1'b0);
        m0_wait_ack(d);
        if (d !== seedA(11'd0)) err("T13: s1-A[0] rdata");
        m2_push(32'h1000_0004, 32'h0, 4'b1111, 1'b0);
        m2_wait_ack(d);
        if (d !== l2[1]) err("T13: s2[1] rdata");
        // ack/push balance
        if (p0 - a0 !== 5) err("T13: m0 ack != push (T8 stuck 5)");
        if (p1 - a1 !== 1) err("T13: m1 ack != push (T9 reset 1)");
        if (p2 - a2 !== 0) err("T13: m2 ack != push");
        if (p0 - a0 === 5 && p1 - a1 === 1 && p2 - a2 === 0)
            ok("T13: ack balance m0-5 m1-1 m2-0");

        // ---- T14 FIFO unit test (u_ut)
        for (i = 0; i < 4; i = i + 1) begin
            @(negedge clk);
            ut_din = 69'h0;
            ut_din[68] = 1'b0;
            ut_din[67:64] = 4'b1111;
            ut_din[63:32] = 32'h8000_0000 + i * 32'h4;
            ut_din[31:0] = 32'h0000_000A * (i + 1);
            ut_push = 1;
            @(posedge clk) #1;
            ut_push = 0;
            if (ut_cnt !== i + 3'd1) err("T14: fifo cnt");
        end
        if (ut_full !== 1'b1) err("T14: fifo full at 4");
        else                   ok("T14: fifo full at 4");
        if (ut_head[31:0] !== 32'h0000_000A) err("T14: fifo head d0");
        // push while full: ignored
        @(negedge clk);
        ut_din[31:0] = 32'h0000_0042;
        ut_push = 1;
        @(posedge clk) #1;
        ut_push = 0;
        if (ut_cnt !== 3'd4) err("T14: push@full not ignored");
        else                 ok("T14: push@full ignored");
        // pop 1
        @(negedge clk);
        ut_pop = 1;
        @(posedge clk) #1;
        ut_pop = 0;
        if (ut_cnt !== 3'd3) err("T14: pop cnt");
        if (ut_head[31:0] !== 32'h0000_0014) err("T14: pop head d1");
        // push+pop same cycle
        @(negedge clk);
        ut_din[31:0] = 32'h0000_0099;
        ut_push = 1;
        ut_pop = 1;
        @(posedge clk) #1;
        ut_push = 0;
        ut_pop = 0;
        if (ut_cnt !== 3'd3) err("T14: push+pop cnt");
        if (ut_head[31:0] !== 32'h0000_001E) err("T14: push+pop head d2");
        // drain to empty
        @(negedge clk); ut_pop = 1;
        @(posedge clk) #1; ut_pop = 0;
        @(negedge clk); ut_pop = 1;
        @(posedge clk) #1; ut_pop = 0;
        if (ut_cnt !== 3'd1) err("T14: drain cnt");
        if (ut_head[31:0] !== 32'h0000_0099) err("T14: drain head pushed");
        @(negedge clk); ut_pop = 1;
        @(posedge clk) #1; ut_pop = 0;
        if (ut_cnt !== 3'd0) err("T14: empty cnt");
        // pop at empty: safe
        @(negedge clk); ut_pop = 1;
        @(posedge clk) #1; ut_pop = 0;
        if (ut_cnt !== 3'd0) err("T14: pop@empty");
        else                 ok("T14: fifo unit");

        // ---- summary
        $display("============================================");
        $display("TB WB_IC: %0d passes, %0d errors (cyc=%0d)",
                 passes, errors, cyc);
        if (errors == 0) $display("ALL TESTS PASSED");
        $display("============================================");
        $finish;
    end

    initial begin
        #50_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
