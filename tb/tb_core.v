// tb_core -- mythrv32 core testbench (growing; check groups C1..C8)
//
// C1: straight-line RV32I ALU (ADD..SRAI, LUI/AUIPC), all rd, x0,
//     forwarding chains. Program assembled from tb/tb_core.S by clang;
//     golden register file from tools/emu32.py -> tb/tb_core_exp.vh.
//     m0 = I-RAM slave (1-cyc ACK); m1/m2 idle in Bare ALU-only tests.
`timescale 1ns/1ps
module tb_core;

    reg clk = 1, rst = 0;
    always #10 clk = ~clk;

    integer errors = 0, passes = 0;
    // NOTE: label must be <= 32 chars (iverilog %s on [255:0] prints 32).
    task err(input [255:0] s);
        begin errors = errors + 1; $display("FAIL: %0s", s); end
    endtask
    task ok(input [255:0] s);
        begin passes = passes + 1; $display("PASS: %0s", s); end
    endtask

    // ---------------- memories / WB slaves ----------------
    reg [31:0] irom [0:4095];      // program (m0)
    reg [31:0] dram [0:4095];      // data    (m1; idle in C1)
    reg [31:0] pram [0:4095];      // PT      (m2; idle in C1)

    wire m0_stb, m0_we, m0_ack;
    wire [3:0] m0_sel;
    wire [31:0] m0_adr, m0_dat;
    wire m1_stb, m1_we, m1_ack;
    wire [3:0] m1_sel;
    wire [31:0] m1_adr, m1_wdat, m1_dat;
    wire m2_stb, m2_we, m2_ack;
    wire [3:0] m2_sel;
    wire [31:0] m2_adr, m2_dat;

    core dut (
        .clk(clk), .rst(rst),
        .meip(1'b0), .seip(1'b0),
        .m0_stb(m0_stb), .m0_sel(m0_sel), .m0_adr(m0_adr), .m0_we(m0_we),
        .m0_ack(m0_ack), .m0_dat(m0_dat),
        .m1_stb(m1_stb), .m1_sel(m1_sel), .m1_adr(m1_adr), .m1_we(m1_we),
        .m1_wdat(m1_wdat), .m1_ack(m1_ack), .m1_dat(m1_dat),
        .m2_stb(m2_stb), .m2_sel(m2_sel), .m2_adr(m2_adr), .m2_we(m2_we),
        .m2_ack(m2_ack), .m2_dat(m2_dat)
    );

    // 1-cyc-ACK RAM slaves (WB B4: ACK >= 1 cycle after STB, 1 cycle long)
    reg ack0 = 0, ack1 = 0, ack2 = 0;
    reg [31:0] dat0 = 0, dat1 = 0, dat2 = 0;
    integer m1_stb_cnt = 0, m2_stb_cnt = 0, sel_bad = 0, we_bad = 0;
    always @(posedge clk) begin
        if (m0_stb) begin
            if (m0_sel !== 4'b1111) sel_bad = sel_bad + 1;
            if (m0_we !== 1'b0)     we_bad  = we_bad  + 1;
            ack0 <= 1'b1;
            dat0 <= irom[m0_adr[13:2]];
        end else ack0 <= 1'b0;
        if (m1_stb) begin
            m1_stb_cnt = m1_stb_cnt + 1;
            ack1 <= 1'b1;
            dat1 <= dram[m1_adr[13:2]];
        end else ack1 <= 1'b0;
        if (m2_stb) begin
            m2_stb_cnt = m2_stb_cnt + 1;
            ack2 <= 1'b1;
            dat2 <= pram[m2_adr[13:2]];
        end else ack2 <= 1'b0;
    end
    assign m0_ack = ack0; assign m0_dat = dat0;
    assign m1_ack = ack1; assign m1_dat = dat1;
    assign m2_ack = ack2; assign m2_dat = dat2;

    // ---------------- program + expectations ----------------
    integer exp [0:31];

    integer nc = 0;                       // retired instructions
    always @(posedge clk) if (dut.commit) nc = nc + 1;
    integer fq_ovf = 0;                   // fetch FIFO overflow guard (bug if set)
    always @(posedge clk) if (dut.fq_c > 3'd4) fq_ovf = 1;

    integer i, bad, t;
    initial begin
        `include "tb/tb_core_exp.vh"
        for (i = 0; i < 4096; i = i + 1) begin
            irom[i] = 32'h00000013;      // ADDI x0,x0,0 (nop) past program
            dram[i] = 32'h0;
            pram[i] = 32'h0;
        end
        $readmemh("tb/tb_core.hex", irom);
        @(negedge clk); rst = 1;
        repeat (3) @(negedge clk); rst = 0;

        // wait for `C1_NCOMMIT` retirements with watchdog, then drain
        t = 0;
        while (nc < `C1_NCOMMIT && t < `C1_NCOMMIT * 40 + 500) begin
            @(posedge clk); t = t + 1;
        end
        repeat (20) @(posedge clk);

        if (nc < `C1_NCOMMIT) err("C1 commit timeout");
        else ok("C1-commit-done");
        if (sel_bad) err("C1 m0 sel/prot");
        else ok("C1-m0-sel-ok");
        if (we_bad) err("C1 m0 we tied 0");
        else ok("C1-m0-we-ok");
        if (m1_stb_cnt) err("C1 m1 idle");
        else ok("C1-m1-idle");
        if (m2_stb_cnt) err("C1 m2 idle (Bare)");
        else ok("C1-m2-idle-bare");
        if (fq_ovf) err("fetch fifo overflow");
        else ok("C-fifo-no-ovf");

        bad = 0;
        for (i = 1; i < 32; i = i + 1) begin
            if (dut.rf[i] !== exp[i]) begin
                bad = bad + 1;
                $display("  x%0d got %h exp %h", i, dut.rf[i], exp[i]);
            end
        end
        if (bad) err("C1 rf golden mismatch");
        else ok("C1-rf-golden31");
        // C2 named checks (subset of golden; label per op/case)
        if (dut.rf[3]  !== exp[3])  err("C2-mul");        else ok("C2-mul");
        if (dut.rf[4]  !== exp[4])  err("C2-mulh");       else ok("C2-mulh");
        if (dut.rf[5]  !== exp[5])  err("C2-mulhsu");     else ok("C2-mulhsu");
        if (dut.rf[6]  !== exp[6])  err("C2-mulhu");      else ok("C2-mulhu");
        if (dut.rf[7]  !== exp[7])  err("C2-div");        else ok("C2-div");
        if (dut.rf[8]  !== exp[8])  err("C2-divu");       else ok("C2-divu");
        if (dut.rf[9]  !== exp[9])  err("C2-rem");        else ok("C2-rem");
        if (dut.rf[10] !== exp[10]) err("C2-remu");       else ok("C2-remu");
        if (dut.rf[11] !== exp[11]) err("C2-div-by-zero");else ok("C2-div-by-zero");
        if (dut.rf[12] !== exp[12]) err("C2-rem-by-zero");else ok("C2-rem-by-zero");
        if (dut.rf[18] !== exp[18]) err("C2-div-overflow");else ok("C2-div-overflow");
        if (dut.rf[20] !== exp[20]) err("C2-rem-overflow");else ok("C2-rem-overflow");
        if (dut.rf[0] !== 32'h0) err("C1 x0 nonzero");
        else ok("C1-x0-immovable");

        if (errors == 0) $display("ALL TESTS PASSED (%0d checks)", passes);
        else $display("%0d ERRORS (%0d passed)", errors, passes);
        $finish;
    end
endmodule
