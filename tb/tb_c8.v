// tb_c8 -- mythrv32 C8: dense hazard/forwarding regression (M-mode Bare)
//
// Program: tb/tb_c8.S (assembled by clang -> tb/tb_c8.hex). 23 controlled RAW
// hazard patterns (ALU d=1..4, load-use d=1 for LW/LB/LH/LBU/LHU,
// store->load/store->store b2b, load->branch, load->jump, MUL/MUL, DIV/DIV,
// REM/REMU b2b, JAL/JALR link, CSR read d=1, taken back-ward loop,
// back-to-back same-reg overwrite, varied ALU d=1).
//
// The C8 bench checks the DRAM scoreboard (words 0..31) + final register file
// against the tools/emu32.py architectural golden (tb/tb_c8_exp.vh), so any
// forwarding/hazard bug in the RTL surfaces as a mismatch. Bare (satp=0):
// VA==PA, DRAM base 0x8000_0000; no page walk (m2 idle).
//
// Build + run (regenerates tb/tb_c8.hex + tb/tb_c8_exp.vh from tb/tb_c8.S):
//   make c8
// (the Makefile compiles, emu32-goldens, seds C1_NCOMMIT->C8_NCOMMIT, and
//  builds vvp; sim log -> $(OUT)/c8.log)
`timescale 1ns/1ps
module tb_c8;

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
    reg [31:0] dram [0:4095];      // data / scoreboard (m1)

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
    integer m1_stb_cnt = 0, m1_we_cnt = 0, m1_ld_cnt = 0, sel_bad = 0, we_bad = 0;
    integer j2 = 0;
    always @(posedge clk) begin
        if (m0_stb) begin
            if (m0_sel !== 4'b1111) sel_bad = sel_bad + 1;
            if (m0_we !== 1'b0)     we_bad  = we_bad  + 1;
            ack0 <= 1'b1;
            dat0 <= irom[m0_adr[13:2]];
        end else ack0 <= 1'b0;
        if (m1_stb) begin
            m1_stb_cnt = m1_stb_cnt + 1;
            if (m1_we) begin
                m1_we_cnt = m1_we_cnt + 1;
                for (j2 = 0; j2 < 4; j2 = j2 + 1)
                    if (m1_sel[j2])
                        dram[m1_adr[13:2]][j2*8 +: 8] <= m1_wdat[j2*8 +: 8];
            end else begin
                m1_ld_cnt = m1_ld_cnt + 1;
            end
            ack1 <= 1'b1;
            dat1 <= dram[m1_adr[13:2]];
        end else ack1 <= 1'b0;
        if (m2_stb) begin ack2 <= 1'b1; dat2 <= 32'h0; end else ack2 <= 1'b0;
    end
    assign m0_ack = ack0; assign m0_dat = dat0;
    assign m1_ack = ack1; assign m1_dat = dat1;
    assign m2_ack = ack2; assign m2_dat = dat2;

    // ---------------- counters / guards ----------------
    integer exp  [0:31];
    integer mexp [0:34];
    integer nc = 0;                       // retired instructions
    always @(posedge clk) if (dut.commit) nc = nc + 1;
    integer fq_ovf = 0;                  // fetch FIFO overflow guard (bug if set)
    always @(posedge clk) if (dut.fq_c > 3'd4) fq_ovf = 1;

    integer i, bad, t;
    initial begin
        for (i = 0; i < 33; i = i + 1) mexp[i] = 32'h0;
        `include "tb/tb_c8_exp.vh"
        for (i = 0; i < 4096; i = i + 1) begin
            irom[i] = 32'h00000013;      // ADDI x0,x0,0 (nop) past program
            dram[i] = 32'h0;
        end
        $readmemh("tb/tb_c8.hex", irom);
        @(negedge clk); rst = 1;
        repeat (3) @(negedge clk); rst = 0;

        // wait for C8_NCOMMIT retirements (incl. final self-loop j) with watchdog
        t = 0;
        while (nc < `C8_NCOMMIT && t < `C8_NCOMMIT * 60 + 2000) begin
            @(posedge clk); t = t + 1;
        end
        repeat (20) @(posedge clk);

        if (nc < `C8_NCOMMIT) err("C8 commit timeout");
        else ok("C8-commit-done");
        if (sel_bad) err("C8 m0 sel/prot");      else ok("C8-m0-sel-ok");
        if (we_bad)  err("C8 m0 we tied 0");     else ok("C8-m0-we-ok");
        if (m1_stb_cnt == 0) err("C8 m1 no traffic");
        else ok("C8-m1-traffic");
        if (m1_we_cnt == 0) err("C8 m1 no stores");
        else ok("C8-m1-stores");
        if (m1_ld_cnt == 0) err("C8 m1 no loads");
        else ok("C8-m1-loads");
        if (fq_ovf) err("C8 fetch fifo ovf");
        else ok("C8-fifo-no-ovf");

        // ---- scoreboard: DRAM words 0..31 vs golden ----
        if (dram[0]  !== mexp[0])  err("C8-T1 alu d1");      else ok("C8-T1 alu d1");
        if (dram[1]  !== mexp[1])  err("C8-T2 alu d2");      else ok("C8-T2 alu d2");
        if (dram[2]  !== mexp[2])  err("C8-T3 alu d3");      else ok("C8-T3 alu d3");
        if (dram[3]  !== mexp[3])  err("C8-T4 alu d4");      else ok("C8-T4 alu d4");
        if (dram[5]  !== mexp[5])  err("C8-T5 lw use d1");   else ok("C8-T5 lw use d1");
        if (dram[7]  !== mexp[7])  err("C8-T6 lb use d1");   else ok("C8-T6 lb use d1");
        if (dram[9]  !== mexp[9])  err("C8-T7 lh use d1");   else ok("C8-T7 lh use d1");
        if (dram[11] !== mexp[11]) err("C8-T8 lbu use d1");  else ok("C8-T8 lbu use d1");
        if (dram[13] !== mexp[13]) err("C8-T9 lhu use d1");  else ok("C8-T9 lhu use d1");
        if (dram[15] !== mexp[15]) err("C8-T10 st->ld b2b"); else ok("C8-T10 st->ld b2b");
        if (dram[17] !== mexp[17]) err("C8-T11 st->st b2b"); else ok("C8-T11 st->st b2b");
        if (dram[19] !== mexp[19]) err("C8-T12 ld->br");     else ok("C8-T12 ld->br");
        if (dram[21] !== mexp[21]) err("C8-T13 ld->jalr");   else ok("C8-T13 ld->jalr");
        if (dram[22] !== mexp[22]) err("C8-T14 mul b2b");    else ok("C8-T14 mul b2b");
        if (dram[23] !== mexp[23]) err("C8-T15 div use");    else ok("C8-T15 div use");
        if (dram[24] !== mexp[24]) err("C8-T16 div b2b");    else ok("C8-T16 div b2b");
        if (dram[25] !== mexp[25]) err("C8-T17 jal link");   else ok("C8-T17 jal link");
        if (dram[26] !== mexp[26]) err("C8-T18 jalr link");  else ok("C8-T18 jalr link");
        if (dram[27] !== mexp[27]) err("C8-T19 csr d1");     else ok("C8-T19 csr d1");
        if (dram[28] !== mexp[28]) err("C8-T20 loop acc");   else ok("C8-T20 loop acc");
        if (dram[29] !== mexp[29]) err("C8-T21 b2b ovr");    else ok("C8-T21 b2b ovr");
        if (dram[30] !== mexp[30]) err("C8-T22 alu d1 mix"); else ok("C8-T22 alu d1 mix");
        if (dram[31] !== mexp[31]) err("C8-T23 rem b2b");    else ok("C8-T23 rem b2b");
        if (dram[32] !== mexp[32]) err("C8-T24 csr-vs-younger"); else ok("C8-T24 csr-vs-younger");
        if (dram[33] !== mexp[33]) err("C8-T25 wb-loss ld");    else ok("C8-T25 wb-loss ld");
        if (dram[34] !== mexp[34]) err("C8-T25 wb-loss link");  else ok("C8-T25 wb-loss link");
        // full scoreboard array (catches anything the named checks miss)
        bad = 0;
        for (i = 0; i < 35; i = i + 1) begin
            if (dram[i] !== mexp[i]) begin
                bad = bad + 1;
                $display("  score[%0d] got %h exp %h", i, dram[i], mexp[i]);
            end
        end
        if (bad) err("C8-scoreboard35");   else ok("C8-scoreboard35");

        // ---- final register file vs golden ----
        bad = 0;
        for (i = 1; i < 32; i = i + 1) begin
            if (dut.rf[i] !== exp[i]) begin
                bad = bad + 1;
                $display("  x%0d got %h exp %h", i, dut.rf[i], exp[i]);
            end
        end
        if (bad) err("C8 rf golden");       else ok("C8-rf-golden31");
        if (dut.rf[0] !== 32'h0) err("C8 x0 nonzero");
        else ok("C8-x0-immovable");

        if (errors == 0) $display("ALL TESTS PASSED (%0d checks)", passes);
        else $display("%0d ERRORS (%0d passed)", errors, passes);
        $finish;
    end
endmodule
