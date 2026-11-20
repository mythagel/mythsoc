// tb_plic -- directed testbench for plic (mythrv32)
//
// Spec: riscv-plic v1.0.0 (16 sources, 2 contexts: 0=M, 1=S).
// Addresses (verified by tools/plic_addr.py):
//   P(N) = 0x0C00_0004*N (N=1..16)   PEND = 0x0C00_1000
//   EN(c) = 0x0C00_2000+0x80*c       THR(c) = 0x0C20_0000+0x1000*c
//   CL(c) = 0x0C20_0004+0x1000*c
`timescale 1ns/1ps
module tb_plic;

    reg clk = 1, rst = 0, STB = 0, WE = 0;
    reg [3:0] SEL = 0;
    reg [31:0] ADR = 0, DAT = 0;
    reg [16:0] IRQ = 0;
    wire [31:0] DATO;
    wire ACK;
    wire [1:0] EIP;

    plic dut (
        .clk(clk), .rst(rst),
        .STB_I(STB), .WE_I(WE), .SEL_I(SEL), .ADR_I(ADR), .DAT_I(DAT),
        .DAT_O(DATO), .ACK_O(ACK),
        .IRQ(IRQ), .EIP(EIP)
    );

    always #10 clk = ~clk;
    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    integer errors = 0, passes = 0;
    // NOTE: label must be <= 32 chars (iverilog %-formats a [255:0] string
    // port as 32 chars; longer literals truncate on print).
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

    // ---- WB access (single STB, word access; SEL passed through)
    task wb_wr(input [31:0] a, input [31:0] d, input [3:0] s);
        begin
            @(negedge clk);
            STB = 1; WE = 1; SEL = s; ADR = a; DAT = d;
            @(posedge clk) #1;
            STB = 0;
        end
    endtask

    task wb_rd(input [31:0] a, input [3:0] s, output [31:0] d);
        begin
            @(negedge clk);
            STB = 1; WE = 0; SEL = s; ADR = a; DAT = 0;
            @(posedge clk) #1;
            d = DATO;
            STB = 0;
        end
    endtask

    // ---- register helpers (word addresses from tools/plic_addr.py)
    task pr_wr(input [7:0] s, input [31:0] v);
        begin
            wb_wr(32'h0C00_0000 + 32'h4 * s, v, 4'b1111);
        end
    endtask
    task pr_rd(input [7:0] s, output [31:0] v);
        begin
            wb_rd(32'h0C00_0000 + 32'h4 * s, 4'b1111, v);
        end
    endtask
    task en_wr(input [1:0] c, input [31:0] v);
        begin
            wb_wr(32'h0C00_2000 + 32'h80 * c, v, 4'b1111);
        end
    endtask
    task en_rd(input [1:0] c, output [31:0] v);
        begin
            wb_rd(32'h0C00_2000 + 32'h80 * c, 4'b1111, v);
        end
    endtask
    task thr_wr(input [1:0] c, input [31:0] v);
        begin
            wb_wr(32'h0C20_0000 + 32'h1000 * c, v, 4'b1111);
        end
    endtask
    task thr_rd(input [1:0] c, output [31:0] v);
        begin
            wb_rd(32'h0C20_0000 + 32'h1000 * c, 4'b1111, v);
        end
    endtask
    task claim_rd(input [1:0] c, output [31:0] v);
        begin
            wb_rd(32'h0C20_0004 + 32'h1000 * c, 4'b1111, v);
        end
    endtask
    task complete_wr(input [1:0] c, input [15:0] id);
        begin
            wb_wr(32'h0C20_0004 + 32'h1000 * c, {16'h0, id}, 4'b1111);
        end
    endtask
    task pend_rd(output [31:0] v);
        begin
            wb_rd(32'h0C00_1000, 4'b1111, v);
        end
    endtask

    task irq_rise(input [4:0] s);
        begin
            @(negedge clk);
            IRQ = IRQ | (17'h1 << s);
        end
    endtask
    task irq_fall(input [4:0] s);
        begin
            @(negedge clk);
            IRQ = IRQ & ~(17'h1 << s);
        end
    endtask
    task wait_cycs(input integer k);
        begin
            repeat (k) @(posedge clk);
        end
    endtask

    reg [31:0] w, v0;
    integer i;

    initial begin
        // reset
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (5) @(negedge clk);

        // ---- P1 reset state
        if (EIP !== 2'b00) err("P1: eip at reset");
        else               ok("P1: eip 0 at reset");
        pr_rd(1, w);  if (w !== 32'h0) err("P1: P1 != 0 at reset");
        pr_rd(16, w); if (w !== 32'h0) err("P1: P16 != 0 at reset");
        pend_rd(w);   if (w !== 32'h0) err("P1: pending != 0 at reset");
        en_rd(0, w);  if (w !== 32'h0) err("P1: EN0 != 0 at reset");
        en_rd(1, w);  if (w !== 32'h0) err("P1: EN1 != 0 at reset");
        thr_rd(0, w); if (w !== 32'h0) err("P1: THR0 != 0 at reset");
        thr_rd(1, w); if (w !== 32'h0) err("P1: THR1 != 0 at reset");
        claim_rd(0, w); if (w !== 32'h0) err("P1: CL0 != 0 at reset");
        claim_rd(1, w); if (w !== 32'h0) err("P1: CL1 != 0 at reset");
        ok("P1: reset state (regs 0)");

        // ---- P2 priority RW + reserved words
        pr_wr(1, 5); pr_wr(3, 5); pr_wr(16, 1);
        pr_rd(1, w);  if (w !== 32'h5)  err("P2: P1 rdata");
        pr_rd(3, w);  if (w !== 32'h5)  err("P2: P3 rdata");
        pr_rd(16, w); if (w !== 32'h1)  err("P2: P16 rdata");
        ok("P2: priority RW");
        pr_wr(0, 7);   pr_rd(0, w);  if (w !== 32'h0) err("P2: P0 reserved");
        pr_wr(17, 9);  pr_rd(17, w); if (w !== 32'h0) err("P2: P17 resv");
        ok("P2: priority reserved words 0/17");

        // ---- P3 level latch (no retraction)
        irq_rise(1);
        wait_cycs(1);
        pend_rd(w); if (w !== 32'h2) err("P3: pending bit1 not set");
        ok("P3: pending latched on assertion");
        irq_fall(1);
        wait_cycs(1);
        pend_rd(w); if (w !== 32'h2) err("P3: pending cleared by deassert");
        if (EIP !== 2'b00) err("P3: eip with en=0");
        ok("P3: latched, eip=0 (en=0)");

        // ---- P4 claim requires enable
        claim_rd(0, w); if (w !== 32'h0) err("P4: claim w/o enable");
        ok("P4: claim 0 without enable");
        en_wr(0, 32'h2);                // enable src1 (bit1)
        claim_rd(0, w); if (w !== 32'h1) err("P4: claim != 1");
        pend_rd(w);     if (w !== 32'h0) err("P4: pending not cleared");
        if (EIP !== 2'b00) err("P4: eip after claim");
        ok("P4: claim=1 clears pending");

        // ---- P5 eip + threshold (src1 prio5)
        irq_rise(1);                // level high again (deasserted in P3)
        complete_wr(0, 1);
        wait_cycs(2);
        pend_rd(w);   if (w !== 32'h2) err("P5: not repending");
        if (EIP !== 2'b01) err("P5: eip0 != 1");
        ok("P5: complete re-arms (level high)");
        thr_wr(0, 5);
        wait_cycs(1);
        if (EIP !== 2'b00) err("P5: prio<=thr not masked");
        thr_wr(0, 4);
        wait_cycs(1);
        if (EIP !== 2'b01) err("P5: prio>thr masked wrong");
        ok("P5: threshold masking");
        en_wr(1, 32'h2);                // ctx1: src1 (bit1)
        wait_cycs(1);
        if (EIP !== 2'b11) err("P5: ctx1 eip");
        thr_wr(0, 5);
        wait_cycs(1);
        if (EIP !== 2'b10) err("P5: thresholds not per-ctx");
        thr_wr(1, 4);
        wait_cycs(1);
        // thr0=5 still masks ctx0 (prio5 > thr5 false); ctx1: 5>4 -> fires
        if (EIP !== 2'b10) err("P5: thr1");
        ok("P5: per-ctx enable+threshold");

        // ---- P6 multicast: both notified, first claim wins
        thr_wr(0, 0);
        wait_cycs(1);
        if (EIP !== 2'b11) err("P6: eip both");
        claim_rd(1, w); if (w !== 32'h1) err("P6: ctx1 claim != 1");
        pend_rd(w);     if (w !== 32'h0) err("P6: pending not cleared");
        if (EIP !== 2'b00) err("P6: eip not cleared (shared IP)");
        claim_rd(0, w); if (w !== 32'h0) err("P6: ctx0 claim != 0");
        ok("P6: first claim wins (multicast)");
        complete_wr(1, 1);
        wait_cycs(2);
        if (EIP !== 2'b11) err("P6: not repending both");
        claim_rd(0, w); if (w !== 32'h1) err("P6: ctx0 claim != 1");
        complete_wr(0, 1);
        wait_cycs(2);
        if (EIP !== 2'b11) err("P6: eip both after");
        ok("P6: complete from either ctx re-arms");

        // ---- P7 priority order + tie-break (src 2,3,4)
        pr_wr(1, 0);                    // src1: never (still pending)
        pr_wr(2, 3); pr_wr(3, 5); pr_wr(4, 5);
        en_wr(0, 32'h0000_001E);       // srcs 1,2,3,4 (bits 1,2,3,4)
        irq_rise(2); irq_rise(3); irq_rise(4);
        wait_cycs(1);
        if (EIP !== 2'b01) err("P7: eip0");
        claim_rd(0, w); if (w !== 32'h3) err("P7: claim order 1 (tie->ID)");
        claim_rd(0, w); if (w !== 32'h4) err("P7: claim order 2");
        claim_rd(0, w); if (w !== 32'h2) err("P7: claim order 3");
        if (EIP !== 2'b00) err("P7: eip not drained (prio0)");
        irq_fall(2); irq_fall(3); irq_fall(4);
        ok("P7: prio desc, tie->low ID");

        // ---- P8 prio0=never + claim ignores threshold (src 5)
        pr_wr(5, 0);
        en_wr(0, 32'h0000_003E);       // srcs 1..5 (bits 1..5)
        irq_rise(5);
        wait_cycs(1);
        pend_rd(w); if (w !== 32'h22) err("P8: pending bits 1,5");
        if (EIP !== 2'b00) err("P8: prio0 eip");
        claim_rd(0, w); if (w !== 32'h0) err("P8: claim w/ prio0");
        ok("P8: prio0 = never interrupt");
        pr_wr(5, 7);
        wait_cycs(1);
        if (EIP !== 2'b01) err("P8: eip0");
        thr_wr(0, 100);
        wait_cycs(1);
        if (EIP !== 2'b00) err("P8: thr0 mask");
        claim_rd(0, w); if (w !== 32'h5) err("P8: claim < threshold");
        pend_rd(w); if (w !== 32'h2) err("P8: pending bit5");
        claim_rd(0, w); if (w !== 32'h0) err("P8: only prio0 left");
        ok("P8: claim not affected by threshold");

        // ---- P9 completion semantics (src 5)
        irq_fall(5);
        wait_cycs(1);
        irq_rise(5);
        wait_cycs(1);
        pend_rd(w); if ((w & 32'h20) !== 0) err("P9: no re-fwd w/o complete");
        if (EIP !== 2'b00) err("P9: eip w/o complete");
        ok("P9: no re-forward without complete");
        complete_wr(0, 5);
        wait_cycs(2);
        pend_rd(w); if ((w & 32'h20) === 0) err("P9: no re-pend");
        thr_wr(0, 0);
        wait_cycs(1);
        if (EIP !== 2'b01) err("P9: eip re-pend");
        claim_rd(0, w); if (w !== 32'h5) err("P9: claim 5");
        irq_fall(5);
        complete_wr(0, 5);
        wait_cycs(2);
        pend_rd(w); if ((w & 32'h20) !== 0) err("P9: re-pend, level low");
        if (EIP !== 2'b00) err("P9: eip level low");
        ok("P9: complete w/o level = no pending");
        complete_wr(0, 17);
        wait_cycs(2);
        pend_rd(w); if ((w & 32'h20) !== 0) err("P9: bad id re-arm");
        ok("P9: complete bad ID ignored");
        irq_rise(5);
        wait_cycs(1);
        pend_rd(w); if ((w & 32'h20) === 0) err("P9: first assert armed");
        claim_rd(0, w); if (w !== 32'h5) err("P9: claim 5 again");
        irq_fall(5);
        complete_wr(0, 5);
        wait_cycs(2);
        ok("P9: full gateway cycle");

        // ---- P10 reserved offsets (read 0 / write ignored)
        wb_rd(32'h0C00_1004, 4'b1111, w); if (w !== 32'h0) err("P10: penw1");
        wb_rd(32'h0C00_2004, 4'b1111, w); if (w !== 32'h0) err("P10: enw1");
        wb_rd(32'h0C00_2100, 4'b1111, w); if (w !== 32'h0) err("P10: en ctx2");
        en_rd(0, v0);
        wb_wr(32'h0C00_2100, 32'h1, 4'b1111);
        en_rd(0, w); if (w !== v0) err("P10: en ctx2 write");
        wb_rd(32'h0C20_0008, 4'b1111, w); if (w !== 32'h0) err("P10: thr8");
        wb_wr(32'h0C20_2000, 32'h7, 4'b1111);
        wb_rd(32'h0C20_2000, 4'b1111, w); if (w !== 32'h0) err("P10: thr ctx2");
        pend_rd(v0);
        wb_wr(32'h0C20_3004, 32'h1, 4'b1111);
        pend_rd(w); if (w !== v0) err("P10: cl ctx3 complete");
        ok("P10: reserved offsets inert");

        // ---- P11 word access (SEL ignored)
        wb_wr(32'h0C00_2080, 32'h10, 4'b0001);
        en_rd(1, w); if (w !== 32'h10) err("P11: SEL byte write");
        wb_rd(32'h0C00_2080, 4'b0001, w); if (w !== 32'h10) err("P11: SEL read");
        ok("P11: word access (SEL ignored)");

        // ---- P12 ACK timing: STB posedge -> ACK posedge = 1 cycle
        v0 = cyc;
        wb_rd(32'h0C00_1000, 4'b1111, w);
        if (cyc - v0 !== 1) err("P12: read ACK != 1 cycle");
        v0 = cyc;
        wb_wr(32'h0C00_2080, 32'h1, 4'b1111);
        if (cyc - v0 !== 1) err("P12: write ACK != 1 cycle");
        ok("P12: ACK 1 cycle after STB");

        // ---- P13 clean shutdown
        IRQ = 0;
        pr_wr(1, 0); pr_wr(2, 0); pr_wr(3, 0); pr_wr(4, 0); pr_wr(5, 0);
        en_wr(0, 0); en_wr(1, 0);
        thr_wr(0, 0); thr_wr(1, 0);
        wait_cycs(2);
        if (EIP !== 2'b00) err("P13: eip after shutdown");
        else               ok("P13: clean shutdown");

        // ---- summary
        $display("============================================");
        $display("TB PLIC: %0d passes, %0d errors (cyc=%0d)",
                 passes, errors, cyc);
        if (errors === 0) $display("ALL TESTS PASSED");
        $display("============================================");
        $finish;
    end

    // global timeout
    initial begin
        #500000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
