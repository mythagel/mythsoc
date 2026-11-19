// tb_clint -- directed testbench for clint (mythrv32)
`timescale 1ns/1ps
module tb_clint;

    reg clk = 1, rst = 0, STB = 0, WE = 0;
    reg [3:0] SEL = 0;
    reg [31:0] ADR = 0, DAT = 0;
    wire [31:0] DATO;
    wire ACK;
    wire ip_ssw, ip_msw, ip_stmr, ip_mtmr, ip_sext, ip_mext;

    clint dut (
        .clk(clk), .rst(rst),
        .STB_I(STB), .WE_I(WE), .SEL_I(SEL), .ADR_I(ADR), .DAT_I(DAT),
        .DAT_O(DATO), .ACK_O(ACK),
        .ip_ssw(ip_ssw), .ip_msw(ip_msw), .ip_stmr(ip_stmr),
        .ip_mtmr(ip_mtmr), .ip_sext(ip_sext), .ip_mext(ip_mext)
    );

    always #10 clk = ~clk;

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

    reg [31:0] w;
    reg [31:0] whi, wlo;
    reg [63:0] v64;

    // mtime snapshot: read hi then lo; value as of the lo-read clock edge
    task snap64;
        begin
            wb_rd(32'h0200_BFFC, 4'b1111, w); whi = w;
            wb_rd(32'h0200_BFF8, 4'b1111, w); wlo = w;
            v64 = {whi, wlo};
        end
    endtask

    reg [63:0] a64, b64, t64;
    integer i, c;

    initial begin
        // reset
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (5) @(negedge clk);

        // --- 1. reset state
        if (ip_ssw || ip_msw || ip_stmr || ip_mtmr || ip_sext || ip_mext)
            err("reset: ip asserted");
        else
            ok("reset: ip all low");
        wb_rd(32'h0200_4000, 4'b1111, w);
        if (w !== 32'hFFFF_FFFF) err("reset: mtimecmp lo != all1");
        else                     ok("reset: mtimecmp lo all1");
        wb_rd(32'h0200_4004, 4'b1111, w);
        if (w !== 32'hFFFF_FFFF) err("reset: 0x4004 read != all1");
        else                     ok("reset: 0x4004 read all1");
        wb_rd(32'h0200_4008, 4'b1111, w);
        if (w !== 32'hFFFF_FFFF) err("reset: stimecmp hi != all1");
        else                     ok("reset: stimecmp hi all1");

        // --- 2. mtime: +1/cycle, monotonic
        snap64; a64 = v64;
        snap64; b64 = v64;
        if (b64 - a64 !== 64'd2) err("mtime: back-to-back delta != 2");
        else                     ok("mtime: back-to-back delta 2");
        for (i = 0; i < 50; i = i + 1) @(posedge clk);
        snap64; b64 = v64;
        if (b64 - a64 !== 64'd54) err("mtime: 54-cycle delta wrong");
        else                     ok("mtime: 54-cycle delta");

        // --- 3. msip -> ip_msw
        if (ip_msw) err("msip: ip pre");
        wb_wr(32'h0200_0000, 32'h1, 4'b1111);
        if (!ip_msw) err("msip: ip_msw not set");
        else         ok("msip: ip_msw set");
        wb_rd(32'h0200_0000, 4'b1111, w);
        if (w !== 32'h1) err("msip: rdata");
        wb_wr(32'h0200_0000, 32'h3, 4'b1111);
        if (!ip_msw) err("msip: ip_msw dropped at 0x3");
        wb_wr(32'h0200_0000, 32'h2, 4'b1111);
        repeat (2) @(posedge clk);
        if (ip_msw) err("msip: ip_msw not clear");
        else       ok("msip: ip_msw clear");
        wb_wr(32'h0200_0000, 32'h0, 4'b1111);

        // --- 4. sstip -> ip_ssw
        wb_wr(32'h0200_0004, 32'h1, 4'b1111);
        if (!ip_ssw) err("sstip: ip_ssw not set");
        else         ok("sstip: ip_ssw set");
        wb_rd(32'h0200_0004, 4'b1111, w);
        if (w !== 32'h1) err("sstip: rdata");
        wb_wr(32'h0200_0004, 32'h0, 4'b1111);
        repeat (2) @(posedge clk);
        if (ip_ssw) err("sstip: ip_ssw not clear");
        else       ok("sstip: ip_ssw clear");

        // --- 5. mtip -> ip_sext & ip_mext
        wb_wr(32'h0200_7000, 32'h1, 4'b1111);
        if (!ip_sext || !ip_mext) err("mtip: sext/mext not set");
        else                      ok("mtip: sext+mext set");
        wb_rd(32'h0200_7000, 4'b1111, w);
        if (w !== 32'h1) err("mtip: rdata");
        wb_wr(32'h0200_7000, 32'h0, 4'b1111);
        repeat (2) @(posedge clk);
        if (ip_sext || ip_mext) err("mtip: sext/mext not clear");
        else                    ok("mtip: sext/mext clear");

        // --- 6. stimecmp: set to now+10, poll, sticky, clear
        snap64; t64 = v64 + 64'd10;
        wb_wr(32'h0200_4008, t64[63:32], 4'b1111);   // stimecmp hi
        wb_wr(32'h0200_4004, t64[31:0], 4'b1111);    // stc lo + mtc hi (overlap)
        wb_rd(32'h0200_4008, 4'b1111, w);
        if (w !== t64[63:32]) err("stc: hi rdata");
        else                  ok("stc: hi rdata");
        wb_rd(32'h0200_4004, 4'b1111, w);
        if (w !== t64[31:0]) err("stc: 0x4004 rdata (mtc hi)");
        else                 ok("stc: 0x4004 rdata (mtc hi)");
        c = 0;
        while (!ip_stmr && c < 200) begin @(posedge clk); c = c + 1; end
        if (c >= 200) err("stmr: never asserted");
        else          ok("stmr: asserted at now+10");
        repeat (10) @(posedge clk);
        if (!ip_stmr) err("stmr: not sticky");
        else          ok("stmr: sticky");
        // clear: stc = all1 (side effect: mtc hi = all1)
        wb_wr(32'h0200_4008, 32'hFFFF_FFFF, 4'b1111);
        wb_wr(32'h0200_4004, 32'hFFFF_FFFF, 4'b1111);
        repeat (4) @(posedge clk);
        if (ip_stmr) err("stmr: not cleared");
        else         ok("stmr: cleared");

        // --- 7. mtimecmp: set to now+20, poll, sticky, clear
        snap64; t64 = v64 + 64'd20;
        wb_wr(32'h0200_4000, t64[31:0], 4'b1111);   // mtc lo
        wb_wr(32'h0200_4004, t64[63:32], 4'b1111);  // mtc hi + stc lo (overlap)
        wb_rd(32'h0200_4000, 4'b1111, w);
        if (w !== t64[31:0]) err("mtc: lo rdata");
        else                 ok("mtc: lo rdata");
        c = 0;
        while (!ip_mtmr && c < 300) begin @(posedge clk); c = c + 1; end
        if (c >= 300) err("mtmr: never asserted");
        else          ok("mtmr: asserted at now+20");
        repeat (10) @(posedge clk);
        if (!ip_mtmr) err("mtmr: not sticky");
        else          ok("mtmr: sticky");
        // clear: mtc = all1 (side effect: stc lo = all1)
        wb_wr(32'h0200_4000, 32'hFFFF_FFFF, 4'b1111);
        wb_wr(32'h0200_4004, 32'hFFFF_FFFF, 4'b1111);
        repeat (4) @(posedge clk);
        if (ip_mtmr) err("mtmr: not cleared");
        else         ok("mtmr: cleared");

        // --- 8. cmp=0: mtime >= 0 always pending
        wb_wr(32'h0200_4000, 32'h0, 4'b1111);
        wb_wr(32'h0200_4004, 32'h0, 4'b1111);
        // mtc = 0, stc = {all1, 0} -> only mtmr pending
        repeat (2) @(posedge clk);
        if (ip_mtmr !== 1'b1 || ip_stmr !== 1'b0)
            err("cmp0: mtmr/stmr state");
        else
            ok("cmp0: mtmr=1 stmr=0");
        wb_wr(32'h0200_4008, 32'h0, 4'b1111);       // stc = 0
        repeat (2) @(posedge clk);
        if (!ip_mtmr || !ip_stmr) err("cmp0: both not pending");
        else                       ok("cmp0: both pending");
        // restore: both all1
        wb_wr(32'h0200_4008, 32'hFFFF_FFFF, 4'b1111);
        wb_wr(32'h0200_4004, 32'hFFFF_FFFF, 4'b1111);
        wb_wr(32'h0200_4000, 32'hFFFF_FFFF, 4'b1111);
        repeat (4) @(posedge clk);
        if (ip_mtmr || ip_stmr) err("cmp0: not cleared");
        else                    ok("cmp0: cleared");

        // --- 9. 0x4004 overlap: write hits mtc hi + stc lo, read = mtc hi
        wb_wr(32'h0200_4004, 32'hDEAD_BEEF, 4'b1111);
        wb_rd(32'h0200_4004, 4'b1111, w);
        if (w !== 32'hDEAD_BEEF)                    err("ovl: 0x4004 readback");
        else if (dut.stimecmp[31:0] !== 32'hDEAD_BEEF) err("ovl: stc lo side effect");
        else                                        ok("ovl: w->mtc hi+stc lo; rd mtc hi");
        // restore
        wb_wr(32'h0200_4008, 32'hFFFF_FFFF, 4'b1111);
        wb_wr(32'h0200_4004, 32'hFFFF_FFFF, 4'b1111);
        wb_wr(32'h0200_4000, 32'hFFFF_FFFF, 4'b1111);
        repeat (4) @(posedge clk);
        if (ip_mtmr || ip_stmr) err("ovl: restore failed");
        else                    ok("ovl: restore, ip low");

        // --- 10. mtime read-only (write ignored)
        snap64; a64 = v64;
        wb_wr(32'h0200_BFF8, 32'hDEAD_BEEF, 4'b1111);
        snap64; b64 = v64;
        if (b64 - a64 !== 64'd3) err("mtime: write disturbed");
        else                     ok("mtime: write ignored");

        // --- 11. LW/SW aligned + unaligned + byte lanes
        wb_wr(32'h0200_0000, 32'hA5C3_F00D, 4'b1111);
        wb_wr(32'h0200_0004, 32'h1122_3344, 4'b1111);
        wb_rd(32'h0200_0000, 4'b1111, w);
        if (w !== 32'hA5C3_F00D) err("lw/sw: msip rdata");
        else                     ok("lw/sw: msip round trip");
        wb_rd(32'h0200_0004, 4'b1111, w);
        if (w !== 32'h1122_3344) err("lw/sw: sstip rdata");
        else                     ok("lw/sw: sstip round trip");
        // unaligned LW at +1: {sstip[7:0], msip[31:8]}
        wb_rd(32'h0200_0001, 4'b1111, w);
        if (w !== 32'h44A5_C3F0) err("unaligned: LW rdata");
        else                     ok("unaligned: LW rdata");
        // unaligned SW at +1: msip <- {D[23:0] bytes, old byte0}, sstip[7:0] <- D[31:24]
        wb_wr(32'h0200_0001, 32'h0D0C_0B0A, 4'b1111);
        wb_rd(32'h0200_0000, 4'b1111, w);
        if (w !== 32'h0C0B_0A0D) err("unaligned: msip after SW");
        else                     ok("unaligned: msip after SW");
        wb_rd(32'h0200_0004, 4'b1111, w);
        if (w !== 32'h1122_330D) err("unaligned: sstip after SW");
        else                     ok("unaligned: sstip after SW");
        // byte-lane reads: data lands in lane b of the word (DAT[8b+7:8b])
        wb_rd(32'h0200_0000, 4'b0001, w); if (w[7:0] !== 8'h0D) err("lane: b0");
        wb_rd(32'h0200_0000, 4'b0010, w); if (w[15:8] !== 8'h0A) err("lane: b1");
        wb_rd(32'h0200_0000, 4'b0100, w); if (w[23:16] !== 8'h0B) err("lane: b2");
        wb_rd(32'h0200_0000, 4'b1000, w); if (w[31:24] !== 8'h0C) err("lane: b3");
        ok("lane: byte reads");
        // clear
        wb_wr(32'h0200_0000, 32'h0, 4'b1111);
        wb_wr(32'h0200_0004, 32'h0, 4'b1111);
        repeat (2) @(posedge clk);
        if (ip_msw || ip_ssw) err("lane: ip left high");
        else                  ok("lane: ip cleared");

        // --- 12. mtimecmp lo round trip, no spurious ip
        wb_wr(32'h0200_4000, 32'h1234_5678, 4'b1111);
        wb_rd(32'h0200_4000, 4'b1111, w);
        if (w !== 32'h1234_5678) err("mtc: lo round trip");
        else                     ok("mtc: lo round trip");
        if (ip_mtmr) err("mtc: ip_mtmr set spuriously");
        wb_wr(32'h0200_4000, 32'hFFFF_FFFF, 4'b1111);
        wb_wr(32'h0200_4004, 32'hFFFF_FFFF, 4'b1111);
        repeat (3) @(posedge clk);
        if (ip_mtmr || ip_stmr) err("mtc: not cleared");

        // --- 13. ACK timing + out-of-range
        @(negedge clk);
        STB = 1; WE = 0; SEL = 4'b1111; ADR = 32'h0200_0004; DAT = 0;
        @(posedge clk) #1;
        if (ACK !== 1'b1) err("ack: not set at T+1");
        STB = 0;
        @(posedge clk) #1;
        if (ACK !== 1'b0) err("ack: still set at T+2");
        @(negedge clk);
        STB = 1; WE = 0; SEL = 4'b1111; ADR = 32'h0000_0004; DAT = 0;
        @(posedge clk) #1;
        if (ACK !== 1'b0) err("ack: oob ACK at T+1");
        STB = 0;
        @(posedge clk) #1; @(posedge clk) #1;
        if (ACK !== 1'b0) err("ack: oob ACK late");
        else              ok("ack: timing + oob");

        // summary
        $display("============================================");
        $display("TB CLINT: %0d passes, %0d errors", passes, errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        $display("============================================");
        $finish;
    end

    initial begin
        #10_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
