// tb_l1_d -- directed testbench for l1_d (mythrv32)
//
// 4 KiB direct-mapped D-cache, 1-word lines, WB B4 RW master,
// write-through, no store-allocate.
// Latency (cycles from REQ cycle to DONE cycle), 1-cyc slave:
//   load hit = 1, load miss = 3, store (hit/miss) = 3.
// RAM pattern: ram[w] = w (word index), so memory data = ADR[17:2].
`timescale 1ns/1ps
module tb_l1_d;

    reg clk = 1, rst = 0;
    reg REQ = 0;
    reg [31:0] PA_I = 0;
    reg WEN = 0;
    reg [31:0] WDI = 0;
    wire DONE, STB, WE;
    wire [3:0] SEL;
    wire [31:0] ADRW, DATAO, WDATO;
    reg ACKR = 0;
    reg [31:0] DATO = 0;

    l1_d dut (
        .clk(clk), .rst(rst),
        .REQ_I(REQ), .PA_I(PA_I), .WEN_I(WEN), .WDAT_I(WDI),
        .DONE_O(DONE), .DATA_O(DATAO),
        .STB_O(STB), .SEL_O(SEL), .ADR_O(ADRW), .WE_O(WE),
        .WDAT_O(WDATO),
        .ACK_I(ACKR), .DAT_I(DATO)
    );

    always #10 clk = ~clk;
    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    integer errors = 0, passes = 0;
    // NOTE: label must be <= 32 chars (iverilog %s on [255:0] prints 32).
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

    // ---- WB RAM slave (word array; DELAY = ACK cycles after STB edge)
    reg [31:0] ram [0:65535];
    integer DELAY = 1;
    reg [31:0] paddr;
    reg        pw;
    reg [31:0] pwd;
    integer pbsy = 0, pcnt = 0;
    always @(posedge clk) begin
        if (STB) begin
            if (WE) ram[ADRW[17:2]] <= WDATO;
            else    DATO <= ram[ADRW[17:2]];
            if (DELAY == 1) begin
                ACKR <= 1'b1;
            end else begin
                paddr <= ADRW;
                pw <= WE;
                pwd <= WDATO;
                pbsy <= 1;
                pcnt <= DELAY - 1;
            end
        end else if (pbsy) begin
            if (pcnt == 1) begin
                pbsy <= 0;
                ACKR <= 1'b1;
                if (pw) ram[paddr[17:2]] <= pwd;
                else    DATO <= ram[paddr[17:2]];
            end else begin
                pcnt <= pcnt - 1;
            end
        end else begin
            ACKR <= 1'b0;
        end
    end
    integer stb_cnt = 0, ack_cnt = 0, stb_tot = 0;
    integer rc, ri;
    reg [31:0] rlog [0:255];
    reg [31:0] wlog [0:255];
    reg        wflag [0:255];
    always @(posedge clk) begin
        if (STB) begin
            stb_cnt = stb_cnt + 1;
            stb_tot = stb_tot + 1;
            rlog[rc] = ADRW;
            wlog[rc] = WDATO;
            wflag[rc] = WE;
            rc = rc + 1;
        end
        if (ACKR) ack_cnt = ack_cnt + 1;
    end

    // ---- load/store: pulse REQ, wait DONE; record result
    reg        q_done;
    reg [31:0] q_data;
    reg [8:0]  q_lat, q_stb;
    integer    lat0, nd;

    task dc_req(input [31:0] pa, input wen, input [31:0] wd);
        begin
            q_done = 1'b0;
            @(negedge clk);
            while (DONE) @(negedge clk);   // ensure idle
            REQ = 1; PA_I = pa; WEN = wen; WDI = wd;
            lat0 = cyc;
            nd = 0;
            while (!DONE) begin
                @(posedge clk); #1;
                REQ = 0;
                nd = nd + 1;
            end
            q_done = DONE;
            q_data = DATAO;
            q_lat = cyc - lat0;
            q_stb = stb_cnt;
            stb_cnt = 0;
        end
    endtask

    initial begin
        DELAY = 1; pbsy = 0; pcnt = 0; rc = 0;
        for (ri = 0; ri < 65536; ri = ri + 1) ram[ri] = ri[31:0];
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        // ---- D1 load miss: 0x1004 -> DATA = word 0x401
        dc_req(32'h00001004, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            q_data === 32'h00000401 && rc === 1 &&
            rlog[0] === 32'h00001004 && wflag[0] === 1'b0)
            ok("D1: load miss lat3");
        else
            err("D1: load miss lat3");

        // ---- D2 load hit
        dc_req(32'h00001004, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd1 && q_stb === 9'd0 &&
            q_data === 32'h00000401)
            ok("D2: load hit lat1");
        else
            err("D2: load hit lat1");

        // ---- D3 store miss: 0x1008, no-allocate
        dc_req(32'h00001008, 1'b1, 32'hDEAD0001);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            rlog[1] === 32'h00001008 && wflag[1] === 1'b1 &&
            wlog[1] === 32'hDEAD0001 &&
            ram[16'h402] === 32'hDEAD0001)
            ok("D3: store miss mem wr");
        else
            err("D3: store miss mem wr");
        dc_req(32'h00001008, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            q_data === 32'hDEAD0001)
            ok("D3: no-allocate (miss)");
        else
            err("D3: no-allocate (miss)");

        // ---- D4 store hit: 0x1004 (filled by D1/D2)
        dc_req(32'h00001004, 1'b1, 32'hBEEF0002);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            ram[16'h401] === 32'hBEEF0002)
            ok("D4: store hit wt mem");
        else
            err("D4: store hit wt mem");
        dc_req(32'h00001004, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd1 && q_stb === 9'd0 &&
            q_data === 32'hBEEF0002)
            ok("D4: line updated hit");
        else
            err("D4: line updated hit");

        // ---- D6 distinct lines (different idx) independent
        dc_req(32'h00008010, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd3 && q_data === 32'h00002004)
            ok("D6: line A miss");
        else
            err("D6: line A miss");
        dc_req(32'h00009014, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd3 && q_data === 32'h00002405)
            ok("D6: line B miss");
        else
            err("D6: line B miss");
        dc_req(32'h00008010, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd1 && q_data === 32'h00002004)
            ok("D6: line A hit");
        else
            err("D6: line A hit");
        dc_req(32'h00009014, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd1 && q_data === 32'h00002405)
            ok("D6: line B hit");
        else
            err("D6: line B hit");

        // ---- D7 DELAY=2: load miss lat4, store miss lat4
        DELAY = 2;
        dc_req(32'h0000A018, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd4 && q_stb === 9'd1 &&
            q_data === 32'h00002806)
            ok("D7: load miss slow");
        else
            err("D7: load miss slow");
        dc_req(32'h0000B01C, 1'b1, 32'hCAFE0003);
        if (q_done && q_lat === 9'd4 && q_stb === 9'd1 &&
            ram[16'h2C07] === 32'hCAFE0003)
            ok("D7: store miss slow");
        else
            err("D7: store miss slow");
        DELAY = 1;
        dc_req(32'h0000B01C, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd3 && q_data === 32'hCAFE0003)
            ok("D7: mem value on load");
        else
            err("D7: mem value on load");

        // ---- D8 reset mid-miss
        @(negedge clk);
        REQ = 1; PA_I = 32'h0000C020; WEN = 0; WDI = 0;
        @(posedge clk); #1;
        REQ = 0;                    // now mid-miss (W1S/W1W)
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);
        dc_req(32'h0000C020, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            q_data === 32'h00003008)
            ok("D8: works after rst mid-miss");
        else
            err("D8: works after rst mid-miss");

        // ---- D9 back-to-back: misses, store hit, hits
        dc_req(32'h0000D024, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd3 && q_data === 32'h00003409)
            ok("D9: miss A");
        else
            err("D9: miss A");
        dc_req(32'h0000E028, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd3 && q_data === 32'h0000380A)
            ok("D9: miss B");
        else
            err("D9: miss B");
        dc_req(32'h0000D024, 1'b1, 32'h12345678);
        if (q_done && q_lat === 9'd3 && ram[16'h3409] === 32'h12345678)
            ok("D9: store hit A");
        else
            err("D9: store hit A");
        dc_req(32'h0000D024, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd1 && q_data === 32'h12345678)
            ok("D9: hit A data");
        else
            err("D9: hit A data");
        dc_req(32'h0000E028, 1'b0, 32'h0);
        if (q_done && q_lat === 9'd1 && q_data === 32'h0000380A)
            ok("D9: hit B intact");
        else
            err("D9: hit B intact");

        // ---- D10 protocol balance: every STB got exactly one ACK
        if (stb_tot === ack_cnt && stb_tot > 0 && stb_cnt === 0)
            ok("D10: stb/ack balanced");
        else
            err("D10: stb/ack balanced");

        $display("============================================");
        $display("TB L1D: %0d passes, %0d errors (cyc=%0d)",
                 passes, errors, cyc);
        if (errors === 0) $display("ALL TESTS PASSED");
        $display("============================================");
        $finish;
    end

    // global timeout
    initial begin
        #1000000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
