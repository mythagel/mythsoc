// tb_l1_i -- directed testbench for l1_i (mythrv32)
//
// 4 KiB direct-mapped I-cache, 1-word lines, WB B4 RO master.
// Latency (cycles from REQ cycle to DONE cycle), 1-cyc slave:
//   hit = 1, miss = 3.  STB cycles: hit = 0, miss = 1.
// RAM pattern: ram[w] = w (word index), so expected DATA = ADR[31:2].
`timescale 1ns/1ps
module tb_l1_i;

    reg clk = 1, rst = 0;
    reg REQ = 0;
    reg [31:0] PA_I = 0;
    reg INVAL = 0;
    wire DONE, STB, WE;
    wire [3:0] SEL;
    wire [31:0] ADRW, DATAO;
    reg ACKR = 0;
    reg [31:0] DATO = 0;

    l1_i dut (
        .clk(clk), .rst(rst),
        .REQ_I(REQ), .PA_I(PA_I), .INVAL_I(INVAL),
        .DONE_O(DONE), .DATA_O(DATAO),
        .STB_O(STB), .SEL_O(SEL), .ADR_O(ADRW), .WE_O(WE),
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
    // index = A[17:2]: max addr 0x1FFFF -> word 0x7FFFF not used; 65536 words.
    reg [31:0] ram [0:65535];
    integer DELAY;
    reg [31:0] paddr;
    integer pbsy, pcnt;
    always @(posedge clk) begin
        if (STB) begin
            if (DELAY == 1) begin
                ACKR <= 1'b1;
                DATO <= ram[ADRW[17:2]];
            end else begin
                paddr <= ADRW;
                pbsy <= 1;
                pcnt <= DELAY - 1;
            end
        end else if (pbsy) begin
            if (pcnt == 1) begin
                pbsy <= 0;
                ACKR <= 1'b1;
                DATO <= ram[paddr[17:2]];
            end else begin
                pcnt <= pcnt - 1;
            end
        end else begin
            ACKR <= 1'b0;
        end
    end
    integer stb_cnt = 0, ack_cnt = 0, stb_tot = 0;
    always @(posedge clk) begin
        if (STB) begin
            stb_cnt = stb_cnt + 1;
            stb_tot = stb_tot + 1;
            rlog[rc] = ADRW;
            rc = rc + 1;
        end
        if (ACKR) ack_cnt = ack_cnt + 1;
    end
    integer rc, ri;
    reg [31:0] rlog [0:255];

    task ram_set(input [31:0] a, input [31:0] d);
        begin
            ram[a[17:2]] = d;
        end
    endtask

    // ---- request: pulse REQ, wait DONE; record result
    reg        q_done;
    reg [31:0] q_data;
    reg [8:0]  q_lat, q_stb;
    integer    lat0, nd;

    task ic_req(input [31:0] pa);
        begin
            q_done = 1'b0;
            @(negedge clk);
            while (DONE) @(negedge clk);   // ensure idle
            REQ = 1; PA_I = pa;
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

    // ---- inval pulse (idle only)
    task inval();
        begin
            @(negedge clk);
            INVAL = 1;
            @(posedge clk) #1;
            INVAL = 0;
        end
    endtask

    initial begin
        DELAY = 1; pbsy = 0; pcnt = 0; rc = 0;
        for (ri = 0; ri < 65536; ri = ri + 1) ram[ri] = ri[31:0];
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        // ---- I1 miss: REQ 0x1004 -> DATA = word idx 0x401
        ic_req(32'h00001004);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            q_data === 32'h00000401 && rc === 1 && rlog[0] === 32'h00001004 &&
            WE === 1'b0)
            ok("I1: miss lat3 data");
        else
            err("I1: miss lat3 data");

        // ---- I2 same PA hit
        ic_req(32'h00001004);
        if (q_done && q_lat === 9'd1 && q_stb === 9'd0 &&
            q_data === 32'h00000401)
            ok("I2: hit lat1");
        else
            err("I2: hit lat1");

        // ---- I3 eviction: same index (0x401), tag 0x5004
        ic_req(32'h00005004);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            q_data === 32'h00001401)
            ok("I3: miss new tag");
        else
            err("I3: miss new tag");
        ic_req(32'h00001004);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            q_data === 32'h00000401)
            ok("I3: old tag evicted");
        else
            err("I3: old tag evicted");
        ic_req(32'h00005004);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            q_data === 32'h00001401)
            ok("I3: 0x1004 line refilled");
        else
            err("I3: 0x1004 line refilled");
        ic_req(32'h00005004);
        if (q_done && q_lat === 9'd1 && q_stb === 9'd0 &&
            q_data === 32'h00001401)
            ok("I3: hit after refill");
        else
            err("I3: hit after refill");

        // ---- I4 INVAL: hit -> inval -> miss
        inval();
        ic_req(32'h00005004);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            q_data === 32'h00001401)
            ok("I4: miss after inval");
        else
            err("I4: miss after inval");

        // ---- I5 memory update picked up on refill
        ram_set(32'h00001004, 32'hDEADBEEF);
        inval();
        ic_req(32'h00001004);
        if (q_done && q_lat === 9'd3 && q_data === 32'hDEADBEEF)
            ok("I5: refill reads mem");
        else
            err("I5: refill reads mem");
        ic_req(32'h00001004);
        if (q_done && q_lat === 9'd1 && q_data === 32'hDEADBEEF)
            ok("I5: hit serves new data");
        else
            err("I5: hit serves new data");

        // ---- I6 distinct lines (different idx) independent
        ic_req(32'h00002008);
        if (q_done && q_lat === 9'd3 && q_data === 32'h00000802)
            ok("I6: line A miss");
        else
            err("I6: line A miss");
        ic_req(32'h0000300C);
        if (q_done && q_lat === 9'd3 && q_data === 32'h00000C03)
            ok("I6: line B miss");
        else
            err("I6: line B miss");
        ic_req(32'h00002008);
        if (q_done && q_lat === 9'd1 && q_data === 32'h00000802)
            ok("I6: line A hit");
        else
            err("I6: line A hit");
        ic_req(32'h0000300C);
        if (q_done && q_lat === 9'd1 && q_data === 32'h00000C03)
            ok("I6: line B hit");
        else
            err("I6: line B hit");

        // ---- I7 3-cyc-equivalent slave (DELAY=2): miss lat 4
        DELAY = 2;
        ic_req(32'h00006010);
        if (q_done && q_lat === 9'd4 && q_stb === 9'd1 &&
            q_data === 32'h00001804)
            ok("I7: miss slow slave");
        else
            err("I7: miss slow slave");
        DELAY = 1;
        ic_req(32'h00006010);
        if (q_done && q_lat === 9'd1 && q_data === 32'h00001804)
            ok("I7: hit after slow miss");
        else
            err("I7: hit after slow miss");

        // ---- I8 reset mid-miss
        @(negedge clk);
        REQ = 1; PA_I = 32'h00009014;
        @(posedge clk); #1;
        REQ = 0;                    // now mid-miss (W1S/W1W)
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);
        ic_req(32'h00009014);
        if (q_done && q_lat === 9'd3 && q_stb === 9'd1 &&
            q_data === 32'h00002405)
            ok("I8: works after rst mid-miss");
        else
            err("I8: works after rst mid-miss");

        // ---- I9 back-to-back: 3 misses then 3 hits
        ic_req(32'h0000A018);
        if (q_done && q_lat === 9'd3 && q_data === 32'h00002806)
            ok("I9: bb miss 1");
        else
            err("I9: bb miss 1");
        ic_req(32'h0000B01C);
        if (q_done && q_lat === 9'd3 && q_data === 32'h00002C07)
            ok("I9: bb miss 2");
        else
            err("I9: bb miss 2");
        ic_req(32'h0000C020);
        if (q_done && q_lat === 9'd3 && q_data === 32'h00003008)
            ok("I9: bb miss 3");
        else
            err("I9: bb miss 3");
        ic_req(32'h0000A018);
        if (q_done && q_lat === 9'd1 && q_data === 32'h00002806)
            ok("I9: bb hit 1");
        else
            err("I9: bb hit 1");
        ic_req(32'h0000B01C);
        if (q_done && q_lat === 9'd1 && q_data === 32'h00002C07)
            ok("I9: bb hit 2");
        else
            err("I9: bb hit 2");
        ic_req(32'h0000C020);
        if (q_done && q_lat === 9'd1 && q_data === 32'h00003008)
            ok("I9: bb hit 3");
        else
            err("I9: bb hit 3");

        // ---- I10 protocol balance: every STB got exactly one ACK
        if (stb_tot === ack_cnt && stb_tot > 0 && stb_cnt === 0)
            ok("I10: stb/ack balanced");
        else
            err("I10: stb/ack balanced");

        // ---- I11 sustained hit throughput (1.0 hit/cyc): back-to-back REQ
        // on consecutive cycles to two resident lines (distinct sets) ->
        // DONE every cycle, no WB. (Hit path pulses DONE and stays IDLE.)
        inval();
        ic_req(32'h00004000);   // fill set 0, data word 0x1000
        ic_req(32'h00004004);   // fill set 1, data word 0x1001
        stb_tot = 0; ack_cnt = 0; stb_cnt = 0; rc = 0; // clear counters
        @(negedge clk);
        while (DONE) @(negedge clk);
        REQ = 1; PA_I = 32'h00004000;   // request A
        @(posedge clk); #1;              // -> result A
        PA_I = 32'h00004004;             // request B (next cycle)
        if (DONE && DATAO === 32'h00001000) ok("I11: b2b hit A"); else err("I11: b2b hit A");
        @(posedge clk); #1;              // -> result B
        if (DONE && DATAO === 32'h00001001) ok("I11: b2b hit B"); else err("I11: b2b hit B");
        @(posedge clk); #1;              // -> result C (REQ held on cycle C)
        REQ = 0;
        if (DONE) ok("I11: b2b hit C"); else err("I11: b2b hit C");
        if (rc === 0) ok("I11: b2b no WB"); else err("I11: b2b no WB");
        if (stb_cnt === 0) ok("I11: b2b no stb"); else err("I11: b2b no stb");

        // ---- summary
        $display("============================================");
        $display("TB L1I: %0d passes, %0d errors (cyc=%0d)",
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
