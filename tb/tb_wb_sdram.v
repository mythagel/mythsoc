// tb_wb_sdram -- pipelined SDRAM controller + M12L64322A(2S) device model.
//
// The device model lives in tb/sdram_model.v (shared with tb_soc_sdram.v,
// the SoC-level board rehearsal bench). It INDEPENDENTLY checks every timing
// rule (tRCD/tRP/tRC/tRAS/tRRD/tRFC/tMRD, open-row requirements, write-data
// and read-launch cycle placement, DQ contention, refresh interval +
// all-banks-idle) so the RTL cannot silently drift. Read launch rule:
// command decoded at the edge closing its visible cycle c; D0 driven on
// edge c+CL (i.e. valid DURING cycle c+CL, captured by the RTL FF at the
// edge closing c+CL -- decision d => visible c=d+1 => capture at d+CL+2).
// WB side checks protocol (1-cycle ACK, <=1 outstanding). Mirror validates
// data.
//
// groups: S0 param cross-check (vs tools/sdram_calc.py --vh header)
//         S1 init sequence observation (PREA + MRS only)
//         S2 single cold write/read + latency bound
//         S3 row-hit streams (16 wr, 16 rd, back-to-back per wb_ic cadence)
//         S4 4-bank interleave (open pages, no re-ACT)
//         S5 bank-boundary + same-bank row-change streams
//         S6 byte RMW (hit + cold paths, all sel patterns)
//         S7 R<->W turnaround sequences + alternating stream
//         S8 refresh: interval observed, rows closed, data survives
//         S9 random stress 400 ops vs mirror + readback
//         S10 reset mid-flight -> recovery
// build+run: make sdram
`default_nettype none
`timescale 1ns / 1ps

module tb_wb_sdram;
    `include "tb/sdram_params.vh"

    localparam integer CLK_FREQ = 25000000;
    localparam integer T_INIT = `SDRAM_T_INIT;
    localparam integer T_RCD = `SDRAM_T_RCD, T_RP = `SDRAM_T_RP,
                     T_RC = `SDRAM_T_RC, T_RAS = `SDRAM_T_RAS,
                     T_RRD = `SDRAM_T_RRD, T_RFC = `SDRAM_T_RFC,
                     T_MRD = `SDRAM_T_MRD, T_REF = `SDRAM_T_REF;
    localparam integer REF_GRACE = 96;
    localparam integer T_REF_LIMIT = T_REF + REF_GRACE;

    integer npass = 0, nfail = 0;

    // ------------------------------------------------------------ DUT wires
    reg  clk = 0;
    always #20 clk = ~clk;              // 25 MHz

    reg         rst = 1;
    reg         stb = 0, we = 0;
    reg  [3:0]  sel = 0;
    reg  [31:0] adr = 0, wd = 0;
    wire [31:0] rd;
    wire        ack;
    wire [1:0]  ba;
    wire [12:0] sa;
    wire        ras_n, cas_n, we_n, cs_n, cke, dq_oe;
    wire [3:0]  dqm;
    wire [31:0] dq_o;
    wire [31:0] dq;

    wb_sdram #(.CLK_FREQ(CLK_FREQ)) dut (
        .clk(clk), .rst(rst),
        .STB_I(stb), .WE_I(we), .SEL_I(sel), .ADR_I(adr), .DAT_I(wd),
        .DAT_O(rd), .ACK_O(ack),
        .sd_ba_o(ba), .sd_a_o(sa), .sd_ras_n_o(ras_n), .sd_cas_n_o(cas_n),
        .sd_we_n_o(we_n), .sd_cs_n_o(cs_n), .sd_cke_o(cke), .sd_dqm_o(dqm),
        .sd_dq_o(dq_o), .sd_dq_oe_o(dq_oe), .sd_dq_i(dq)
    );

    sdram_model mdl (
        .clk(clk), .rst(rst),
        .ba(ba), .sa(sa), .ras_n(ras_n), .cas_n(cas_n), .we_n(we_n),
        .cs_n(cs_n), .cke(cke), .dqm(dqm), .dq_oe(dq_oe), .dq_o(dq_o),
        .dq(dq)
    );

    // model views (S1/S3/S4/S8 checks + final tally)
    wire [31:0] mcyc       = mdl.mcyc;
    wire [31:0] ref_total  = mdl.ref_total;
    wire [31:0] m_act_any  = mdl.m_act_any;
    wire        m_mrs_done = mdl.m_mrs_done;
    wire [2:0]  m_bl       = mdl.m_bl;
    wire [3:0]  m_cl       = mdl.m_cl;
    wire [31:0] mfail      = mdl.nfail;

    integer i;      // (j,k/t0/rcnt declared with the stimulus)

    // WB protocol monitor
    integer wb_out = 0, n_ack = 0;
    reg     ack_seen = 0;
    always @(posedge clk) begin
        if (rst) wb_out = 0;
        else begin
            if (ack) begin
                if (wb_out == 0) begin
                    nfail = nfail + 1;
                    $display("FAIL WB: ACK with no outstanding transfer, cyc %0d", mcyc);
                end
                wb_out = wb_out - 1;
                n_ack = n_ack + 1;
                ack_seen = 1;
            end
            if (stb) wb_out = wb_out + 1;
            if (wb_out > 1) begin
                nfail = nfail + 1;
                $display("FAIL WB: >1 outstanding at cyc %0d", mcyc);
                wb_out = 1;
            end
        end
    end

    // ------------------------------------------------------------ helpers
    task chk(input cond, input [319:0] name);
    begin
        if (cond) npass = npass + 1;
        else begin
            nfail = nfail + 1;
            $display("FAIL @%0t: %0s", $time, name);
        end
    end endtask

    // one WB transfer, waits ack
    task wb_wr(input [19:0] wad, input [3:0] s, input [31:0] d);
    begin
        @(negedge clk); stb=1; we=1; sel=s; adr={wad,2'b00}; wd=d;
        @(negedge clk); stb=0; we=0;
        while (!ack_seen) @(negedge clk);
        ack_seen = 0;
    end endtask

    task wb_rd(input [19:0] wad, output [31:0] d, output [31:0] lat);
    integer t0;
    begin
        t0 = mcyc;
        @(negedge clk); stb=1; we=0; sel=4'hF; adr={wad,2'b00};
        @(negedge clk); stb=0;
        while (!ack_seen) @(negedge clk);
        ack_seen = 0; d = rd; lat = mcyc - t0;
    end endtask

    // quiesce: queue/ring/inflight empty AND the next REF is at least half
    // a window away (keeps the timing-gated groups deterministic)
    task wait_quiet;
    integer p; reg pend;
    begin
        @(negedge clk);
        pend = 1;
        while (pend || dut.ref_due || dut.ref_sent ||
               dut.ref_cnt < (T_REF/2)) begin
            pend = dut.q0_v || dut.q1_v || dut.rd_v || dut.pa_v ||
                   dut.rwm_w || dut.wa1;
            for (p=0; p<3; p=p+1)
                if (dut.sr_v[p] && !dut.sr_rdy[p]) pend = 1;
            @(negedge clk);
        end
    end endtask

    // wait for the next completed REF event
    task wait_for_ref;
    integer rt;
    begin
        rt = ref_total;
        @(negedge clk);
        while (ref_total == rt) @(negedge clk);
    end endtask

    // ------------------------------------------------------------- stimulus
    reg [31:0] d;
    reg [31:0] lat;
    integer j, k;
    integer t0;
    integer rcnt;

    initial begin : MAIN
        #100; rst = 0;

        // ===== S0: RTL params == generated header (sdram_calc.py cross-check)
        chk(dut.CL     === `SDRAM_CL,    "S0 CL");
        chk(dut.T_RCD  === T_RCD,        "S0 T_RCD");
        chk(dut.T_RP   === T_RP,         "S0 T_RP");
        chk(dut.T_RC   === T_RC,         "S0 T_RC");
        chk(dut.T_RAS  === T_RAS,        "S0 T_RAS");
        chk(dut.T_RRD  === T_RRD,        "S0 T_RRD");
        chk(dut.T_RFC  === T_RFC,        "S0 T_RFC");
        chk(dut.T_MRD  === T_MRD,        "S0 T_MRD");
        chk(dut.T_REF  === T_REF,        "S0 T_REF");
        chk(dut.T_INIT === T_INIT,       "S0 T_INIT");

        // ===== S1: init = 200us NOP, then PREA + MRS, nothing else
        #(T_INIT*40 + 1000);
        chk(m_mrs_done,                  "S1 MRS seen");
        chk(m_bl === 3'd1,               "S1 BL=1");
        chk(m_cl === 4'd2,               "S1 CL=2");
        chk(mdl.m_act[0]==-100000 && mdl.m_act[3]==-100000, "S1 no ACT in init");

        wait_quiet;

        // ===== S2: single cold write + cold read
        wb_wr(20'd100, 4'hF, 32'hDEADBEEF);
        wb_rd(20'd100, d, lat);
        chk(d === 32'hDEADBEEF,          "S2 rd data");
        $display("S2 cold read latency = %0d cyc", lat);
        chk(lat <= T_RCD + T_RFC + 24,   "S2 rd latency");
        wb_wr(20'd100, 4'hF, 32'h01020304);
        wb_rd(20'd100, d, lat);
        chk(d === 32'h01020304,          "S2 rewrite");

        wait_quiet;

        // ===== S3: streams (STB pulses at the wb_ic cadence: next request
        // asserted at the ack cycle's negedge -> sampled ack+2)
        // Pre-touch the stream row/bank so the timed loop starts hot; the
        // gate allows 8 cyc per refresh that lands inside the window
        // (drain PREs+REF+tRFC, unavoidable at the 390-cyc period).
        wb_wr(600, 4'hF, 32'h5EED0000);
        wait_quiet;
        rcnt = ref_total;
        t0 = mcyc;
        @(negedge clk);
        for (j=0; j<16; j=j+1) begin
            stb=1; we=1; sel=4'hF;
            adr={20'd512+j, 2'b00}; wd=32'hA0000000+j;
            @(negedge clk); stb=0; we=0;
            while (!ack_seen) @(negedge clk);
            ack_seen = 0;
        end
        stb=0; we=0;
        $display("S3 stream: 16 posted writes in %0d cyc", mcyc - t0);
        chk(mcyc - t0 <= 36 + 8*(ref_total - rcnt), "S3 wr 1/2cyc");

        wait_quiet;

        rcnt = ref_total;
        t0 = mcyc;
        @(negedge clk);
        for (j=0; j<16; j=j+1) begin
            stb=1; we=0; sel=4'hF; adr={20'd512+j, 2'b00};
            @(negedge clk); stb=0;
            while (!ack_seen) @(negedge clk);
            ack_seen = 0;
            if (rd !== 32'hA0000000 + j) begin
                nfail=nfail+1;
                $display("FAIL S3 stream rd %0d = %h", j, rd);
            end
        end
        stb=0;
        $display("S3 stream: 16 prefetch reads in %0d cyc", mcyc - t0);
        chk(mcyc - t0 <= 44 + 8*(ref_total - rcnt), "S3 rd stream");
        npass = npass + 1;               // S3 rd data loop counted once

        // ===== S4: 4-bank interleave, rows stay open
        for (j=0; j<4; j=j+1) wb_wr(1024 + j*256, 4'hF, 32'hB0000000+j);
        for (j=0; j<4; j=j+1) begin
            wb_rd(1024 + j*256, d, lat);
            chk(d === 32'hB0000000+j,    "S4 bank data");
        end
        t0 = m_act_any;                  // snapshot ACT stamp (visible cyc)
        for (j=0; j<4; j=j+1) wb_rd(1024 + j*256, d, lat);
        chk(m_act_any === t0,            "S4 no re-ACT");

        // ===== S5: bank-boundary stream (250..261: bank0 row0 -> bank1)
        for (j=250; j<262; j=j+1) wb_wr(j, 4'hF, 32'hC0000000+j);
        k = 1;
        for (j=250; j<262; j=j+1) begin
            wb_rd(j, d, lat);
            if (d !== 32'hC0000000+j) begin
                k=0; $display("FAIL S5 word %0d = %h", j, d);
            end
        end
        chk(k, "S5 bank-cross data");
        // same-bank row change (bank2: word 4608=row4, 5632=row5, same row
        // bits? row = w>>10: 4608>>10=4, 5632>>10=5, bank=(w>>8)&3: 0,0)
        wb_wr(4608, 4'hF, 32'h55AA55AA);
        wb_wr(5632, 4'hF, 32'hAA55AA55);
        wb_rd(4608, d, lat);  chk(d === 32'h55AA55AA, "S5 row-chg old");
        wb_rd(5632, d, lat);  chk(d === 32'hAA55AA55, "S5 row-chg new");

        // ===== S6: byte RMW all patterns (row-open) + cold RMW
        wb_wr(300, 4'hF, 32'h00000000);
        wb_wr(300, 4'h1, 32'h000000AA);
        wb_wr(300, 4'h2, 32'h0000BB00);
        wb_wr(300, 4'h4, 32'h00CC0000);
        wb_wr(300, 4'h8, 32'hDD000000);
        wb_rd(300, d, lat);
        chk(d === 32'hDDCCBBAA,          "S6 RMW lane merge");
        wb_wr(301, 4'hF, 32'hFF00FF00);
        wb_wr(301, 4'h6, 32'h11223344);   // sel=0110 -> bytes 1,2 replaced
        wb_rd(301, d, lat);
        chk(d === 32'hFF223300,          "S6 RMW half pair");
        wb_wr(9000, 4'h9, 32'h12345678);  // cold-row RMW: sel 1001 -> bytes
        wb_rd(9000, d, lat);              // 0,3 kept: 0x12000078
        chk(d === 32'h12000078,          "S6 cold RMW");

        // ===== S7: turnaround sequences
        wb_wr(700, 4'hF, 32'h11111111);
        wb_wr(701, 4'hF, 32'h22222222);
        wb_rd(700, d, lat); chk(d === 32'h11111111, "S7 R after W");
        wb_wr(700, 4'hF, d + 32'h1);
        wb_rd(700, d, lat); chk(d === 32'h11111112, "S7 RW interleave");
        // alternating wr/rd same row, back-to-back (max bus pressure)
        wait_quiet;
        @(negedge clk);
        k = 1;
        for (j=0; j<8; j=j+1) begin
            stb=1; we=((j & 1) == 0); sel=4'hF;           // even=write
            adr={20'd700+(j>>1), 2'b00};
            wd=32'hE0000000+(j>>1);
            @(negedge clk); stb=0; we=0;
            while (!ack_seen) @(negedge clk);
            ack_seen=0;
            if (j & 1) begin                            // read of word j>>1
                if (rd !== 32'hE0000000+(j>>1)) begin
                    k=0;
                    $display("FAIL S7 alt %0d: got %h exp %h", j, rd,
                             32'hE0000000+(j>>1));
                end
            end
        end
        stb=0;
        chk(k, "S7 alt rd data");

        // ===== S8: refresh over an idle open row
        wb_wr(1000, 4'hF, 32'h5A5A5A5A);
        wait_for_ref;                   // row must have been force-closed
        chk(dut.open_v === 4'b0,        "S8 rows closed for REF");
        t0 = ref_total;
        #( (T_REF_LIMIT + 10) * 40 );   // >1 more refresh window
        chk(ref_total > t0,             "S8 refresh repeats");
        wb_rd(1000, d, lat);
        chk(d === 32'h5A5A5A5A,         "S8 data survived refresh");

        // ===== S9: random stress 400 ops vs mirror
        seed = 32'h12345;
        for (i=0; i<=2047; i=i+1) refm[i] = 32'h0;
        for (j=0; j<400; j=j+1) begin
            seed = lcg(seed);
            wad = seed[19:0] & 20'h7FF;         // keep in 2 KiB word region
            seed = lcg(seed);
            if (seed[0]) begin
                s = seed[3:0];
                v = {seed[31:4], 4'h0} ^ 32'h5C000000;
                wb_wr(wad, s, v);
                if (s[0]) refm[wad][7:0]   = v[7:0];
                if (s[1]) refm[wad][15:8]  = v[15:8];
                if (s[2]) refm[wad][23:16] = v[23:16];
                if (s[3]) refm[wad][31:24] = v[31:24];
            end else begin
                wb_rd(wad, d, lat);
                if (d !== refm[wad]) begin
                    nfail=nfail+1;
                    $display("FAIL S9 @%0d rd %h exp %h", wad, d, refm[wad]);
                end
            end
        end
        npass = npass + 1;              // S9 stress counted once (inline fail)
        for (j=0; j<64; j=j+1) begin
            wb_rd(j, d, lat);
            if (d !== refm[j]) begin
                nfail=nfail+1;
                $display("FAIL S9 readback %0d %h != %h", j, d, refm[j]);
            end
        end
        npass = npass + 1;

        // ===== S10: reset mid-flight -> full recovery
        @(negedge clk); stb=1; we=1; sel=4'hF;
        adr={20'd60,2'b00}; wd=32'hFEEDFACE;
        @(negedge clk); stb=0;
        @(negedge clk);                 // ack pending when reset hits
        rst = 1; #100; rst = 0;         // hold over >=2 posedges
        ack_seen = 0;
        #(T_INIT*40 + 1000);             // re-init
        chk(m_mrs_done,                 "S10 re-MRS");
        wb_wr(60, 4'hF, 32'hCAFEBABE);
        wb_rd(60, d, lat);
        chk(d === 32'hCAFEBABE,         "S10 recovery");

        $display("wb_sdram: %0d checks pass, %0d fail (%0d model)",
                 npass, nfail + mfail, mfail);
        if (nfail + mfail != 0) $display("FAILURES PRESENT");
        else                    $display("ALL TESTS PASSED");
        $finish;
    end

    // S9 support
    integer seed;
    reg [19:0] wad; reg [3:0] s; reg [31:0] v;
    reg [31:0] refm [0:2047];
    function [31:0] lcg; input [31:0] x;
        lcg = x*32'd1103515245 + 32'd12345;
    endfunction

    // watchdog (100k cycles: init 5000 + S8 ~1000 + S9 ~6k + slack)
    initial begin
        #(4_000_000);
        $display("FAIL: WATCHDOG (nfail=%0d mfail=%0d npass=%0d)", nfail, mfail, npass);
        $finish;
    end
endmodule
`default_nettype wire
