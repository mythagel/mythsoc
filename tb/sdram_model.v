// sdram_model -- behavior model of the M12L64322A(2S) SDRAM device
// (4 banks x 512K x 32b; BL1 sequential; the v8.2 wiring: CS#/CKE/DQM
// hard-wired, so the model ASSERTS the controller keeps them static).
//
// Extracted verbatim from tb/tb_wb_sdram.v (session 28) so the SoC-level
// board-rehearsal bench (tb/tb_soc_sdram.v, DRAM_SDRAM=1) can run the SAME
// independently-asserted device rules against the full core:wb_ic path,
// not just the controller unit test. Every FAIL MODEL message is unchanged.
//
// Timing constants come from tb/sdram_params.vh (tools/sdram_calc.py is the
// source of truth; 25 MHz set). Host testbenches reference internals
// hierarchically (mem[], mcyc, ref_total, m_mrs_done, nfail, ...).
`default_nettype none

module sdram_model (
    input  clk, rst,
    // device-side pads (as wired on the board)
    input  [1:0]  ba,
    input  [12:0] sa,
    input         ras_n, cas_n, we_n, cs_n, cke,
    input  [3:0]  dqm,
    input         dq_oe,
    input  [31:0] dq_o,
    inout  [31:0] dq
);
    `include "tb/sdram_params.vh"

    localparam integer CLK_FREQ   = 25000000;   // params header is for 25 MHz
    localparam integer T_INIT     = `SDRAM_T_INIT;
    localparam integer T_RCD      = `SDRAM_T_RCD, T_RP = `SDRAM_T_RP,
                     T_RC         = `SDRAM_T_RC, T_RAS = `SDRAM_T_RAS,
                     T_RRD        = `SDRAM_T_RRD, T_RFC = `SDRAM_T_RFC,
                     T_MRD        = `SDRAM_T_MRD, T_REF = `SDRAM_T_REF;
    localparam integer REF_GRACE  = 96;
    localparam integer T_REF_LIMIT = T_REF + REF_GRACE;

    integer mcyc = 0;
    integer nfail = 0;

    // --------------------------------------------------------- refresh book
    // rows written are marked with the current window generation; when the
    // window (T_REF + grace) expires without a REF covering them -> FAIL.
    integer ref_gen = 0, ref_timer = 0;
    integer ref_mark [0:4*2048-1];
    integer dirty_left;

    // -------------------------------------------------------- device model
    reg [31:0] mem [0:4*524288-1];      // 8 MiB
    reg        m_open [0:3];
    reg [10:0] m_row  [0:3];
    integer m_act [0:3], m_pre [0:3], m_act_any, m_ref;
    reg [3:0] m_cl = 0;
    reg [2:0] m_bl = 0;
    reg       m_mrs_done = 0;
    integer m_mrs_c = -1;
    // read launch FIFO: BL=1, but back-to-back RDs pipeline; one beat per
    // RD command, driven CL edges after its decode edge (up to 6 in flight)
    reg [31:0] bf_val [0:5];
    integer    bf_cnt [0:5];             // 0 = free; else countdown edges
    reg        dq_en = 0;                // model drives this cycle
    reg  [31:0] dq_out = 0;
    // write capture stage: WR decoded -> data sampled on the NEXT edge
    reg        wr_pend = 0;
    integer    wr_bank = 0;
    reg [10:0] wr_row = 0;
    reg [7:0]  wr_col = 0;
    integer rd_cmd_c [0:3];              // last RD/WR command cycle per bank

    assign dq = dq_oe ? dq_o : (dq_en ? dq_out : 32'hzzzzzzzz);

    // refresh event counter (for S8)
    integer ref_total = 0;

    integer i, b;
    integer idx;

    always @(posedge clk) begin : MODEL
        if (rst) begin
            for (b=0; b<4; b=b+1) begin
                m_open[b] = 0; m_act[b] = -100000; m_pre[b] = -100000;
                rd_cmd_c[b] = -100000;
            end
            for (i=0; i<6; i=i+1) begin bf_cnt[i] = 0; bf_val[i] = 32'h0; end
            m_act_any = -100000; m_ref = -100000; m_mrs_c = -1;
            m_cl = 0; m_bl = 0; m_mrs_done = 0;
            dq_en = 0; dq_out = 32'h0; wr_pend = 0;
            // fresh device: prior dirty rows are irrelevant post-reset
            for (i=0; i<4*2048; i=i+1) ref_mark[i] = -1;
            ref_gen = ref_gen + 1; ref_timer = 0;
        end else begin
            // ---- model data-cycle processing (events scheduled last edge)
            dq_en <= 0;
            if (wr_pend) begin
                wr_pend <= 0;
                if (!dq_oe) begin
                    nfail = nfail + 1;
                    $display("FAIL MODEL: write at cyc %0d: no DQ drive", mcyc);
                end else if (^dq === 1'bx) begin
                    nfail = nfail + 1;
                    $display("FAIL MODEL: write data X at cyc %0d", mcyc);
                end else begin
                    mem[{wr_bank, wr_row, wr_col}] <= dq;
                    ref_mark[wr_bank*2048 + wr_row] = ref_gen;
                end
            end
            // read launch FIFO countdown: beat set to CL at its decode
            // edge -> drives CL edges later for one cycle
            for (i=0; i<6; i=i+1) if (bf_cnt[i] != 0) begin
                if (bf_cnt[i] == 1) begin
                    bf_cnt[i] <= 0;
                    dq_out <= bf_val[i];
                    dq_en  <= 1;
                end else bf_cnt[i] <= bf_cnt[i] - 1;
            end
            // refresh window watchdog (dirty rows written since last REF)
            ref_timer = ref_timer + 1;
            if (ref_timer >= T_REF_LIMIT) begin
                dirty_left = 0;
                for (i=0; i<4*2048; i=i+1)
                    if (ref_mark[i] == ref_gen) dirty_left = 1;
                if (dirty_left && mcyc > (T_INIT + T_REF + 64)) begin
                    nfail = nfail + 1;
                    $display("FAIL MODEL: refresh window expired, cyc %0d", mcyc);
                end
                ref_gen = ref_gen + 1;
                ref_timer = 0;
            end

            // ---- command decode (inputs visible during cycle mcyc, the
            // value before this edge; stamps are VISIBLE cycles)
            if (!cs_n) begin
                if (!ras_n && !cas_n && !we_n) begin        // MODE REGISTER SET
                    if (m_open[0]||m_open[1]||m_open[2]||m_open[3]) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: MRS with open bank, cyc %0d", mcyc);
                    end
                    m_bl <= sa[2:0]; m_cl <= sa[6:4];
                    if (sa[3] !== 1'b0 || sa[10:7] !== 4'h0) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: MRS reserved bits nonzero");
                    end
                    m_mrs_done <= 1; m_mrs_c <= mcyc;
                end else if (!ras_n && !cas_n && we_n) begin // AUTO REFRESH
                    for (b=0; b<4; b=b+1) if (m_open[b]) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: REF with bank %0d open at cyc %0d",
                                 b, mcyc);
                    end
                    if (m_ref > 0 && mcyc - m_ref < T_RFC) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: REF-REF gap %0d < tRFC at cyc %0d",
                                 mcyc-m_ref, mcyc);
                    end
                    if (m_mrs_done && m_ref > 0 &&
                        mcyc - m_ref > T_REF + REF_GRACE) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: REF interval %0d too long (cyc %0d)",
                                 mcyc-m_ref, mcyc);
                    end
                    m_ref <= mcyc;
                    ref_total = ref_total + 1;
                    // every row in every bank is refreshed by this command
                    for (i=0; i<4*2048; i=i+1) ref_mark[i] = -1;
                    ref_gen = ref_gen + 1;
                    ref_timer = 0;
                end else if (!ras_n && cas_n && we_n) begin  // ACTIVATE (L H H)
                    idx = ba;
                    if (m_open[idx]) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: ACT on open bank %0d cyc %0d",
                                 idx, mcyc);
                    end
                    if (mcyc - m_act[idx] < T_RC || mcyc - m_pre[idx] < T_RP ||
                        mcyc - m_act_any < T_RRD ||
                        (m_ref > 0 && mcyc - m_ref < T_RFC) ||
                        (m_mrs_done && mcyc - m_mrs_c < T_MRD)) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: ACT timing viol bank %0d cyc %0d (rc=%0d rp=%0d rrd=%0d rfc=%0d mrd=%0d)",
                                 idx, mcyc, mcyc-m_act[idx], mcyc-m_pre[idx],
                                 mcyc-m_act_any, m_ref>0 ? mcyc-m_ref : 9999,
                                 m_mrs_done ? mcyc-m_mrs_c : 9999);
                    end
                    m_open[idx] <= 1;
                    m_row[idx]  <= sa[10:0];
                    m_act[idx]  <= mcyc; m_act_any <= mcyc;
                end else if (!ras_n && cas_n && !we_n) begin  // PRECHARGE (L H L)
                    if (sa[10]) begin                        // ALL
                        for (b=0; b<4; b=b+1) if (m_open[b]) begin
                            if (mcyc - m_act[b] < T_RAS) begin
                                nfail = nfail + 1;
                                $display("FAIL MODEL: PREA tRAS viol bank %0d cyc %0d",
                                         b, mcyc);
                            end
                            if (rd_cmd_c[b] >= 0 && mcyc - rd_cmd_c[b] < m_cl + 2) begin
                                nfail = nfail + 1;
                                $display("FAIL MODEL: PREA interrupts burst bank %0d cyc %0d",
                                         b, mcyc);
                            end
                            m_open[b] <= 0; m_pre[b] <= mcyc;
                        end
                    end else if (m_open[ba]) begin
                        if (mcyc - m_act[ba] < T_RAS) begin
                            nfail = nfail + 1;
                            $display("FAIL MODEL: PRE tRAS viol bank %0d cyc %0d",
                                     ba, mcyc);
                        end
                        if (rd_cmd_c[ba] >= 0 &&
                            mcyc - rd_cmd_c[ba] < m_cl + 2) begin
                            nfail = nfail + 1;
                            $display("FAIL MODEL: PRE interrupts burst bank %0d cyc %0d",
                                     ba, mcyc);
                        end
                        m_open[ba] <= 0; m_pre[ba] <= mcyc;
                    end
                end else if (ras_n && !cas_n && we_n) begin    // COLUMN READ (H L H)
                    idx = ba;
                    if (!m_open[idx] || !m_mrs_done) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: RD on closed bank %0d cyc %0d",
                                 idx, mcyc);
                    end else if (mcyc - m_act[idx] < T_RCD) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: RD tRCD viol bank %0d cyc %0d",
                                 idx, mcyc);
                    end else begin
                        rd_cmd_c[idx] <= mcyc;
                        begin : PUSH
                            integer p;
                            p = 0;
                            while (p<6 && bf_cnt[p] != 0) p = p + 1;
                            if (p == 6) begin
                                nfail = nfail + 1;
                                $display("FAIL MODEL: launch FIFO overflow cyc %0d", mcyc);
                            end else begin
                                bf_cnt[p] <= m_cl;
                                bf_val[p] <= mem[{idx, m_row[idx], sa[7:0]}];
                            end
                        end
                    end
                end else if (ras_n && !cas_n && !we_n) begin   // COLUMN WRITE (H L L)
                    idx = ba;
                    if (!m_open[idx] || !m_mrs_done) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: WR on closed bank %0d cyc %0d",
                                 idx, mcyc);
                    end else if (mcyc - m_act[idx] < T_RCD) begin
                        nfail = nfail + 1;
                        $display("FAIL MODEL: WR tRCD viol bank %0d cyc %0d",
                                 idx, mcyc);
                    end else begin
                        wr_pend  <= 1; wr_bank <= idx;   // (no rd_cmd_c: a WR
                        wr_row   <= m_row[idx]; wr_col <= sa[7:0];  // does not
                    end                                   // drive DQ out)
                end
            end
            // DQ contention: device drives while controller drives
            if (dq_oe && dq_en) begin
                nfail = nfail + 1;
                $display("FAIL MODEL: DQ bus contention at cyc %0d", mcyc);
            end
            if (cs_n !== 1'b0 || cke !== 1'b1 || dqm !== 4'h0) begin
                nfail = nfail + 1;
                $display("FAIL MODEL: static pad misdrive cs=%b cke=%b dqm=%h",
                         cs_n, cke, dqm);
            end
            mcyc = mcyc + 1;
        end
    end

    initial begin
        for (i=0; i<4*2048; i=i+1)  ref_mark[i] = -1;
        for (i=0; i<4*524288; i=i+1) mem[i] = 32'h0;
    end

endmodule
`default_nettype wire
