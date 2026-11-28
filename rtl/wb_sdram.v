// wb_sdram -- pipelined Wishbone B4 SDRAM controller for the M12L64322A(2S)
// (Colorlight 5A-75B v8.2 SDRAM U29: 4 banks x 524288 x 32-bit words,
//  refresh 64 ms / 4K cycles; timing from the datasheet -7 worst column).
//
// Protocol: native wb_ic SLAVE side (design.md sec.6) -- STB_I is a 1-cycle
// pulse per transfer; ACK_O is a 1-cycle pulse >=1 cycle later with DAT_O
// valid on it; completions strictly in issue order; no ERR, no STALL.
// wb_ic presents one transfer per slave until ACK, and re-presents earliest
// on the cycle AFTER the ACK cycle, so the interface-rate ceiling is
// 1 word / 2 cycles (50 MB/s at 25 MHz -- 2x wb_ram-class miss-free path);
// the datapath is built to run AT that ceiling:
//   * FAST PATHS (request queue idle, no refresh pending):
//       - read whose prefetched word matches: DAT_O/ACK_O registered on the
//         STB cycle -> complete at STB+1;
//       - full-word write on an open row: command issued directly from the
//         STB cycle (posted ACK at STB+1), zero queue latency.
//   * SPECULATIVE READ RING (4 slots): the chase pointer issues addr+1..+N
//     into idle command slots while the row is open; a serve KEEPS a
//     still-leading pointer (re-arms only on stream discontinuity), skips
//     targets the ring already holds, and an STB read miss FLUSHES the ring
//     (except an in-flight fill of the new address, which still serves the
//     queue head at its capture edge). A hit whose capture edge IS the STB
//     cycle is served straight off the DQ bus (one cycle of latency back).
//     A stale-but-legal entry is dropped when a write touches its address
//     (write-after-prefetch coherence).
//   * queued path (misses, RMW, q1 traffic): 2-entry queue, ACT/PRE chains,
//     byte writes read-modify-write (v8.2 ties DQM0..3 to GND -> the device
//     cannot mask lanes; merge taken live off the bus on the capture edge).
//   * refresh: tREF counter -> drain -> PRECHARGE open rows (also bounds
//     row-open << tRAS_max=100us) -> AUTO-REFRESH -> tRFC quiet window.
//
// CYCLE CONVENTION: a decision at cycle d loads the command register at the
// edge closing cycle d -> command VISIBLE during c = d+1 -> the device
// latches it at the edge closing c. READ (BL=1, CL=2): the device drives
// D0 on the edge (latching edge + CL), so data is valid DURING cycle
// c+CL = d+CL+1 and the capture condition is (cyc == d+CL+2) (plain
// rising-edge FF needs the full cycle); result/ACK visible d+CL+3. WRITE:
// data valid during c+1 = d+2 (dq_a -> dq_r chain with the one-cycle-
// offset oe_r); the device samples on the edge after its latching edge.
// The tb device model encodes the same launch rule and asserts it.
// All timestamps are DECISION cycles, so decision-to-decision spacing
// equals visible-to-visible spacing.
//
// Device setup (datasheet MODE REGISTER FIELD TABLE): BL=1 (A[2:0]=001),
// BT=sequential (A3=0), CL=2 (A[6:4]=010), A[10:7]=0 -> MRS = 13'h021.
// Address map of the 4 MiB DRAM window (wb_ic decodes ADR[31:22]==10'h200):
//   word = ADR[21:2] = { row[9:0]->RA, bank[1:0]->BA, col[7:0]->CA }
//   (RA10 of 2048 rows / 1M of 2M words unused -> no address wrap).
//
// Timing (datasheet AC OPERATING PARAMETER, -7 worst column):
//   tRRD 14, tRCD 21, tRP 21, tRAS 42, tRC 63, tRFC 63 ns; tMRD 2 CLK;
//   tCCD 1 CLK. DQ turnaround (half-duplex, BL=1): write data drives at
//   cycle d_w+2, read data occupies cycle d_r+CL+2 -> R->W decision gap >=
//   CL+2, W->R >= 2, same-type >= 1. tools/sdram_calc.py derives the
//   same cycle counts; tb/tb_wb_sdram.v S0 cross-checks every T_* and its
//   device model independently asserts every gap/latency/coherency rule.
//
// Not implemented (documented): self-refresh/power-down (CKE tied high),
// DQM drive (tied 0; SEL handled by RMW). SDRAM clock pin synthesis
// (plain registered or phase-advanced ODDF) is the board top's job.
`default_nettype none

module wb_sdram #(
    parameter CLK_FREQ = 25000000                 // Hz; see sdram_calc.py
)(
    input  clk, rst,

    // Wishbone B4 slave (wb_ic side)
    input         STB_I,
    input         WE_I,
    input  [3:0]  SEL_I,
    /* verilator lint_off UNUSEDSIGNAL */ // word-aligned SDRAM window only:
    input  [31:0] ADR_I,                   // ADR[1:0], ADR[31:22] unused
    /* verilator lint_on UNUSEDSIGNAL */
    input  [31:0] DAT_I,
    output reg [31:0] DAT_O,
    output reg        ACK_O,

    // SDRAM device (all registered; dq is tristated in the board top)
    output [1:0]  sd_ba_o,
    output [12:0] sd_a_o,
    output        sd_ras_n_o, sd_cas_n_o, sd_we_n_o,
    output        sd_cs_n_o, sd_cke_o,
    output [3:0]  sd_dqm_o,
    output [31:0] sd_dq_o,
    output        sd_dq_oe_o,
    input  [31:0] sd_dq_i
);

    // ns x MHz / 1000, ceil (identical math to tools/sdram_calc.py); MHz form
    // keeps every intermediate < 2^31.
    localparam integer CLK_MHZ  = CLK_FREQ/1000000;
    localparam integer T_RRD  = (14*CLK_MHZ + 999)/1000;
    localparam integer T_RCD  = (21*CLK_MHZ + 999)/1000;
    localparam integer T_RP   = (21*CLK_MHZ + 999)/1000;
    localparam integer T_RAS  = (42*CLK_MHZ + 999)/1000;
    localparam integer T_RC   = (63*CLK_MHZ + 999)/1000;
    localparam integer T_RFC  = (63*CLK_MHZ + 999)/1000;
    localparam integer T_MRD  = 2;
    localparam integer T_REF  = (64*(CLK_FREQ/1000))/4096; // 64ms/4K, floor
    localparam integer T_INIT = 200*CLK_MHZ;               // 200 us power-up
    localparam [2:0]  CL      = 3'd2;              // CAS latency (MRS[6:4])
    localparam [12:0] MRS_V   = {6'b000000, CL, 4'b0001};  // BL1 seq, CL2
    localparam integer NS     = 4;                       // spec ring slots
    localparam [15:0] RDLEAD  = 16'd4;   // CL+2 (CL=2 via MRS_V) issue->capture
    localparam [19:0] CHLEAD  = 20'd5;   // NS+1 (NS=4) chase-lead window

    localparam [3:0] C_NOP = 4'd0, C_ACT = 4'd1, C_RD = 4'd2, C_WR = 4'd3,
                     C_PRE = 4'd4, C_PREA = 4'd5, C_REF = 4'd6, C_MRS = 4'd7;

    reg [15:0] cyc;
    always @(posedge clk or posedge rst)
        if (rst) cyc <= 16'd0; else cyc <= cyc + 16'd1;

    // ------------------------------------------------------ request queue
    // holds ops that the fast paths cannot serve (miss, RMW, overlap);
    // q1 absorbs a STB landing while q0 is being served (IC keeps one
    // outstanding per slave, so depth 2 is sufficient by construction).
    reg        q0_v, q0_we;
    reg [3:0]  q0_sel;
    reg [19:0] q0_ad;
    reg [31:0] q0_dat;
    reg        q1_v, q1_we;
    reg [3:0]  q1_sel;
    reg [19:0] q1_ad;
    reg [31:0] q1_dat;

    // --------------------------------------------------- scheduler state
    reg [3:0]  open_v;
    reg [9:0]  orow  [0:3];
    reg [15:0] ts_act [0:3], ts_pre [0:3], ts_act_any;
    reg [15:0] ts_rd  [0:3];                  // last C_RD issue per bank
    reg        ts_rd_v [0:3];                 // (valid flag: FFFF sentinel
                                              //  would falsely look stale)
    reg [15:0] col_ts;  reg col_is_wr;
    reg [15:0] ts_ref;

    reg        pa_v;                              // pending ACT for head
    reg [15:0] pa_due;                            // col decision cycle
    reg        pa_we; reg [3:0] pa_sel;

    reg        rd_v;  reg [15:0] rd_due;          // queued head read
    reg        rd_rwm; reg [3:0] rd_sel; reg [31:0] rd_wdat;
    reg        rwm_w;                             // COL_WR decision next cyc
    reg        wa1;                               // queued-write ACK chain

    // ------------------------------------------------- speculative ring
    reg        sr_v   [0:NS-1];                   // slot allocated
    reg        sr_rdy [0:NS-1];                   // capture complete
    reg [19:0] sr_ad  [0:NS-1];
    reg [15:0] sr_due [0:NS-1];                   // capture condition cyc
    reg [31:0] sr_dat [0:NS-1];
    reg        ch_v;  reg [19:0] ch_ad;           // next speculation target

    reg [1:0]  ist;  reg [15:0] ist_wait;         // 0 NOP 1 PREA 2 MRS 3 RUN
    reg [15:0] ref_cnt;
    reg        ref_due, ref_sent; reg [15:0] ref_quiet;

    // --------------------------------------------------- command output
    reg [3:0]  cmd_r;
    reg [1:0]  ba_r;
    reg [12:0] a_r;
    reg [31:0] dq_a, dq_r;
    reg        dq_v, oe_r;

    // ------------------------------------------------- comb: legality
    wire [15:0] s_col   = cyc - col_ts;
    wire        gap_rd  = col_is_wr ? (s_col >= 16'd2) : (s_col >= 16'd1);
    wire        gap_wr  = col_is_wr ? (s_col >= 16'd1) : (s_col >= RDLEAD);
    wire        gap_any = (s_col >= 16'd1);
    wire        rfc_ok  = (cyc - ts_ref) >= T_RFC[15:0];
    wire        trcd_ok = (cyc - ts_act_any) >= T_RRD[15:0];

    // ------------------------------------------------- fast path inputs
    wire [19:0] f_ad = ADR_I[21:2];
    wire [1:0]  f_ba = f_ad[9:8];
    wire [9:0]  f_row = f_ad[19:10];
    wire        f_hit = open_v[f_ba] && (orow[f_ba] == f_row) &&
                        ((cyc - ts_act[f_ba]) >= T_RCD[15:0]);
    wire        q_idle = !q0_v && !q1_v && !rd_v && !pa_v && !rwm_w && !wa1;

    // ring match: READY slot, or an in-flight slot whose capture edge IS
    // this cycle (data present on the DQ bus: serve straight from it --
    // one cycle of speculative latency recovered).
    integer ms; reg m_hit; reg [31:0] m_dat;
    always @(*) begin
        m_hit = 1'b0; m_dat = 32'h0; ms = 0;
        for (ms=0; ms<NS; ms=ms+1)
            if (sr_v[ms] && sr_ad[ms] == f_ad &&
                (sr_rdy[ms] || sr_due[ms] == cyc)) begin
                m_hit = 1'b1; m_dat = sr_rdy[ms] ? sr_dat[ms] : sd_dq_i;
            end
    end

    wire fp_rd_serve = STB_I && !WE_I && q_idle && !ref_due && m_hit;
    wire fp_wr       = STB_I &&  WE_I && SEL_I == 4'hF && q_idle && !ref_due
                       && f_hit && gap_wr;

    // speculative-stream dedup: the chase pointer must not re-fill an
    // address the ring already holds (a serve re-arm can land ch_ad on a
    // still-in-flight slot) -- skip such slots instead of wasting one.
    wire ch_dup = (sr_v[0] && sr_ad[0] == ch_ad) ||
                  (sr_v[1] && sr_ad[1] == ch_ad) ||
                  (sr_v[2] && sr_ad[2] == ch_ad) ||
                  (sr_v[3] && sr_ad[3] == ch_ad);

    // first free ring slot for a speculative fill (precomputed: keeps the
    // registered loop body assignment-free for lint)
    integer fk; reg [1:0] fill_slot; reg fill_ok;
    always @(*) begin
        fill_ok = 1'b0; fill_slot = 2'b00; fk = 0;
        for (fk=0; fk<NS; fk=fk+1)
            if (!sr_v[fk] && !fill_ok) begin
                fill_ok = 1'b1; fill_slot = fk[1:0];
            end
    end

    // queued head already covered by an in-flight ring fill: the head must
    // NOT issue a redundant C_RD -- it gets served by h_hit at capture.
    reg head_pend; integer hp;
    always @(*) begin
        head_pend = 1'b0; hp = 0;
        for (hp=0; hp<NS; hp=hp+1)
            if (sr_v[hp] && !sr_rdy[hp] && sr_ad[hp] == q0_ad)
                head_pend = 1'b1;
    end

    // queued-head ring match (request that arrived before its spec data)
    integer hs; reg h_hit; reg [31:0] h_dat; reg [1:0] h_slot;
    always @(*) begin
        h_hit = 1'b0; h_dat = 32'h0; h_slot = 0; hs = 0;
        for (hs=0; hs<NS; hs=hs+1)
            if (q0_v && !q0_we && sr_v[hs] && sr_ad[hs] == q0_ad &&
                (sr_rdy[hs] || sr_due[hs] == cyc)) begin
                h_hit = 1'b1; h_slot = hs[1:0];
                h_dat = sr_rdy[hs] ? sr_dat[hs] : sd_dq_i;
            end
    end

    wire rd_cap_fire = rd_v && (cyc == rd_due);

    // refresh drain + PRE burst guard: a bank with a C_RD whose DQ window
    // (issue + CL + 2) has not closed cannot take a PRECHARGE/REFRESH --
    // keyed on DEVICE-side ts_rd so a ring-slot flush cannot erase it.
    integer pc; reg anypend; reg [3:0] pend_b;
    always @(*) begin
        anypend = 1'b0; pend_b = 4'h0; pc = 0;
        for (pc=0; pc<4; pc=pc+1)
            if (ts_rd_v[pc] && (cyc - ts_rd[pc]) < RDLEAD) begin
                anypend = 1'b1;
                pend_b[pc] = 1'b1;
            end
    end
    // (q0/q1 traffic is NOT drained: ref_due simply stops new issue; in-flight
    // work is bounded to <=CL+3 cycles, so REF always meets its window)
    wire rfs_drain = rd_v || pa_v || rwm_w || wa1 || anypend;

    wire head_act  = pa_v && (cyc >= pa_due) && q0_v;
    wire head_busy = rd_v || pa_v || rwm_w || wa1;

    integer b, s;
    reg [3:0] ncmd; reg [1:0] nba; reg [12:0] na; reg [31:0] ndq;
    reg       spec_now;                           // C_RD is a ring fill

    always @(*) begin
        ncmd = C_NOP; nba = 2'b00; na = 13'h0; ndq = 32'h0; spec_now = 1'b0;
        if (ist == 2'd3 && rfc_ok) begin
            // ---- 1. in-flight head chains complete even while ref_due
            // (rfs_drain WAITS on them; blocking them would deadlock) ----
            if (q0_v && rwm_w) begin
                // RMW read captured last edge: merge-write now
                ncmd = C_WR; nba = q0_ad[9:8];
                na = {5'b0, q0_ad[7:0]}; ndq = rd_wdat;
            end else if (q0_v && head_act &&
                         ((pa_we && pa_sel == 4'hF) ? gap_wr : gap_rd)
                         && !h_hit) begin
                ncmd  = (pa_we && pa_sel == 4'hF) ? C_WR : C_RD;
                nba   = q0_ad[9:8]; na = {5'b0, q0_ad[7:0]};
                if (pa_we && pa_sel == 4'hF) ndq = q0_dat;
            end
            // ---- 2. refresh sequence once nothing is in flight ----
            else if (ref_due && !ref_sent) begin
                if (!rfs_drain) begin
                    if (open_v != 4'b0) begin
                        for (b=3; b>=0; b=b-1)
                            if (open_v[b] && (cyc - ts_act[b]) >= T_RAS[15:0]
                                && !pend_b[b])
                                begin ncmd = C_PRE; nba = b[1:0]; end
                    end else if (gap_any) ncmd = C_REF;
                end
            end
            // ---- 3. fresh issue (never while ref_due) ----
            else if (!ref_due && q0_v) begin
                if (!head_busy && !h_hit && !(head_pend && !q0_we)) begin
                    if (open_v[q0_ad[9:8]] &&
                        orow[q0_ad[9:8]] == q0_ad[19:10] &&
                        (cyc - ts_act[q0_ad[9:8]]) >= T_RCD[15:0]) begin
                        nba = q0_ad[9:8]; na = {5'b0, q0_ad[7:0]};
                        if (q0_we && q0_sel == 4'hF) begin
                            if (gap_wr) begin ncmd = C_WR; ndq = q0_dat; end
                        end else if (gap_rd) ncmd = C_RD;   // read or RMW read
                    end else if (!open_v[q0_ad[9:8]] &&
                                 (cyc - ts_pre[q0_ad[9:8]]) >= T_RP[15:0] &&
                                 (cyc - ts_act[q0_ad[9:8]]) >= T_RC[15:0] &&
                                 trcd_ok) begin
                        ncmd = C_ACT; nba = q0_ad[9:8];
                        na   = {3'b0, q0_ad[19:10]};
                    end else if (open_v[q0_ad[9:8]] &&
                                 orow[q0_ad[9:8]] != q0_ad[19:10] &&
                                 (cyc - ts_act[q0_ad[9:8]]) >= T_RAS[15:0] &&
                                 gap_any && !pend_b[q0_ad[9:8]]) begin
                        ncmd = C_PRE; nba = q0_ad[9:8];
                    end
                end
                // ---- ring fill in an idle slot ----
                if (ncmd == C_NOP && ch_v && !ch_dup && gap_rd &&
                    open_v[ch_ad[9:8]] &&
                    orow[ch_ad[9:8]] == ch_ad[19:10] &&
                    (cyc - ts_act[ch_ad[9:8]]) >= T_RCD[15:0]) begin
                    for (s=0; s<NS; s=s+1)
                        if (!sr_v[s] && ncmd == C_NOP) begin
                            ncmd = C_RD; nba = ch_ad[9:8];
                            na   = {5'b0, ch_ad[7:0]};
                            spec_now = 1'b1;
                        end
                end
            end else if (ncmd == C_NOP && ch_v && !ch_dup && !ref_due && !head_act &&
                         q_idle && !fp_wr && gap_rd &&   // fp_wr steals slot
                         open_v[ch_ad[9:8]] &&
                         orow[ch_ad[9:8]] == ch_ad[19:10] &&
                         (cyc - ts_act[ch_ad[9:8]]) >= T_RCD[15:0]) begin
                // queue fully idle: speculation chain keeps running
                for (s=0; s<NS; s=s+1)
                    if (!sr_v[s] && ncmd == C_NOP) begin
                        ncmd = C_RD; nba = ch_ad[9:8];
                        na   = {5'b0, ch_ad[7:0]};
                        spec_now = 1'b1;
                    end
            end
        end
    end

    // =========================================================== seq block
    always @(posedge clk or posedge rst) begin : SEQ
        integer k;
        if (rst) begin
            q0_v<=0; q0_we<=0; q0_sel<=0; q0_ad<=0; q0_dat<=0;
            q1_v<=0; q1_we<=0; q1_sel<=0; q1_ad<=0; q1_dat<=0;
            open_v<=0; cmd_r<=C_NOP; ba_r<=0; a_r<=0;
            dq_a<=0; dq_r<=0; dq_v<=0; oe_r<=0;
            rd_v<=0; rd_due<=0; rd_rwm<=0; rd_sel<=0; rd_wdat<=0; rwm_w<=0;
            wa1<=0; ch_v<=0; ch_ad<=0;
            pa_v<=0; pa_due<=0; pa_we<=0; pa_sel<=0;
            col_ts<=16'hFFFF; col_is_wr<=0; ts_ref<=16'd0;
            ts_act_any<=16'hFFFF;
            for (b=0; b<4; b=b+1) begin
                ts_act[b]<=16'hFFFF; ts_pre[b]<=16'hFFFF; orow[b]<=0;
                ts_rd[b]<=16'hFFFF; ts_rd_v[b]<=1'b0;
            end
            for (k=0; k<NS; k=k+1) begin
                sr_v[k]<=0; sr_rdy[k]<=0; sr_ad[k]<=0;
                sr_due[k]<=0; sr_dat[k]<=0;
            end
            ist<=2'd0; ist_wait<=T_INIT[15:0];
            ref_cnt<=T_REF[15:0]; ref_due<=0; ref_sent<=0; ref_quiet<=0;
            DAT_O<=32'h0; ACK_O<=1'b0;
        end else begin
            // ---------------- defaults: NOP bus, one-cycle ACK, dq chain
            cmd_r <= C_NOP;
            ACK_O <= 1'b0;
            dq_r  <= dq_a;
            oe_r  <= dq_v;
            dq_v  <= 1'b0;
            wa1   <= 1'b0;
            rwm_w <= 1'b0;

            // ---------------- ring capture edges
            for (k=0; k<NS; k=k+1)
                if (sr_v[k] && !sr_rdy[k] && cyc == sr_due[k]) begin
                    sr_rdy[k] <= 1'b1; sr_dat[k] <= sd_dq_i;
                end

            // ---------------- refresh counter / quiet window
            if (ist == 2'd3) begin
                if (!ref_due && ref_cnt != 16'd0) ref_cnt <= ref_cnt - 16'd1;
                else if (!ref_due) ref_due <= 1'b1;
                if (ref_due && ref_sent && cyc == ref_quiet) begin
                    ref_due <= 1'b0; ref_sent <= 1'b0;
                    ref_cnt <= T_REF[15:0];
                end
            end else begin
                if (ist_wait != 16'd0) ist_wait <= ist_wait - 16'd1;
                else case (ist)
                2'd0: begin
                    cmd_r <= C_PREA; a_r <= 13'h400;   // A10=1 => PRECHARGE ALL
                    ist <= 2'd1; ist_wait <= T_RP[15:0]; col_ts <= cyc;
                end
                2'd1: begin
                    cmd_r <= C_MRS; a_r <= MRS_V;
                    ist <= 2'd2; ist_wait <= T_MRD[15:0]; col_ts <= cyc;
                end
                2'd2: ist <= 2'd3;
                default: ;
                endcase
            end

            // ---------------- FAST PATH 1: ring serve on STB cycle
            if (fp_rd_serve) begin
                DAT_O <= m_dat;
                ACK_O <= 1'b1;
                for (k=0; k<NS; k=k+1)
                    if (sr_v[k] && sr_ad[k] == f_ad &&
                        (sr_rdy[k] || sr_due[k] == cyc))
                        sr_v[k] <= 1'b0;
                // keep a still-leading chase pointer (ring filled ahead of
                // the host); only a discontinuity re-arms to f_ad+1.
                if (!(ch_v && ch_ad > f_ad && (ch_ad - f_ad) <= CHLEAD)) begin
                    ch_v  <= (f_ad[7:0] != 8'hFF);
                    ch_ad <= f_ad + 20'd1;
                end
            end
            // stream break: an STB read the ring can't serve means its
            // contents belong to an abandoned chase -- flush everything
            // except a still-in-flight fill of the new address (the queue
            // head can be served by it via h_hit).
            if (STB_I && !WE_I && !fp_rd_serve)
                for (k=0; k<NS; k=k+1)
                    if (!(sr_v[k] && !sr_rdy[k] && sr_ad[k] == f_ad))
                        sr_v[k] <= 1'b0;

            // ---------------- FAST PATH 2: posted full write, direct issue
            if (fp_wr) begin
                ACK_O <= 1'b1;
                for (k=0; k<NS; k=k+1)
                    if (sr_v[k] && sr_ad[k] == f_ad) sr_v[k] <= 1'b0;
            end

            // ---------------- queued head: ring serve (already queued).
            // Suppressed while a queued read capture is in flight (its
            // ack arrives at rd_due+1; a double-ACK would corrupt order).
            if (h_hit && !fp_rd_serve && !rd_v) begin
                DAT_O <= h_dat;
                ACK_O <= 1'b1;
                sr_v[h_slot] <= 1'b0;
                q0_v <= 1'b0;
                if (q1_v) begin
                    q0_v<=1; q0_we<=q1_we; q0_sel<=q1_sel;
                    q0_ad<=q1_ad; q0_dat<=q1_dat; q1_v<=1'b0;
                end
                if (!(ch_v && ch_ad > q0_ad && (ch_ad - q0_ad) <= CHLEAD)) begin
                    if (!(ch_v && ch_ad > q0_ad && (ch_ad - q0_ad) <= CHLEAD)) begin
                        ch_v  <= !ref_due && (q0_ad[7:0] != 8'hFF);
                        ch_ad <= q0_ad + 20'd1;
                    end
                end
            end

            // ---------------- queued head read capture (plain / RMW)
            if (rd_cap_fire) begin
                rd_v <= 1'b0;
                if (rd_rwm) begin
                    rd_rwm <= 1'b0;
                    rd_wdat[7:0]   <= rd_sel[0] ? rd_wdat[7:0]   : sd_dq_i[7:0];
                    rd_wdat[15:8]  <= rd_sel[1] ? rd_wdat[15:8]  : sd_dq_i[15:8];
                    rd_wdat[23:16] <= rd_sel[2] ? rd_wdat[23:16] : sd_dq_i[23:16];
                    rd_wdat[31:24] <= rd_sel[3] ? rd_wdat[31:24] : sd_dq_i[31:24];
                    rwm_w <= 1'b1;
                end else begin
                    DAT_O <= sd_dq_i; ACK_O <= 1'b1;
                    q0_v <= 1'b0;
                    if (q1_v) begin
                        q0_v<=1; q0_we<=q1_we; q0_sel<=q1_sel;
                        q0_ad<=q1_ad; q0_dat<=q1_dat; q1_v<=1'b0;
                    end
                    if (!(ch_v && ch_ad > q0_ad && (ch_ad - q0_ad) <= CHLEAD)) begin
                        ch_v  <= !ref_due && (q0_ad[7:0] != 8'hFF);
                        ch_ad <= q0_ad + 20'd1;
                    end
                end
            end

            // ---------------- chase-pointer dedup advance: if no re-arm
            // happened this edge and the ring already holds ch_ad, skip
            // forward (stop at row end) so fills are never wasted.
            if (ch_v && ch_dup && !fp_rd_serve &&
                !(h_hit && !rd_v) && !(rd_cap_fire && !rd_rwm)) begin
                if (ch_ad[7:0] == 8'hFF) ch_v <= 1'b0;
                else                      ch_ad <= ch_ad + 20'd1;
            end

            // ---------------- queued write posted-ack / pop
            if (wa1) begin
                ACK_O <= 1'b1;
                q0_v <= 1'b0;
                if (q1_v) begin
                    q0_v<=1; q0_we<=q1_we; q0_sel<=q1_sel;
                    q0_ad<=q1_ad; q0_dat<=q1_dat;
                end
                q1_v <= 1'b0;
            end

            // ---------------- queue push (STB not served by a fast path).
            // A pop edge never coincides with STB (wb_ic presents the next
            // transfer at least one cycle AFTER the ACK cycle), so q0 is
            // only ever written when empty or freshly popped.
            if (STB_I && !fp_rd_serve && !fp_wr) begin
                if (!q0_v) begin
                    q0_v<=1; q0_we<=WE_I; q0_sel<=SEL_I;
                    q0_ad<=f_ad; q0_dat<=DAT_I;
                end else begin
                    q1_v<=1; q1_we<=WE_I; q1_sel<=SEL_I;
                    q1_ad<=f_ad; q1_dat<=DAT_I;
                end
            end

            // ---------------- command commit
            case (ncmd)
            C_ACT: begin
                cmd_r <= C_ACT; ba_r <= nba; a_r <= na;
                col_ts <= cyc; ts_act[nba] <= cyc; ts_act_any <= cyc;
                open_v[nba] <= 1'b1; orow[nba] <= q0_ad[19:10];
                pa_v <= 1'b1; pa_we <= q0_we; pa_sel <= q0_sel;
                pa_due <= cyc + T_RCD[15:0];
                if (q0_we) ch_v <= 1'b0;          // stream broken by write
            end
            C_RD: begin
                cmd_r <= C_RD; ba_r <= nba; a_r <= na;
                col_ts <= cyc; col_is_wr <= 1'b0;
                ts_rd[nba] <= cyc; ts_rd_v[nba] <= 1'b1; // device-side guard
                if (!spec_now) begin
                    rd_v <= 1'b1; rd_due <= cyc + RDLEAD;
                    rd_rwm <= q0_we;            // write: this is the RMW read
                    rd_sel <= q0_sel;
                    rd_wdat <= q0_dat;
                    pa_v <= 1'b0;
                    // chain speculation only behind pure reads (an RMW read
                    // is an internal op of a WRITE stream)
                    ch_v  <= !ref_due && !q0_we && (q0_ad[7:0] != 8'hFF);
                    ch_ad <= q0_ad + 20'd1;
                end else begin
                    if (fill_ok) begin
                        sr_v[fill_slot]   <= 1'b1;
                        sr_rdy[fill_slot] <= 1'b0;
                        sr_ad[fill_slot]  <= ch_ad;
                        sr_due[fill_slot] <= cyc + RDLEAD;
                    end
                    ch_v  <= (ch_ad[7:0] != 8'hFF); // +1 crosses row: stop
                    ch_ad <= ch_ad + 20'd1;
                end
            end
            C_WR: begin
                cmd_r <= C_WR; ba_r <= nba; a_r <= na;
                dq_a <= ndq; dq_v <= 1'b1;
                col_ts <= cyc; col_is_wr <= 1'b1;
                wa1 <= 1'b1;
                pa_v <= 1'b0;
                ch_v <= 1'b0;
                for (k=0; k<NS; k=k+1)
                    if (sr_v[k] && sr_ad[k] == q0_ad)
                        sr_v[k] <= 1'b0;            // WA-prefetch coherence
            end
            C_PRE: begin
                cmd_r <= C_PRE; ba_r <= nba; a_r <= 13'h0;
                col_ts <= cyc; ts_pre[nba] <= cyc; open_v[nba] <= 1'b0;
                // drop ring entries whose row closed
                for (k=0; k<NS; k=k+1)
                    if (sr_v[k] && sr_ad[k][9:8] == nba) sr_v[k] <= 1'b0;
                if (ch_v && ch_ad[9:8] == nba) ch_v <= 1'b0;
            end
            C_REF: begin
                cmd_r <= C_REF; ba_r <= 2'b00; a_r <= 13'h0;
                col_ts <= cyc; ts_ref <= cyc;
                ref_sent <= 1'b1;
                ref_quiet <= cyc + (T_RFC[15:0] - 16'd1);
                for (k=0; k<NS; k=k+1) sr_v[k] <= 1'b0;
                ch_v <= 1'b0;
            end
            default: ;
            endcase

            // ---------------- fast-path command override (later block wins)
            if (fp_wr) begin
                cmd_r <= C_WR; ba_r <= f_ba; a_r <= {5'b0, f_ad[7:0]};
                dq_a <= DAT_I; dq_v <= 1'b1;
                col_ts <= cyc; col_is_wr <= 1'b1;
            end
        end
    end

    // -------------------------------------------------- pad assignments
    // datasheet SIMPLIFIED TRUTH TABLE (strobe polarity, active low).
    // COMB decode of cmd_r: the command FF IS the one-cycle interface
    // register (cmd_r loaded at edge closing d -> pads visible cycle d+1).
    reg ras_n_c, cas_n_c, we_n_c;
    always @(*) case (cmd_r)
        C_ACT:  begin ras_n_c=1'b0; cas_n_c=1'b1; we_n_c=1'b1; end
        C_RD:   begin ras_n_c=1'b1; cas_n_c=1'b0; we_n_c=1'b1; end
        C_WR:   begin ras_n_c=1'b1; cas_n_c=1'b0; we_n_c=1'b0; end
        C_PRE,
        C_PREA: begin ras_n_c=1'b0; cas_n_c=1'b1; we_n_c=1'b0; end
        C_REF:  begin ras_n_c=1'b0; cas_n_c=1'b0; we_n_c=1'b1; end
        C_MRS:  begin ras_n_c=1'b0; cas_n_c=1'b0; we_n_c=1'b0; end
        default:begin ras_n_c=1'b1; cas_n_c=1'b1; we_n_c=1'b1; end
    endcase
    assign sd_ras_n_o = ras_n_c;
    assign sd_cas_n_o = cas_n_c;
    assign sd_we_n_o  = we_n_c;
    assign sd_cs_n_o  = 1'b0;        // CS# is tied GND on v8.2
    assign sd_cke_o   = 1'b1;        // CKE tied 3V3; power-down unused
    assign sd_dqm_o   = 4'h0;        // DQM tied GND; SEL handled by RMW
    assign sd_ba_o    = ba_r;
    assign sd_a_o     = a_r;
    assign sd_dq_o    = dq_r;
    assign sd_dq_oe_o = oe_r;

endmodule
`default_nettype wire
