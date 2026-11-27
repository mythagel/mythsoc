// mmu -- Sv32 MMU (mythrv32): page-table walker + 2-level TLB, WB B4 RO master
//
// Spec: riscv-privileged 12.3 (Sv32), 12.1.11 (satp). Address math and the
// reference walk model live in tools/mmu_addr.py (source of truth for
// tb/tb_mmu.v expectations).
//
// Request protocol (1-ahead pipelined): REQ_I is a 1-cycle pulse, asserted
// in an idle cycle (DONE_O low) OR during the previous request's DONE cycle
// (DONE_O high). Not accepted during a walk. SF_I must never coincide with
// REQ_I and must not coincide with a walk-starting REQ (core serializes
// sfence.vma). The requester samples PA_O/FAULT_O/CAUSE_O while DONE_O is
// high (1 cycle each). Sustained hit rate = 1 translation/cycle.
//   DONE is presented the cycle the last PTE ACK lands (miss) / after REQ
//   (hit / M-mode / Bare):
//     hit / M-mode / Bare : DONE 1 cycle after REQ
//     miss 4M            : DONE 3 cycles after REQ (1 PTE read, 1-cyc slave)
//     miss 4K            : DONE 5 cycles after REQ (2 PTE reads, 1-cyc slave)
//   STB cycles from REQ: hit/M/Bare = 0, 4M = 1, 4K = 2.
// Response: PA_O (don't-care on fault), FAULT_O, CAUSE_O (12/13/15).
//
// Translation (Sv32, LEVELS=2):
//   L1 PTE = MEM[satp.ppn*4096 + VA[31:22]*4]   (index VA[31:22])
//   L0 PTE = MEM[L1.pte.ppn*4096 + VA[21:12]*4] (index VA[21:12])
//   PTE bits: V[0] R[1] W[2] X[3] U[4] G[5] A[6] D[7] RSW[9:8]
//             PPN0[19:10] PPN1[31:20]
//   M-mode / satp.MODE==0 (Bare): identity PA=VA, no walk, no TLB use.
//   structural faults (no TLB fill): v=0; R=0&W=1; non-leaf with D|A|U set;
//     4K leaf (L0) with R=X=0; 4M leaf (L1) with PPN[9:0]!=0.
//   permission/A-D (re-checked on hit, so per access type): fetch needs X;
//     load R (MXR=1: R|X); store/AMO W (AMO also R; cause always store).
//     S-mode: U=1 pages fault unless (SUM=1 && !fetch); fetch on U=1 always
//     faults in S. U-mode: needs U=1. A/D (Svade fault scheme): A=0 fault;
//     store/AMO with D=0 fault. (No hardware A/D update.)
// TLB (per instance):
//   L1 16e = 4 sets x 4 ways:  4K: set VA[13:12] tag VA[31:14]
//                              4M: set VA[21:20] tag VA[31:22]
//   L2 256e = 32 sets x 8 ways: 4K: set VA[16:12] tag VA[31:17]
//                              4M: set VA[22:20] tag VA[31:22]
//   entry: v tag sz g asid[8:0] pa12=PA[31:12] r w x u a d
//   match: entry.g==1 || entry.asid==satp.asid (asid = SATP_I[30:22])
//   GRANT_I/WANT_O/BUSY_O: walk-start handshake with the core's m2 arbiter
//     (two instances share one WB read port): a miss without grant latches
//     its context (pend) in S_IDLE (or steps back from S_RESP) and starts
//     when granted -> at most one walker ever presents STB on m2.
//   fill (walk structurally OK, even if this access faults): L1+L2, way =
//     INVALID way first (lowest index), else min stamp; stamp = global 4-bit
//     fill counter (FIFO within sets; hits do not refresh). Invalid-first is
//     required: invalidated ways keep stale stamps, and raw min-stamp across
//     a 4-bit wrap would evict just-filled valid entries (t04 chk42).
//   PA on hit: 4K {pa12, VA[11:0]}; 4M {pa12[11:2], VA[21:0]}
// sfence.vma: SF_I: 00=none 01=all 10=asid (SFASID_I) 11=page(VA_I);
//   page-granular over-flush (spec-safe). A sfence that arrives mid-walk is
//   held pending and applied after the walk's TLB fill (a speculative walk
//   must not create entries the sfence invalidates). satp change:
//   NO auto-flush (software sfence.vma).
// WB B4: read-only, STB asserted 1 cycle per PTE, waits ACK_I (any latency).
//
// Pipelined timing: the result + TLB fill are computed combinationally from
// the incoming PTE (DAT_I) in the ACK cycle, so no S_R bubble remains:
//   hit/Bare DONE on REQ+1; 4M DONE on L1-ACK+1; 4K DONE on L0-ACK+1.
`default_nettype none

module mmu (
    input clk, rst,
    input        REQ_I,
    input  [31:0] VA_I,
    input  [1:0]  ACC_I,     // 0=fetch 1=load 2=store 3=amo
    input  [1:0]  MODE_I,   // 0=M 1=S 2=U
    input        SUM_I, MXR_I,
    input  [31:0] SATP_I,   // MODE[31] ASID[30:22] PPN[21:0]
    input        GRANT_I,   // walk-start grant (core m2 arbiter); hold until 1
    output       DONE_O,
    output       BUSY_O,    // 1 while in a walk (st != S_IDLE): m2 held
    output       WANT_O,    // 1 while wanting to start a walk (S_IDLE + miss/pend)
    output reg [31:0] PA_O,
    output reg   FAULT_O,
    output reg [3:0] CAUSE_O,
    input  [1:0] SF_I,       // 00=none 01=all 10=asid 11=page(VA_I)
    input  [8:0] SFASID_I,
    // WB B4 master (read-only)
    output       STB_O,
    output [3:0] SEL_O,
    output [31:0] ADR_O,
    output       WE_O,
    input        ACK_I,
    input  [31:0] DAT_I
);

    // ------------------------------------------------------------- TLB state
    reg        l1_v   [0:15];
    reg  [17:0] l1_tag [0:15];
    reg        l1_sz  [0:15];
    reg        l1_g   [0:15];
    reg  [8:0]  l1_asid [0:15];
    reg  [19:0] l1_pa12 [0:15];
    reg        l1_r [0:15], l1_w [0:15], l1_x [0:15], l1_u [0:15];
    reg        l1_a [0:15], l1_d [0:15];
    reg  [3:0]  l1_st  [0:15];

    reg        l2_v   [0:255];
    reg  [14:0] l2_tag [0:255];
    reg        l2_sz  [0:255];
    reg        l2_g   [0:255];
    reg  [8:0]  l2_asid [0:255];
    reg  [19:0] l2_pa12 [0:255];
    reg        l2_r [0:255], l2_w [0:255], l2_x [0:255], l2_u [0:255];
    reg        l2_a [0:255], l2_d [0:255];
    reg  [3:0]  l2_st  [0:255];

    reg [3:0] lru;

    // ------------------------------------------------------------- FSM / regs
    localparam S_IDLE = 3'd0, S_W1S = 3'd1, S_W1W = 3'd2,
               S_W2S = 3'd3, S_W2W = 3'd4, S_RESP = 3'd5;
    reg [2:0] st;
    reg [31:0] va_r, satp_r, pte0a_r;
    reg        pend;          // miss latched, awaiting GRANT_I (m2 arbiter)
    reg  [1:0]  acc_r, mode_r;
    reg        sum_r, mxr_r;
    reg  [8:0]  asid_r;
    // sf pending (latched while walking; applied at walk end, post-fill)
    reg        sf_p;
    reg  [1:0]  sf_cls_p;
    reg  [8:0]  sfasid_p;
    reg  [31:0] sfva_p;

    // ------------------------------------------------------------- helpers
    // permission/A-D fault for leaf bits + access context (1 = fault)
    function chk_fault;
        input [1:0] acc, mode;
        input r, w, x, u, a, d;
        input sum, mxr;
        begin
            chk_fault = 1'b0;
            if (mode == 2'd1)
                if (u && (acc == 2'd0 || !sum)) chk_fault = 1'b1;
            if (mode == 2'd2 && !u) chk_fault = 1'b1;
            if (acc == 2'd0 && !x) chk_fault = 1'b1;
            if (acc == 2'd1 && !r && !(mxr && x)) chk_fault = 1'b1;
            if ((acc == 2'd2 || acc == 2'd3) && !w) chk_fault = 1'b1;
            if (acc == 2'd3 && !r) chk_fault = 1'b1;
            if (!a) chk_fault = 1'b1;
            if ((acc == 2'd2 || acc == 2'd3) && !d) chk_fault = 1'b1;
        end
    endfunction

    // ------------------------------------------------------------- lookup
    wire [1:0]  lk1s0 = VA_I[13:12];
    wire [1:0]  lk1s1 = VA_I[21:20];
    wire [4:0]  lk2s0 = VA_I[16:12];
    wire [4:0]  lk2s1 = {2'b00, VA_I[22:20]};
    wire [17:0] lk1t0 = VA_I[31:14];
    wire [9:0]  lk1t1 = VA_I[31:22];
    wire [14:0] lk2t0 = VA_I[31:17];
    wire [9:0]  lk2t1 = VA_I[31:22];
    wire [8:0]  lk_asid = SATP_I[30:22];

    reg        m1a, m1b, m2a, m2b;   // class matches
    reg  [3:0] e1a, e1b;              // L1 matched entry idx (4K / 4M)
    reg  [7:0] e2a, e2b;              // L2 matched entry idx (4K / 4M)
    integer   j;
    always @(*) begin
        m1a = 0; m1b = 0; m2a = 0; m2b = 0;
        e1a = 0; e1b = 0; e2a = 0; e2b = 0;
        for (j = 0; j < 4; j = j + 1) begin
            if (l1_v[{lk1s0, 2'b00} + j] && l1_sz[{lk1s0, 2'b00} + j] == 1'b0 &&
                l1_tag[{lk1s0, 2'b00} + j] == lk1t0 &&
                (l1_g[{lk1s0, 2'b00} + j] ||
                 l1_asid[{lk1s0, 2'b00} + j] == lk_asid)) begin
                m1a = 1; e1a = {lk1s0, 2'b00} + j;
            end
            if (l1_v[{lk1s1, 2'b00} + j] && l1_sz[{lk1s1, 2'b00} + j] == 1'b1 &&
                l1_tag[{lk1s1, 2'b00} + j][9:0] == lk1t1 &&
                (l1_g[{lk1s1, 2'b00} + j] ||
                 l1_asid[{lk1s1, 2'b00} + j] == lk_asid)) begin
                m1b = 1; e1b = {lk1s1, 2'b00} + j;
            end
        end
        for (j = 0; j < 8; j = j + 1) begin
            if (l2_v[{lk2s0, 3'b000} + j] && l2_sz[{lk2s0, 3'b000} + j] == 1'b0 &&
                l2_tag[{lk2s0, 3'b000} + j] == lk2t0 &&
                (l2_g[{lk2s0, 3'b000} + j] ||
                 l2_asid[{lk2s0, 3'b000} + j] == lk_asid)) begin
                m2a = 1; e2a = {lk2s0, 3'b000} + j;
            end
            if (l2_v[{lk2s1, 3'b000} + j] && l2_sz[{lk2s1, 3'b000} + j] == 1'b1 &&
                l2_tag[{lk2s1, 3'b000} + j][9:0] == lk2t1 &&
                (l2_g[{lk2s1, 3'b000} + j] ||
                 l2_asid[{lk2s1, 3'b000} + j] == lk_asid)) begin
                m2b = 1; e2b = {lk2s1, 3'b000} + j;
            end
        end
    end

    // hit fields (priority: L1-4K > L1-4M > L2-4K > L2-4M)
    reg        h_hit, h_sz;
    reg  [19:0] h_pa12;
    reg        h_r, h_w, h_x, h_u, h_a, h_d;
    always @(*) begin
        h_hit = 0; h_sz = 0; h_pa12 = 0;
        h_r = 0; h_w = 0; h_x = 0; h_u = 0; h_a = 0; h_d = 0;
        if (m1a) begin
            h_hit = 1; h_sz = 0; h_pa12 = l1_pa12[e1a];
            h_r = l1_r[e1a]; h_w = l1_w[e1a]; h_x = l1_x[e1a];
            h_u = l1_u[e1a]; h_a = l1_a[e1a]; h_d = l1_d[e1a];
        end else if (m1b) begin
            h_hit = 1; h_sz = 1; h_pa12 = l1_pa12[e1b];
            h_r = l1_r[e1b]; h_w = l1_w[e1b]; h_x = l1_x[e1b];
            h_u = l1_u[e1b]; h_a = l1_a[e1b]; h_d = l1_d[e1b];
        end else if (m2a) begin
            h_hit = 1; h_sz = 0; h_pa12 = l2_pa12[e2a];
            h_r = l2_r[e2a]; h_w = l2_w[e2a]; h_x = l2_x[e2a];
            h_u = l2_u[e2a]; h_a = l2_a[e2a]; h_d = l2_d[e2a];
        end else if (m2b) begin
            h_hit = 1; h_sz = 1; h_pa12 = l2_pa12[e2b];
            h_r = l2_r[e2b]; h_w = l2_w[e2b]; h_x = l2_x[e2b];
            h_u = l2_u[e2b]; h_a = l2_a[e2b]; h_d = l2_d[e2b];
        end
    end
    // pa12 = PA[31:12] (20b). 4K: PA = {pa12, VA[11:0]}.
    // 4M: PA = {PA[31:22], VA[21:0]} = {pa12[19:10], VA[21:0]}.
    wire [31:0] pa_hit = h_sz ? {h_pa12[19:10], VA_I[21:0]} : {h_pa12, VA_I[11:0]};
    wire        sfault = chk_fault(ACC_I, MODE_I, h_r, h_w, h_x, h_u, h_a, h_d,
                                   SUM_I, MXR_I);
    wire        mbare = !SATP_I[31] || MODE_I == 2'd0;

    // m2-arbiter handshake (core: two MMU walkers share the m2 read port).
    // BUSY_O: holding a walk / an m2 read is outstanding (must not preempt).
    // WANT_O: wants to start a walk now (fresh miss or a held/pend miss).
    // The core grants walk-starts so at most one walker is BUSY at a time;
    // a walker holds in S_IDLE (pend) until GRANT_I, so its latched miss is
    // not lost and its STB never collides with the other walker's on m2.
    assign BUSY_O = (st != S_IDLE);
    assign WANT_O = (st == S_IDLE) &&
                    (pend || (REQ_I && !mbare && !h_hit));
    wire [3:0]  cause_req = (ACC_I == 2'd0) ? 4'd12 :
                             (ACC_I == 2'd1) ? 4'd13 : 4'd15;

    // ------------------------------------------------------------- PTE addrs
    wire [33:0] a1  = {12'b0, satp_r[21:0], 12'b0} + {22'b0, va_r[31:22], 2'b0};
    wire [33:0] a2c = {12'b0, DAT_I[31:10], 12'b0} + {22'b0, va_r[21:12], 2'b0};
    assign STB_O = (st == S_W1S) || (st == S_W2S);
    assign SEL_O = 4'b1111;
    assign WE_O  = 1'b0;
    assign ADR_O = (st == S_W1S) ? a1[31:0] : pte0a_r;

    // W1W decode (structural): leaf = R|X; reserved perms = R=0,W=1
    wire        w1_leaf = DAT_I[1] | DAT_I[3];
    wire        w1_bad = !DAT_I[0] ||
                          (!DAT_I[1] && DAT_I[2]) ||
                          (!DAT_I[1] && !DAT_I[3] &&
                           (DAT_I[7] | DAT_I[6] | DAT_I[4]));

    // -------------------------------------------------- incoming PTE compute
    // PTE = DAT_I (the word the WB read just returned). frm_eff is known per
    // state: 1 in W1W (4M leaf), 0 in W2W (4K leaf). Computed combinationally
    // so the result + fill are ready the same cycle the PTE ACK lands.
    wire        frm_eff = (st == S_W1W);
    wire [31:0] pte_i   = DAT_I;
    wire [21:0] pi_ppn  = pte_i[31:10];
    wire pi_v = pte_i[0], pi_r = pte_i[1], pi_w = pte_i[2], pi_x = pte_i[3];
    wire pi_u = pte_i[4], pi_g = pte_i[5], pi_a = pte_i[6], pi_d = pte_i[7];
    wire pi_leaf = pi_r | pi_x;
    wire r_bad_i = !pi_v ||
                   (!pi_r && pi_w) ||
                   (!pi_leaf && (pi_d | pi_a | pi_u)) ||
                   (frm_eff && pi_leaf && pi_ppn[9:0] != 10'd0) ||
                   (!frm_eff && !pi_leaf);
    wire [31:0] pa_i = frm_eff ? {pi_ppn[19:10], va_r[21:0]} :
                                    {pi_ppn[19:0], va_r[11:0]};
    wire r_pf_i = chk_fault(acc_r, mode_r, pi_r, pi_w, pi_x, pi_u, pi_a, pi_d,
                            sum_r, mxr_r);
    wire [3:0]  cause_i = (acc_r == 2'd0) ? 4'd12 :
                          (acc_r == 2'd1) ? 4'd13 : 4'd15;
    wire fl_ok_i = !r_bad_i;

    // 1-ahead: DONE_O is high for the S_RESP cycle (1 cycle per serviced req).
    // A new REQ may arrive during S_RESP (sampled at the leaving edge): hit
    // latches its result and stays in S_RESP (DONE next cycle); miss latches
    // ctx and starts a walk. Sustained rate on hits = 1 translation/cycle.
    assign DONE_O = (st == S_RESP);

    // ------------------------------------------------------------- fill idx
    wire [1:0]  f1s = frm_eff ? va_r[21:20] : va_r[13:12];
    wire [4:0]  f2s = frm_eff ? {2'b00, va_r[22:20]} : va_r[16:12];
    wire [17:0] f1t = frm_eff ? {8'b0, va_r[31:22]} : va_r[31:14];
    wire [14:0] f2t = frm_eff ? {5'b0, va_r[31:22]} : va_r[31:17];
    // victim way: an INVALID way first (lowest index), else min stamp. Stale
    // stamps on invalidated ways must never make a fill evict a VALID entry
    // while the set has spare capacity (raw min-stamp alone mis-picks when
    // the 4-bit counter wraps vs pre-sfence stamps -- found by t04 chk42).
    reg  [1:0]  fw1;
    reg         iv1;
    integer     k1;
    always @(*) begin
        fw1 = 2'd0; iv1 = 1'b0;
        for (k1 = 0; k1 < 4; k1 = k1 + 1)
            if (!iv1 && !l1_v[{f1s, 2'b00} + k1]) begin
                iv1 = 1'b1; fw1 = k1[1:0];
            end
        if (!iv1)
            for (k1 = 1; k1 < 4; k1 = k1 + 1)
                if (l1_st[{f1s, 2'b00} + k1] < l1_st[{f1s, 2'b00} + 2'd0])
                    fw1 = k1[1:0];
    end
    reg  [2:0]  fw2;
    reg         iv2;
    integer     k2;
    always @(*) begin
        fw2 = 3'd0; iv2 = 1'b0;
        for (k2 = 0; k2 < 8; k2 = k2 + 1)
            if (!iv2 && !l2_v[{f2s, 3'b000} + k2]) begin
                iv2 = 1'b1; fw2 = k2[2:0];
            end
        if (!iv2)
            for (k2 = 1; k2 < 8; k2 = k2 + 1)
                if (l2_st[{f2s, 3'b000} + k2] < l2_st[{f2s, 3'b000} + 2'd0])
                    fw2 = k2[2:0];
    end
    wire [3:0] fe1 = {f1s, fw1};
    wire [7:0] fe2 = {f2s, fw2};
    // fill only on a structurally-valid LEAF PTE (4M in W1W, 4K in W2W)
    wire do_fill = fl_ok_i &&
                   (((st == S_W1W) && ACK_I && w1_leaf) ||
                    ((st == S_W2W) && ACK_I));

    // sfence sources: sfa = pending (latched mid-walk), sfb = direct input
    reg  [1:0]  sfa_cls;
    reg  [8:0]  sfa_asid;
    reg  [31:0] sfa_va;
    reg  [1:0]  sfb_cls;
    reg  [8:0]  sfb_asid;
    reg  [31:0] sfb_va;
    always @(*) begin
        sfa_cls = 2'd0; sfa_asid = 0; sfa_va = 0;
        if (sf_p) begin
            sfa_cls = sf_cls_p; sfa_asid = sfasid_p; sfa_va = sfva_p;
        end
        sfb_cls = SF_I; sfb_asid = SFASID_I; sfb_va = VA_I;
    end

    // ------------------------------------------------------------- update
    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st <= S_IDLE;
            PA_O <= 32'h0;
            FAULT_O <= 1'b0;
            CAUSE_O <= 4'h0;
            sf_p <= 1'b0;
            pend <= 1'b0;
            lru <= 4'd0;
            for (i = 0; i < 16; i = i + 1)
                l1_v[i] <= 1'b0;
            for (i = 0; i < 256; i = i + 1)
                l2_v[i] <= 1'b0;
        end else begin
            // sf pending: latch while walking; clear at walk end / idle
            if (SF_I && st != S_IDLE) begin
                sf_p <= 1'b1;
                sf_cls_p <= SF_I;
                sfasid_p <= SFASID_I;
                sfva_p <= VA_I;
            end
            if (st != S_W1S && st != S_W1W && st != S_W2S && st != S_W2W)
                sf_p <= 1'b0;

            case (st)
            S_IDLE: begin
                if (pend) begin
                    // held miss awaiting the m2-arbiter grant; start once granted
                    if (GRANT_I) begin st <= S_W1S; pend <= 1'b0; end
                end else if (REQ_I) begin
                    if (mbare) begin
                        PA_O <= VA_I; FAULT_O <= 1'b0; CAUSE_O <= 4'h0;
                        st <= S_RESP;
                    end else if (!h_hit) begin
                        va_r <= VA_I;
                        satp_r <= SATP_I;
                        acc_r <= ACC_I;
                        mode_r <= MODE_I;
                        sum_r <= SUM_I;
                        mxr_r <= MXR_I;
                        asid_r <= SATP_I[30:22];
                        if (GRANT_I) begin st <= S_W1S; pend <= 1'b0; end
                        else pend <= 1'b1;
                    end else begin
                        PA_O <= pa_hit; FAULT_O <= sfault;
                        CAUSE_O <= cause_req;
                        st <= S_RESP;
                    end
                end
            end
            S_W1S: st <= S_W1W;
            S_W1W: if (ACK_I) begin
                if (w1_bad || w1_leaf) begin
                    // 4M terminal result (frm_eff=1), computed from DAT_I
                    PA_O <= pa_i; FAULT_O <= !(fl_ok_i && !r_pf_i);
                    CAUSE_O <= cause_i;
                    st <= S_RESP;
                end else begin
                    pte0a_r <= a2c[31:0];
                    st <= S_W2S;
                end
            end
            S_W2S: st <= S_W2W;
            S_W2W: if (ACK_I) begin
                // 4K terminal result (frm_eff=0), computed from DAT_I
                PA_O <= pa_i; FAULT_O <= !(fl_ok_i && !r_pf_i);
                CAUSE_O <= cause_i;
                st <= S_RESP;
            end
            S_RESP: begin
                // 1-ahead: accept a new request during this DONE cycle
                if (REQ_I) begin
                    if (mbare) begin
                        PA_O <= VA_I; FAULT_O <= 1'b0; CAUSE_O <= 4'h0;
                    end else if (!h_hit) begin
                        va_r <= VA_I;
                        satp_r <= SATP_I;
                        acc_r <= ACC_I;
                        mode_r <= MODE_I;
                        sum_r <= SUM_I;
                        mxr_r <= MXR_I;
                        asid_r <= SATP_I[30:22];
                        // GRANT_I must be honored here too (m2 arbiter):
                        // entering S_W1S un-granted would collide this
                        // walker's STB with the other walker's m2 read.
                        if (GRANT_I) st <= S_W1S;
                        else begin st <= S_IDLE; pend <= 1'b1; end
                    end else begin
                        PA_O <= pa_hit; FAULT_O <= sfault;
                        CAUSE_O <= cause_req;
                    end
                end else
                    st <= S_IDLE;
            end
            default: ;
            endcase

            // TLB fill: in the present cycle (4M in W1W, 4K in W2W)
            if (do_fill) begin
                l1_v[fe1] <= 1'b1;
                l1_tag[fe1] <= f1t;
                l1_sz[fe1] <= frm_eff;
                l1_g[fe1] <= pi_g;
                l1_asid[fe1] <= asid_r;
                l1_pa12[fe1] <= pa_i[31:12];
                l1_r[fe1] <= pi_r; l1_w[fe1] <= pi_w; l1_x[fe1] <= pi_x;
                l1_u[fe1] <= pi_u; l1_a[fe1] <= pi_a; l1_d[fe1] <= pi_d;
                l1_st[fe1] <= lru;
                l2_v[fe2] <= 1'b1;
                l2_tag[fe2] <= f2t;
                l2_sz[fe2] <= frm_eff;
                l2_g[fe2] <= pi_g;
                l2_asid[fe2] <= asid_r;
                l2_pa12[fe2] <= pa_i[31:12];
                l2_r[fe2] <= pi_r; l2_w[fe2] <= pi_w; l2_x[fe2] <= pi_x;
                l2_u[fe2] <= pi_u; l2_a[fe2] <= pi_a; l2_d[fe2] <= pi_d;
                l2_st[fe2] <= lru;
                lru <= lru + 1'b1;
            end

            // sfence flush: after fill above, so a pending/direct sfence
            // beats an in-flight walk's fill (spec: speculative walks must
            // not create entries the sfence invalidates)
            for (i = 0; i < 16; i = i + 1) begin
                if (sfa_cls == 2'b01 ||
                    (sfa_cls == 2'b10 && !l1_g[i] && l1_asid[i] == sfa_asid) ||
                    (sfa_cls == 2'b11 &&
                     ((l1_sz[i] == 1'b0 &&
                       {l1_tag[i], i[3:2]} == sfa_va[31:12]) ||
                      (l1_sz[i] == 1'b1 && l1_tag[i][9:0] == sfa_va[31:22]))))
                    l1_v[i] <= 1'b0;
                if (sfb_cls == 2'b01 ||
                    (sfb_cls == 2'b10 && !l1_g[i] && l1_asid[i] == sfb_asid) ||
                    (sfb_cls == 2'b11 &&
                     ((l1_sz[i] == 1'b0 &&
                       {l1_tag[i], i[3:2]} == sfb_va[31:12]) ||
                      (l1_sz[i] == 1'b1 && l1_tag[i][9:0] == sfb_va[31:22]))))
                    l1_v[i] <= 1'b0;
            end
            for (i = 0; i < 256; i = i + 1) begin
                if (sfa_cls == 2'b01 ||
                    (sfa_cls == 2'b10 && !l2_g[i] && l2_asid[i] == sfa_asid) ||
                    (sfa_cls == 2'b11 &&
                     ((l2_sz[i] == 1'b0 &&
                       {l2_tag[i], i[7:3]} == sfa_va[31:12]) ||
                      (l2_sz[i] == 1'b1 && l2_tag[i][9:0] == sfa_va[31:22]))))
                    l2_v[i] <= 1'b0;
                if (sfb_cls == 2'b01 ||
                    (sfb_cls == 2'b10 && !l2_g[i] && l2_asid[i] == sfb_asid) ||
                    (sfb_cls == 2'b11 &&
                     ((l2_sz[i] == 1'b0 &&
                       {l2_tag[i], i[7:3]} == sfb_va[31:12]) ||
                      (l2_sz[i] == 1'b1 && l2_tag[i][9:0] == sfb_va[31:22]))))
                    l2_v[i] <= 1'b0;
            end
        end
    end

endmodule
`default_nettype wire
