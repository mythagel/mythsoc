// mmu -- Sv32 MMU (mythrv32): page-table walker + 2-level TLB, WB B4 RO master
//
// Spec: riscv-privileged 12.3 (Sv32), 12.1.11 (satp). Address math and the
// reference walk model live in tools/mmu_addr.py (source of truth for
// tb/tb_mmu.v expectations).
//
// Request protocol: REQ_I is a 1-cycle pulse, asserted only when idle
// (DONE_O low). The requester samples results while DONE_O is high.
//   hit / M-mode / Bare : DONE next cycle (1)
//   miss 4K            : 6 cyc (2 PTE reads, 1-cyc slave)
//   miss 4M            : 4 cyc (1 PTE read)
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
//   permission/A-D (re-checked on hit, so per-access): fetch needs X; load
//     R (MXR=1: R|X); store/AMO W (AMO also R; cause always store). S: U=1
//     pages fault unless (SUM=1 && !fetch); fetch on U=1 always faults in S.
//     U-mode: needs U=1. A/D (Svade fault scheme): A=0 fault; store/AMO
//     with D=0 fault. (No hardware A/D update.)
// TLB (per instance):
//   L1 16e = 4 sets x 4 ways:  4K: set VA[13:12] tag VA[31:14]
//                              4M: set VA[21:20] tag VA[31:22]
//   L2 256e = 32 sets x 8 ways: 4K: set VA[16:12] tag VA[31:17]
//                              4M: set VA[22:20] tag VA[31:22]
//   entry: v tag sz g asid[8:0] pa12=PA[31:12] r w x u a d
//   match: entry.g==1 || entry.asid==satp.asid (asid = SATP_I[30:22])
//   fill (walk structurally OK, even if this access faults): L1+L2, way =
//     min stamp, stamp = global 4-bit LRU counter.
//   PA on hit: 4K {pa12, VA[11:0]}; 4M {pa12[11:2], VA[21:0]}
// sfence.vma: SF_I: 00=none 01=all 10=asid (SFASID_I) 11=page(VA_I);
//   page-granular over-flush (spec-safe). A pending sfence is applied at
//   the end of the in-flight walk, before its TLB fill. satp change:
//   NO auto-flush (software sfence.vma).
// WB B4: read-only, STB asserted 1 cycle per PTE, waits ACK_I (any latency).
`default_nettype none

module mmu (
    input clk, rst,
    input        REQ_I,
    input  [31:0] VA_I,
    input  [1:0]  ACC_I,     // 0=fetch 1=load 2=store 3=amo
    input  [1:0]  MODE_I,   // 0=M 1=S 2=U
    input        SUM_I, MXR_I,
    input  [31:0] SATP_I,   // MODE[31] ASID[30:22] PPN[21:0]
    output reg   DONE_O,
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
    reg  [11:0] l1_pa12 [0:15];
    reg        l1_r [0:15], l1_w [0:15], l1_x [0:15], l1_u [0:15];
    reg        l1_a [0:15], l1_d [0:15];
    reg  [3:0]  l1_st  [0:15];

    reg        l2_v   [0:255];
    reg  [14:0] l2_tag [0:255];
    reg        l2_sz  [0:255];
    reg        l2_g   [0:255];
    reg  [8:0]  l2_asid [0:255];
    reg  [11:0] l2_pa12 [0:255];
    reg        l2_r [0:255], l2_w [0:255], l2_x [0:255], l2_u [0:255];
    reg        l2_a [0:255], l2_d [0:255];
    reg  [3:0]  l2_st  [0:255];

    reg [3:0] lru;

    // ------------------------------------------------------------- FSM / regs
    localparam S_IDLE = 3'd0, S_W1S = 3'd1, S_W1W = 3'd2,
               S_W2S = 3'd3, S_W2W = 3'd4, S_R = 3'd5, S_RESP = 3'd6;
    reg [2:0] st;
    reg [31:0] va_r, satp_r, pte0a_r;
    reg  [1:0]  acc_r, mode_r;
    reg        sum_r, mxr_r;
    reg  [8:0]  asid_r;
    reg  [31:0] pte1_r, pte0_r;
    reg        from1;             // R: result from L1 PTE (4M leaf / L1 fault)
    // result (latched in R or IDLE-hit; presented in RESP)
    reg        res_ok;
    reg  [31:0] res_pa;
    reg  [3:0]  res_cause;
    // sf pending (latched while busy; applied at end of walk, pre-fill)
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
    integer   k1, k2;
    always @(*) begin
        m1a = 0; m1b = 0; m2a = 0; m2b = 0;
        e1a = 0; e1b = 0; e2a = 0; e2b = 0;
        for (j = 0; j < 4; j = j + 1) begin
            if (l1_v[lk1s0 * 4 + j] && l1_sz[lk1s0 * 4 + j] == 1'b0 &&
                l1_tag[lk1s0 * 4 + j] == lk1t0 &&
                (l1_g[lk1s0 * 4 + j] || l1_asid[lk1s0 * 4 + j] == lk_asid)) begin
                m1a = 1; e1a = lk1s0 * 4 + j;
            end
            if (l1_v[lk1s1 * 4 + j] && l1_sz[lk1s1 * 4 + j] == 1'b1 &&
                l1_tag[lk1s1 * 4 + j][9:0] == lk1t1 &&
                (l1_g[lk1s1 * 4 + j] || l1_asid[lk1s1 * 4 + j] == lk_asid)) begin
                m1b = 1; e1b = lk1s1 * 4 + j;
            end
        end
        for (j = 0; j < 8; j = j + 1) begin
            if (l2_v[lk2s0 * 8 + j] && l2_sz[lk2s0 * 8 + j] == 1'b0 &&
                l2_tag[lk2s0 * 8 + j] == lk2t0 &&
                (l2_g[lk2s0 * 8 + j] || l2_asid[lk2s0 * 8 + j] == lk_asid)) begin
                m2a = 1; e2a = lk2s0 * 8 + j;
            end
            if (l2_v[lk2s1 * 8 + j] && l2_sz[lk2s1 * 8 + j] == 1'b1 &&
                l2_tag[lk2s1 * 8 + j][9:0] == lk2t1 &&
                (l2_g[lk2s1 * 8 + j] || l2_asid[lk2s1 * 8 + j] == lk_asid)) begin
                m2b = 1; e2b = lk2s1 * 8 + j;
            end
        end
    end

    // hit fields (priority: L1-4K > L1-4M > L2-4K > L2-4M)
    reg        h_hit, h_sz;
    reg  [11:0] h_pa12;
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
    wire [31:0] pa_hit = h_sz ? {h_pa12[11:2], VA_I[21:0]} : {h_pa12, VA_I[11:0]};
    wire        sfault = chk_fault(ACC_I, MODE_I, h_r, h_w, h_x, h_u, h_a, h_d,
                                   SUM_I, MXR_I);
    wire        mbare = !SATP_I[31] || MODE_I == 2'd0;
    wire [3:0]  cause_req = (ACC_I == 2'd0) ? 4'd12 :
                            (ACC_I == 2'd1) ? 4'd13 : 4'd15;

    // ------------------------------------------------------------- PTE addrs
    wire [33:0] a1  = {2'b0, satp_r[21:0], 12'b0} + {34'b0, va_r[31:22], 2'b0};
    wire [33:0] a2c = {2'b0, DAT_I[31:10], 12'b0} + {34'b0, va_r[21:12], 2'b0};
    assign STB_O = (st == S_W1S) || (st == S_W2S);
    assign SEL_O = 4'b1111;
    assign WE_O  = 1'b0;
    assign ADR_O = (st == S_W1S) ? a1[31:0] : pte0a_r;

    // W1W decode (structural)
    wire        w1_leaf = DAT_I[1] | DAT_I[3];
    wire        w1_bad = !DAT_I[0] ||
                         (!DAT_I[1] && DAT_I[2]) ||
                         (!DAT_I[1] && !DAT_I[3] && (DAT_I[6] | DAT_I[5] | DAT_I[4]));

    // ------------------------------------------------------------- R compute
    wire [31:0] pte_c = from1 ? pte1_r : pte0_r;
    wire [21:0] pc_ppn = pte_c[31:10];
    wire pc_v = pte_c[0], pc_r = pte_c[1], pc_w = pte_c[2], pc_x = pte_c[3];
    wire pc_u = pte_c[4], pc_g = pte_c[5], pc_a = pte_c[6], pc_d = pte_c[7];
    wire pc_leaf = pc_r | pc_x;
    wire r_bad = !pc_v || (!pc_r && pc_w) ||
                 (from1 && pc_leaf && pc_ppn[9:0] != 10'd0) ||
                 (!from1 && !pc_leaf);
    wire [31:0] pa_c = from1 ? {pc_ppn[19:10], va_r[21:0]} :
                                       {pc_ppn[19:0], va_r[11:0]};
    wire r_pf = chk_fault(acc_r, mode_r, pc_r, pc_w, pc_x, pc_u, pc_a, pc_d,
                          sum_r, mxr_r);
    wire [3:0]  cause_r = (acc_r == 2'd0) ? 4'd12 :
                          (acc_r == 2'd1) ? 4'd13 : 4'd15;

    // fill fields (valid in R, applied leaving R)
    wire        fl_ok = !r_bad;
    wire [1:0]  f1s = from1 ? va_r[21:20] : va_r[13:12];
    wire [4:0]  f2s = from1 ? {2'b00, va_r[22:20]} : va_r[16:12];
    wire [17:0] f1t = from1 ? {8'b0, va_r[31:22]} : va_r[31:14];
    wire [14:0] f2t = from1 ? {5'b0, va_r[31:22]} : va_r[31:17];
    reg  [1:0]  fw1;
    always @(*) begin
        fw1 = 2'd0;
        for (k1 = 1; k1 < 4; k1 = k1 + 1)
            if (l1_st[f1s * 4 + k1] < l1_st[f1s * 4 + 2'd0]) fw1 = k1[1:0];
    end
    reg  [2:0]  fw2;
    always @(*) begin
        fw2 = 3'd0;
        for (k2 = 1; k2 < 8; k2 = k2 + 1)
            if (l2_st[f2s * 8 + k2] < l2_st[f2s * 8 + 2'd0]) fw2 = k2[2:0];
    end
    wire [3:0] fe1 = f1s * 4 + fw1;
    wire [7:0] fe2 = {f2s, fw2};

    // sfence kill (inline; cls 01=all 10=asid 11=page)
    // applied leaving R (pending) or at IDLE (direct/pending)
    reg [1:0]  sfa_cls;
    reg [8:0]  sfa_asid;
    reg [31:0] sfa_va;
    reg [1:0]  sfb_cls;
    reg [8:0]  sfb_asid;
    reg [31:0] sfb_va;
    integer    i;
    always @(*) begin
        sfa_cls = 2'd0; sfa_asid = 0; sfa_va = 0;
        sfb_cls = 2'd0; sfb_asid = 0; sfb_va = 0;
        if (sf_p) begin
            sfa_cls = sf_cls_p; sfa_asid = sfasid_p; sfa_va = sfva_p;
        end
        if (SF_I) begin
            sfb_cls = SF_I; sfb_asid = SFASID_I; sfb_va = VA_I;
        end
    end

    // ------------------------------------------------------------- update
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st <= S_IDLE;
            DONE_O <= 1'b0;
            PA_O <= 32'h0;
            FAULT_O <= 1'b0;
            CAUSE_O <= 4'h0;
            res_ok <= 1'b0;
            res_pa <= 32'h0;
            res_cause <= 4'h0;
            sf_p <= 1'b0;
            lru <= 4'd0;
            for (i = 0; i < 16; i = i + 1) begin
                l1_v[i] <= 1'b0; l1_st[i] <= 4'd0;
            end
            for (i = 0; i < 256; i = i + 1) begin
                l2_v[i] <= 1'b0; l2_st[i] <= 4'd0;
            end
        end else begin
            // sfence latch (direct apply happens in flush below)
            if (SF_I && st != S_IDLE) begin
                sf_p <= 1'b1;
                sf_cls_p <= SF_I;
                sfasid_p <= SFASID_I;
                sfva_p <= VA_I;
            end

            // flush (before fill when leaving R: fill wins conflicts,
            // correct order -- sfence precedes the walk architecturally)
            for (i = 0; i < 16; i = i + 1) begin
                if (sfa_cls == 2'b01 ||
                    (sfa_cls == 2'b10 && !l1_g[i] && l1_asid[i] == sfa_asid) ||
                    (sfa_cls == 2'b11 &&
                     ((l1_sz[i] == 1'b0 &&
                       {l1_tag[i], 2'(i)} == sfa_va[31:12]) ||
                      (l1_sz[i] == 1'b1 && l1_tag[i][9:0] == sfa_va[31:22]))))
                    l1_v[i] <= 1'b0;
                if (sfb_cls == 2'b01 ||
                    (sfb_cls == 2'b10 && !l1_g[i] && l1_asid[i] == sfb_asid) ||
                    (sfb_cls == 2'b11 &&
                     ((l1_sz[i] == 1'b0 &&
                       {l1_tag[i], 2'(i)} == sfb_va[31:12]) ||
                      (l1_sz[i] == 1'b1 && l1_tag[i][9:0] == sfb_va[31:22]))))
                    l1_v[i] <= 1'b0;
            end
            for (i = 0; i < 256; i = i + 1) begin
                if (sfa_cls == 2'b01 ||
                    (sfa_cls == 2'b10 && !l2_g[i] && l2_asid[i] == sfa_asid) ||
                    (sfa_cls == 2'b11 &&
                     ((l2_sz[i] == 1'b0 &&
                       {l2_tag[i], i[4:0]} == sfa_va[31:12]) ||
                      (l2_sz[i] == 1'b1 && l2_tag[i][9:0] == sfa_va[31:22]))))
                    l2_v[i] <= 1'b0;
                if (sfb_cls == 2'b01 ||
                    (sfb_cls == 2'b10 && !l2_g[i] && l2_asid[i] == sfb_asid) ||
                    (sfb_cls == 2'b11 &&
                     ((l2_sz[i] == 1'b0 &&
                       {l2_tag[i], i[4:0]} == sfb_va[31:12]) ||
                      (l2_sz[i] == 1'b1 && l2_tag[i][9:0] == sfb_va[31:22]))))
                    l2_v[i] <= 1'b0;
            end

            case (st)
            S_IDLE: begin
                if (SF_I) sf_p <= 1'b0;
                if (REQ_I) begin
                    if (mbare) begin
                        res_ok <= 1'b1;
                        res_pa <= VA_I;
                        res_cause <= 4'h0;
                    end else if (h_hit) begin
                        res_ok <= !sfault;
                        res_pa <= pa_hit;
                        res_cause <= cause_req;
                    end else begin
                        va_r <= VA_I;
                        satp_r <= SATP_I;
                        acc_r <= ACC_I;
                        mode_r <= MODE_I;
                        sum_r <= SUM_I;
                        mxr_r <= MXR_I;
                        asid_r <= SATP_I[30:22];
                    end
                    st <= mbare || !h_hit ?
                          (mbare ? S_RESP : S_W1S) : S_RESP;
                end
            end
            S_W1S: st <= S_W1W;
            S_W1W: if (ACK_I) begin
                pte1_r <= DAT_I;
                if (w1_bad || w1_leaf) begin
                    from1 <= 1'b1;
                    st <= S_R;
                end else begin
                    pte0a_r <= a2[31:0];
                    st <= S_W2S;
                end
            end
            S_W2S: st <= S_W2W;
            S_W2W: if (ACK_I) begin
                pte0_r <= DAT_I;
                from1 <= 1'b0;
                st <= S_R;
            end
            S_R: begin
                sf_p <= 1'b0;
                res_ok <= fl_ok && !r_pf;
                res_pa <= pa_c;
                res_cause <= cause_r;
                if (fl_ok) begin
                    l1_v[fe1] <= 1'b1;
                    l1_tag[fe1] <= f1t;
                    l1_sz[fe1] <= from1;
                    l1_g[fe1] <= pc_g;
                    l1_asid[fe1] <= asid_r;
                    l1_pa12[fe1] <= pa_c[31:12];
                    l1_r[fe1] <= pc_r; l1_w[fe1] <= pc_w; l1_x[fe1] <= pc_x;
                    l1_u[fe1] <= pc_u; l1_a[fe1] <= pc_a; l1_d[fe1] <= pc_d;
                    l1_st[fe1] <= lru;
                    l2_v[fe2] <= 1'b1;
                    l2_tag[fe2] <= f2t;
                    l2_sz[fe2] <= from1;
                    l2_g[fe2] <= pc_g;
                    l2_asid[fe2] <= asid_r;
                    l2_pa12[fe2] <= pa_c[31:12];
                    l2_r[fe2] <= pc_r; l2_w[fe2] <= pc_w; l2_x[fe2] <= pc_x;
                    l2_u[fe2] <= pc_u; l2_a[fe2] <= pc_a; l2_d[fe2] <= pc_d;
                    l2_st[fe2] <= lru;
                    lru <= lru + 1'b1;
                end
                st <= S_RESP;
            end
            S_RESP: st <= S_IDLE;
            default: ;
            endcase

            // outputs (presented in RESP)
            DONE_O <= (st == S_RESP);
            if (st == S_RESP) begin
                PA_O <= res_pa;
                FAULT_O <= !res_ok;
                CAUSE_O <= res_cause;
            end
        end
    end

endmodule
`default_nettype wire
