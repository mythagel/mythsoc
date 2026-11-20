// plic -- RISC-V PLIC v1.0.0 (16 sources, 2 contexts: 0=M, 1=S), WB B4 slave
// (mythrv32)
//
// Spec: specs/riscv-plic.pdf (ratified v1.0.0) ch.3-9; register map and
// decode verified by tools/plic_addr.py (1M-word region scan).
// Base 0x0C00_0000 (wb_ic decode ADR[31:22]==10'h030); word access only
// (spec ch.3: atomic LW/SW; SEL ignored).
//
// Registers (offset from base; N=1..16, c=0/1):
//   +0x000004*N       source N priority (32b WARL; word0=src0 reserved;
//                     words 17..1023 not implemented)
//   +0x001000        pending word 0 (bit N = src N, bit 0 = 0) [RO]
//   +0x002000+c*0x80  enable word 0 for ctx c (bit N = src N)
//   +0x200000+c*0x1000  priority threshold for ctx c
//   +0x200004+c*0x1000  claim/complete for ctx c
//   all other offsets: reserved (read 0, write ignored)
//
// Semantics (spec ch.4-9):
//   prio: 0 = never interrupt; higher = higher; tie -> lower ID.
//   IP: latched on first assertion of the level input while the gateway is
//       armed; not cleared by deassert; cleared by claim; re-set 1 cycle
//       after completion if the level is still asserted (no other path).
//   claim (read claim/complete): atomically select highest (prio desc,
//       ID asc) among IP & enable[c] & prio>0, clear its IP, return ID
//       (0 if none). Not affected by threshold.
//   complete (write ID in 1..16 to claim/complete): re-arms the gateway;
//       ID 0 / out of range: silently ignored.
//   threshold: sources with prio <= thr[c] masked from eip[c].
//   EIP[c] (level out -> meip/seip): OR over N of IP[N] & en[c][N] &
//       prio[N] > thr[c].
//
// Reset: prio=0 (disabled), en=0, thr=0, IP=0, gateways armed.
// WB B4: single STB, ACK 1 cycle later, RDAT valid on ACK cycle.
`default_nettype none

module plic (
    input clk, rst,
    // Wishbone B4 slave
    input        STB_I,
    input        WE_I,
    input  [3:0] SEL_I,
    input  [31:0] ADR_I,
    input  [31:0] DAT_I,
    output reg [31:0] DAT_O,
    output reg        ACK_O,
    // interrupt inputs (level-sensitive, bit N = source N, N=1..16)
    input  [16:0] IRQ,
    // interrupt notifications (level, bit c = ctx c: 0=M, 1=S)
    output [1:0] EIP
);

    // ------------------------------------------------------------- decode
    // ADR[21:12] == offset[21:12] (no carry: base[21:12]==0, region 4 MiB;
    // verified by tools/plic_addr.py over the full 1M-word region).
    wire [11:0] h12 = ADR_I[21:12];
    wire [9:0]  src = ADR_I[11:2];         // priority word = source (1..1023)
    wire [3:0]  srcidx = src[4:0] - 5'd1;  // prio[] index (src 1..16 -> 0..15)
    wire [5:0]  ectx  = ADR_I[12:7];       // enable ctx (0..1 valid)
    wire [1:0]  ectxi = ectx[1:0];
    wire [5:0]  tc    = h12 - 12'h200;     // thr/claim ctx (0..1 valid)
    wire [1:0]  tcx   = tc[1:0];

    wire d_pr  = (h12 == 12'h000);
    wire d_pen = (h12 == 12'h001) && (ADR_I[11:2] == 10'd0);
    wire d_en  = (h12 == 12'h002) && (ectx < 6'd2) && (ADR_I[6:2] == 5'd0);
    wire d_tc  = (h12 >= 12'h200) && (h12 <= 12'h201);
    wire d_thr = d_tc && (ADR_I[11:2] == 10'd0);
    wire d_cl  = d_tc && (ADR_I[11:2] == 10'd1);

    // ------------------------------------------------------------- state
    // source N stored at index N-1 (prio/ip/arm); en bits: bit N = src N
    reg [31:0] prio [0:15];
    reg        ip   [0:15];
    reg        arm  [0:15];
    reg [16:0] en   [0:1];
    reg [31:0] thr  [0:1];
    integer    n;

    // claim/complete this cycle (one WB transfer/cycle: read XOR write)
    // claim selection: max(prio desc, tie -> low ID) among IP & en[c] &
    // prio>0. (always block: iverilog mis-evaluates the equivalent
    // function with an integer loop index over the arrays.)
    reg [4:0]  cid0, cid1;
    reg [31:0] bp0, bp1;
    reg [4:0]  bi;
    always @(*) begin
        bp0  = 32'h0; cid0 = 5'd0;
        bp1  = 32'h0; cid1 = 5'd0;
        for (bi = 5'd0; bi < 5'd16; bi = bi + 5'd1) begin
            if (ip[bi] && en[0][bi + 5'd1] && prio[bi] > 32'h0)
                if (prio[bi] > bp0) begin
                    bp0  = prio[bi];
                    cid0 = bi[4:0] + 5'd1;
                end
            if (ip[bi] && en[1][bi + 5'd1] && prio[bi] > 32'h0)
                if (prio[bi] > bp1) begin
                    bp1  = prio[bi];
                    cid1 = bi[4:0] + 5'd1;
                end
        end
    end

    wire [4:0] cid_c = (tcx == 2'd0) ? cid0 : cid1;
    wire claiming = d_cl && !WE_I && (cid_c != 5'd0);
    wire completing = d_cl && WE_I;
    wire [9:0] comp_id = DAT_I[9:0];
    wire [4:0] comp_idx = comp_id[4:0] - 5'd1;
    wire comp_ok = (comp_id >= 10'd1) && (comp_id <= 10'd16);

    // ------------------------------------------------------------- update
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (n = 0; n < 16; n = n + 1) begin
                prio[n] <= 32'h0;
                ip[n]   <= 1'b0;
                arm[n]  <= 1'b1;
            end
            en[0]  <= 16'h0;
            en[1]  <= 16'h0;
            thr[0] <= 32'h0;
            thr[1] <= 32'h0;
        end else begin
            // gateways: set on first assertion while armed (level latch);
            // claim clears (and disarms); complete re-arms (level re-latches
            // 1 cycle later if still asserted).
            for (n = 0; n < 16; n = n + 1) begin
                ip[n] <= (ip[n] | (arm[n] && IRQ[n + 1] && !ip[n])) &
                         ~(claiming && (cid_c - 5'd1) == n[4:0]);
                arm[n] <= (arm[n] & ~(claiming && (cid_c - 4'd1) == n[3:0])) |
                          (completing && comp_ok && (comp_idx == n[4:0]));
            end
            // register writes (reserved: read 0 / write ignored)
            if (STB_I && WE_I) begin
                if (d_pr) begin
                    if (src >= 10'd1 && src <= 10'd16)
                        prio[srcidx] <= DAT_I;
                end else if (d_en) begin
                    en[ectxi] <= DAT_I[16:0];
                end else if (d_thr) begin
                    thr[tcx] <= DAT_I;
                end
                // d_cl write = complete (handled above); else: reserved
            end
        end
    end

    // ------------------------------------------------------------- reads
    reg [16:0] pendb;
    always @(*) begin
        pendb = 17'h0;
        for (n = 0; n < 16; n = n + 1) pendb[n + 1] = ip[n];
    end

    reg [31:0] rdata_c;
    always @(*) begin
        rdata_c = 32'h0;
        if (d_pr) begin
            if (src >= 10'd1 && src <= 10'd16) rdata_c = prio[srcidx];
        end else if (d_pen) begin
            rdata_c = {15'h0, pendb};
        end else if (d_en) begin
            rdata_c = {15'h0, en[ectxi][16:0]};
        end else if (d_thr) begin
            rdata_c = thr[tcx];
        end else if (d_cl) begin
            rdata_c = {27'h0, cid_c};
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ACK_O <= 1'b0;
            DAT_O <= 32'h0;
        end else begin
            ACK_O <= STB_I;
            if (STB_I && !WE_I)
                DAT_O <= rdata_c;
        end
    end

    // ------------------------------------------------------------- eip
    reg [16:0] gt0, gt1;
    always @(*) begin
        gt0 = 17'h0;
        gt1 = 17'h0;
        for (n = 0; n < 16; n = n + 1) begin
            gt0[n + 1] = ip[n] && en[0][n + 1] && (prio[n] > thr[0]);
            gt1[n + 1] = ip[n] && en[1][n + 1] && (prio[n] > thr[1]);
        end
    end
    assign EIP[0] = |gt0;
    assign EIP[1] = |gt1;

endmodule
`default_nettype wire
