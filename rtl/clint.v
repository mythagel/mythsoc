// clint -- RISC-V CLINT, Wishbone B4 slave.  (mythrv32)
//
// Registers (byte addressable, base 0x0200_0000, region 64 KiB;
// word index = byte_off[15:2], byte lane = byte_off[1:0]):
//   0x0000  msip[0]    [0]=hart0 machine software interrupt pending
//   0x0004  sstip[0]   [0]=hart0 supervisor software interrupt pending (ext)
//   0x4000  mtimecmp[0] [31:0]
//   0x4004  mtimecmp[0] [63:32]  (overlaps stimecmp[0][31:0])
//   0x4008  stimecmp[0] [63:32]
//   0x7000  mtip[0]    [0]=hart0 machine timer interrupt pending
//   0xBFF8  mtime [31:0]   (read-only)
//   0xBFFC  mtime [63:32]  (read-only)
// 0x4004 overlap: reads return mtimecmp[63:32]; writes update BOTH
// mtimecmp[63:32] and stimecmp[31:0] (CLINT-spec address overlap).
//
// mtime: 64-bit free-running, +1 per cycle.
// Interrupt outputs (level, to PLIC sources):
//   ip_ssw  = sstip[0]            (src 2)
//   ip_msw  = msip[0]             (src 3)
//   ip_stmr = mtime >= stimecmp   (src 6)
//   ip_mtmr = mtime >= mtimecmp   (src 7)
//   ip_sext = mtip[0]             (src 10)
//   ip_mext = mtip[0]             (src 11)
// Comparisons unsigned (priv spec: "greater than or equal", unsigned).
//
// Reset: mtimecmp/stimecmp = all 1 (no pending), msip/sstip/mtip = 0.
// BIT FORMAT (priv 3.2 msip; design sec.4; review F14): these are
// per-hart-flag registers -- bits 31:1 read as ZERO, bit 0 = hart 0 flag.
// WB B4: single cycle; DAT_O valid when ACK_O; no ERR/STALL.
`default_nettype none

module clint (
    input clk, rst,
    // Wishbone B4 slave
    input        STB_I,
    input        WE_I,
    input  [3:0] SEL_I,
    input  [31:0] ADR_I,
    input  [31:0] DAT_I,
    output reg [31:0] DAT_O,
    output reg        ACK_O,
    // interrupt outputs (level)
    output ip_ssw, ip_msw, ip_stmr, ip_mtmr, ip_sext, ip_mext
);

    wire in_range = (ADR_I[31:16] == 16'h0200);

    // ------------------------------------------------------------- mtime
    reg [63:0] mtime;
    always @(posedge clk or posedge rst) begin
        if (rst) mtime <= 64'h0;
        else     mtime <= mtime + 64'h1;
    end

    // ------------------------------------------------------------- regs
    reg [31:0] msip;
    reg [31:0] sstip;
    reg [31:0] mtip;
    reg [63:0] mtimecmp;
    reg [63:0] stimecmp;

    assign ip_ssw  = sstip[0];
    assign ip_msw  = msip[0];
    assign ip_stmr = (mtime >= stimecmp);
    assign ip_mtmr = (mtime >= mtimecmp);
    assign ip_sext = mtip[0];
    assign ip_mext = mtip[0];

    // byte offsets of the 4 lanes within the region (ADR[15:0] + lane)
    wire [15:0] b0 = ADR_I[15:0];
    wire [15:0] b1 = b0 + 16'h1;
    wire [15:0] b2 = b0 + 16'h2;
    wire [15:0] b3 = b0 + 16'h3;

    // register word at byte offset b (0 if none)
    function [31:0] reg_word;
        input [15:0] b;
        begin
            case (b[15:2])
            14'h0000:  reg_word = msip;
            14'h0001:  reg_word = sstip;
            14'h1C00:  reg_word = mtip;
            14'h1000:  reg_word = mtimecmp[31:0];
            14'h1001:  reg_word = mtimecmp[63:32];
            14'h1002:  reg_word = stimecmp[63:32];
            14'h2FFE:  reg_word = mtime[31:0];
            14'h2FFF:  reg_word = mtime[63:32];
            default:   reg_word = 32'h0;
            endcase
        end
    endfunction

    // byte of the register word at byte offset b
    function [7:0] reg_byte;
        input [15:0] b;
        reg [31:0] w;
        begin
            w = reg_word(b);
            case (b[1:0])
            2'd0:  reg_byte = w[7:0];
            2'd1:  reg_byte = w[15:8];
            2'd2:  reg_byte = w[23:16];
            2'd3:  reg_byte = w[31:24];
            endcase
        end
    endfunction

    // merge byte v into byte position s of word cur
    function [31:0] merge_byte;
        input [31:0] cur;
        input [7:0]  v;
        input [1:0]  s;
        begin
            case (s)
            2'd0:  merge_byte = {cur[31:8],  v};
            2'd1:  merge_byte = {cur[31:16], v, cur[7:0]};
            2'd2:  merge_byte = {cur[31:24], v, cur[15:0]};
            2'd3:  merge_byte = {v, cur[23:0]};
            endcase
        end
    endfunction

    reg [31:0] wt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            msip <= 32'h0;
            sstip <= 32'h0;
            mtip <= 32'h0;
            mtimecmp <= 64'hFFFF_FFFF_FFFF_FFFF;
            stimecmp <= 64'hFFFF_FFFF_FFFF_FFFF;
            ACK_O <= 1'b0;
            DAT_O <= 32'h0;
        end else begin
            ACK_O <= 1'b0;
            if (in_range && STB_I) begin
                ACK_O <= 1'b1;
                if (WE_I) begin
                    // msip (0x0000)
                    wt = msip;
                    if (SEL_I[0] && b0[15:2] == 14'h0000) wt = merge_byte(wt, DAT_I[7:0],  b0[1:0]);
                    if (SEL_I[1] && b1[15:2] == 14'h0000) wt = merge_byte(wt, DAT_I[15:8], b1[1:0]);
                    if (SEL_I[2] && b2[15:2] == 14'h0000) wt = merge_byte(wt, DAT_I[23:16], b2[1:0]);
                    if (SEL_I[3] && b3[15:2] == 14'h0000) wt = merge_byte(wt, DAT_I[31:24], b3[1:0]);
                    msip <= {31'h0, wt[0]};
                    // sstip (0x0004)
                    wt = sstip;
                    if (SEL_I[0] && b0[15:2] == 14'h0001) wt = merge_byte(wt, DAT_I[7:0],  b0[1:0]);
                    if (SEL_I[1] && b1[15:2] == 14'h0001) wt = merge_byte(wt, DAT_I[15:8], b1[1:0]);
                    if (SEL_I[2] && b2[15:2] == 14'h0001) wt = merge_byte(wt, DAT_I[23:16], b2[1:0]);
                    if (SEL_I[3] && b3[15:2] == 14'h0001) wt = merge_byte(wt, DAT_I[31:24], b3[1:0]);
                    sstip <= {31'h0, wt[0]};
                    // mtip (0x7000)
                    wt = mtip;
                    if (SEL_I[0] && b0[15:2] == 14'h1C00) wt = merge_byte(wt, DAT_I[7:0],  b0[1:0]);
                    if (SEL_I[1] && b1[15:2] == 14'h1C00) wt = merge_byte(wt, DAT_I[15:8], b1[1:0]);
                    if (SEL_I[2] && b2[15:2] == 14'h1C00) wt = merge_byte(wt, DAT_I[23:16], b2[1:0]);
                    if (SEL_I[3] && b3[15:2] == 14'h1C00) wt = merge_byte(wt, DAT_I[31:24], b3[1:0]);
                    mtip <= {31'h0, wt[0]};
                    // mtimecmp[31:0] (0x4000)
                    wt = mtimecmp[31:0];
                    if (SEL_I[0] && b0[15:2] == 14'h1000) wt = merge_byte(wt, DAT_I[7:0],  b0[1:0]);
                    if (SEL_I[1] && b1[15:2] == 14'h1000) wt = merge_byte(wt, DAT_I[15:8], b1[1:0]);
                    if (SEL_I[2] && b2[15:2] == 14'h1000) wt = merge_byte(wt, DAT_I[23:16], b2[1:0]);
                    if (SEL_I[3] && b3[15:2] == 14'h1000) wt = merge_byte(wt, DAT_I[31:24], b3[1:0]);
                    mtimecmp[31:0] <= wt;
                    // mtimecmp[63:32] / stimecmp[31:0] (0x4004 overlap)
                    wt = mtimecmp[63:32];
                    if (SEL_I[0] && b0[15:2] == 14'h1001) wt = merge_byte(wt, DAT_I[7:0],  b0[1:0]);
                    if (SEL_I[1] && b1[15:2] == 14'h1001) wt = merge_byte(wt, DAT_I[15:8], b1[1:0]);
                    if (SEL_I[2] && b2[15:2] == 14'h1001) wt = merge_byte(wt, DAT_I[23:16], b2[1:0]);
                    if (SEL_I[3] && b3[15:2] == 14'h1001) wt = merge_byte(wt, DAT_I[31:24], b3[1:0]);
                    mtimecmp[63:32] <= wt;
                    wt = stimecmp[31:0];
                    if (SEL_I[0] && b0[15:2] == 14'h1001) wt = merge_byte(wt, DAT_I[7:0],  b0[1:0]);
                    if (SEL_I[1] && b1[15:2] == 14'h1001) wt = merge_byte(wt, DAT_I[15:8], b1[1:0]);
                    if (SEL_I[2] && b2[15:2] == 14'h1001) wt = merge_byte(wt, DAT_I[23:16], b2[1:0]);
                    if (SEL_I[3] && b3[15:2] == 14'h1001) wt = merge_byte(wt, DAT_I[31:24], b3[1:0]);
                    stimecmp[31:0] <= wt;
                    // stimecmp[63:32] (0x4008)
                    wt = stimecmp[63:32];
                    if (SEL_I[0] && b0[15:2] == 14'h1002) wt = merge_byte(wt, DAT_I[7:0],  b0[1:0]);
                    if (SEL_I[1] && b1[15:2] == 14'h1002) wt = merge_byte(wt, DAT_I[15:8], b1[1:0]);
                    if (SEL_I[2] && b2[15:2] == 14'h1002) wt = merge_byte(wt, DAT_I[23:16], b2[1:0]);
                    if (SEL_I[3] && b3[15:2] == 14'h1002) wt = merge_byte(wt, DAT_I[31:24], b3[1:0]);
                    stimecmp[63:32] <= wt;
                    // mtime (0xBFF8/0xBFFC): read-only
                end else begin
                    if (SEL_I[0]) DAT_O[7:0]    <= reg_byte(b0);
                    if (SEL_I[1]) DAT_O[15:8]  <= reg_byte(b1);
                    if (SEL_I[2]) DAT_O[23:16] <= reg_byte(b2);
                    if (SEL_I[3]) DAT_O[31:24] <= reg_byte(b3);
                end
            end
        end
    end

endmodule
`default_nettype wire
