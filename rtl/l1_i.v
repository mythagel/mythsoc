// l1_i -- 4 KiB direct-mapped L1 instruction cache (mythrv32)
//
// 1-word lines: 1024 x 32b. Index PA[11:2], tag PA[31:12].
// WB B4 read-only master (SoC master m0): 1 STB pulse per transfer,
// ADR valid on the STB cycle, waits ACK_I (any latency >= 1), 1
// outstanding. No STALL/ERR/CYC (wb_ic sec.6 protocol).
//
// Request interface (from IF; 1 outstanding):
//   REQ_I / INVAL_I are 1-cycle pulses, asserted only while idle
//   (DONE_O low, no transfer in flight).
//   hit   : DONE_O 1 cycle after REQ (latency 1); DATA_O valid on DONE.
//   miss  : one WB read; DONE_O 1 cycle after the memory ACK
//           (1-cyc slave: latency 3 from REQ).
//   INVAL : clears all lines (fence.i / mret / sret), 1 cycle, no DONE.
// Reset: v[]=0; tag/mem left X (dead lines: v=0 gates the tag compare).

`default_nettype none

module l1_i (
    input clk, rst,
    input        REQ_I,
    input [31:0] PA_I,
    input        INVAL_I,
    output reg   DONE_O,
    output reg [31:0] DATA_O,
    // WB B4 master (read-only)
    output       STB_O,
    output [3:0] SEL_O,
    output [31:0] ADR_O,
    output       WE_O,
    input        ACK_I,
    input [31:0] DAT_I
);

    localparam W_IDLE = 3'd0, W_W1S = 3'd1, W_W1W = 3'd2, W_RESP = 3'd3;
    reg [1:0] st;

    reg        v   [0:1023];
    reg [19:0] tag [0:1023];
    reg [31:0] mem [0:1023];

    reg [31:0] adr_r;

    wire [9:0]  idx = PA_I[11:2];
    wire       hit = v[idx] && (tag[idx] == PA_I[31:12]);

    assign STB_O = (st == W_W1S);
    assign SEL_O = 4'b1111;
    assign WE_O  = 1'b0;
    assign ADR_O = adr_r;

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st <= W_IDLE;
            DONE_O <= 1'b0;
            adr_r <= 32'h0;
            for (i = 0; i < 1024; i = i + 1)
                v[i] <= 1'b0;
        end else begin
            DONE_O <= 1'b0;
            case (st)
            W_IDLE: begin
                if (REQ_I) begin
                    if (hit) begin
                        DATA_O <= mem[idx];
                        DONE_O <= 1'b1;
                    end else begin
                        adr_r <= PA_I;
                        st <= W_W1S;
                    end
                end else if (INVAL_I) begin
                    for (i = 0; i < 1024; i = i + 1)
                        v[i] <= 1'b0;
                end
            end
            W_W1S: st <= W_W1W;
            W_W1W: if (ACK_I) begin
                v[adr_r[11:2]] <= 1'b1;
                tag[adr_r[11:2]] <= adr_r[31:12];
                mem[adr_r[11:2]] <= DAT_I;
                DATA_O <= DAT_I;
                DONE_O <= 1'b1;
                st <= W_RESP;
            end
            W_RESP: st <= W_IDLE;
            default: ;
            endcase
        end
    end

endmodule
`default_nettype wire
