// l1_d -- 4 KiB direct-mapped L1 data cache (mythrv32)
//
// 1-word lines: 1024 x 32b. Index PA[11:2], tag PA[31:12].
// Write-through; STORE miss writes memory directly (line NOT filled); store
// hit writes line + writes through. LOAD miss fetches AND FILLS — but only
// for the cacheable window (design.md sec.2: DRAM PA[31:22]==10'h200, plus
// ROM PA[31:22]==10'h000 -- immutable).
// DEVICE PAs (PLIC/CLINT/UART) are strict pass-through: no hit ever,
// no fill — device reads have side effects (PLIC claim, UART R0 pop) and
// must never see stale line data (tb_soc p2 storm, session 17).
// WB B4 master (SoC master m1, read/write): 1 STB pulse per transfer,
// {WE,ADR,DAT} valid on the STB cycle, waits ACK_I (any latency >= 1),
// 1 outstanding. No STALL/ERR/CYC (wb_ic sec.6 protocol).
//
// Request interface (from MA; 1 outstanding, 1-ahead on DONE):
//   REQ_I is a 1-cycle pulse, asserted while idle (DONE_O low) OR
//   during the previous transfer's DONE cycle (hit or miss; DATA_O is
//   a held register, sampled the DONE cycle).
//   WEN_I: 0=load, 1=store; WDAT_I/SEL_I valid on REQ for stores
//   (SEL_I byte lanes pass to the WB write; loads/fills stay 4'b1111).
//   load hit  : DONE_O 1 cycle after REQ (latency 1); DATA_O valid.
//   load miss : WB read, line filled, DONE 1 cycle after memory ACK
//               (1-cyc slave: latency 3 from REQ).
//   store (hit or miss): WB write, DONE 1 cycle after memory ACK
//               (1-cyc slave: latency 3 from REQ).
// Reset: v[]=0; tag/mem left X (dead lines: v=0 gates the tag compare).

`default_nettype none

module l1_d (
    input clk, rst,
    input        REQ_I,
    input [3:0]  SEL_I,
    input [31:0] PA_I,
    input        WEN_I,
    input [31:0] WDAT_I,
    output reg   DONE_O,
    output reg [31:0] DATA_O,
    // WB B4 master (read/write)
    output       STB_O,
    output [3:0] SEL_O,
    output [31:0] ADR_O,
    output       WE_O,
    output [31:0] WDAT_O,
    input        ACK_I,
    input [31:0] DAT_I
);

    localparam W_IDLE = 3'd0, W_W1S = 3'd1, W_W1W = 3'd2;
    localparam [9:0] DRAM_REG = 10'h200;   // PA[31:22] of DRAM (design.md sec.2)
    reg [1:0] st;

    reg        v   [0:1023];
    reg [19:0] tag [0:1023];
    reg [31:0] mem [0:1023];

    reg [31:0] adr_r, wd_r;
    reg [3:0]  sel_r;
    reg        wen_r;

    wire [9:0]  idx = PA_I[11:2];
    // cacheable: DRAM (0x8000_0000) or ROM (0x0000_0000, immutable).
    // Devices (CLINT/PLIC/UART) must NEVER fill/hit: reads have side effects.
    wire        cacheable = (PA_I[31:22] == 10'h200) || (PA_I[31:22] == 10'h000);
    wire       hit = cacheable && v[idx] && (tag[idx] == PA_I[31:12]);

    assign STB_O = (st == W_W1S);
    assign SEL_O = (st == W_W1S) && wen_r ? sel_r : 4'b1111;
    assign WE_O  = (st == W_W1S) && wen_r;
    assign ADR_O = adr_r;
    assign WDAT_O = wd_r;

    integer i, j;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st <= W_IDLE;
            DONE_O <= 1'b0;
            adr_r <= 32'h0;
            wd_r <= 32'h0;
            sel_r <= 4'h0;
            wen_r <= 1'b0;
            for (i = 0; i < 1024; i = i + 1)
                v[i] <= 1'b0;
        end else begin
            DONE_O <= 1'b0;
            case (st)
            W_IDLE: begin
                if (REQ_I) begin
                    adr_r <= PA_I;
                    wen_r <= WEN_I;
                    wd_r  <= WDAT_I;
                    sel_r <= SEL_I;
                    if (WEN_I) begin
                        // store: byte-merge line if present (hit), WB write
                        if (hit) begin
                            for (j = 0; j < 4; j = j + 1)
                                if (SEL_I[j])
                                    mem[idx][j*8 +: 8] <= WDAT_I[j*8 +: 8];
                        end
                        st <= W_W1S;
                    end else if (hit) begin
                        // load hit: serve from line
                        DATA_O <= mem[idx];
                        DONE_O <= 1'b1;
                    end else begin
                        // load miss: WB read + fill
                        st <= W_W1S;
                    end
                end
            end
            W_W1S: st <= W_W1W;
            W_W1W: if (ACK_I) begin
                if (!wen_r) begin
                    // load: data always from memory; fill ONLY cacheable lines
                    DATA_O <= DAT_I;
                    if (adr_r[31:22] == DRAM_REG || adr_r[31:22] == 10'h000) begin
                        v[adr_r[11:2]] <= 1'b1;
                        tag[adr_r[11:2]] <= adr_r[31:12];
                        mem[adr_r[11:2]] <= DAT_I;
                    end
                end
                DONE_O <= 1'b1;
                st <= W_IDLE;  // DONE cycle accepts 1-ahead REQ (hit or miss)
            end
            default: ;
            endcase
        end
    end

endmodule
`default_nettype wire
