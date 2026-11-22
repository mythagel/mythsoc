// wb_ram -- Wishbone B4 RAM slave (mythrv32 DRAM).
//
// Protocol (wb_ic sec. slave): STB_I = 1-cycle pulse (one transfer);
// {WE_I,SEL_I,ADR_I,DAT_I} valid on that cycle. ACK_O = 1 cycle later (1-cycle
// slave). DAT_O valid on ACK_O (reads). Byte-select writes (SEL_I).
// Word index = ADR_I[21:2] (the 4 MiB DRAM region, base 0x8000_0000).
//
// DEPTH is a power of two; for the board use 1048576 (4 MiB / 4). Sim can
// use a smaller DEPTH as long as firmware stays in range (index < DEPTH).
// mem is NOT pre-zeroed (X) to keep elab fast; firmware must write before
// reading a word (the TB checks only written words).
`default_nettype none

module wb_ram #(
    parameter DEPTH = 1048576
)(
    input clk, rst,
    input         STB_I,
    input         WE_I,
    input  [3:0]  SEL_I,
    input  [31:0] ADR_I,
    input  [31:0] DAT_I,
    output reg [31:0] DAT_O,
    output reg        ACK_O
);
    localparam [19:0] IDXW_HI = 20;
    wire [19:0] idx = ADR_I[21:2];

    reg [31:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (STB_I) begin
            if (WE_I) begin
                if (SEL_I[0]) mem[idx][7:0]   <= DAT_I[7:0];
                if (SEL_I[1]) mem[idx][15:8]  <= DAT_I[15:8];
                if (SEL_I[2]) mem[idx][23:16] <= DAT_I[23:16];
                if (SEL_I[3]) mem[idx][31:24] <= DAT_I[31:24];
            end
            ACK_O <= 1'b1;
            DAT_O <= mem[idx];
        end else ACK_O <= 1'b0;
    end

endmodule
`default_nettype wire
