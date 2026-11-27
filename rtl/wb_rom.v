// wb_rom -- Wishbone B4 ROM slave (mythrv32 boot ROM, base 0x0, 32 KiB).
//
// Read-only, 1-cycle ACK. Word index = ADR_I[14:2] (32 KiB / 4 = 8192 words).
// Contents loaded from the HEX file (parameter ROM_HEX) via $readmemh; words
// not in the file default to 32'h00000013 (ADDI x0,x0,0 = nop). WE/SEL
// ignored (read-only).
`default_nettype none

module wb_rom #(
    parameter DEPTH  = 8192,
    parameter ROM_HEX = ""
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
    wire [13:0] idx = ADR_I[14:2];

    reg [31:0] mem [0:DEPTH-1];
    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1) mem[i] = 32'h00000013;
        if (ROM_HEX != "") $readmemh(ROM_HEX, mem);
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin           // WB B4 3.1.1: interfaces init to a defined
            ACK_O <= 1'b0;       // state (review F15; rst port was unused)
            DAT_O <= 32'h0;
        end else if (STB_I) begin
            ACK_O <= 1'b1;
            DAT_O <= mem[idx];
        end else ACK_O <= 1'b0;
    end

endmodule
`default_nettype wire
