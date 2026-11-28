// wb_ic_mfifo -- 4-entry shift FIFO, per-master queue of wb_ic (mythrv32)
//
// Entry: 69 bits {we, sel[3:0], adr[31:0], wdat[31:0]}.
//   push: 1-cycle pulse, din valid; ignored when full (protocol: master must
//          not push while full/STALL).
//   pop:  1-cycle pulse; head = oldest entry. push+pop in same cycle is legal
//          (oldest popped, din pushed to tail, count unchanged).
//   head: comb, valid when cnt > 0.
//   full: cnt == 4 (drives wb_ic STALL).
`default_nettype none

module wb_ic_mfifo (
    input  clk, rst,
    input  push, pop,
    input  [68:0] din,
    output [68:0] head,
    output [2:0] cnt,
    output       full
);

    reg [68:0] e [0:3];
    reg [2:0]  c;
    integer    i;

    assign head = e[0];
    assign cnt  = c;
    assign full = (c == 3'd4);

    // SYNCHRONOUS reset: the e[] entries make this the array-writer
    // process; clk-only keeps it $mem-inferable (async-reset writers get
    // replaced by a register list). Entry array is fully reset to 0 here,
    // so the sync clear preserves the old defined-state behavior.
    always @(posedge clk) begin
        if (rst) begin
            c   <= 3'd0;
            e[0] <= 69'h0;
            e[1] <= 69'h0;
            e[2] <= 69'h0;
            e[3] <= 69'h0;
        end else begin
            if (push && pop) begin
                if (c == 3'd0) begin
                    // the pushed entry IS the popped one: no state change
                end else begin
                    for (i = 1; i < 4; i = i + 1) e[i-1] <= e[i];
                    if (c < 3'd4) begin
                        e[c - 3'd1] <= din;   // tail after shift; c unchanged
                    end else begin
                        e[3] <= din;          // c == 4: shift out, tail in
                    end
                end
            end else if (push) begin
                if (c < 3'd4) begin
                    e[c] <= din;
                    c <= c + 3'd1;
                end
            end else if (pop) begin
                if (c != 3'd0) begin
                    for (i = 0; i < 3; i = i + 1) e[i] <= e[i+1];
                    c <= c - 3'd1;
                end
            end
        end
    end

endmodule
`default_nettype wire
