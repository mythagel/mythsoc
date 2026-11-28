// tb_soc_sdram -- board data-path rehearsal: FULL SoC with DRAM_SDRAM=1
// (wb_sdram controller instead of wb_ram) against tb/sdram_model.v.
//
// This is what tb_wb_sdram.v cannot prove: the core + wb_ic + caches +
// fetch/PC tables driving the SDRAM slave under real traffic, i.e. the
// board bring-up path (rtl/top.v + the wb_sdram generate branch of
// rtl/soc.v) minus the pin-level pads. The device model applies its
// independent timing/coherency assertions to everything the core does.
//
// Protocol: same fw result contract as tb_fw.v -- here we PEEK THE MODEL:
//   word w of DRAM (w = PA[21:2] = {row[9:0], bank[1:0], col[7:0]}) lives
//   at mem[{bank, row, col}] (the model's device ordering).
// Default image: t01 ISA battery (ROM fetches -> wb_ic s0; loads/stores ->
// s1 SDRAM). Knob: -DROM_HEX + -DWDT=<cycles>.
// build+run: make soc_sdram
`default_nettype none
`timescale 1ns / 1ps

module tb_soc_sdram;
    `include "tb/sdram_params.vh"

    localparam [31:0] MARK   = 32'h5A5A_0000;
`ifdef WDT
    localparam integer WDT_CYC = `WDT;
`else
    localparam integer WDT_CYC = 300000;   // t01 ~10k commits; cold SDRAM
                                           // refill cost adds a few x
`endif

    reg clk = 0;
    always #20 clk = ~clk;                 // 25 MHz = board sysclk
    reg rst = 1;

    // ------------------------------------------------------------ SoC (board
    wire [1:0]  ba;
    wire [12:0] sa;
    wire        ras_n, cas_n, we_n, cs_n, cke, dq_oe;
    wire [3:0]  dqm;
    wire [31:0] dq_o, dq_i;
    wire        txd;

    soc #(.ROM_DEPTH(8192), .ROM_HEX(`ROM_HEX),
          .UART_CLK(100), .UART_BAUD(20),
          .CLK_HZ(25000000), .DRAM_SDRAM(1)) dut (
        .clk(clk), .rst(rst),
        .uart_rxd(1'b1), .uart_txd(txd),
        .sd_ba_o(ba), .sd_a_o(sa),
        .sd_ras_n_o(ras_n), .sd_cas_n_o(cas_n), .sd_we_n_o(we_n),
        .sd_cs_n_o(cs_n), .sd_cke_o(cke), .sd_dqm_o(dqm),
        .sd_dq_o(dq_o), .sd_dq_oe_o(dq_oe), .sd_dq_i(dq_i)
    );

    wire [31:0] dq;
    assign dq_i = dq;

    sdram_model mdl (
        .clk(clk), .rst(rst),
        .ba(ba), .sa(sa), .ras_n(ras_n), .cas_n(cas_n), .we_n(we_n),
        .cs_n(cs_n), .cke(cke), .dqm(dqm), .dq_oe(dq_oe), .dq_o(dq_o),
        .dq(dq)
    );

    // DRAM-word view of the device array (PA[21:2] -> device {ba,row,col})
    function [31:0] dw;
        input [19:0] w;
        begin
            dw = mdl.mem[{w[9:8], w[19:10], w[7:0]}];
        end
    endfunction

    // ------------------------------------------------ WB s1 usage counters
    integer s1_stb_c = 0, s1_ack_c = 0;
    always @(posedge clk) begin
        if (dut.s1_stb) s1_stb_c = s1_stb_c + 1;
        if (dut.s1_ack) s1_ack_c = s1_ack_c + 1;
    end

    integer npass = 0, nfail = 0;
    task chk(input cond, input [319:0] name);
    begin
        if (cond) npass = npass + 1;
        else begin
            nfail = nfail + 1;
            $display("FAIL @%0t: %0s", $time, name);
        end
    end endtask

    // ------------------------------------------------------------ run
    reg done = 0;
    integer settle = 0;

    initial begin : MAIN
        #200; rst = 0;
        while (!done && mdl.mcyc < WDT_CYC) @(negedge clk);
        chk(mdl.mcyc < WDT_CYC,               "R1 finished inside watchdog");
        chk(done,                             "R2 pass marker in SDRAM");
        chk(dw(0) === 32'h0,                  "R3 fw result = 0");
        chk(mdl.m_mrs_done,                   "R4 MRS programmed");
        chk(mdl.ref_total > 0,                "R5 refresh ran during battery");
        chk(mdl.nfail == 0,                   "R6 device model: zero violations");
        chk(s1_stb_c == s1_ack_c,             "R7 s1 STB/ACK balance");
        $display("soc_sdram: %0d cyc, %0d s1 ops (stb=%0d ack=%0d), %0d REFs",
                 mdl.mcyc, s1_stb_c, s1_stb_c, s1_ack_c, mdl.ref_total);
        if (nfail != 0) $display("FAILURES PRESENT (%0d)", nfail);
        else            $display("ALL TESTS PASSED (%0d checks)", npass);
        $finish;
    end

    always @(posedge clk) begin
        if (!rst && !done) begin
            // marker written LAST by fw_done: result word then marker word
            if (dw(1) === MARK) begin
                settle = settle + 1;
                if (settle > 32) done = 1;   // drain posted writes
            end
        end
    end

    initial begin
        #(WDT_CYC * 40 + 20000);
        $display("FAIL: WATCHDOG (nfail=%0d npass=%0d cyc=%0d dw1=%h)",
                 nfail, npass, mdl.mcyc, dw(1));
        $display("pc=%h fpc=%h", dut.core0.exma_pc, dut.core0.fpc);
        $finish;
    end

endmodule
`default_nettype wire
