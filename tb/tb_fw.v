// tb_fw -- generic firmware harness for the tests/ suite (design.md sec.10)
//
// Boots soc from a ROM hex (macro `ROM_HEX, set via iverilog -D), waits for
// the result-block marker (DRAM[1] bit 23, written last by fw_tb.S
// fw_done), then checks DRAM[0]:
//   0          = pass
//   0x8000_000N = check N failed
//   0x8000_1000+c = unexpected trap (cause c)
// Watchdog: no marker within 1M cycles -> FAIL + core state dump.
//
// Run: see scripts/fw_asm.sh (build) -- the Makefile builds each test group
// into its own hex and invokes this TB with -D ROM_HEX='"<hex>"'.
`timescale 1ns/1ps
module tb_fw;

    reg clk = 1, rst = 0;
    always #10 clk = ~clk;

    integer errors = 0, passes = 0;
    task err(input [255:0] s);
        begin errors = errors + 1; $display("FAIL: %0s", s); end
    endtask
    task ok(input [255:0] s);
        begin passes = passes + 1; $display("PASS: %0s", s); end
    endtask

    // UART RX held idle (1); tests that need RX drive it via dut ports later
    // (loopback mux kept for UART tests: loop=1 feeds txd to rxd).
    reg rxd_r = 1'b1;
    reg loop = 1'b0;
    wire txd;
    soc #(.ROM_DEPTH(4096), .ROM_HEX(`ROM_HEX), .RAM_DEPTH(65536),
          .UART_CLK(100), .UART_BAUD(20)) dut (
        .clk(clk), .rst(rst),
        .uart_rxd(loop ? txd : rxd_r), .uart_txd(txd)
    );

`ifndef WDT
`define WDT 300000
`endif

    // dump the result block + data area (helps localize a failing check)
    integer di;
    task dram_dump();
        begin
            for (di = 0; di < 32; di = di + 1)
                $display("  dram[%0d] = %h", di, dut.s1ram.mem[di]);
        end
    endtask

    integer t;
    initial begin
        @(negedge clk); rst = 1;
        repeat (3) @(negedge clk); rst = 0;

        // wait for the result marker (DRAM word 1, written last) + watchdog
        t = 0;
        while (dut.s1ram.mem[1] !== {16'h5A5A, 16'h0000} && t < `WDT) begin
            @(negedge clk); t = t + 1;
        end
        if (dut.s1ram.mem[1] !== {16'h5A5A, 16'h0000}) begin
            err("FW-timeout");
            $display("  result=%h (DRAM[0]) fpc=%h exma_pc=%h mcause=%h mepc=%h priv=%b",
                     dut.s1ram.mem[0], dut.core0.fpc, dut.core0.exma_pc,
                     dut.core0.mcause, dut.core0.mepc, dut.core0.priv);
            dram_dump();
            $finish;
        end
        if (dut.s1ram.mem[0] === 32'h0) begin ok("FW-pass"); end
        else begin
            err("FW-result");
            $display("  result=%h mcause=%h mepc=%h priv=%b",
                     dut.s1ram.mem[0], dut.core0.mcause, dut.core0.mepc,
                     dut.core0.priv);
            dram_dump();
        end
        if ($test$plusargs("DUMP")) dram_dump();
        if (errors == 0) $display("ALL TESTS PASSED (%0d checks)", passes);
        else $display("%0d ERRORS (%0d passed)", errors, passes);
        $finish;
    end
endmodule
