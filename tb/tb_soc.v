// tb_soc -- mythrv32 SoC top testbench (simulation)
//
// Instantiates soc (core + wb_ic + ROM/DRAM/16550/CLINT/PLIC) with a small
// firmware loaded into ROM (tb/tb_soc_fw.hex). The firmware runs in ROM
// (M-mode Bare), uses DRAM, and writes a sentinel to DRAM word 5 last. The
// TB waits for the sentinel, then checks DRAM words 0..5 against the
// emu32 golden.
//
// Build: see tb/tb_soc_fw.S -> tb/tb_soc_fw.hex (clang + llvm-objcopy).
// Run:
//   iverilog -g2005 -s tb_soc tb/tb_soc.v rtl/soc.v rtl/core.v rtl/mmu.v \
//     rtl/l1_i.v rtl/l1_d.v rtl/wb_ic.v rtl/wb_ic_mfifo.v rtl/wb_rom.v \
//     rtl/wb_ram.v rtl/16550.v rtl/clint.v rtl/plic.v -o tb_soc.vvp
//   vvp tb_soc.vvp
`timescale 1ns/1ps
module tb_soc;

    reg clk = 1, rst = 0;
    always #10 clk = ~clk;

    integer errors = 0, passes = 0;
    task err(input [255:0] s);
        begin errors = errors + 1; $display("FAIL: %0s", s); end
    endtask
    task ok(input [255:0] s);
        begin passes = passes + 1; $display("PASS: %0s", s); end
    endtask

    reg uart_rxd = 1'b1;
    wire uart_txd;

    // sim sizes: ROM 8 KiB (firmware is tiny), DRAM 256 KiB (firmware < 64 B),
    // UART fast (CLK_FREQ=100, BAUD=20 -> DIV=5).
    soc #(.ROM_DEPTH(2048), .ROM_HEX("tb/tb_soc_fw.hex"), .RAM_DEPTH(65536),
          .UART_CLK(100), .UART_BAUD(20)) dut (
        .clk(clk), .rst(rst),
        .uart_rxd(uart_rxd), .uart_txd(uart_txd)
    );

    // golden (emu32): DRAM words 0..5
    reg [31:0] mexp [0:5];
    integer i;
    initial begin
        mexp[0] = 32'haabbccdd;
        mexp[1] = 32'h557799ba;
        mexp[2] = 32'h0000002a;   // 42
        mexp[3] = 32'h0000000e;   // 14
        mexp[4] = 32'h000013ba;   // 5050 = sum 1..100
        mexp[5] = 32'hdeadbeef;   // sentinel
    end

    integer t, done = 0;
    initial begin
        @(negedge clk); rst = 1;
        repeat (3) @(negedge clk); rst = 0;

        // wait for the sentinel (DRAM word 5) with a watchdog
        t = 0;
        while (dut.s1ram.mem[5] !== 32'hdeadbeef && t < 200000) begin
            @(posedge clk); t = t + 1;
        end
        if (dut.s1ram.mem[5] === 32'hdeadbeef) begin done = 1; ok("SOC-sentinel"); end
        else err("SOC-timeout");

        // check DRAM words 0..5
        for (i = 0; i < 6; i = i + 1) begin
            if (dut.s1ram.mem[i] !== mexp[i])
                $display("  dram[%0d] got %h exp %h", i, dut.s1ram.mem[i], mexp[i]);
        end
        if (dut.s1ram.mem[0] !== mexp[0]) err("SOC-dram0-ld");
        else ok("SOC-dram0-ld");
        if (dut.s1ram.mem[1] !== mexp[1]) err("SOC-dram1-lduse");
        else ok("SOC-dram1-lduse");
        if (dut.s1ram.mem[2] !== mexp[2]) err("SOC-dram2-mul");
        else ok("SOC-dram2-mul");
        if (dut.s1ram.mem[3] !== mexp[3]) err("SOC-dram3-div");
        else ok("SOC-dram3-div");
        if (dut.s1ram.mem[4] !== mexp[4]) err("SOC-dram4-sum");
        else ok("SOC-dram4-sum");
        if (dut.s1ram.mem[5] !== mexp[5]) err("SOC-dram5-sentinel");
        else ok("SOC-dram5-sentinel");

        if (errors == 0) $display("ALL TESTS PASSED (%0d checks)", passes);
        else $display("%0d ERRORS (%0d passed)", errors, passes);
        $finish;
    end
endmodule
