// tb_soc -- mythrv32 SoC top testbench (simulation)
//
// Instantiates soc (core + wb_ic + ROM/DRAM/16550/CLINT/PLIC) with firmware
// from ROM (tb/tb_soc_fw.hex, built by scripts/soc_asm.sh).
//
// Boot phase (firmware, M-mode Bare): ROM fetch -> wb_ic -> DRAM store/load,
// mul/div, loop sum -> DRAM words 0..4, sentinel word 5 last.
//
// Peripheral interrupt phases (end-to-end through the SoC):
//   p1: TB drives 0x2A onto rxd -> 16550 RX -> irq -> PLIC src1 -> EIP[0]
//       -> meip -> core take (mcause 0x8000_000B) -> m_hnd: PLIC claim,
//       pop FIFO byte -> DRAM[6], complete.
//   p2: TB connects txd->rxd (loopback) once p1 is consumed; firmware TXs
//       0x5A -> same path -> DRAM[8]. (Verifies TX engine + loopback RX.)
//   mtime: firmware latches mtime lo around a delay loop -> DRAM[7] < [11].
//   msip: firmware sets CLINT msip -> PLIC src3 -> handler clears it,
//       DRAM[10]=1.
//   m_hnd latches mcause/mepc -> DRAM[12]/[13] on every take.
//
// All waits are bounded (CAP cycles) so the TB always terminates; a stuck
// phase prints P<phase>-STUCK with core/PLIC/UART/CLINT state.
//
// Build: scripts/soc_asm.sh tb/tb_soc_fw.S tb/tb_soc_fw.hex
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

    // sim sizes: ROM 8 KiB, DRAM 256 KiB, UART fast (DIV = 100/20 = 5).
    soc #(.ROM_DEPTH(2048), .ROM_HEX("tb/tb_soc_fw.hex"), .RAM_DEPTH(65536),
          .UART_CLK(100), .UART_BAUD(20)) dut (
        .clk(clk), .rst(rst),
        .uart_rxd(rxd_t), .uart_txd(uart_txd)
    );

    // golden (boot phase): DRAM words 0..5
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

    // RX input: p1 driven by the TB (rxd_r); p2 loopback (rxd_t = txd).
    reg rxd_r = 1'b1;
    reg loop = 1'b0;
    wire rxd_t;
    wire uart_txd;
    assign rxd_t = loop ? uart_txd : rxd_r;
    // storm probes: print every change with sim time
    always @(uart_txd) $display("PROBE txd  t=%0t -> %b (loop=%b)", $time, uart_txd, loop);
    always @(rxd_t)    $display("PROBE rxd_t t=%0t -> %b (loop=%b)", $time, rxd_t, loop);

    // RX bit driver: 1 bit per 5-cycle baud tick (DIV)
    task send_bit(input b);
        begin
            @(negedge clk);
            rxd_r = b;
            repeat (4) @(negedge clk);
        end
    endtask

    task send_rx(input [7:0] byte);
        integer i;
        begin
            send_bit(1'b0);
            for (i = 0; i < 8; i = i + 1)
                send_bit(byte[i]);
            send_bit(1'b1);
        end
    endtask

    // progress checkpoint (counters MUST be initialized: X breaks `%`)
    integer cyc = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (cyc % 2000 == 0) $display("EDGE t=%t cyc=%0d fpc=%h irq=%b ip1=%b eip=%b",
                                      $time, cyc, dut.core0.fpc, dut.s2uart.irq,
                                      dut.s4plic.ip[0], dut.s4plic.EIP[0]);
    end

    // bounded wait: cond expr passed as 4 args (a bop b cop c) evaluated by
    // the caller is impossible in verilog; use a task with a wire probe:
    //   wait_until(expr, cap, label)
    integer t;
    task dump_state(input [255:0] label);
        begin
            $display("  state: fpc=%h priv=%b ier=%b eip0=%b ip1=%b ip3=%b msip=%h m5=%h m9=%h m6=%h",
                     dut.core0.fpc, dut.core0.priv, dut.s2uart.ier[0],
                     dut.s4plic.EIP[0], dut.s4plic.ip[0], dut.s4plic.ip[2],
                     dut.s3clint.msip, dut.s1ram.mem[5], dut.s1ram.mem[9],
                     dut.s1ram.mem[6]);
            $display("  %0s", label);
        end
    endtask

    // bounded waits on observable state (event-driven; no always blocks)
    integer ier_seen = 0, m9_seen = 0, sent = 0;
    reg [31:0] wv;
    initial begin
        @(negedge clk); rst = 1;
        repeat (3) @(negedge clk); rst = 0;

        // wait: firmware sets UART IER (bit0) as the LAST setup step
        t = 0;
        while (ier_seen == 0 && t < 200000) begin
            @(posedge clk);
            if (dut.s2uart.ier[0] === 1'b1) ier_seen = 1;
            t = t + 1;
        end
        if (ier_seen == 0) begin err("SOC-ier-stuck"); dump_state("before-IER"); end
        else               begin ok("SOC-ier"); end
        repeat (3) @(negedge clk);
        send_rx(8'h2A);                          // p1 character
        $display("P1 rx 0x2A sent");

        // p1 byte consumed by the handler (count 0->1) -> loopback for p2
        t = 0;
        while (m9_seen == 0 && t < 200000) begin
            @(posedge clk);
            if (dut.s1ram.mem[9] === 32'h1) m9_seen = 1;
            t = t + 1;
        end
        if (m9_seen == 0) begin err("SOC-p1-stuck"); dump_state("after-rx"); end
        else               begin ok("SOC-p1-consumed"); loop = 1; end
        $display("P2 loopback on");

        // wait for the sentinel (DRAM word 5, written last) with a watchdog
        t = 0;
        while (sent == 0 && t < 400000) begin
            @(posedge clk);
            if (dut.s1ram.mem[5] === 32'hdeadbeef) sent = 1;
            t = t + 1;
        end
        if (sent == 0) begin
            err("SOC-sentinel-stuck"); dump_state("no-sentinel");
            $finish;
        end
        ok("SOC-sentinel");

        // boot phase: DRAM words 0..5
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

        // peripheral phases
        if (dut.s1ram.mem[6] !== 32'h0000002A) err("SOC-rx1-byte");
        else ok("SOC-rx1-byte");
        if (dut.s1ram.mem[8] !== 32'h0000005A) err("SOC-loop-echo");
        else ok("SOC-loop-echo");
        if (dut.s1ram.mem[9] !== 32'h3) err("SOC-irq-cnt");   // uart,echo,msip
        else ok("SOC-irq-cnt");
        if (dut.s1ram.mem[10] !== 32'h1) err("SOC-msip-done");
        else ok("SOC-msip-done");
        if (!(dut.s1ram.mem[7] > 32'h0 && dut.s1ram.mem[11] > dut.s1ram.mem[7]))
            err("SOC-mtime-mono");
        else ok("SOC-mtime-mono");
        if (dut.s1ram.mem[12] !== 32'h8000000B) err("SOC-mcause");
        else ok("SOC-mcause");
        if (!(dut.s1ram.mem[13] > 32'h0 && dut.s1ram.mem[13] <= 32'h8000))
            err("SOC-mepc");
        else ok("SOC-mepc");

        if (errors == 0) $display("ALL TESTS PASSED (%0d checks)", passes);
        else $display("%0d ERRORS (%0d passed)", errors, passes);
        $finish;
    end
endmodule
