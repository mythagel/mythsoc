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

    // UART RX held idle (1); +LOOP feeds txd to rxd (16550 loopback, used
    // by tests that drive the UART from the firmware side).
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

    // heartbeat (diagnostics): +HB prints core state every 2000 cycles
    integer hbn;
    initial begin
        if ($test$plusargs("HB")) begin
            hbn = 0;
            while (hbn < 200) begin
                repeat (2000) @(negedge clk);
                if (hbn == 1) begin  // per-cycle window at HB1
                    repeat (60) begin
                        @(negedge clk);
                        $display("  C stall=%b mawait=%b pawait=%b mldone=%b reqld=%b ld_mio=%b idex_v=%b exma_v=%b mawb_v=%b ifid_v=%b fpc=%h exma_pc=%h l1d_st=%0d",
                                 dut.core0.stall, dut.core0.ma_wait, dut.core0.pa_wait,
                                 dut.core0.mld_done, dut.core0.req_ld, dut.core0.ld_mio,
                                 dut.core0.idex_v, dut.core0.exma_v, dut.core0.mawb_v,
                                 dut.core0.ifid_v, dut.core0.fpc, dut.core0.exma_pc,
                                 dut.core0.l1d.st);
                    end
                end
                $display("HB %0d fpc=%h exma_pc=%h st=%0d l1d_st=%0d m1stb=%b m1ack=%b",
                         hbn, dut.core0.fpc, dut.core0.exma_pc, dut.core0.stall,
                         dut.core0.l1d.st, dut.core0.m1_stb, dut.core0.m1_ack);
                $display("   mio=%b pav=%b pavf=%b lio=%b dmio=%b dlio=%b fqc=%0d mmui_st=%0d mmui_req=%b mmui_done=%b",
                         dut.core0.mio, dut.core0.pav, dut.core0.pav_f,
                         dut.core0.lio, dut.core0.d_mio, dut.core0.d_lio,
                         dut.core0.fq_c, dut.core0.mmu_i.st,
                         dut.core0.mmu_i.REQ_I, dut.core0.mmu_i.DONE_O);
                $display("   ifid_v=%b idex_v=%b exma_v=%b mawb_v=%b lu=%b stallif=%b div=%b pawait=%b mawait=%b l1i_st=%0d l1done=%b",
                         dut.core0.ifid_v, dut.core0.idex_v, dut.core0.exma_v,
                         dut.core0.mawb_v, dut.core0.lu_stall, dut.core0.stall_if,
                         dut.core0.div_stall, dut.core0.pa_wait, dut.core0.ma_wait,
                         dut.core0.l1i.st, dut.core0.l1_done);
                hbn = hbn + 1;
            end
        end
    end

    // +VCD: dump core + ic + peripheral hierarchies for freeze forensics.
    // Parse with tools/vcd_last.py <vcd> <keep-samples> [substr ...].
    initial begin
        if ($test$plusargs("VCD")) begin
            $dumpfile("/tmp/opencode/tb_fw.vcd");
            $dumpvars(0, dut.core0);
            $dumpvars(0, dut.ic0);
            $dumpvars(0, dut.s2uart);
            $dumpvars(0, dut.s3clint);
            $dumpvars(0, dut.s4plic);
        end
    end

    integer t;
    initial begin
        if ($test$plusargs("LOOP")) loop = 1;
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
