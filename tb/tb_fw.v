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
    soc #(.ROM_DEPTH(8192), .ROM_HEX(`ROM_HEX), .RAM_DEPTH(65536),
          .UART_CLK(100), .UART_BAUD(20)) dut (
        .clk(clk), .rst(rst),
        .uart_rxd(loop ? txd : rxd_r), .uart_txd(txd)
    );

`ifndef WDT
`define WDT 300000
`endif

    // dump the result block + data area (helps localize a failing check).
    // Range 0..115 covers result words, M hist (21..43), data 96..101,
    // S counts/hist (100..112) -- needed to localize livelocks (t03 P9).
    integer di;
    task dram_dump();
        begin
            for (di = 0; di < 116; di = di + 1)
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

    // +TRACE: per-interrupt-handler-entry trace (t03 forensics).
    // Handler writes m_hist_n (DRAM word 20) / s_hist_n (word 101) AFTER
    // storing id/epc at words 22+3n/23+3n (M) and 103+3n/104+3n (S).
    integer tn;
    reg [31:0] tr_mn, tr_sn;
    task ent_trace(input [31:0] mn, input [31:0] sn);
        begin
            $display("ENT %s n=%0d id=%0d epc=%h priv=%b meip=%b seip=%b pend=%b ip_stmr=%b ip_mtmr=%b mtip=%b msip=%b sstip=%b mtc=%h stc=%h mtc32=%h t=%h",
                     (mn ? "M" : "S"),
                     (mn ? (mn - 1) : (sn - 1)),
                     (mn ? dut.s1ram.mem[22 + 3 * (mn - 1)]
                         : dut.s1ram.mem[103 + 3 * (sn - 1)]),
                     (mn ? dut.s1ram.mem[23 + 3 * (mn - 1)]
                         : dut.s1ram.mem[104 + 3 * (sn - 1)]),
                     dut.core0.priv, dut.core0.meip, dut.core0.seip,
                     dut.s4plic.pendb, dut.s3clint.ip_stmr, dut.s3clint.ip_mtmr,
                     dut.s3clint.mtip[0], dut.s3clint.msip[0], dut.s3clint.sstip[0],
                     dut.s3clint.mtimecmp[31:0], dut.s3clint.stimecmp[31:0],
                     dut.s3clint.mtimecmp[63:32], dut.s3clint.mtime[31:0]);
        end
    endtask
    initial begin
        tn = 0; tr_mn = 0; tr_sn = 0;
        if ($test$plusargs("TRACE")) begin
            forever begin
                @(negedge clk);
                tn = tn + 1;
                if (dut.s1ram.mem[20] !== tr_mn) begin
                    tr_mn = dut.s1ram.mem[20];
                    ent_trace(tr_mn, tr_sn);
                end
                if (dut.s1ram.mem[101] !== tr_sn) begin
                    tr_sn = dut.s1ram.mem[101];
                    ent_trace(tr_mn, tr_sn);
                end
            end
        end
    end

    // +PLICMON: per-change trace of CLINT levels + PLIC gateway state (t03).
    reg [31:0] pm_v;
    reg pm_last;
    task plmon_print;
        begin
            $display("PL t=%h msip=%b sstip=%b mtip=%b ip10=%b ip11=%b arm10=%b arm11=%b pend=%b EIP=%b",
                     dut.s3clint.mtime[31:0],
                     dut.s3clint.msip[0], dut.s3clint.sstip[0], dut.s3clint.mtip[0],
                     dut.s4plic.ip[9], dut.s4plic.ip[10],
                     dut.s4plic.arm[9], dut.s4plic.arm[10],
                     dut.s4plic.pendb, dut.s4plic.EIP);
        end
    endtask
    initial begin
        pm_last = -1;
        if ($test$plusargs("PLICMON")) begin
            forever begin
                @(negedge clk);
                pm_v = {dut.s3clint.msip[0], dut.s3clint.sstip[0], dut.s3clint.mtip[0],
                        dut.s4plic.ip[9], dut.s4plic.ip[10],
                        dut.s4plic.arm[9], dut.s4plic.arm[10], dut.s4plic.EIP[1],
                        dut.s4plic.EIP[0]};
                if (pm_v !== pm_last) begin
                    pm_last = pm_v;
                    if (dut.s3clint.mtime[31:0] >= 32'h3000)
                        plmon_print;
                end
            end
        end
    end

    // +MTCMON: log every mtimecmp[31:0]/stimecmp[31:0] change (t03 X hunt).
    reg [31:0] mt_last, st_last;
    initial begin
        mt_last = -1; st_last = -1;
        if ($test$plusargs("MTCMON")) begin
            forever begin
                @(negedge clk);
                if (dut.s3clint.mtimecmp[31:0] !== mt_last) begin
                    mt_last = dut.s3clint.mtimecmp[31:0];
                    $display("MTC t=%h mtc_lo=%h mtc_hi=%h stc_lo=%h msip=%b mtip=%b ip_mtmr=%b meip=%b fpc=%h",
                             dut.s3clint.mtime[31:0], dut.s3clint.mtimecmp[31:0],
                             dut.s3clint.mtimecmp[63:32], dut.s3clint.stimecmp[31:0],
                             dut.s3clint.msip[0], dut.s3clint.mtip[0],
                             dut.s3clint.ip_mtmr, dut.core0.meip, dut.core0.fpc);
                end
            end
        end
    end

    // +DMON: log l1_d REQ_I (PA) + DONE_O (DATA) in CLINT range, and the
    // l1_d master (m1) STB/ACK in CLINT range.
    initial begin
        if ($test$plusargs("DMON")) begin
            forever begin
                @(negedge clk);
                if (dut.core0.l1d.REQ_I)
                    $display("DM-L1REQ t=%h pa=%h p32=%02d wen=%b pc=%h",
                             dut.s3clint.mtime[31:0], dut.core0.l1d.PA_I,
                             dut.core0.l1d.PA_I[31:22], dut.core0.l1d.WEN_I,
                             dut.core0.exma_pc);
                if (dut.core0.req_md)
                    $display("DM-MDREQ t=%h va=%h pc=%h mode=%b",
                             dut.s3clint.mtime[31:0], dut.core0.ex_addr,
                             dut.core0.idex_pc, dut.core0.mode_d);
                if (dut.core0.fresh_md)
                    $display("DM-MDDONE t=%h pa=%h pc=%h",
                             dut.s3clint.mtime[31:0], dut.core0.mmd_pa,
                             dut.core0.idex_pc);
                if (dut.core0.l1d.DONE_O && dut.core0.l1d.DATA_O !== 32'h0)
                    $display("DM-L1DONE t=%h dat=%h",
                             dut.s3clint.mtime[31:0], dut.core0.l1d.DATA_O);
                if (dut.core0.m1_stb && dut.core0.m1_adr[31:16] == 16'h0200)
                    $display("DM-M1STB t=%h adr=%h we=%b",
                             dut.s3clint.mtime[31:0], dut.core0.m1_adr, dut.core0.m1_we);
            end
        end
    end

    // +BUSMON: CLINT bus ops + scoreboard RAM writes + trap/xret/priv
    // transitions (t03 P9 store-loss forensics). Negedge-sampled like TRACE.
    integer bmw;
    reg [31:0] bm_old [0:115];
    initial begin
        for (bmw = 0; bmw < 116; bmw = bmw + 1) bm_old[bmw] = 32'hx;
        if ($test$plusargs("BUSMON")) begin
            forever begin
                @(negedge clk);
                if (dut.s3clint.STB_I && dut.s3clint.WE_I)
                    $display("CW  t=%h off=%h d=%h fpc=%h priv=%b",
                             dut.s3clint.mtime[31:0], dut.s3clint.ADR_I[15:0],
                             dut.s3clint.DAT_I, dut.core0.fpc, dut.core0.priv);
                else if (dut.s3clint.STB_I &&
                         (dut.s3clint.ADR_I[15:14] == 2'b01))
                    $display("CR  t=%h off=%h fpc=%h priv=%b",
                             dut.s3clint.mtime[31:0], dut.s3clint.ADR_I[15:0],
                             dut.core0.fpc, dut.core0.priv);
                if (dut.core0.ma_trap)
                    $display("TRP t=%h to_s=%b cause=%h tval=%h priv=%b",
                             dut.s3clint.mtime[31:0], dut.core0.trap_to_s,
                             dut.core0.trap_cause, dut.core0.trap_tval,
                             dut.core0.priv);
                if (dut.core0.ma_xret)
                    $display("XRT t=%h mret=%b pc=%h priv=%b",
                             dut.s3clint.mtime[31:0], dut.core0.ma_mret,
                             dut.core0.xret_pc, dut.core0.priv);
                for (bmw = 16; bmw <= 20; bmw = bmw + 1)
                    if (dut.s1ram.mem[bmw] !== bm_old[bmw]) begin
                        $display("RW  t=%h w=%0d v=%h fpc=%h priv=%b",
                                 dut.s3clint.mtime[31:0], bmw,
                                 dut.s1ram.mem[bmw], dut.core0.fpc,
                                 dut.core0.priv);
                        bm_old[bmw] = dut.s1ram.mem[bmw];
                    end
                for (bmw = 100; bmw <= 110; bmw = bmw + 1)
                    if (dut.s1ram.mem[bmw] !== bm_old[bmw]) begin
                        $display("RW  t=%h w=%0d v=%h fpc=%h priv=%b",
                                 dut.s3clint.mtime[31:0], bmw,
                                 dut.s1ram.mem[bmw], dut.core0.fpc,
                                 dut.core0.priv);
                        bm_old[bmw] = dut.s1ram.mem[bmw];
                    end
            end
        end
    end

    // final PLIC/CLINT/core state (timeout forensics)
    integer pn;
    task int_state();
        begin
            for (pn = 0; pn < 16; pn = pn + 1)
                $display("  ip[%0d]=%b arm[%0d]=%b prio[%0d]=%h",
                         pn + 1, dut.s4plic.ip[pn], pn + 1, dut.s4plic.arm[pn],
                         pn + 1, dut.s4plic.prio[pn]);
            $display("  en0=%h en1=%h thr0=%h thr1=%h pendb=%h EIP=%b",
                     dut.s4plic.en[0], dut.s4plic.en[1],
                     dut.s4plic.thr[0], dut.s4plic.thr[1],
                     dut.s4plic.pendb, dut.s4plic.EIP);
            $display("  CLINT msip=%b sstip=%b mtip=%b mtc=%h stc=%h t=%h ip_stmr=%b ip_mtmr=%b",
                     dut.s3clint.msip[0], dut.s3clint.sstip[0], dut.s3clint.mtip[0],
                     dut.s3clint.mtimecmp, dut.s3clint.stimecmp, dut.s3clint.mtime,
                     dut.s3clint.ip_stmr, dut.s3clint.ip_mtmr);
            $display("  CORE priv=%b mstatus=%h mie=%h mideleg=%h mcause=%h mepc=%h scause=%h sepc=%h mtvec=%h stvec=%h",
                     dut.core0.priv, dut.core0.mstatus, dut.core0.mie,
                     dut.core0.mideleg, dut.core0.mcause, dut.core0.mepc,
                     dut.core0.scause, dut.core0.sepc, dut.core0.mtvec,
                     dut.core0.stvec);
        end
    endtask

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
            int_state();
            dram_dump();
            $finish;
        end
        if (dut.s1ram.mem[0] === 32'h0) begin ok("FW-pass"); end
        else begin
            err("FW-result");
            $display("  result=%h mcause=%h mepc=%h priv=%b",
                     dut.s1ram.mem[0], dut.core0.mcause, dut.core0.mepc,
                     dut.core0.priv);
            int_state();
            dram_dump();
        end
        if ($test$plusargs("DUMP")) dram_dump();
        if (errors == 0) $display("ALL TESTS PASSED (%0d checks)", passes);
        else $display("%0d ERRORS (%0d passed)", errors, passes);
        $finish;
    end
endmodule
