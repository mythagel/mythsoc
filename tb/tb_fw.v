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

    // final GPR / save-area forensics. rf is the core regfile; sav_s/g_abort
    // indices are for the CURRENT t04 build (tools/pa_word.py) -- best-effort.
    integer rgv;
    task regdump();
        begin
            for (rgv = 0; rgv < 32; rgv = rgv + 1) begin
                $write("  x%0d=%h", rgv, dut.core0.rf[rgv]);
                if (rgv % 4 == 3) $display("");
            end
            $display("");
            for (rgv = 0; rgv < 16; rgv = rgv + 1)
                $display("  sav_s[%0d]=%h", rgv, dut.s1ram.mem[1024 + rgv]);
            $display("  g_abort=%h", dut.s1ram.mem[15360]);
        end
    endtask

    // +MMON: MMU transaction trace, both sides (I + D walkers): every REQ
    // (VA page, acc, mode, current asid, TLB hit bit), every DONE (PA/fault/
    // cause), TLB fills (L1/L2 index, tag, asid, PA), sfence commits. The hit
    // bit + fill log settles TLB/asid/eviction questions without guessing.
    initial begin
        if ($test$plusargs("MMON")) begin
            forever begin
                @(negedge clk);
                if (!rst) begin
                    if (dut.core0.req_mmu)
                        $display("MI-R %h a%0d m%0d as%0d satp=%h hit=%b",
                                 dut.core0.mmu_i.VA_I[31:12],
                                 dut.core0.mmu_i.ACC_I, dut.core0.mmu_i.MODE_I,
                                 dut.core0.mmu_i.SATP_I[30:22],
                                 dut.core0.mmu_i.SATP_I, dut.core0.mmu_i.h_hit);
                    if (dut.core0.mmu_i.DONE_O)
                        $display("MI-D %h fau=%b c%0d fpc=%h",
                                 dut.core0.mmu_i.PA_O,
                                 dut.core0.mmu_i.FAULT_O,
                                 dut.core0.mmu_i.CAUSE_O, dut.core0.fpc);
                    if (dut.core0.req_md)
                        $display("MD-R %h a%0d m%0d as%0d satp=%h hit=%b pc=%h",
                                 dut.core0.mmu_d.VA_I[31:12],
                                 dut.core0.mmu_d.ACC_I, dut.core0.mmu_d.MODE_I,
                                 dut.core0.mmu_d.SATP_I[30:22],
                                 dut.core0.mmu_d.SATP_I, dut.core0.mmu_d.h_hit,
                                 dut.core0.idex_pc);
                    if (dut.core0.mmu_d.DONE_O)
                        $display("MD-D %h fau=%b c%0d",
                                 dut.core0.mmu_d.PA_O,
                                 dut.core0.mmu_d.FAULT_O,
                                 dut.core0.mmu_d.CAUSE_O);
                    if (dut.core0.mmu_i.do_fill)
                        $display("MFI l1i=%0d l2i=%0d tag=%h as=%0d pa=%h sz=%b g=%b",
                                 dut.core0.mmu_i.fe1, dut.core0.mmu_i.fe2,
                                 dut.core0.mmu_i.f1t, dut.core0.mmu_i.asid_r,
                                 dut.core0.mmu_i.pa_i[31:12],
                                 dut.core0.mmu_i.frm_eff, dut.core0.mmu_i.pi_g);
                    if (dut.core0.mmu_d.do_fill)
                        $display("MFD l1i=%0d l2i=%0d tag=%h as=%0d pa=%h sz=%b g=%b",
                                 dut.core0.mmu_d.fe1, dut.core0.mmu_d.fe2,
                                 dut.core0.mmu_d.f1t, dut.core0.mmu_d.asid_r,
                                 dut.core0.mmu_d.pa_i[31:12],
                                 dut.core0.mmu_d.frm_eff, dut.core0.mmu_d.pi_g);
                    if (dut.core0.sf_now)
                        $display("SF cls=%0d fpc=%h", dut.core0.sf_typ,
                                 dut.core0.fpc);
                end
            end
        end
    end

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

    // X-catcher: 64-deep ring of fetch-engine control state; the first cycle
    // any key control signal is X triggers a full oldest->newest dump.
    localparam XR = 64;
    integer xk, xi, xn;
    reg xcaught;
    reg [31:0] xfpc [0:XR-1], xrpc [0:XR-1];
    reg [63:0] x_mio, x_pav, x_pf, x_lio, x_dmio, x_dlio, x_rmu, x_rl1, x_syn2;
    reg [63:0] x_flush, x_stall, x_mdone, x_mfault, x_ldone, x_sfn, x_sfh;
    reg [63:0] x_priv, x_trap, x_xret, x_redir, x_fqc2;
    reg [2:0]  x_mst [0:XR-1];
    reg [1:0]  x_lst [0:XR-1];
    initial begin
        xcaught = 1'b0; xk = 0;
        forever begin
            @(negedge clk);
            if (rst) begin
                xk = xk + 1;
            end else if (!xcaught) begin
                xfpc[xk % XR]     = dut.core0.fpc;
                xrpc[xk % XR]     = dut.core0.redirect_pc;
                x_mio  [xk % XR]  = dut.core0.mio;
                x_pav  [xk % XR]  = dut.core0.pav;
                x_pf   [xk % XR]  = dut.core0.pav_f;
                x_lio  [xk % XR]  = dut.core0.lio;
                x_dmio [xk % XR]  = dut.core0.d_mio;
                x_dlio [xk % XR]  = dut.core0.d_lio;
                x_rmu  [xk % XR]  = dut.core0.req_mmu;
                x_rl1  [xk % XR]  = dut.core0.req_l1;
                x_syn2 [xk % XR]  = dut.core0.syn;
                x_flush[xk % XR]  = dut.core0.flush;
                x_stall[xk % XR]  = dut.core0.stall;
                x_mdone[xk % XR]  = dut.core0.mmu_done;
                x_mfault[xk % XR] = dut.core0.mmu_fault;
                x_ldone[xk % XR]  = dut.core0.l1_done;
                x_sfn  [xk % XR]  = dut.core0.sf_now;
                x_sfh  [xk % XR]  = dut.core0.sf_h;
                x_priv [xk % XR]  = dut.core0.priv;
                x_trap [xk % XR]  = dut.core0.ma_trap;
                x_xret [xk % XR]  = dut.core0.ma_xret;
                x_redir[xk % XR]  = dut.core0.redirect;
                x_fqc2 [xk % XR]  = |dut.core0.fq_c;
                x_mst  [xk % XR]  = dut.core0.mmu_i.st;
                x_lst  [xk % XR]  = dut.core0.l1i.st;
                if (dut.core0.mio === 1'bx || dut.core0.pav === 1'bx ||
                    dut.core0.lio === 1'bx || dut.core0.req_mmu === 1'bx ||
                    dut.core0.mmu_done === 1'bx || dut.core0.l1_done === 1'bx ||
                    dut.core0.sf_now === 1'bx || dut.core0.mmu_i.st === 3'bx ||
                    dut.core0.l1i.st === 2'bx || dut.core0.fq_c === 3'bx ||
                    dut.core0.flush === 1'bx || dut.core0.stall === 1'bx ||
                    dut.core0.ma_xret === 1'bx || dut.core0.ma_trap === 1'bx) begin
                    xcaught = 1'b1;
                    $display("XCATCH first-X at negedge %0d", xk);
                    for (xi = 0; xi < XR; xi = xi + 1) begin
                        xn = (xk + 1 + xi) % XR;   // oldest-first ring walk
                        $display("XC %0d fpc=%h mio=%b pav=%b pf=%b lio=%b dmio=%b dlio=%b rmu=%b rl1=%b syn=%b fl=%b st=%b md=%b mf=%b ld=%b sfn=%b sfh=%b priv=%b tr=%b xr=%b rd=%b rpc=%h fq=%b mst=%b lst=%b",
                                 xn, xfpc[xn], x_mio[xn], x_pav[xn], x_pf[xn], x_lio[xn],
                                 x_dmio[xn], x_dlio[xn], x_rmu[xn], x_rl1[xn], x_syn2[xn],
                                 x_flush[xn], x_stall[xn], x_mdone[xn], x_mfault[xn],
                                 x_ldone[xn], x_sfn[xn], x_sfh[xn], x_priv[xn],
                                 x_trap[xn], x_xret[xn], x_redir[xn], xrpc[xn],
                                 x_fqc2[xn], x_mst[xn], x_lst[xn]);
                    end
                end
                xk = xk + 1;
            end
        end
    end

    // +TLBDUMP: full valid-entry dump of both TLBs (both sides) at the moment
    // the run ends (marker seen) -- pair with +MMON's hit/fill log.
    integer td1, td2;
    task tlbdump(input [255:0] side);
        begin
            for (td1 = 0; td1 < 16; td1 = td1 + 1)
                if (side == "I" ? dut.core0.mmu_i.l1_v[td1]
                                : dut.core0.mmu_d.l1_v[td1])
                    $display("  %0s-L1 %0d tag=%h sz=%b g=%b as=%0d pa=%h st=%0d",
                             side, td1,
                             side == "I" ? dut.core0.mmu_i.l1_tag[td1]
                                         : dut.core0.mmu_d.l1_tag[td1],
                             side == "I" ? dut.core0.mmu_i.l1_sz[td1]
                                         : dut.core0.mmu_d.l1_sz[td1],
                             side == "I" ? dut.core0.mmu_i.l1_g[td1]
                                         : dut.core0.mmu_d.l1_g[td1],
                             side == "I" ? dut.core0.mmu_i.l1_asid[td1]
                                         : dut.core0.mmu_d.l1_asid[td1],
                             side == "I" ? dut.core0.mmu_i.l1_pa12[td1]
                                         : dut.core0.mmu_d.l1_pa12[td1],
                             side == "I" ? dut.core0.mmu_i.l1_st[td1]
                                         : dut.core0.mmu_d.l1_st[td1]);
            for (td2 = 0; td2 < 256; td2 = td2 + 1)
                if (side == "I" ? dut.core0.mmu_i.l2_v[td2]
                                : dut.core0.mmu_d.l2_v[td2])
                    $display("  %0s-L2 %0d tag=%h sz=%b g=%b as=%0d pa=%h st=%0d",
                             side, td2,
                             side == "I" ? dut.core0.mmu_i.l2_tag[td2]
                                         : dut.core0.mmu_d.l2_tag[td2],
                             side == "I" ? dut.core0.mmu_i.l2_sz[td2]
                                         : dut.core0.mmu_d.l2_sz[td2],
                             side == "I" ? dut.core0.mmu_i.l2_g[td2]
                                         : dut.core0.mmu_d.l2_g[td2],
                             side == "I" ? dut.core0.mmu_i.l2_asid[td2]
                                         : dut.core0.mmu_d.l2_asid[td2],
                             side == "I" ? dut.core0.mmu_i.l2_pa12[td2]
                                         : dut.core0.mmu_d.l2_pa12[td2],
                             side == "I" ? dut.core0.mmu_i.l2_st[td2]
                                         : dut.core0.mmu_d.l2_st[td2]);
        end
    endtask

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
                    $display("TRP t=%h to_s=%b cause=%h tval=%h priv=%b ms=%h",
                             dut.s3clint.mtime[31:0], dut.core0.trap_to_s,
                             dut.core0.trap_cause, dut.core0.trap_tval,
                             dut.core0.priv, dut.core0.mstatus);
                if (dut.core0.ma_xret)
                    $display("XRT t=%h mret=%b pc=%h priv=%b ms=%h",
                             dut.s3clint.mtime[31:0], dut.core0.ma_mret,
                             dut.core0.xret_pc, dut.core0.priv,
                             dut.core0.mstatus);
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

    // +FIP: t05 E9 redirect-boundary probe. Ring runs continuously; on the
    // FIRST exma_v && exma_pc==0x17a4 (fence.i) the ring resets; dump happens
    // on exma_pc==0x17ac (jalr) -- or 0x17b0 as fallback (jalr entered exma
    // with a corrupted pc) -- printing the 24 edges of fetch/forward state.
    localparam FR = 24;
    reg [31:0] f_ifpc[0:FR-1], f_ipc[0:FR-1], f_xpc[0:FR-1], f_wpc[0:FR-1];
    reg [31:0] f_fwda[0:FR-1], f_ixa[0:FR-1], f_ares[0:FR-1], f_xres[0:FR-1];
    reg [31:0] f_wres[0:FR-1], f_fpc[0:FR-1], f_rpc[0:FR-1], f_rf5[0:FR-1];
    reg [31:0] f_instr[0:FR-1], f_tgt[0:FR-1];
    reg [31:0] f_pmmu[0:FR-1], f_ppav[0:FR-1], f_pl1[0:FR-1];
    reg [31:0] f_l1dat[0:FR-1], f_mpa[0:FR-1], f_ifis[0:FR-1];
    reg [13:0] f_b1[0:FR-1]; // {req_mmu,req_l1,enq,deq,bypass,syn,fresh_mmu,
                             //  fresh_l1,flush,pav,lio,inval_p,d_mio,d_lio}
    reg [2:0]  f_l1st[0:FR-1];
    reg [4:0]  f_rs1[0:FR-1], f_ird[0:FR-1], f_xrd[0:FR-1], f_wrd[0:FR-1];
    reg [7:0]  f_f[0:FR-1];   // {idex_v, exma_v, mawb_v, iwr, xwr, wwr, lu, st}
    reg [3:0]  f_fqc[0:FR-1];
    integer fk;
    integer fi2, fj;
    reg fcaught = 1'b0;
    reg farmed = 1'b0;
    initial begin
        fk = 0;
        if ($test$plusargs("FIP")) forever begin
            @(negedge clk);
            if (!rst && !fcaught) begin
                f_ifpc[fk % FR]  = dut.core0.ifid_pc;
                f_ipc [fk % FR]  = dut.core0.idex_pc;
                f_xpc [fk % FR]  = dut.core0.exma_pc;
                f_wpc [fk % FR]  = dut.core0.mawb_pc;
                f_fwda[fk % FR]  = dut.core0.fwd_a;
                f_ixa [fk % FR]  = dut.core0.idex_a;
                f_ares[fk % FR]  = dut.core0.alu_res;
                f_xres[fk % FR]  = dut.core0.exma_res;
                f_wres[fk % FR]  = dut.core0.mawb_res;
                f_fpc [fk % FR]  = dut.core0.fpc;
                f_rpc [fk % FR]  = dut.core0.redirect_pc;
                f_rf5 [fk % FR]  = dut.core0.rf[1];
                f_instr[fk % FR] = dut.core0.exma_instr;
                f_tgt [fk % FR]  = dut.core0.exma_target;
                f_ifis[fk % FR]  = dut.core0.ifid_instr;
                f_pmmu[fk % FR]  = dut.core0.pc_mmu;
                f_ppav[fk % FR]  = dut.core0.pc_pav;
                f_pl1 [fk % FR]  = dut.core0.pc_l1;
                f_l1dat[fk % FR] = dut.core0.l1_data;
                f_mpa [fk % FR]  = dut.core0.mmu_pa;
                f_rs1 [fk % FR]  = dut.core0.rs1;
                f_ird [fk % FR]  = dut.core0.idex_rd;
                f_xrd [fk % FR]  = dut.core0.exma_rd;
                f_wrd [fk % FR]  = dut.core0.mawb_rd;
                f_f   [fk % FR]  = {dut.core0.idex_v, dut.core0.exma_v,
                                    dut.core0.mawb_v, dut.core0.idex_wr,
                                    dut.core0.exma_wr, dut.core0.mawb_wr,
                                    dut.core0.lu_stall, dut.core0.stall};
                f_fqc [fk % FR]  = {dut.core0.ifid_v, dut.core0.fq_c};
                f_b1  [fk % FR]  = {dut.core0.req_mmu, dut.core0.req_l1,
                                    dut.core0.enq, dut.core0.deq,
                                    dut.core0.bypass, dut.core0.syn,
                                    dut.core0.fresh_mmu, dut.core0.fresh_l1,
                                    dut.core0.flush, dut.core0.pav,
                                    dut.core0.lio, dut.core0.inval_p,
                                    dut.core0.d_mio, dut.core0.d_lio};
                f_l1st[fk % FR]  = dut.core0.l1i.st;
                fk = fk + 1;
                if (dut.core0.exma_v && dut.core0.exma_pc === 32'h0000_17a4
                    && dut.core0.exma_instr === 32'h0000_100f
                    && dut.core0.fi_now) begin
                    fk = 0;   // ring restarts at the fence.i COMMIT sample
                    farmed = 1'b1;
                end else if (farmed && dut.core0.exma_v &&
                             dut.core0.exma_instr === 32'h8980_80e7) begin
                    $display("FIP trigger negedge %0d (xpc=%h)", fk,
                             dut.core0.exma_pc);
                    for (fi2 = 0; fi2 < FR; fi2 = fi2 + 1) begin
                        fj = (fk + fi2) % FR;   // oldest first
                        $display("F %0d fpc=%h rpc=%h ip=%h is=%h v=%b%b%b%b xp=%h xis=%h b=%b l1st=%d fqc=%o pmm=%h ppv=%h pl1=%h pa=%h ld=%h fw=%h ia=%h ar=%h ra1=%h",
                                 fi2, f_fpc[fj], f_rpc[fj], f_ipc[fj], f_ifis[fj],
                                 f_f[fj][7], f_f[fj][6], f_f[fj][5], f_f[fj][2],
                                 f_xpc[fj], f_instr[fj], f_b1[fj], f_l1st[fj],
                                 f_fqc[fj], f_pmmu[fj], f_ppav[fj], f_pl1[fj],
                                 f_mpa[fj], f_l1dat[fj], f_fwda[fj], f_ixa[fj],
                                 f_ares[fj], f_rf5[fj]);
                    end
                    fcaught = 1'b1;
                end
            end
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
            regdump();
            if ($test$plusargs("TLBDUMP")) begin
                tlbdump("I"); tlbdump("D");
            end
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
            regdump();
            if ($test$plusargs("TLBDUMP")) begin
                tlbdump("I"); tlbdump("D");
            end
        end
        if ($test$plusargs("DUMP")) dram_dump();
        if (errors == 0) $display("ALL TESTS PASSED (%0d checks)", passes);
        else $display("%0d ERRORS (%0d passed)", errors, passes);
        $finish;
    end
endmodule
