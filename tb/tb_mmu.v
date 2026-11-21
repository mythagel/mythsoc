// tb_mmu -- directed testbench for mmu (mythrv32)
//
// Sv32 walker + 2-level TLB, WB B4 RO master.
// All constants (RAM loads, SATP words, expected PA/cause/latency) come from
// tools/mmu_addr.py (reference walk model, spec 12.3).
//
// Latency (cycles from REQ cycle to DONE cycle): hit / M / Bare = 2,
// 4M miss = 5, 4K miss = 7.  STB cycles: hit/M/Bare = 0, 4M = 1, 4K = 2.
`timescale 1ns/1ps
module tb_mmu;

    reg clk = 1, rst = 0;
    reg REQ = 0;
    reg [1:0] SF = 0;
    reg [8:0] SFASID = 0;
    reg [31:0] VA_I = 0, SATP_I = 0;
    reg [1:0] ACC_I = 0, MODE_I = 0;
    reg SUM = 0, MXR = 0;
    wire STB;
    wire [31:0] ADRW;
    reg ACKR = 0;
    reg [31:0] DATO = 0;
    wire DONE;
    wire [31:0] PAO;
    wire FAU;
    wire [3:0] CAU;

    mmu dut (
        .clk(clk), .rst(rst),
        .REQ_I(REQ), .VA_I(VA_I), .ACC_I(ACC_I), .MODE_I(MODE_I),
        .SUM_I(SUM), .MXR_I(MXR), .SATP_I(SATP_I),
        .DONE_O(DONE), .PA_O(PAO), .FAULT_O(FAU), .CAUSE_O(CAU),
        .SF_I(SF), .SFASID_I(SFASID),
        .STB_O(STB), .SEL_O(), .ADR_O(ADRW), .WE_O(),
        .ACK_I(ACKR), .DAT_I(DATO)
    );

    always #10 clk = ~clk;
    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    integer errors = 0, passes = 0;
    // NOTE: label must be <= 32 chars (iverilog %s on [255:0] prints 32).
    task err(input [255:0] s);
        begin
            errors = errors + 1;
            $display("FAIL: %0s", s);
        end
    endtask

    task ok(input [255:0] s);
        begin
            passes = passes + 1;
            $display("PASS: %0s", s);
        end
    endtask

    // ---- 1-cyc WB RAM slave (word array; PTE space)
    // index = A[17:2]: max addr used 0x20020 (map C L0) -> word 0x8008
    reg [31:0] ram [0:65535];
    integer ri;
    always @(posedge clk) begin
        if (STB) begin
            DATO <= ram[ADRW[17:2]];
            ACKR <= 1'b1;
            rlog[rc] = ADRW;
            rc = rc + 1;
        end else
            ACKR <= 1'b0;
    end
    integer rc;
    reg [31:0] rlog [0:255];
    integer stb_cnt = 0;
    always @(posedge clk) if (STB) stb_cnt = stb_cnt + 1;

    task ram_set(input [31:0] a, input [31:0] d);
        begin
            ram[a[17:2]] = d;
        end
    endtask
    task ram_clear();
        begin
            rc = 0;
        end
    endtask

    // temp: dump TLB entries for current VA_I (hierarchical)
    task tlb_dump;
        integer s1, s2, k;
        begin
            s1 = VA_I[13:12];
            s2 = VA_I[16:12];
            for (k = 0; k < 4; k = k + 1)
                $display("  TLB1 s%0d w%0d v=%b tag=%05x sz=%b g=%b as=%0d",
                         s1, k, dut.l1_v[s1 * 4 + k], dut.l1_tag[s1 * 4 + k],
                         dut.l1_sz[s1 * 4 + k], dut.l1_g[s1 * 4 + k],
                         dut.l1_asid[s1 * 4 + k]);
            for (k = 0; k < 8; k = k + 1)
                $display("  TLB2 s%0d w%0d v=%b tag=%05x sz=%b g=%b as=%0d",
                         s2, k, dut.l2_v[s2 * 8 + k], dut.l2_tag[s2 * 8 + k],
                         dut.l2_sz[s2 * 8 + k], dut.l2_g[s2 * 8 + k],
                         dut.l2_asid[s2 * 8 + k]);
        end
    endtask

    // ---- request: pulse REQ, wait DONE; record result
    integer dn;
    reg        q_ok, q_fau;
    reg [31:0] q_pa;
    reg [3:0]  q_cause;
    reg [8:0]  q_lat, q_stb;
    integer    lat0, nd;

    task mmu_req(input [31:0] va, input [1:0] acc, input [1:0] mode,
                 input [31:0] satp, input sum, input mxr);
        begin
            q_ok = 1'b0;
            @(negedge clk);
            while (DONE) @(negedge clk);   // ensure DUT idle (prev DONE gone)
            REQ = 1; VA_I = va; ACC_I = acc; MODE_I = mode;
            SATP_I = satp; SUM = sum; MXR = mxr;
            lat0 = cyc;
            nd = 0;
            while (!DONE) begin
                @(posedge clk); #1;
                REQ = 0;
                nd = nd + 1;
            end
            q_ok = DONE;
            q_fau = FAU;
            q_pa = PAO;
            q_cause = CAU;
            q_lat = cyc - lat0;
            q_stb = stb_cnt;
            stb_cnt = 0;
            dn = dn + 1;
            $display("DBG %0d lat=%0d stb=%0d rc=%0d r0=%h r1=%h PA=%h FAU=%b CAU=%0d",
                     dn, q_lat, q_stb, rc, rlog[0], rlog[1], q_pa, q_fau, q_cause);
            tlb_dump;
        end
    endtask

    // ---- sfence pulse (idle only)
    task sfence(input [1:0] cls, input [8:0] asid, input [31:0] va);
        begin
            @(negedge clk);
            SF = cls; SFASID = asid; VA_I = va;
            @(posedge clk) #1;
            SF = 0;
        end
    endtask

    reg [31:0] w;
    integer i;

    // SATP words (tools/mmu_addr.py)
    localparam [31:0] S_A0 = 32'h80000004, S_A1 = 32'h80400004,
                     S_B0 = 32'h80000008, S_C0 = 32'h8000000C,
                     S_C1 = 32'h8040000C;

    initial begin
        for (ri = 0; ri < 32768; ri = ri + 1) ram[ri] = 32'h0;
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        // base RAM map (tools/mmu_addr.py)
        ram_set(32'h00004000, 32'h00004001);
        ram_set(32'h00008004, 32'h008000CF);
        ram_set(32'h0000C000, 32'h00008001);
        ram_set(32'h00010004, 32'h000004CF);
        ram_set(32'h00010008, 32'h000008CF);
        ram_set(32'h00020004, 32'h000004C3);
        ram_set(32'h00020008, 32'h000004C9);
        ram_set(32'h0002000C, 32'h00000447);
        ram_set(32'h00020010, 32'h000004C5);
        ram_set(32'h00020014, 32'h000004DF);
        ram_set(32'h00020018, 32'h0000040B);
        ram_set(32'h0002001C, 32'h000004EF);

        // ---- M1 reset: M-mode / Bare identity (no walk, TLB untouched)
        ram_clear();
        mmu_req(32'h00001004, 2'd0, 2'd0, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd2 && q_stb === 9'd0)
            ok("M1: M-mode identity");
        else
            err("M1: M-mode identity");
        mmu_req(32'h00001004, 2'd1, 2'd1, 32'h0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd2 && q_stb === 9'd0)
            ok("M1: Bare identity");
        else
            err("M1: Bare identity");

        // ---- M2 4K miss walk (map A): VA 0x1004 -> PA 0x1004
        ram_clear();
        mmu_req(32'h00001004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004)
            ok("M2: 4K miss PA");
        else
            err("M2: 4K miss PA");
        if (q_lat === 9'd7 && q_stb === 9'd2)
            ok("M2: 4K miss lat7 stb2");
        else
            err("M2: 4K miss lat/stb");
        if (rc === 2 && rlog[0] === 32'h00004000 && rlog[1] === 32'h00010004)
            ok("M2: PTE addrs L1,L0");
        else
            err("M2: PTE addrs L1,L0");

        // ---- M3 4K hits (L1/L2): same page, other offset; store (D=1)
        mmu_req(32'h0000100C, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h0000100C && q_lat === 9'd2 && q_stb === 9'd0)
            ok("M3: 4K hit lat2");
        else
            err("M3: 4K hit lat2");
        mmu_req(32'h00001004, 2'd2, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd2)
            ok("M3: 4K hit store");
        else
            err("M3: 4K hit store");
        mmu_req(32'h00001004, 2'd3, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd2)
            ok("M3: 4K hit amo");
        else
            err("M3: 4K hit amo");

        // ---- M4 2nd page (new set): VA 0x2004 -> PA 0x2004
        ram_clear();
        mmu_req(32'h00002004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00002004 && q_lat === 9'd7)
            ok("M4: 2nd page miss");
        else
            err("M4: 2nd page miss");
        if (rc === 2 && rlog[0] === 32'h00004000 && rlog[1] === 32'h00010008)
            ok("M4: L0 addr page2");
        else
            err("M4: L0 addr page2");
        mmu_req(32'h0000200C, 2'd1, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h0000200C && q_lat === 9'd2)
            ok("M4: page2 hit");
        else
            err("M4: page2 hit");

        // ---- M5 4M mega (map B): PA = PPN1[9:0]<<22 | VA[21:0]
        ram_clear();
        mmu_req(32'h00400004, 2'd0, 2'd1, S_B0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h02000004 && q_lat === 9'd5 && q_stb === 9'd1)
            ok("M5: 4M miss lat5 PA");
        else
            err("M5: 4M miss lat5 PA");
        if (rc === 1 && rlog[0] === 32'h00008004)
            ok("M5: 4M L1 addr");
        else
            err("M5: 4M L1 addr");
        mmu_req(32'h00404000, 2'd1, 2'd1, S_B0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h02004000 && q_lat === 9'd2)
            ok("M5: 4M hit PA");
        else
            err("M5: 4M hit PA");
        ram_clear();
        mmu_req(32'h7FFFFFFF, 2'd0, 2'd1, S_B0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h023FFFFF && q_lat === 9'd5)
            ok("M5: 4M new set miss");
        else
            err("M5: 4M new set miss");

        // ---- M6 4M structural faults (no fill): misalign, R0W1
        sfence(2'b01, 9'd0, 32'h0);
        ram_set(32'h00008004, 32'h008014CF);
        mmu_req(32'h00400004, 2'd0, 2'd1, S_B0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd12 && q_lat === 9'd5)
            ok("M6: 4M misalign fault");
        else
            err("M6: 4M misalign fault");
        mmu_req(32'h00400004, 2'd0, 2'd1, S_B0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd12 && q_lat === 9'd5)
            ok("M6: fault not filled");
        else
            err("M6: fault not filled");
        ram_set(32'h00008004, 32'h008000C5);
        mmu_req(32'h00400004, 2'd1, 2'd1, S_B0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd13 && q_lat === 9'd5)
            ok("M6: 4M R0W1 fault13");
        else
            err("M6: 4M R0W1 fault13");
        ram_set(32'h00008004, 32'h008000CF);
        sfence(2'b01, 9'd0, 32'h0);

        // ---- M7 perms (map C): R/X/RW, A/D, U/SUM/MXR, amo, U-mode
        sfence(2'b01, 9'd0, 32'h0);
        mmu_req(32'h00001004, 2'd1, 2'd1, S_C0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd7)
            ok("M7: R-only load OK");
        else
            err("M7: R-only load OK");
        mmu_req(32'h00001004, 2'd0, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd12 && q_lat === 9'd2)
            ok("M7: R-only fetch 12");
        else
            err("M7: R-only fetch 12");
        mmu_req(32'h00001004, 2'd2, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd15)
            ok("M7: R-only store 15");
        else
            err("M7: R-only store 15");
        mmu_req(32'h00001004, 2'd3, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd15)
            ok("M7: R-only amo 15");
        else
            err("M7: R-only amo 15");
        mmu_req(32'h00002004, 2'd0, 2'd1, S_C0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd7)
            ok("M7: X-only fetch OK");
        else
            err("M7: X-only fetch OK");
        mmu_req(32'h00002004, 2'd1, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd13)
            ok("M7: X-only load 13");
        else
            err("M7: X-only load 13");
        mmu_req(32'h00002004, 2'd1, 2'd1, S_C0, 1'b0, 1'b1);
        if (!q_fau && q_pa === 32'h00001004)
            ok("M7: X-only load MXR");
        else
            err("M7: X-only load MXR");
        mmu_req(32'h00002004, 2'd2, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd15)
            ok("M7: X-only store 15");
        else
            err("M7: X-only store 15");
        mmu_req(32'h00002004, 2'd3, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd15)
            ok("M7: X-only amo 15");
        else
            err("M7: X-only amo 15");
        mmu_req(32'h00003004, 2'd1, 2'd1, S_C0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd7)
            ok("M7: RW load OK");
        else
            err("M7: RW load OK");
        mmu_req(32'h00003004, 2'd2, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd15)
            ok("M7: D=0 store 15");
        else
            err("M7: D=0 store 15");
        mmu_req(32'h00004004, 2'd1, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd13 && q_lat === 9'd7)
            ok("M7: R0W1 fault13");
        else
            err("M7: R0W1 fault13");
        mmu_req(32'h00005004, 2'd1, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd13 && q_lat === 9'd7)
            ok("M7: S U=1 load 13");
        else
            err("M7: S U=1 load 13");
        mmu_req(32'h00005004, 2'd1, 2'd1, S_C0, 1'b1, 1'b0);
        if (!q_fau && q_pa === 32'h00001004)
            ok("M7: S U=1 SUM load");
        else
            err("M7: S U=1 SUM load");
        mmu_req(32'h00005004, 2'd2, 2'd1, S_C0, 1'b1, 1'b0);
        if (!q_fau && q_pa === 32'h00001004)
            ok("M7: S U=1 SUM store");
        else
            err("M7: S U=1 SUM store");
        mmu_req(32'h00005004, 2'd0, 2'd1, S_C0, 1'b1, 1'b0);
        if (q_fau && q_cause === 4'd12)
            ok("M7: S fetch U=1 12");
        else
            err("M7: S fetch U=1 12");
        mmu_req(32'h00005004, 2'd1, 2'd2, S_C0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004)
            ok("M7: U load U=1 OK");
        else
            err("M7: U load U=1 OK");
        mmu_req(32'h00005004, 2'd0, 2'd2, S_C0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004)
            ok("M7: U fetch U=1 OK");
        else
            err("M7: U fetch U=1 OK");
        mmu_req(32'h00008004, 2'd1, 2'd2, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd13 && q_lat === 9'd7)
            ok("M7: U load U=0 13");
        else
            err("M7: U load U=0 13");
        mmu_req(32'h00006004, 2'd1, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd13 && q_lat === 9'd7)
            ok("M7: A=0 fault13");
        else
            err("M7: A=0 fault13");
        mmu_req(32'h00009004, 2'd1, 2'd1, S_C0, 1'b0, 1'b0);
        if (q_fau && q_cause === 4'd13 && q_lat === 9'd7)
            ok("M7: v=0 unmapped 13");
        else
            err("M7: v=0 unmapped 13");
        mmu_req(32'h00007004, 2'd1, 2'd1, S_C0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd7)
            ok("M7: G page load OK");
        else
            err("M7: G page load OK");
        mmu_req(32'h00008004, 2'd1, 2'd1, S_C0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd7)
            ok("M7: S load U=0 OK");
        else
            err("M7: S load U=0 OK");

        // ---- M8 ASID tagging: mismatch -> miss even on same VA
        mmu_req(32'h00001004, 2'd0, 2'd1, S_A1, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd7)
            ok("M8: asid1 miss refill");
        else
            err("M8: asid1 miss refill");
        mmu_req(32'h00001004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd7)
            ok("M8: asid0 miss again");
        else
            err("M8: asid0 miss again");
        mmu_req(32'h0000100C, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h0000100C && q_lat === 9'd2)
            ok("M8: asid0 hit");
        else
            err("M8: asid0 hit");

        // ---- M9 sfence all: hit -> flush -> miss
        mmu_req(32'h00001004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd2)
            ok("M9: hit before flush");
        else
            err("M9: hit before flush");
        sfence(2'b01, 9'd0, 32'h0);
        mmu_req(32'h00001004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd7)
            ok("M9: miss after all");
        else
            err("M9: miss after all");

        // ---- M10 sfence asid: G entries survive, others flushed
        mmu_req(32'h00007004, 2'd1, 2'd1, S_C1, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd7)
            ok("M10: C asid1 G fill");
        else
            err("M10: C asid1 G fill");
        mmu_req(32'h00008004, 2'd1, 2'd1, S_C1, 1'b0, 1'b0);
        if (!q_fau && q_pa === 32'h00001004 && q_lat === 9'd7)
            ok("M10: C asid1 NG fill");
        else
            err("M10: C asid1 NG fill");
        sfence(2'b10, 9'd1, 32'h0);
        mmu_req(32'h00007004, 2'd1, 2'd1, S_C1, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd2)
            ok("M10: G survives asid");
        else
            err("M10: G survives asid");
        mmu_req(32'h00008004, 2'd1, 2'd1, S_C1, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd7)
            ok("M10: NG flushed asid");
        else
            err("M10: NG flushed asid");

        // ---- M11 sfence page: one 4K page flushed, others kept
        mmu_req(32'h00001004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd2)
            ok("M11: page1 hit");
        else
            err("M11: page1 hit");
        mmu_req(32'h00002004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd7)
            ok("M11: page2 refill");
        else
            err("M11: page2 refill");
        mmu_req(32'h00002004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd2)
            ok("M11: page2 hit");
        else
            err("M11: page2 hit");
        sfence(2'b11, 9'd0, 32'h00002004);
        mmu_req(32'h00001004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd2)
            ok("M11: page1 kept");
        else
            err("M11: page1 kept");
        mmu_req(32'h00002004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd7)
            ok("M11: page2 flushed");
        else
            err("M11: page2 flushed");

        // ---- M12 mid-walk sfence page: flush applies AFTER fill
        sfence(2'b01, 9'd0, 32'h0);
        @(negedge clk);
        REQ = 1; VA_I = 32'h00001004; ACC_I = 2'd0; MODE_I = 2'd1;
        SATP_I = S_A0; SUM = 0; MXR = 0;
        lat0 = cyc;
        @(posedge clk); #1;
        REQ = 0;                       // C0 done (st->W1S)
        @(posedge clk); #1;            // C1 (W1S)
        SF = 2'b11; SFASID = 9'd0; VA_I = 32'h00001004;  // page flush
        @(posedge clk); #1;
        SF = 0;                        // sampled in W1W: sf_p latch
        while (!DONE) begin
            @(posedge clk); #1;
        end
        if (FAU)
            err("M12: mid-walk done");
        else
            ok("M12: mid-walk done");
        if (cyc - lat0 !== 9'd7)
            err("M12: mid-walk lat7");
        else
            ok("M12: mid-walk lat7");
        mmu_req(32'h00001004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd7)
            ok("M12: flush after fill");
        else
            err("M12: flush after fill");
        mmu_req(32'h00001004, 2'd0, 2'd1, S_A0, 1'b0, 1'b0);
        if (!q_fau && q_lat === 9'd2)
            ok("M12: refill hit");
        else
            err("M12: refill hit");

        // ---- summary
        $display("============================================");
        $display("TB MMU: %0d passes, %0d errors (cyc=%0d)",
                 passes, errors, cyc);
        if (errors === 0) $display("ALL TESTS PASSED");
        $display("============================================");
        $finish;
    end

    // global timeout
    initial begin
        #1000000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
