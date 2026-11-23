// tb_16550 -- directed testbench for u16550
// DUT: CLK_FREQ=100 BAUD=20 -> DIV=5 ticks of 5 cycles.
`timescale 1ns/1ps
module tb_16550;

    reg clk = 1, rst = 0, STB = 0, WE = 0, rxd = 1;
    reg [3:0] SEL = 0;
    reg [31:0] ADR = 0, DAT = 0;
    wire [31:0] DATO;
    wire ACK, txd, irq;

    u16550 #(.CLK_FREQ(100), .BAUD(20)) dut (
        .clk(clk), .rst(rst),
        .STB_I(STB), .WE_I(WE), .SEL_I(SEL), .ADR_I(ADR), .DAT_I(DAT),
        .DAT_O(DATO), .ACK_O(ACK),
        .rxd(rxd), .txd(txd), .irq(irq)
    );

    always #10 clk = ~clk;

    integer errors = 0, passes = 0;
    // NOTE: label must be <= 32 chars (iverilog %-formats a [255:0] string
    // port as 32 chars; longer literals truncate on print).

    task err(input [255:0] s);
        begin
            errors = errors + 1;
            $display("FAIL t=%0d: %0s", $time, s);
        end
    endtask

    task ok(input [255:0] s);
        begin
            passes = passes + 1;
            $display("PASS t=%0d: %0s", $time, s);
        end
    endtask

    // ---- WB helpers -----------------------------------------------------
    task wb_wr(input [31:0] a, input [31:0] d, input [3:0] s);
        begin
            @(negedge clk);
            STB = 1; WE = 1; SEL = s; ADR = a; DAT = d;
            @(posedge clk) #1;
            STB = 0;
        end
    endtask

    task wb_rd(input [31:0] a, input [3:0] s, output [31:0] d);
        begin
            @(negedge clk);
            STB = 1; WE = 0; SEL = s; ADR = a; DAT = 0;
            @(posedge clk) #1;
            d = DATO;
            STB = 0;
        end
    endtask

    // core-protocol register access: byte stores replicate data across all
    // 4 lanes with SEL=1<<(addr[1:0]); loads: SEL=1111, addr = byte address,
    // the CONSUMER extracts the lane. Helpers mirror that exactly.
    reg [7:0] rb;
    task wr_reg(input [2:0] r, input [7:0] v);
        begin wb_wr({r[2], 2'b00}, {v, v, v, v}, 4'b0001 << r[1:0]); end
    endtask
    task rd_reg(input [2:0] r, input unused);
        begin
            wb_rd({r[2], 2'b00}, 4'b1111, w);
            rb = w[8 * r[1:0] +: 8];
        end
    endtask

    // ---- RX bit driver (aligned to baud ticks) --------------------------
    task send_bit(input b);
        begin
            @(negedge clk);
            rxd = b;
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

    task send_rx_badstop(input [7:0] byte);
        integer i;
        begin
            send_bit(1'b0);
            for (i = 0; i < 8; i = i + 1)
                send_bit(byte[i]);
            send_bit(1'b0);
            rxd = 1;            // line returns to idle after the character
        end
    endtask

    task wait_ticks(input [31:0] n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                @(negedge clk);
                repeat (4) @(negedge clk);
            end
        end
    endtask

    // ---- TX capture (mirrors DUT rx engine, once per baud tick) --------
    reg tick_ff;
    always @(posedge clk) tick_ff <= dut.tick;
    reg dec_act = 0;
    integer dec_n;
    reg [7:0] dec_d, last_b;
    integer n_tx = 0, stop_err = 0;

    always @(posedge clk) begin
        #1;
        if (tick_ff) begin
            if (!dec_act) begin
                if (!txd) begin
                    dec_act = 1; dec_n = 0; dec_d = 0;
                end
            end else begin
                if (dec_n < 8) begin
                    dec_d = {txd, dec_d[7:1]};
                    dec_n = dec_n + 1;
                end else begin
                    dec_act = 0;
                    if (!txd) stop_err = stop_err + 1;
                    else begin
                        last_b = dec_d;
                        n_tx = n_tx + 1;
                    end
                end
            end
        end
    end

    // ---- main ------------------------------------------------------------
    reg [31:0] w;
    integer i, nt0;

    initial begin
        // reset
        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        wait_ticks(2);

        // --- TX test: send 'H', expect decoded byte, TEMT behaviour
        w = 0;
        wb_wr(32'h0, 32'h48, 4'b0001);          // write R0 = 'H'
        wait_ticks(3);
        // LSR: TEMT should be 0 while shifting
        rd_reg(3'h7, 0);
        if (rb[5] !== 1'b0) err("tx: temt=1 while shifting");
        wait_ticks(14);                         // whole char: 10 ticks + margin
        if (n_tx !== 1)          err("tx: no byte decoded");
        else if (last_b !== 8'h48) err("tx: wrong byte decoded");
        else                     ok("tx: byte decoded");
        if (stop_err !== 0) err("tx: bad stop bit");
        else                ok("tx: stop bits clean");
        rd_reg(3'h7, 0);
        if (rb[5] !== 1'b1) err("tx: temt=0 after tx");

        // --- TX drop when busy
        wb_wr(32'h0, 32'h55, 4'b0001);
        wait_ticks(1);
        w = 0;
        // write while busy: TEMT=0 -> dropped
        @(negedge clk);
        STB = 1; WE = 1; SEL = 4'b0001; ADR = 32'h0; DAT = 32'hAA;
        @(posedge clk) #1;
        STB = 0;
        wait_ticks(14);
        if (n_tx !== 2) err("tx: busy byte not dropped");
        else if (last_b !== 8'h55) err("tx: busy-dropped order");
        else ok("tx: busy write dropped");

        // --- RX single byte
        send_rx(8'h2A);
        wait_ticks(2);
        wb_rd(32'h0, 4'b0001, w);
        if (w[7:0] !== 8'h2A) err("rx: single byte wrong");
        else                  ok("rx: single byte");

        // --- RX 4-byte drain (1 byte popped per R0 read)
        send_rx(8'hAA); send_rx(8'hBB); send_rx(8'hCC); send_rx(8'hDD);
        wait_ticks(6);
        wb_rd(32'h0, 4'b0001, w);
        if (w[7:0] !== 8'hAA) err("rx: pop1 wrong");
        else                  ok("rx: pop1");
        wb_rd(32'h0, 4'b0001, w);
        if (w[7:0] !== 8'hBB) err("rx: pop2");
        wb_rd(32'h0, 4'b0001, w);
        if (w[7:0] !== 8'hCC) err("rx: pop3");
        wb_rd(32'h0, 4'b0001, w);
        if (w[7:0] !== 8'hDD) err("rx: pop4");
        wb_rd(32'h0, 4'b0001, w);
        if (w[7:0] !== 8'h0) err("rx: fifo not empty after pop");

        // --- LSR: DR, THRE
        rd_reg(3'h7, 0);
        if (rb !== 8'b01100000) err("lsr: dr/temt/thre wrong");
        else                         ok("lsr: dr/temt/thre");

        // --- IER DR irq
        wr_reg(3'h1, 8'h01);                // IER bit0
        wait_ticks(2);
        send_rx(8'h5A);
        wait_ticks(3);
        #1;
        if (!irq) err("irq: DR not asserted");
        else      ok("irq: DR asserted");
        rd_reg(3'h2, 0);
        if (rb !== 8'd6) err("iir: RDI value");
        else                 ok("iir: RDI value");
        wb_rd(32'h0, 4'b0001, w);              // pop
        wait_ticks(1);
        #1;
        if (irq) err("irq: DR still asserted after pop");
        else     ok("irq: DR deasserted");
        rd_reg(3'h2, 0);
        if (rb !== 8'd1) err("iir: no-irq value");

        // --- IER THRE irq
        wr_reg(3'h1, 8'h02);                // IER bit1 only
        wait_ticks(2);
        #1;
        if (!irq) err("irq: THRE not asserted");
        else      ok("irq: THRE asserted");
        rd_reg(3'h2, 0);
        if (rb !== 8'd7) err("iir: THR value");
        else                 ok("iir: THR value");

        // --- SCR, MSR
        wr_reg(3'h5, 8'hC5);
        rd_reg(3'h5, 0);
        if (rb !== 8'hC5) err("scr: rw");
        else                  ok("scr: rw");
        rd_reg(3'h4, 0);
        if (rb !== 8'h9F) err("msr: value");
        else                  ok("msr: value");

        // --- frame error
        send_rx_badstop(8'h60);
        wait_ticks(2);
        rd_reg(3'h7, 0);
        if (rb[3] !== 1'b1) err("lsr: FE not set");
        else               ok("lsr: FE set");
        wb_rd(32'h0, 4'b0001, w);              // dummy read clears FE
        rd_reg(3'h7, 0);
        if (rb[3] !== 1'b0) err("lsr: FE not cleared");

        // --- overflow -> OE
        for (i = 0; i < 17; i = i + 1) begin
            send_rx(i[7:0]);
            wait_ticks(1);
        end
        rd_reg(3'h7, 0);
        if (rb[1] !== 1'b1) err("lsr: OE not set");
        else               ok("lsr: OE set");

        // --- FCR rx reset
        wr_reg(3'h2, 8'h01);                // FCR rx reset
        wait_ticks(2);
        rd_reg(3'h7, 0);
        if (rb[1] !== 1'b0) err("lsr: OE not cleared by fcr");
        wb_rd(32'h0, 4'b0001, w);
        if (w[7:0] !== 8'h0) err("fifo: not empty after fcr reset");
        else                 ok("fifo: empty after fcr reset");

        // --- U17: byte store to THR must not touch IER/FCR/LCR (p2 storm
        //     regression: IER picked 0x5A from replicated data -> THRE irq)
        wr_reg(3'h1, 8'h00);                     // IER = 0
        wr_reg(3'h0, 8'h5A);                     // THR byte store (lane 0)
        wait_ticks(14);
        rd_reg(3'h1, 0);
        if (rb !== 8'h00) err("u17: thr st clobbered ier");
        else              ok("u17: thr st keeps ier");
        #1;
        if (irq !== 1'b0) err("u17: irq stuck after thr st");
        else              ok("u17: irq low after thr st");
        rd_reg(3'h2, 0);
        if (rb !== 8'd1) err("u17: fcr lane clobbered");
        else             ok("u17: fcr lane kept");
        // --- U18: byte store to SCR must not start TX (old wrap bug: w_r0
        //     saw SCR as THR on sel[3])
        nt0 = n_tx;
        wr_reg(3'h5, 8'h33);                     // SCR byte (grp4, lane 1)
        wait_ticks(3);
        if (n_tx !== nt0) err("u18: scr write started TX");
        else            ok("u18: scr write no TX");
        rd_reg(3'h5, 0);
        if (rb !== 8'h33) err("u18: scr readback");
        else              ok("u18: scr readback");

        // --- summary
        $display("============================================");
        $display("TB 16550: %0d passes, %0d errors", passes, errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        $display("============================================");
        $finish;
    end

    initial begin
        #1000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
