// u16550 -- 16550-compatible UART, Wishbone B4 slave.  (mythrv32)
//
// Registers at byte offset ADR[2:0] (session-17 aligned-window decode:
// a LW/SW transfers the 4-register window {grp4,0..3}, grp4=ADR[2];
// each access gated per SEL lane, see wb decode below):
//   0  RX FIFO / TX holding : read pops 1 FIFO byte (at the R0 position in
//      the word); write loads TX holding byte (dropped if TX busy) and
//      clears OE/FE
//   1  IER  [0]=DR, [1]=THRE  (other bits reserved=0)
//   2  IIR (read) / FCR (write, FIFO mode always on)
//      FCR: [0]RX reset, [1]TX reset, [7:6] RX trigger level 1/2/4/8
//   3  LCR  (latched; fixed 8N1)
//   4  MSR  (read-only: CTS/DSR/DCD/RI/THD/TEMT=1, OUT1/OUT2=0)
//   5  SCR  (read/write scratch)
//   7  LSR  [0]DR [1]OE [2]PE=0 [3]FE [4]BI=0 [5]TEMT [6]THRE=1
//
// 8N1, no parity. RX FIFO 16x8 (implementation limit of the 64-byte standard;
// trigger levels mapped 1/2/4/8). Overflow sets OE, byte dropped.
// TX: holding register only (no TX FIFO; byte written while TX busy dropped).
// IRQ (level) = (IER0 & rx_active) | (IER1 & THRE); THRE always 1.
// Baud: one tick every DIV = round(CLK_FREQ/BAUD) cycles; rxd sampled on tick.
// WB B4: single cycle; DAT_O valid when ACK_O (one cycle after STB); no ERR.
`default_nettype none

module u16550 #(
    parameter CLK_FREQ = 25_000_000,
    parameter BAUD     = 115_200
)(
    input  clk, rst,
    // Wishbone B4 slave
    input        STB_I,
    input        WE_I,
    input  [3:0] SEL_I,
    input  [31:0] ADR_I,
    input  [31:0] DAT_I,
    output reg [31:0] DAT_O,
    output reg        ACK_O,
    // UART
    input  rxd,
    output txd,
    output irq
);

    localparam DIV = (CLK_FREQ + BAUD/2) / BAUD;

    localparam R_RXFIFO = 0, R_IER = 1, R_IIR_FCR = 2, R_LCR = 3,
               R_MSR = 4, R_SCR = 5, R_LSR = 7;

    // ------------------------------------------------------------- baud
    reg [31:0] cnt;
    always @(posedge clk or posedge rst) begin
        if (rst)             cnt <= 0;
        else if (cnt == 0)   cnt <= DIV - 1;
        else                 cnt <= cnt - 1;
    end
    wire tick = (cnt == 0);

    // ------------------------------------------------------------- fifo
    reg [7:0]  mem [0:15];
    reg [3:0]  fwr, frd;
    reg [4:0]  fcnt;
    reg oe, fe;

    // rx shift engine: 8N1, LSB first
    localparam RX_IDLE = 0, RX_DATA = 1, RX_STOP = 2;
    reg [1:0] rx_st;
    reg [2:0] rx_n;
    reg [7:0] rx_shift;

    // regs
    reg [1:0] ier;
    reg [7:0] fcr, lcr, scr, iir_last;

    // ---------------------------------------------------------- wb decode
    wire [2:0] byte_ofs = ADR_I[2:0];
    // 4-byte window covers regs {0..3} (grp4=0) or {4..7} (grp4=1); byte
    // lane L (SEL_I[L]) hits reg {grp4,L}. Access MUST gate on the SEL lane
    // of that register: a byte store to one register must not clobber
    // others (the old (8+k-ofs) lane math wrote IER from a THR byte store
    // -- THRE irq stuck high, tb_soc p2 storm -- and started TX from an
    // SCR/MSR byte store; session 17).
    wire       grp4  = byte_ofs[2];
    wire reg0_wr = STB_I && WE_I  && !grp4 && SEL_I[0];   // THR (TX start)
    wire reg0_rd = STB_I && !WE_I && !grp4 && SEL_I[0];   // RBR pop
    wire wr_ier  = STB_I && WE_I  && !grp4 && SEL_I[1];
    wire reg2_wr = STB_I && WE_I  && !grp4 && SEL_I[2];   // FCR
    wire wr_lcr  = STB_I && WE_I  && !grp4 && SEL_I[3];
    wire wr_scr  = STB_I && WE_I  &&  grp4 && SEL_I[1];
    wire rd_iir  = STB_I && !WE_I && !grp4 && SEL_I[2];   // IIR sample
    // value lanes (sub-word store data is replicated by the core; lane
    // choice is irrelevant for them, and full-word uses the true lanes)
    wire [7:0] b_r0  = DAT_I[7:0];
    wire [7:0] b_ier = DAT_I[15:8];
    wire [7:0] b_iir = DAT_I[23:16];
    wire [7:0] b_lcr = DAT_I[31:24];
    wire [7:0] b_scr = DAT_I[15:8];
    wire fcr_sel = iir_last[0];            // FIFO mode: always
    wire fwr_reset = reg2_wr && fcr_sel && b_iir[0];
    wire ftx_reset = reg2_wr && fcr_sel && b_iir[1];

    // push: valid stop bit received
    wire rx_push = tick && (rx_st == RX_STOP) && rxd;
    wire pop1 = reg0_rd && (fcnt > 0);

    // SYNCHRONOUS reset: this is the RX-FIFO storage-array writer process
    // (mem[]); clk-only keeps the array $mem-inferable (async-reset writers
    // fall back to a register list). Reset held across >=2 edges everywhere.
    always @(posedge clk) begin
        if (rst) begin
            fwr <= 0; frd <= 0; fcnt <= 0; oe <= 0; fe <= 0;
            rx_st <= RX_IDLE; rx_n <= 0; rx_shift <= 0;
        end else begin
            // shift engine
            if (tick) begin
                case (rx_st)
                RX_IDLE: if (!rxd) begin
                             rx_st <= RX_DATA; rx_n <= 0; rx_shift <= 0;
                         end
                RX_DATA: begin
                    rx_shift <= {rxd, rx_shift[7:1]};
                    if (rx_n == 7) begin        // 8th bit shifted: to stop
                        rx_st <= RX_STOP; rx_n <= 0;
                    end else begin
                        rx_n <= rx_n + 1;
                    end
                end
                RX_STOP: begin
                    rx_st <= RX_IDLE;
                    if (!rxd) fe <= 1;           // frame error: char dropped
                end
                default: ;
                endcase
            end
            // push
            if (rx_push && fcnt != 16) begin
                mem[fwr] <= rx_shift;
                fwr <= fwr + 1;
            end else if (rx_push) begin
                oe <= 1;                         // overflow: byte dropped
            end
            // pop
            if (pop1) begin
                frd <= frd + 1;
            end
            // dummy read or write of R0 clears OE/FE
            if (reg0_wr || reg0_rd) begin oe <= 0; fe <= 0; end
            // count
            if (fwr_reset) begin
                fwr <= 0; frd <= 0; fcnt <= 0; oe <= 0;
            end else begin
                fcnt <= fcnt
                      + (rx_push && fcnt != 16 ? 5'd1 : 5'd0)
                      - (pop1 ? 5'd1 : 5'd0);
            end
        end
    end

    // ------------------------------------------------------------- tx
    reg [7:0] tx_data;
    reg [3:0] tx_n;        // 0=start, 1..8=data bits 0..7, 9=stop
    reg       tx_valid;
    reg       tx_pending;  // holding loaded, wait for baud tick
    wire tx_busy = tx_valid;
    wire [3:0] tx_bit_sel = tx_n - 1;
    assign txd = tx_busy ?
                ((tx_n == 0) ? 1'b0 :
                 (tx_n < 9)  ? tx_data[tx_bit_sel] :
                               1'b1) :
                1'b1;
    wire temt = !tx_busy;
    wire thre = 1'b1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx_data <= 0; tx_n <= 0; tx_valid <= 0; tx_pending <= 0;
        end else begin
            // write to holding register (dropped if TX in flight)
            if (reg0_wr && !tx_valid && !tx_pending) begin
                tx_data <= b_r0;
                tx_n    <= 0;
                tx_pending <= 1;
            end
            if (tick) begin
                // start on the tick (full start-bit duration)
                if (tx_pending && !tx_valid) begin
                    tx_pending <= 0;
                    tx_valid   <= 1;
                    tx_n       <= 0;
                end
                if (tx_valid && !tx_pending) begin
                    if (tx_n == 9) tx_valid <= 0;   // stop bit done
                    else           tx_n     <= tx_n + 1;
                end
            end
            if (ftx_reset) begin                   // fcr tx reset
                tx_valid <= 0; tx_pending <= 0;
            end
        end
    end

    // ------------------------------------------------------------- irq/iir
    wire [4:0] trig_lvl = (fcr[7:6] == 0) ? 5'd1  :
                          (fcr[7:6] == 1) ? 5'd2  :
                          (fcr[7:6] == 2) ? 5'd4  : 5'd8;
    wire rx_active  = (fcnt >= trig_lvl);
    wire ier_dr_irq = ier[0] & rx_active;
    wire [2:0] iir  = ier_dr_irq ? 3'd6 : ier[1] ? 3'd7 : 3'd1;
    assign irq = ier_dr_irq | ier[1];

    wire dr = fcnt > 0;
    wire [7:0] lsr = {1'b0, thre, temt, 1'b0, fe, 1'b0, oe, dr};

    // ------------------------------------------------------------- regs
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ier <= 0; fcr <= 0; lcr <= 8'h03; scr <= 0; iir_last <= 3'd1;
            ACK_O <= 0; DAT_O <= 0;
        end else begin
            ACK_O <= 0;
            if (STB_I) begin
                ACK_O <= 1;
                if (WE_I) begin
                    if (reg0_wr) begin oe <= 0; fe <= 0; end
                    if (wr_ier) ier <= b_ier[1:0];
                    if (reg2_wr) fcr <= b_iir;
                    if (wr_lcr) lcr <= b_lcr;
                    if (wr_scr) scr <= b_scr;
                end else begin
                    if (rd_iir) iir_last <= iir;
                    DAT_O[7:0]  <= rb0;
                    DAT_O[15:8] <= rb1;
                    DAT_O[23:16] <= rb2;
                    DAT_O[31:24] <= rb3;
                end
            end
        end
    end

    // read lane values: aligned 4-byte window, lane L -> reg {grp4, L}
    // (0 RBR/THR, 1 IER, 2 IIR, 3 LCR | 4 MSR, 5 SCR, 6 -, 7 LSR)
    reg  [7:0] rb0, rb1, rb2, rb3;
    always @(*) begin
        if (!grp4) begin
            rb0 = pop1 ? mem[frd] : 8'h00;      // RBR (pop advances FIFO)
            rb1 = {6'h0, ier};
            rb2 = {5'h0, iir};
            rb3 = lcr;
        end else begin
            rb0 = 8'h9F;                        // MSR: THD|TEMT (RO)
            rb1 = scr;
            rb2 = 8'h00;
            rb3 = lsr;
        end
    end

endmodule
`default_nettype wire
