// soc -- mythrv32 SoC top (simulation; board adds SDRAM/LED/JTAG later)
//
// Integrates the RV32IM core (mythrv32) with its three Wishbone B4 masters
// (m0=I-cache, m1=D-cache, m2=MMU walk) through the pipelined interconnect
// (wb_ic) to the five slaves:
//   s0 ROM   (boot, base 0x0000_0000, 32 KiB)
//   s1 DRAM  (base 0x8000_0000, 4 MiB): wb_ram when DRAM_SDRAM=0 (sim),
//            wb_sdram controller when DRAM_SDRAM=1 (board; pads below)
//   s2 UART  (16550, base 0x1000_0000)
//   s3 CLINT (base 0x0200_0000)
//   s4 PLIC  (base 0x0C00_0000)
//
// Interrupt fabric (per notes/design.md sec.3):
//   PLIC source 1  = 16550.irq
//   PLIC source 2  = clint.ip_ssw   (S software)
//   PLIC source 3  = clint.ip_msw   (M software)
//   PLIC source 6  = clint.ip_stmr  (S timer)
//   PLIC source 7  = clint.ip_mtmr  (M timer)
//   PLIC source 10 = clint.ip_sext  (S external)
//   PLIC source 11 = clint.ip_mext  (M external)
//   PLIC.EIP[0] = meip (ctx M), PLIC.EIP[1] = seip (ctx S) -> core.
//
// The core's WB masters are 1-outstanding (l1_i/l1_d/mmu each issue at most
// one in-flight STB and wait for ACK), so wb_ic's STALL_k never asserts and
// is left unconnected.
`default_nettype none

// DRAM_SDRAM default is macro-driven so the SYNTHESIS ELABORATION of this
// module can select the board path before any instance override happens:
// yosys elaborates generate blocks at READ time with the module's own
// (default) parameters -- the derived per-instance copy (top.v passes
// DRAM_SDRAM=1) comes later. Without this, the base \soc elaborates
// wb_ram #(.DEPTH(1048576)) = a 32 Mbit register list (yosys OOM; it
// killed the session-27 synth attempt). The board flow reads the file list
// WITHOUT rtl/wb_ram.v and adds -DDRAM_SDRAM_DEFAULT=1 (Makefile synth);
// simulations never define the macro and keep parameter-driven choice.
`ifndef DRAM_SDRAM_DEFAULT
  `define DRAM_SDRAM_DEFAULT 0
`endif

module soc #(
    parameter ROM_DEPTH  = 8192,
    parameter ROM_HEX    = "",
    parameter RAM_DEPTH  = 1048576,   // wb_ram path only; sim TBs reduce
    parameter UART_CLK   = 25_000_000,
    parameter UART_BAUD  = 115_200,
    parameter CLK_HZ     = 25_000_000,  // sysclk frequency (wb_sdram timing)
    parameter DRAM_SDRAM = `DRAM_SDRAM_DEFAULT, // 0: wb_ram (sim); 1: wb_sdram (board/synth)
    parameter integer LINES_I = 1024, // I$ lines
    parameter integer LINES_D = 1024  // D$ lines
)(
    input clk, rst,
    input  uart_rxd,
    output uart_txd,

    // SDRAM pads (active with DRAM_SDRAM=1; the board top owns sd_clk and
    // the dq tristate -- wb_sdram drives registered outputs only, sec.6a)
    output [1:0]  sd_ba_o,
    output [12:0] sd_a_o,
    output        sd_ras_n_o, sd_cas_n_o, sd_we_n_o,
    output        sd_cs_n_o, sd_cke_o,
    output [3:0]  sd_dqm_o,
    output [31:0] sd_dq_o,
    output        sd_dq_oe_o,
    input  [31:0] sd_dq_i
);

    // ------------------------------------------------------------- core
    wire        m0_stb, m1_stb, m2_stb;
    wire [3:0]  m0_sel, m1_sel, m2_sel;
    wire [31:0] m0_adr, m1_adr, m2_adr;
    wire        m0_we, m1_we, m2_we;
    wire [31:0] m1_wdat;
    wire        m0_ack, m1_ack, m2_ack;
    wire [31:0] m0_dat, m1_dat, m2_dat;
    wire        meip, seip;

    core #(.LINES_I(LINES_I), .LINES_D(LINES_D)) core0 (
        .clk(clk), .rst(rst),
        .meip(meip), .seip(seip),
        .m0_stb(m0_stb), .m0_sel(m0_sel), .m0_adr(m0_adr), .m0_we(m0_we),
        .m0_ack(m0_ack), .m0_dat(m0_dat),
        .m1_stb(m1_stb), .m1_sel(m1_sel), .m1_adr(m1_adr), .m1_we(m1_we),
        .m1_wdat(m1_wdat), .m1_ack(m1_ack), .m1_dat(m1_dat),
        .m2_stb(m2_stb), .m2_sel(m2_sel), .m2_adr(m2_adr), .m2_we(m2_we),
        .m2_ack(m2_ack), .m2_dat(m2_dat)
    );

    // ------------------------------------------------------------ wb_ic
    wire        ic_m0_stall, ic_m1_stall, ic_m2_stall;   // unused (1-outstanding masters)
    wire        s0_stb, s0_we, s1_stb, s1_we, s2_stb, s2_we, s3_stb, s3_we, s4_stb, s4_we;
    wire [3:0]  s0_sel, s1_sel, s2_sel, s3_sel, s4_sel;
    wire [31:0] s0_adr, s0_wdat, s1_adr, s1_wdat, s2_adr, s2_wdat, s3_adr, s3_wdat, s4_adr, s4_wdat;
    wire        s0_ack, s1_ack, s2_ack, s3_ack, s4_ack;
    wire [31:0] s0_rdat, s1_rdat, s2_rdat, s3_rdat, s4_rdat;

    // ic_m*_stall are wb_ic OUTPUTS left dangling (masters are 1-outstanding;
    // FIFO depth 4 can never fill). No constant drivers here: iverilog
    // tolerated the multi-driven net (const-0 + cell output), yosys errors.

    wb_ic ic0 (
        .clk(clk), .rst(rst),
        .m0_STB(m0_stb), .m0_WE(m0_we), .m0_SEL(m0_sel), .m0_ADR(m0_adr), .m0_WDAT(32'h0),
        .m0_STALL(ic_m0_stall), .m0_ACK(m0_ack), .m0_RDAT(m0_dat),
        .m1_STB(m1_stb), .m1_WE(m1_we), .m1_SEL(m1_sel), .m1_ADR(m1_adr), .m1_WDAT(m1_wdat),
        .m1_STALL(ic_m1_stall), .m1_ACK(m1_ack), .m1_RDAT(m1_dat),
        .m2_STB(m2_stb), .m2_WE(m2_we), .m2_SEL(m2_sel), .m2_ADR(m2_adr), .m2_WDAT(32'h0),
        .m2_STALL(ic_m2_stall), .m2_ACK(m2_ack), .m2_RDAT(m2_dat),
        .s0_STB(s0_stb), .s0_WE(s0_we), .s0_SEL(s0_sel), .s0_ADR(s0_adr), .s0_WDAT(s0_wdat),
        .s0_ACK(s0_ack), .s0_RDAT(s0_rdat),
        .s1_STB(s1_stb), .s1_WE(s1_we), .s1_SEL(s1_sel), .s1_ADR(s1_adr), .s1_WDAT(s1_wdat),
        .s1_ACK(s1_ack), .s1_RDAT(s1_rdat),
        .s2_STB(s2_stb), .s2_WE(s2_we), .s2_SEL(s2_sel), .s2_ADR(s2_adr), .s2_WDAT(s2_wdat),
        .s2_ACK(s2_ack), .s2_RDAT(s2_rdat),
        .s3_STB(s3_stb), .s3_WE(s3_we), .s3_SEL(s3_sel), .s3_ADR(s3_adr), .s3_WDAT(s3_wdat),
        .s3_ACK(s3_ack), .s3_RDAT(s3_rdat),
        .s4_STB(s4_stb), .s4_WE(s4_we), .s4_SEL(s4_sel), .s4_ADR(s4_adr), .s4_WDAT(s4_wdat),
        .s4_ACK(s4_ack), .s4_RDAT(s4_rdat)
    );

    // ------------------------------------------------------------ slaves
    wb_rom #(.DEPTH(ROM_DEPTH), .ROM_HEX(ROM_HEX)) s0rom (
        .clk(clk), .rst(rst), .STB_I(s0_stb), .WE_I(s0_we), .SEL_I(s0_sel),
        .ADR_I(s0_adr), .DAT_I(s0_wdat), .ACK_O(s0_ack), .DAT_O(s0_rdat)
    );
    generate
    if (DRAM_SDRAM) begin : g_dram_sdram
        wb_sdram #(.CLK_FREQ(CLK_HZ)) s1sdram (
            .clk(clk), .rst(rst), .STB_I(s1_stb), .WE_I(s1_we), .SEL_I(s1_sel),
            .ADR_I(s1_adr), .DAT_I(s1_wdat), .ACK_O(s1_ack), .DAT_O(s1_rdat),
            .sd_ba_o(sd_ba_o), .sd_a_o(sd_a_o),
            .sd_ras_n_o(sd_ras_n_o), .sd_cas_n_o(sd_cas_n_o),
            .sd_we_n_o(sd_we_n_o), .sd_cs_n_o(sd_cs_n_o), .sd_cke_o(sd_cke_o),
            .sd_dqm_o(sd_dqm_o), .sd_dq_o(sd_dq_o), .sd_dq_oe_o(sd_dq_oe_o),
            .sd_dq_i(sd_dq_i)
        );
    end else begin : g_dram_ram
        wb_ram #(.DEPTH(RAM_DEPTH)) s1ram (
            .clk(clk), .rst(rst), .STB_I(s1_stb), .WE_I(s1_we), .SEL_I(s1_sel),
            .ADR_I(s1_adr), .DAT_I(s1_wdat), .ACK_O(s1_ack), .DAT_O(s1_rdat)
        );
    end
    endgenerate

    wire        uart_irq;
    wire [5:0]  clint_ip;
    wire [1:0]  eip;
    u16550 #(.CLK_FREQ(UART_CLK), .BAUD(UART_BAUD)) s2uart (
        .clk(clk), .rst(rst), .STB_I(s2_stb), .WE_I(s2_we), .SEL_I(s2_sel),
        .ADR_I(s2_adr), .DAT_I(s2_wdat), .ACK_O(s2_ack), .DAT_O(s2_rdat),
        .rxd(uart_rxd), .txd(uart_txd), .irq(uart_irq)
    );
    clint s3clint (
        .clk(clk), .rst(rst), .STB_I(s3_stb), .WE_I(s3_we), .SEL_I(s3_sel),
        .ADR_I(s3_adr), .DAT_I(s3_wdat), .ACK_O(s3_ack), .DAT_O(s3_rdat),
        .ip_ssw(clint_ip[0]), .ip_msw(clint_ip[1]),
        .ip_stmr(clint_ip[2]), .ip_mtmr(clint_ip[3]),
        .ip_sext(clint_ip[4]), .ip_mext(clint_ip[5])
    );

    // PLIC IRQ fan-in (bit N = source N, N=1..16; see design.md sec.3)
    wire [16:0] plic_irq;
    assign plic_irq[0]  = 1'b0;         // src 0 does not exist (spec ch.6)
    assign plic_irq[1]  = uart_irq;
    assign plic_irq[2]  = clint_ip[0];    // ip_ssw
    assign plic_irq[3]  = clint_ip[1];    // ip_msw
    assign plic_irq[6]  = clint_ip[2];    // ip_stmr
    assign plic_irq[7]  = clint_ip[3];    // ip_mtmr
    assign plic_irq[10] = clint_ip[4];    // ip_sext
    assign plic_irq[11] = clint_ip[5];    // ip_mext
    assign plic_irq[16:12] = 5'b0;
    assign plic_irq[5:4]   = 2'b0;
    assign plic_irq[9:8]   = 2'b0;

    plic s4plic (
        .clk(clk), .rst(rst), .STB_I(s4_stb), .WE_I(s4_we), .SEL_I(s4_sel),
        .ADR_I(s4_adr), .DAT_I(s4_wdat), .ACK_O(s4_ack), .DAT_O(s4_rdat),
        .IRQ(plic_irq), .EIP(eip)
    );

    assign meip = eip[0];    // ctx0 = M
    assign seip = eip[1];    // ctx1 = S

endmodule
`default_nettype wire
