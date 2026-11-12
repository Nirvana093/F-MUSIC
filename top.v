`timescale 1ns / 1ps

module top (
    input  wire        clk_50m,       // 板载 50MHz 晶振 (W19)
    output wire [8:0]  led,           // U22, V22, W21, W22, Y21, Y22, N13, N17, P17
    input  wire [4:0]  s_keys,        // S[4:0] 轻触按键 (D21, E21, G22, G21, B22)
    output wire        seg7_dio,      // U20
    output wire        seg7_rclk,     // V20
    output wire        seg7_sclk,     // T21
    output wire        tp_scl,        // L13
    inout  wire        tp_sda,        // L14
    output wire        tp_rst,        // L15
    inout  wire        tp_int,        // M13
    input  wire        uart_rxd,      // L21
    output wire        uart_txd,      // M21
    output wire        lcd_de,
    output wire        lcd_hsync,
    output wire        lcd_vsync,
    output wire [4:0]  lcd_r,
    output wire [5:0]  lcd_g,
    output wire [4:0]  lcd_b,
    output wire        lcd_bl,
    output wire        lcd_clk,
    output wire [14:0] O_ddr_addr,
    output wire [2:0]  O_ddr_ba,
    output wire        O_ddr_cs_n,
    output wire        O_ddr_ras_n,
    output wire        O_ddr_cas_n,
    output wire        O_ddr_we_n,
    output wire        O_ddr_clk,
    output wire        O_ddr_clk_n,
    output wire        O_ddr_cke,
    output wire        O_ddr_odt,
    output wire        O_ddr_reset_n,
    output wire [1:0]  O_ddr_dqm,
    inout  wire [15:0] IO_ddr_dq,
    inout  wire [1:0]  IO_ddr_dqs,
    inout  wire [1:0]  IO_ddr_dqs_n,
    inout  wire        flash_csn,     // T19 (QSPI_CS)
    inout  wire        flash_mosi,    // P22 (QSPI_DQ0)
    inout  wire        flash_miso,    // R22 (QSPI_DQ1)
    inout  wire        flash_wpn,     // P21 (QSPI_DQ2)
    inout  wire        flash_holdn,   // R21 (QSPI_DQ3)
    inout  wire        flash_clk,     // L12 (QSPI_SCK)
    input  wire        midi_rx_in,    // P11 Pin 1  (C18, 保持不变)
    output wire        wm_mode,       // 新 P11 Pin 16 (D15) -> 杜邦线接扩展板 Pin 15
    output wire        wm_csb_gpio1,  // 新 P11 Pin 14 (E17) -> 杜邦线接扩展板 Pin 13
    output wire        wm_i2c_sclk,   // 新 P11 Pin 8  (E18) -> 杜邦线接扩展板 Pin 7
    inout  wire        wm_i2c_sdin,   // 新 P11 Pin 10 (D16) -> 杜邦线接扩展板 Pin 9
    output wire        wm_mclk,       // 新 P11 Pin 22 (B18) -> 杜邦线接扩展板 Pin 21
    inout  wire        wm_bclk,       // 新 P11 Pin 24 (C17) -> 杜邦线接扩展板 Pin 23
    inout  wire        wm_lrc,        // 新 P11 Pin 26 (B16) -> 杜邦线接扩展板 Pin 25
    output wire        wm_dacdat,     // 新 P11 Pin 28 (A19) -> 杜邦线接扩展板 Pin 27
    input  wire        wm_adcdat,     // 新 P11 Pin 32 (C15) -> 杜邦线接扩展板 Pin 31
    output wire [4:0]  iso_guard,     // P11 Pin 5, 17, 19, 33, 39
    output wire [17:0] vgnd_shield    // 其中 [6:3] 与 [13:9] 已置换至原奇数脚，设为高阻态 1'bz
);
    // 单按住板载 S3 键 (且未按 S4) 时，新偶数输出引脚全部拉高为 3.3V 供万用表排查通断
    wire pin_dc3v3_test = (~s_keys[3]) & s_keys[4];

    wire raw_wm_mode;
    wire raw_wm_csb;
    wire raw_wm_sclk;
    wire raw_wm_mclk;
    wire raw_wm_dacdat;

    assign wm_mode      = pin_dc3v3_test ? 1'b1 : raw_wm_mode;
    assign wm_csb_gpio1 = pin_dc3v3_test ? 1'b1 : raw_wm_csb;
    assign wm_i2c_sclk  = pin_dc3v3_test ? 1'b1 : raw_wm_sclk;
    assign wm_mclk      = pin_dc3v3_test ? 1'b1 : raw_wm_mclk;
    assign wm_dacdat    = pin_dc3v3_test ? 1'b1 : raw_wm_dacdat;

    assign iso_guard    = 5'd0;

    // 关键保护：将置换到原奇数排针 (Pin 7, 9, 13, 15, 21, 23, 25, 27, 31) 的通道设为高阻态 z，
    // 彻底杜绝原奇数脚输出 0V 拉低或干扰新偶数脚信号！其余未置换的偶数屏蔽脚保持 0V。
    assign vgnd_shield[2:0]   = 3'b000;   // Pin 2, 4, 6
    assign vgnd_shield[6:3]   = 4'bzzzz;  // 原奇数脚 Pin 7, 9, 13, 15 (高阻态释放)
    assign vgnd_shield[8:7]   = 2'b00;    // Pin 18, 20
    assign vgnd_shield[13:9]  = 5'bzzzzz; // 原奇数脚 Pin 21, 23, 25, 27, 31 (高阻态释放)
    assign vgnd_shield[17:14] = 4'b0000;  // Pin 34, 36, 38, 40

    wire clk_10m;
    wire clk_33m;
    wire clk_400m;
    wire pll_lock;
    wire pll_stop;
    wire ddr_ui_clk;
    wire ddr_ui_rst;
    wire init_calib_complete;
    wire vram_init_done;
    assign lcd_clk = clk_33m;

    reg [24:0] por_cnt    = 25'd0;
    reg        rst_n_sync = 1'b0;
    always @(posedge clk_50m) begin
        por_cnt <= por_cnt + 25'd1;
        if (&por_cnt[15:0])
            rst_n_sync <= 1'b1;
    end

    (* syn_preserve = 1 *) reg [1:0] vram_done_50m = 2'b00;
    always @(posedge clk_50m) begin
        vram_done_50m <= {vram_done_50m[0], vram_init_done};
    end

    // 1. 例化 PLL IP 核
    Gowin_PLL u_pll (
        .clkin    (clk_50m),
        .init_clk (clk_50m),
        .clkout0  (clk_10m),
        .clkout1  (clk_33m),
        .clkout2  (clk_400m),
        .lock     (pll_lock),
        .reset    (1'b0),
        .enclk0   (1'b1),
        .enclk1   (1'b1),
        .enclk2   (pll_stop)
    );

    // 2. 例化 Gowin_GoRV32 软核
    wire [31:0] exts_hrdata;
    wire        exts_hreadyin;
    wire [1:0]  exts_hresp;
    wire        exts_hreadyout;
    wire [31:0] exts_haddr;
    wire [2:0]  exts_hburst;
    wire [3:0]  exts_hprot;
    wire        exts_hsel;
    wire [2:0]  exts_hsize;
    wire [1:0]  exts_htrans;
    wire [31:0] exts_hwdata;
    wire        exts_hwrite;
    wire        exts_hclk;
    wire        exts_hrstn;
    wire [31:0] gorv_gpio_oe, gorv_gpio_out;
    wire        cpu_por_n = rst_n_sync & vram_done_50m[1];

    Gowin_GoRV32_Top u_gorv32 (
        .HCLK             (clk_50m),
        .PCLK             (clk_50m),
        .POR_N            (cpu_por_n),
        .EXT_GPIO_CLK     (clk_50m),
        .GPIO_IN          ({27'd0, ~s_keys}),
        .GPIO_OE          (gorv_gpio_oe),
        .GPIO_OUT         (gorv_gpio_out),
        .UART1_CTSN       (1'b0),
        .UART1_DCDN       (1'b0),
        .UART1_DSRN       (1'b0),
        .UART1_RIN        (1'b1),
        .UART1_RXD        (uart_rxd),
        .UART1_DTRN       (),
        .UART1_OUT1N      (),
        .UART1_OUT2N      (),
        .UART1_RTSN       (),
        .UART1_TXD        (uart_txd),
        .FLASH_QSPI_CSN   (flash_csn),
        .FLASH_QSPI_MISO  (flash_miso),
        .FLASH_QSPI_MOSI  (flash_mosi),
        .FLASH_QSPI_CLK   (flash_clk),
        .FLASH_QSPI_HOLDN (flash_holdn),
        .FLASH_QSPI_WPN   (flash_wpn),
        .I2C_SCL_IN       (1'b1),
        .I2C_SDA_IN       (1'b1),
        .I2C_SCL_OUT      (),
        .I2C_SDA_OUT      (),
        .EXT_INT          (8'd0),
        .EXTS_HRDATA      (exts_hrdata),
        .EXTS_HREADYIN    (exts_hreadyin),
        .EXTS_HRESP       (exts_hresp),
        .EXTS_HREADYOUT   (exts_hreadyout),
        .EXTS_HADDR       (exts_haddr),
        .EXTS_HBURST      (exts_hburst),
        .EXTS_HPROT       (exts_hprot),
        .EXTS_HSEL        (exts_hsel),
        .EXTS_HSIZE       (exts_hsize),
        .EXTS_HTRANS      (exts_htrans),
        .EXTS_HWDATA      (exts_hwdata),
        .EXTS_HWRITE      (exts_hwrite),
        .EXTS_HCLK        (exts_hclk),
        .EXTS_HRSTN       (exts_hrstn),
        .JTAG_TMS         (1'b1),
        .JTAG_TDI         (1'b1),
        .JTAG_TDO         (),
        .JTAG_TCK         (1'b0)
    );

    // 3. GoRV32 全链路 9-LED 硬件诊断状态机
    reg [7:0]  hclk_alive_cnt  = 8'd0;
    reg        diag_hclk_ok    = 1'b0;
    reg        diag_gpio_alive = 1'b0;
    reg [15:0] gpio_snap       = 16'h0000;
    wire       diag_ahb_hsel_hit;
    wire       diag_ahb_reg_wr;
    wire       diag_ahb_vram_wr;
    wire       diag_ahb_tp_rd_tog;
    wire       wm_init_done;
    wire       wm_i2c_err;

    always @(posedge exts_hclk or negedge cpu_por_n) begin
        if (!cpu_por_n) begin
            hclk_alive_cnt <= 8'd0;
            diag_hclk_ok   <= 1'b0;
        end else begin
            if (hclk_alive_cnt != 8'hFF)
                hclk_alive_cnt <= hclk_alive_cnt + 8'd1;
            else
                diag_hclk_ok <= 1'b1;
        end
    end

    always @(posedge clk_50m) begin
        if (!cpu_por_n) begin
            diag_gpio_alive <= 1'b0;
            gpio_snap       <= 16'h0000;
        end else if ((gorv_gpio_oe != 32'd0) || (gorv_gpio_out != 32'd0)) begin
            diag_gpio_alive <= 1'b1;
            gpio_snap       <= gorv_gpio_out[15:0] | gorv_gpio_oe[15:0];
        end
    end

    assign led[0] = pll_lock & init_calib_complete & vram_done_50m[1];
    assign led[1] = cpu_por_n;
    assign led[2] = exts_hrstn;
    assign led[3] = diag_hclk_ok;
    assign led[4] = diag_gpio_alive;
    assign led[5] = diag_ahb_hsel_hit;
    assign led[6] = diag_ahb_reg_wr;
    assign led[7] = diag_ahb_vram_wr;
    assign led[8] = diag_ahb_tp_rd_tog;

    wire [23:0] diag_chain_word_24b;
    wire [31:0] diag_cc_word = {8'd0, diag_chain_word_24b};

    // 4. 例化 DDR3 IP 核与 AHB-LCD 调制桥接器
    wire         cmd_ready;
    wire [2:0]   bridge_cmd;
    wire         bridge_cmd_en;
    wire [27:0]  bridge_addr_28b;
    wire         wr_data_rdy;
    wire [127:0] bridge_wr_data;
    wire         bridge_wr_data_en;
    wire         bridge_wr_data_end;
    wire [15:0]  bridge_wr_data_mask;
    wire [127:0] rd_data;
    wire         rd_data_valid;
    wire         rd_data_end;
    wire         sr_req, ref_req, sr_ack, ref_ack, burst;
    wire [15:0]  bridge_rgb565;
    wire [31:0]  touch_xy_coord;
    wire [511:0] mod_regs_flat;

    DDR3_Memory_Interface_Top u_ddr3 (
        .clk                 (clk_50m),
        .pll_stop            (pll_stop),
        .memory_clk          (clk_400m),
        .pll_lock            (pll_lock),
        .rst_n               (rst_n_sync),
        .clk_out             (ddr_ui_clk),
        .ddr_rst             (ddr_ui_rst),
        .init_calib_complete (init_calib_complete),
        .cmd_ready           (cmd_ready),
        .cmd                 (bridge_cmd),
        .cmd_en              (bridge_cmd_en),
        .addr                ({1'b0, bridge_addr_28b}),
        .wr_data_rdy         (wr_data_rdy),
        .wr_data             (bridge_wr_data),
        .wr_data_en          (bridge_wr_data_en),
        .wr_data_end         (bridge_wr_data_end),
        .wr_data_mask        (bridge_wr_data_mask),
        .rd_data             (rd_data),
        .rd_data_valid       (rd_data_valid),
        .rd_data_end         (rd_data_end),
        .sr_req              (sr_req),
        .ref_req             (ref_req),
        .sr_ack              (sr_ack),
        .ref_ack             (ref_ack),
        .burst               (burst),
        .O_ddr_addr          (O_ddr_addr),
        .O_ddr_ba            (O_ddr_ba),
        .O_ddr_cs_n          (O_ddr_cs_n),
        .O_ddr_ras_n         (O_ddr_ras_n),
        .O_ddr_cas_n         (O_ddr_cas_n),
        .O_ddr_we_n          (O_ddr_we_n),
        .O_ddr_clk           (O_ddr_clk),
        .O_ddr_clk_n         (O_ddr_clk_n),
        .O_ddr_cke           (O_ddr_cke),
        .O_ddr_odt           (O_ddr_odt),
        .O_ddr_reset_n       (O_ddr_reset_n),
        .O_ddr_dqm           (O_ddr_dqm),
        .IO_ddr_dq           (IO_ddr_dq),
        .IO_ddr_dqs          (IO_ddr_dqs),
        .IO_ddr_dqs_n        (IO_ddr_dqs_n)
    );

    ahb_ddr3_lcd_bridge u_bridge (
        .ahb_hclk            (exts_hclk),
        .ahb_hrstn           (exts_hrstn),
        .ahb_hsel            (exts_hsel),
        .ahb_haddr           (exts_haddr),
        .ahb_htrans          (exts_htrans),
        .ahb_hwrite          (exts_hwrite),
        .ahb_hsize           (exts_hsize),
        .ahb_hwdata          (exts_hwdata),
        .ahb_hreadyin        (exts_hreadyout),
        .ahb_hrdata          (exts_hrdata),
        .ahb_hreadyout       (exts_hreadyin),
        .ahb_hresp           (exts_hresp),
        .touch_xy_coord_in   (touch_xy_coord),
        .diag_cc_in          (diag_cc_word),
        .mod_regs_flat       (mod_regs_flat),
        .diag_ahb_hsel_hit   (diag_ahb_hsel_hit),
        .diag_ahb_reg_wr     (diag_ahb_reg_wr),
        .diag_ahb_vram_wr    (diag_ahb_vram_wr),
        .diag_ahb_tp_rd_tog  (diag_ahb_tp_rd_tog),
        .ddr_clk             (ddr_ui_clk),
        .ddr_rst             (ddr_ui_rst),
        .init_calib_complete (init_calib_complete),
        .cmd_ready           (cmd_ready),
        .cmd                 (bridge_cmd),
        .cmd_en              (bridge_cmd_en),
        .addr                (bridge_addr_28b),
        .wr_data_rdy         (wr_data_rdy),
        .wr_data             (bridge_wr_data),
        .wr_data_en          (bridge_wr_data_en),
        .wr_data_end         (bridge_wr_data_end),
        .wr_data_mask        (bridge_wr_data_mask),
        .rd_data             (rd_data),
        .rd_data_valid       (rd_data_valid),
        .rd_data_end         (rd_data_end),
        .sr_req              (sr_req),
        .ref_req             (ref_req),
        .burst               (burst),
        .lcd_clk             (clk_33m),
        .lcd_vsync           (lcd_vsync),
        .lcd_de              (lcd_de),
        .lcd_rgb565          (bridge_rgb565),
        .init_done           (vram_init_done)
    );

    // 5. 例化 GT9147 硬件自动触控采集模块
    gt9147_touch_ctrl u_touch (
        .clk_50m        (clk_50m),
        .rst_n          (rst_n_sync),
        .tp_scl         (tp_scl),
        .tp_sda         (tp_sda),
        .tp_rst         (tp_rst),
        .tp_int         (tp_int),
        .touch_xy_coord (touch_xy_coord)
    );

    // 6. 例化按键状态机与 74HC595 八位数码管监控探针
    seg7_key_probe u_seg7_probe (
        .clk_50m       (clk_50m),
        .rst_n         (rst_n_sync),
        .s_keys        (s_keys),
        .mod_regs_flat (mod_regs_flat),
        .seg7_dio      (seg7_dio),
        .seg7_rclk     (seg7_rclk),
        .seg7_sclk     (seg7_sclk)
    );

    // 7. 例化 LCD 时序驱动模块
    wire signed [15:0] synth_scope_pcm;

    lcd_rgb_timing u_lcd_timing (
        .lcd_clk      (clk_33m),
        .sys_rst_n    (rst_n_sync),
        .pixel_data   (vram_init_done ? bridge_rgb565 : 16'h001F),
        .ui_page_id   (mod_regs_flat[19:16]),
        .scope_pcm_in (synth_scope_pcm),
        .lcd_de       (lcd_de),
        .lcd_hsync    (lcd_hsync),
        .lcd_vsync    (lcd_vsync),
        .lcd_r        (lcd_r),
        .lcd_g        (lcd_g),
        .lcd_b        (lcd_b),
        .lcd_bl       (lcd_bl)
    );

    // 8. 例化 F-MUSIC 合成器子系统
    wire synth_enable_n = rst_n_sync & vram_done_50m[1];
    fmusic_synth_subsystem #(
        .USE_TOUCH_UI_REGS  (1),
        .DIRECT_ALG_TO_MODE (0),
        .WM_USE_INT_PLL     (0)
    ) u_fmusic_synth (
        .clk_50m             (clk_50m),
        .rst_n               (synth_enable_n),
        .mod_regs_flat       (mod_regs_flat),
        .midi_rx_in          (midi_rx_in),
        .test_key_note       (~s_keys[4]),
        .pure_sine_byp       ((~s_keys[3]) & (~s_keys[4])),
        .ext_pwr_ok          (pll_lock & vram_done_50m[1]),
        .wm_mode             (raw_wm_mode),
        .wm_csb_gpio1        (raw_wm_csb),
        .wm_i2c_sclk         (raw_wm_sclk),
        .wm_i2c_sdin         (wm_i2c_sdin),
        .wm_mclk             (raw_wm_mclk),
        .wm_bclk             (wm_bclk),
        .wm_lrc              (wm_lrc),
        .wm_dacdat           (raw_wm_dacdat),
        .wm_adcdat           (wm_adcdat),
        .scope_pcm_out       (synth_scope_pcm),
        .wm_init_done        (wm_init_done),
        .wm_i2c_err          (wm_i2c_err),
        .diag_chain_word_24b (diag_chain_word_24b)
    );
endmodule