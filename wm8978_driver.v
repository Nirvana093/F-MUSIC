`timescale 1ns / 1ps

module wm8978_driver #(
    parameter        SYS_CLK_FREQ   = 50_000_000,
    parameter        I2C_FREQ       = 25_000,
    parameter        USE_INT_PLL    = 1,
    parameter        AUDIO_WORD_LEN = 24,
    parameter [21:0] PWR_WAIT_CLKS  = 22'd2_500_000
)(
    input  wire                        clk_50m,
    input  wire                        rst_n,
    input  wire                        ext_pwr_ok,
    input  wire                        reinit_trig,
    input  wire [7:0]                  master_vol_pct,
    input  wire [AUDIO_WORD_LEN-1:0]   pcm_left_in,
    input  wire [AUDIO_WORD_LEN-1:0]   pcm_right_in,
    output wire                        sample_req_pulse,
    output wire [4:0]                  i2c_fifo_cnt,
    output reg                         init_done,
    output reg                         i2c_ack_err,
    output wire                        wm_mode,
    output wire                        wm_csb_gpio1,
    output wire                        wm_i2c_sclk,
    inout  wire                        wm_i2c_sdin,
    output wire                        wm_mclk,
    inout  wire                        wm_bclk,
    inout  wire                        wm_lrc,
    output reg                         wm_dacdat,
    input  wire                        wm_adcdat
);
    assign wm_mode      = 1'b0; // 2-wire I2C 模式 (P11 Pin 15, D14)
    assign wm_csb_gpio1 = 1'b1; // P11 Pin 13, F16
    localparam [6:0] WM8978_DEV_ADDR = 7'b0011010; // 7'h1A

    localparam [1:0] R4_WL_BITS = (AUDIO_WORD_LEN == 24) ? 2'b10 :
                                  (AUDIO_WORD_LEN == 20) ? 2'b01 :
                                  (AUDIO_WORD_LEN == 32) ? 2'b11 : 2'b00;

    localparam [16:0] SOFT_RST_GAP_CLKS = (PWR_WAIT_CLKS < 22'd10_000) ? 17'd500 : 17'd100_000;
    localparam [16:0] STEP_GAP_CLKS     = (PWR_WAIT_CLKS < 22'd10_000) ? 17'd200 : 17'd5_000;

    // 1. MCLK 时钟生成 (12.5MHz = 256 * 48.828kHz)
    reg [1:0] mclk_div_cnt = 2'd0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            mclk_div_cnt <= 2'd0;
        else
            mclk_div_cnt <= mclk_div_cnt + 2'd1;
    end
    assign wm_mclk = mclk_div_cnt[1];

    // 2. 外部电源就绪监测与 50ms 上电防抖延时
    reg [2:0]  pwr_ok_sync  = 3'b000;
    reg [21:0] pwr_wait_cnt = 22'd0;
    reg        pwr_ready    = 1'b0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            pwr_ok_sync  <= 3'b000;
            pwr_wait_cnt <= 22'd0;
            pwr_ready    <= 1'b0;
        end else begin
            pwr_ok_sync <= {pwr_ok_sync[1:0], ext_pwr_ok};
            if (!pwr_ok_sync[2]) begin
                pwr_wait_cnt <= 22'd0;
                pwr_ready    <= 1'b0;
            end else if (pwr_wait_cnt < PWR_WAIT_CLKS) begin
                pwr_wait_cnt <= pwr_wait_cnt + 22'd1;
                pwr_ready    <= 1'b0;
            end else begin
                pwr_ready    <= 1'b1;
            end
        end
    end

    // 3. 模拟音量映射 (85% -> 57 = 0dB，100% -> 63 = +6dB, ZC=0 立即生效)
    wire [7:0]  clamped_vol    = (master_vol_pct > 8'd100) ? 8'd100 : master_vol_pct;
    wire [16:0] scaled_vol_q10 = clamped_vol * 10'd490;
    wire [5:0]  calc_vol_6b    = 6'd16 + scaled_vol_q10[15:10];
    wire [8:0]  dynamic_vol_9b = (clamped_vol == 8'd0) ?
                                 9'b1_0100_0000 :
                                 {3'b100, calc_vol_6b[5:0]};

    // 4. WM8978 23 步初始化 ROM 表 (全输出接口激活：LOUT1/ROUT1 + LOUT2/ROUT2 + OUT3/OUT4)
    localparam [4:0] TOTAL_INIT_STEPS = 5'd22;
    reg [4:0]  cfg_idx = 5'd0;
    reg [15:0] rom_reg_data;
    always @(*) begin
        case (cfg_idx)
            5'd0:  rom_reg_data = {7'd0,  9'b0_0000_0000}; // Reg 0 : 软复位 (仅首次上电发送一次，重试严禁发送!)
            5'd1:  rom_reg_data = {7'd1,  9'b1_1110_1111}; // Reg 1 : BUFDCOPEN=1, OUT3/4MIXEN=1, PLLEN=1, BIASEN=1, BUFIOEN=1, VMIDSEL=11
            5'd2:  rom_reg_data = {7'd2,  9'b1_1000_0000}; // Reg 2 : ROUT1EN=1, LOUT1EN=1 (开启 CN1 耳机/音响驱动)
            5'd3:  rom_reg_data = {7'd3,  9'b1_1110_1111}; // Reg 3 : OUT3/4EN=1, LOUT2/ROUT2EN=1 (开启 CN2/CN3 驱动), LMIX/RMIX=1, DACENL/R=1
            5'd4:  rom_reg_data = {7'd36, 9'b0_0000_0111}; // Reg 36: PLL N = 7
            5'd5:  rom_reg_data = {7'd37, 9'b0_0011_0111}; // Reg 37: PLL K1
            5'd6:  rom_reg_data = {7'd38, 9'b0_1010_0010}; // Reg 38: PLL K2
            5'd7:  rom_reg_data = {7'd39, 9'b0_0001_0011}; // Reg 39: PLL K3
            5'd8:  rom_reg_data = USE_INT_PLL ?
                                  {7'd6,  9'b1_0100_1001} :
                                  {7'd6,  9'b0_0000_0000}; // Reg 6 : CLKSEL=0(MCLK=256fs), MS=0(Slave)
            5'd9:  rom_reg_data = {7'd4,  {2'b00, R4_WL_BITS, 5'b1_0000}}; // Reg 4 : 24-bit 标准 I2S
            5'd10: rom_reg_data = {7'd5,  9'b0_0000_0000}; // Reg 5 : 线性无压缩模式
            5'd11: rom_reg_data = {7'd7,  9'b0_0000_0001}; // Reg 7 : SR=48kHz(000), SLOWCLKEN=1
            5'd12: rom_reg_data = {7'd10, 9'b0_0000_1000}; // Reg 10: DACOSR128=1, SOFTMUTE=0 (解除数字静音)
            5'd13: rom_reg_data = {7'd11, 9'b1_1111_1111}; // Reg 11: 左 DAC 数字音量 0dB
            5'd14: rom_reg_data = {7'd12, 9'b1_1111_1111}; // Reg 12: 右 DAC 数字音量 0dB
            5'd15: rom_reg_data = {7'd43, 9'b0_0001_0000}; // Reg 43: INVROUT2=1
            5'd16: rom_reg_data = {7'd49, 9'b0_0000_0010}; // Reg 49: TSDEN=1, SPKBOOST=0 (3.3V 标准 AVDD/2 偏置)
            5'd17: rom_reg_data = {7'd50, 9'b0_0000_0001}; // Reg 50: DACL2LMIX=1
            5'd18: rom_reg_data = {7'd51, 9'b0_0000_0001}; // Reg 51: DACR2RMIX=1
            5'd19: rom_reg_data = {7'd52, dynamic_vol_9b}; // Reg 52: LOUT1 音量
            5'd20: rom_reg_data = {7'd53, dynamic_vol_9b}; // Reg 53: ROUT1 音量
            5'd21: rom_reg_data = {7'd54, dynamic_vol_9b}; // Reg 54: LOUT2 音量
            5'd22: rom_reg_data = {7'd55, dynamic_vol_9b}; // Reg 55: ROUT2 音量
            default: rom_reg_data = 16'h0000;
        endcase
    end

    wire [15:0] cmd_fifo_dout;
    wire        cmd_fifo_full;
    wire        cmd_fifo_empty;
    reg         i2c_start;
    wire        i2c_busy;
    wire        i2c_err_wire;
    reg         init_push_done = 1'b0;

    // 若发生 I2C 应答错误，每隔 1.5 秒从 cfg_idx = 5'd1 (Reg 1) 重试，严禁发 Reg 0 软复位!
    reg [26:0] err_retry_cnt = 27'd0;
    reg        err_retry_req = 1'b0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            err_retry_cnt <= 27'd0;
            err_retry_req <= 1'b0;
        end else if (init_done && i2c_ack_err && cmd_fifo_empty && !i2c_busy) begin
            if (err_retry_cnt >= 27'd75_000_000) begin
                err_retry_cnt <= 27'd0;
                err_retry_req <= 1'b1;
            end else begin
                err_retry_cnt <= err_retry_cnt + 27'd1;
                err_retry_req <= 1'b0;
            end
        end else begin
            err_retry_cnt <= 27'd0;
            err_retry_req <= 1'b0;
        end
    end

    wire cmd_fifo_push = pwr_ready && (!reinit_trig) && (!err_retry_req) && (!init_push_done) && (!cmd_fifo_full);

    lut_sync_fifo #(
        .DATA_WIDTH (16),
        .ADDR_WIDTH (4),
        .FWFT_MODE  (1)
    ) u_i2c_cmd_lut_fifo (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .wr_en      (cmd_fifo_push),
        .din        (rom_reg_data),
        .full       (cmd_fifo_full),
        .rd_en      (i2c_start),
        .dout       (cmd_fifo_dout),
        .empty      (cmd_fifo_empty),
        .data_count (i2c_fifo_cnt)
    );

    i2c_master #(
        .SYS_CLK_FREQ (SYS_CLK_FREQ),
        .I2C_FREQ     (I2C_FREQ)
    ) u_wm_i2c (
        .clk       (clk_50m),
        .rst_n     (rst_n),
        .i2c_start (i2c_start),
        .dev_addr  (WM8978_DEV_ADDR),
        .reg_data  (cmd_fifo_dout),
        .busy      (i2c_busy),
        .ack_error (i2c_err_wire),
        .i2c_scl   (wm_i2c_sclk),
        .i2c_sda   (wm_i2c_sdin)
    );

    localparam [2:0] S_WAIT_PWR  = 3'd0,
                     S_POP_SEND  = 3'd1,
                     S_WAIT_ACK  = 3'd2,
                     S_WAIT_DONE = 3'd3,
                     S_STEP_GAP  = 3'd4;
    reg [2:0]  state        = S_WAIT_PWR;
    reg [16:0] gap_cnt      = 17'd0;
    reg [7:0]  last_vol_pct = 8'd90;
    reg        is_soft_rst  = 1'b0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            cfg_idx        <= 5'd0;
            init_push_done <= 1'b0;
            last_vol_pct   <= 8'd90;
        end else if (!pwr_ready || reinit_trig) begin
            cfg_idx        <= 5'd0;
            init_push_done <= 1'b0;
        end else if (err_retry_req) begin
            cfg_idx        <= 5'd1; // 从第 1 步 (Reg 1) 开始重发，绝不发 Reg 0 软复位!
            init_push_done <= 1'b0;
        end else if (!init_push_done) begin
            if (!cmd_fifo_full) begin
                if (cfg_idx == TOTAL_INIT_STEPS) begin
                    init_push_done <= 1'b1;
                    last_vol_pct   <= clamped_vol;
                end else begin
                    cfg_idx <= cfg_idx + 5'd1;
                end
            end
        end else if (clamped_vol != last_vol_pct && !cmd_fifo_full) begin
            cfg_idx        <= 5'd19;
            init_push_done <= 1'b0;
        end
    end

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_WAIT_PWR;
            i2c_start   <= 1'b0;
            init_done   <= 1'b0;
            i2c_ack_err <= 1'b0;
            gap_cnt     <= 17'd0;
            is_soft_rst <= 1'b0;
        end else if (!pwr_ready || reinit_trig) begin
            state       <= S_WAIT_PWR;
            i2c_start   <= 1'b0;
            init_done   <= 1'b0;
            i2c_ack_err <= 1'b0;
        end else if (err_retry_req) begin
            state       <= S_POP_SEND;
            i2c_start   <= 1'b0;
            init_done   <= 1'b0;
            i2c_ack_err <= 1'b0;
        end else begin
            case (state)
                S_WAIT_PWR: begin
                    if (pwr_ready) begin
                        i2c_ack_err <= 1'b0;
                        state       <= S_POP_SEND;
                    end
                end
                S_POP_SEND: begin
                    if (!cmd_fifo_empty && !i2c_busy) begin
                        is_soft_rst <= (cmd_fifo_dout[15:9] == 7'd0);
                        i2c_start   <= 1'b1;
                        state       <= S_WAIT_ACK;
                    end else if (cmd_fifo_empty && init_push_done) begin
                        init_done <= 1'b1;
                    end
                end
                S_WAIT_ACK: begin
                    i2c_start <= 1'b0;
                    if (i2c_busy) begin
                        state <= S_WAIT_DONE;
                    end
                end
                S_WAIT_DONE: begin
                    if (!i2c_busy) begin
                        if (i2c_err_wire)
                            i2c_ack_err <= 1'b1;
                        gap_cnt <= 17'd0;
                        state   <= S_STEP_GAP;
                    end
                end
                S_STEP_GAP: begin
                    if (gap_cnt >= (is_soft_rst ? SOFT_RST_GAP_CLKS : STEP_GAP_CLKS)) begin
                        state <= S_POP_SEND;
                    end else begin
                        gap_cnt <= gap_cnt + 17'd1;
                    end
                end
                default: state <= S_WAIT_PWR;
            endcase
        end
    end

    // 5. 恢复原版正常发声的 24-bit I2S 串行音频发送状态机
    reg [2:0] bclk_sync      = 3'b000;
    reg [2:0] lrc_sync       = 3'b000;
    (* syn_preserve = 1 *) reg [1:0] adcdat_sync = 2'b00;
    reg [4:0] slave_bclk_cnt = 5'd0;
    reg [4:0] slave_lrc_cnt  = 5'd0;
    reg       slave_bclk_reg = 1'b0;
    reg       slave_lrc_reg  = 1'b0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            slave_bclk_cnt <= 5'd0;
            slave_bclk_reg <= 1'b0;
            slave_lrc_cnt  <= 5'd0;
            slave_lrc_reg  <= 1'b0;
        end else if (slave_bclk_cnt == 5'd7) begin
            slave_bclk_cnt <= 5'd0;
            slave_bclk_reg <= ~slave_bclk_reg;
            if (slave_bclk_reg == 1'b1) begin
                slave_lrc_cnt <= slave_lrc_cnt + 5'd1;
                if (slave_lrc_cnt == 5'd31)
                    slave_lrc_reg <= ~slave_lrc_reg;
            end
        end else begin
            slave_bclk_cnt <= slave_bclk_cnt + 5'd1;
        end
    end

    assign wm_bclk = (USE_INT_PLL == 1) ? 1'bz : slave_bclk_reg;
    assign wm_lrc  = (USE_INT_PLL == 1) ? 1'bz : slave_lrc_reg;
    wire bclk_in   = (USE_INT_PLL == 1) ? wm_bclk : slave_bclk_reg;
    wire lrc_in    = (USE_INT_PLL == 1) ? wm_lrc  : slave_lrc_reg;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            bclk_sync   <= 3'b000;
            lrc_sync    <= 3'b000;
            adcdat_sync <= 2'b00;
        end else begin
            bclk_sync   <= {bclk_sync[1:0],   bclk_in};
            lrc_sync    <= {lrc_sync[1:0],    lrc_in};
            adcdat_sync <= {adcdat_sync[0],   wm_adcdat};
        end
    end

    wire bclk_falling       = (bclk_sync[2:1] == 2'b10);
    assign sample_req_pulse = (lrc_sync[2:1] == 2'b10);

    reg bclk_falling_d1 = 1'b0, bclk_falling_d2 = 1'b0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) {bclk_falling_d2, bclk_falling_d1} <= 2'b00;
        else        {bclk_falling_d2, bclk_falling_d1} <= {bclk_falling_d1, bclk_falling};
    end

    reg                        lrc_at_last_bclk = 1'b0;
    reg [AUDIO_WORD_LEN-1:0]   shift_out_reg    = {AUDIO_WORD_LEN{1'b0}};
    reg [4:0]                  i2s_bit_cnt      = 5'd0;
    localparam [4:0]           INIT_BIT_CNT     = (AUDIO_WORD_LEN == 24) ? 5'd23 : 5'd15;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            lrc_at_last_bclk <= 1'b0;
            shift_out_reg    <= {AUDIO_WORD_LEN{1'b0}};
            i2s_bit_cnt      <= 5'd0;
            wm_dacdat        <= 1'b0;
        end else if (bclk_falling_d2) begin
            lrc_at_last_bclk <= lrc_sync[2];
            if (lrc_sync[2] != lrc_at_last_bclk) begin
                if (lrc_sync[2] == 1'b0) begin
                    wm_dacdat     <= pcm_left_in[AUDIO_WORD_LEN-1];
                    shift_out_reg <= {pcm_left_in[AUDIO_WORD_LEN-2:0], 1'b0};
                end else begin
                    wm_dacdat     <= pcm_right_in[AUDIO_WORD_LEN-1];
                    shift_out_reg <= {pcm_right_in[AUDIO_WORD_LEN-2:0], 1'b0};
                end
                i2s_bit_cnt <= INIT_BIT_CNT;
            end else if (i2s_bit_cnt > 5'd0) begin
                wm_dacdat     <= shift_out_reg[AUDIO_WORD_LEN-1];
                shift_out_reg <= {shift_out_reg[AUDIO_WORD_LEN-2:0], 1'b0};
                i2s_bit_cnt   <= i2s_bit_cnt - 5'd1;
            end else begin
                wm_dacdat <= 1'b0;
            end
        end
    end
endmodule

module wm8978_driver_fifo #(
    parameter        SYS_CLK_FREQ   = 50_000_000,
    parameter        I2C_FREQ       = 25_000,
    parameter        USE_INT_PLL    = 0,
    parameter        AUDIO_WORD_LEN = 24,
    parameter [21:0] PWR_WAIT_CLKS  = 22'd2_500_000
)(
    input  wire                        clk_50m,
    input  wire                        rst_n,
    input  wire                        ext_pwr_ok,
    input  wire                        reinit_trig,
    input  wire [7:0]                  master_vol_pct,
    input  wire [AUDIO_WORD_LEN-1:0]   pcm_left_in,
    input  wire [AUDIO_WORD_LEN-1:0]   pcm_right_in,
    output wire                        sample_req_pulse,
    output wire [4:0]                  i2c_fifo_cnt,
    output wire                        init_done,
    output wire                        i2c_ack_err,
    output wire                        wm_mode,
    output wire                        wm_csb_gpio1,
    output wire                        wm_i2c_sclk,
    inout  wire                        wm_i2c_sdin,
    output wire                        wm_mclk,
    inout  wire                        wm_bclk,
    inout  wire                        wm_lrc,
    output wire                        wm_dacdat,
    input  wire                        wm_adcdat
);
    wm8978_driver #(
        .SYS_CLK_FREQ   (SYS_CLK_FREQ),
        .I2C_FREQ       (I2C_FREQ),
        .USE_INT_PLL    (USE_INT_PLL),
        .AUDIO_WORD_LEN (AUDIO_WORD_LEN),
        .PWR_WAIT_CLKS  (PWR_WAIT_CLKS)
    ) u_drv_core (
        .clk_50m          (clk_50m),
        .rst_n            (rst_n),
        .ext_pwr_ok       (ext_pwr_ok),
        .reinit_trig      (reinit_trig),
        .master_vol_pct   (master_vol_pct),
        .pcm_left_in      (pcm_left_in),
        .pcm_right_in     (pcm_right_in),
        .sample_req_pulse (sample_req_pulse),
        .i2c_fifo_cnt     (i2c_fifo_cnt),
        .init_done        (init_done),
        .i2c_ack_err      (i2c_ack_err),
        .wm_mode          (wm_mode),
        .wm_csb_gpio1     (wm_csb_gpio1),
        .wm_i2c_sclk      (wm_i2c_sclk),
        .wm_i2c_sdin      (wm_i2c_sdin),
        .wm_mclk          (wm_mclk),
        .wm_bclk          (wm_bclk),
        .wm_lrc           (wm_lrc),
        .wm_dacdat        (wm_dacdat),
        .wm_adcdat        (wm_adcdat)
    );
endmodule