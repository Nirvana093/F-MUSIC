`timescale 1ns / 1ps

module lcd_rgb_timing (
    input  wire               lcd_clk,      // 33.3MHz 像素时钟
    input  wire               sys_rst_n,    // 复位信号，低有效
    input  wire [15:0]        pixel_data,   // 来自 DDR3 桥接器的 RGB565 像素数据
    input  wire [3:0]         ui_page_id,   // 来自 mod_regs_flat[19:16] 的当前界面编号 (5=实时波形页)
    input  wire signed [15:0] scope_pcm_in, // 来自 u_fmusic_synth 的 16-bit 实时合成音频
    output wire               lcd_de,       // 数据有效使能信号
    output wire               lcd_hsync,    // 行同步信号
    output wire               lcd_vsync,    // 场同步信号
    output wire [4:0]         lcd_r,        // 红色分量 (5-bit)
    output wire [5:0]         lcd_g,        // 绿色分量 (6-bit)
    output wire [4:0]         lcd_b,        // 蓝色分量 (5-bit)
    output wire               lcd_bl        // 背光控制信号
);
    localparam [10:0] H_SYNC  = 11'd128;
    localparam [10:0] H_BACK  = 11'd88;
    localparam [10:0] H_DISP  = 11'd800;
    localparam [10:0] H_FRONT = 11'd40;
    localparam [10:0] H_TOTAL = 11'd1056;
    localparam [10:0] V_SYNC  = 11'd2;
    localparam [10:0] V_BACK  = 11'd33;
    localparam [10:0] V_DISP  = 11'd480;
    localparam [10:0] V_FRONT = 11'd10;
    localparam [10:0] V_TOTAL = 11'd525;

    reg [10:0] h_cnt = 11'd0;
    reg [10:0] v_cnt = 11'd0;

    always @(posedge lcd_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            h_cnt <= 11'd0;
        else if (h_cnt == H_TOTAL - 1'b1)
            h_cnt <= 11'd0;
        else
            h_cnt <= h_cnt + 11'd1;
    end

    always @(posedge lcd_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            v_cnt <= 11'd0;
        else if (h_cnt == H_TOTAL - 1'b1) begin
            if (v_cnt == V_TOTAL - 1'b1)
                v_cnt <= 11'd0;
            else
                v_cnt <= v_cnt + 11'd1;
        end
    end

    assign lcd_hsync = (h_cnt < H_SYNC) ? 1'b0 : 1'b1;
    assign lcd_vsync = (v_cnt < V_SYNC) ? 1'b0 : 1'b1;
    assign lcd_de = ((h_cnt >= H_SYNC + H_BACK) && (h_cnt < H_SYNC + H_BACK + H_DISP) &&
                     (v_cnt >= V_SYNC + V_BACK) && (v_cnt < V_SYNC + V_BACK + V_DISP)) ? 1'b1 : 1'b0;
    assign lcd_bl = 1'b1;

    // =========================================================================
    // 硬件过零触发实时音频示波器引擎 (1024 分频 = 32.5kHz 采样率，512点完整呈现 15.7ms 约 3~7 个完整正弦周期!)
    // 采用采集缓存 (cap_mem) 与显示缓存 (disp_mem) 帧消隐同步拷贝，100% 杜绝边写边读撕裂!
    // =========================================================================
    reg signed [15:0] pcm_sync1  = 16'sd0, pcm_sync2  = 16'sd0, pcm_prev = 16'sd0;
    reg [3:0]         page_sync1 = 4'd0,   page_sync2 = 4'd0;
    always @(posedge lcd_clk) begin
        pcm_sync1  <= scope_pcm_in;
        pcm_sync2  <= pcm_sync1;
        page_sync1 <= ui_page_id;
        page_sync2 <= page_sync1;
    end

    reg [9:0] div_cnt     = 10'd0;
    wire      sample_tick = (div_cnt == 10'd1023);
    always @(posedge lcd_clk) begin
        div_cnt <= div_cnt + 10'd1;
        if (sample_tick)
            pcm_prev <= pcm_sync2;
    end

    wire signed [8:0] pcm_hi9  = $signed(pcm_sync2[15:7]);
    wire signed [9:0] y_mapped = 10'sd128 - $signed({pcm_hi9[8], pcm_hi9});
    wire [7:0]        sample_y = (y_mapped < 10'sd4)   ? 8'd4   :
                                 (y_mapped > 10'sd251) ? 8'd251 : y_mapped[7:0];

    reg [7:0] cap_mem  [0:511];
    reg [7:0] disp_mem [0:511];
    reg [8:0] wr_addr      = 9'd0;
    reg [8:0] copy_addr    = 9'd0;
    reg       capturing    = 1'b0;
    reg       cap_ready    = 1'b0;
    reg       copying      = 1'b0;
    reg [9:0] trig_timeout = 10'd0;

    wire zc_rising = (pcm_prev < 16'sd0) && (pcm_sync2 >= 16'sd0);

    always @(posedge lcd_clk) begin
        // 1. 后台连续过零捕获 512 点到 cap_mem
        if (!capturing && !cap_ready) begin
            if (v_cnt == 11'd0 && h_cnt == 11'd0) begin
                if (trig_timeout < 10'd6)
                    trig_timeout <= trig_timeout + 10'd1;
            end
            if (sample_tick && (zc_rising || (trig_timeout >= 10'd3))) begin
                capturing    <= 1'b1;
                wr_addr      <= 9'd0;
                trig_timeout <= 10'd0;
            end
        end else if (capturing && sample_tick) begin
            cap_mem[wr_addr] <= sample_y;
            if (wr_addr == 9'd511) begin
                capturing <= 1'b0;
                cap_ready <= 1'b1;
            end else begin
                wr_addr <= wr_addr + 9'd1;
            end
        end

        // 2. 在垂直消隐期 (v_cnt == 1) 用 512 个 33.3MHz 周期 (仅半条扫描线) 瞬间同步到 disp_mem!
        if (cap_ready && (v_cnt == 11'd1) && (h_cnt == 11'd0)) begin
            copying   <= 1'b1;
            copy_addr <= 9'd0;
        end else if (copying) begin
            disp_mem[copy_addr] <= cap_mem[copy_addr];
            if (copy_addr == 9'd511) begin
                copying   <= 1'b0;
                cap_ready <= 1'b0;
            end else begin
                copy_addr <= copy_addr + 9'd1;
            end
        end
    end

    wire [10:0] scr_x = (h_cnt >= H_SYNC + H_BACK) ? (h_cnt - (H_SYNC + H_BACK)) : 11'd0;
    wire [10:0] scr_y = (v_cnt >= V_SYNC + V_BACK) ? (v_cnt - (V_SYNC + V_BACK)) : 11'd0;

    wire in_scope_x = (scr_x >= 11'd192) && (scr_x < 11'd704);
    wire in_scope_y = (scr_y >= 11'd150) && (scr_y < 11'd406);

    wire [8:0] rd_idx = (scr_x >= 11'd191 && scr_x < 11'd703) ? (scr_x - 11'd191) : 9'd0;
    reg [7:0] y_cur     = 8'd128;
    reg [7:0] y_prev_d1 = 8'd128;

    always @(posedge lcd_clk) begin
        y_cur     <= disp_mem[rd_idx];
        y_prev_d1 <= y_cur;
    end

    wire [7:0] y_min = (y_cur < y_prev_d1) ? y_cur : y_prev_d1;
    wire [7:0] y_max = (y_cur > y_prev_d1) ? y_cur : y_prev_d1;
    wire [7:0] rel_y = scr_y - 11'd150;

    wire trace_hit = (page_sync2 == 4'd5) && lcd_de && in_scope_x && in_scope_y &&
                     (rel_y + 8'd1 >= y_min) && (rel_y <= y_max + 8'd1);

    wire [15:0] trace_color = (rel_y < 8'd48 || rel_y > 8'd208) ? 16'hFFE0 : 16'h07E0;
    wire [15:0] final_pixel = trace_hit ? trace_color : pixel_data;

    assign {lcd_r, lcd_g, lcd_b} = lcd_de ? final_pixel : 16'h0000;

endmodule