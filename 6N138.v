`timescale 1ns / 1ps

// ============================================================================
// 模块名称: midi_optocoupler_rx_poly4
// 功能描述: 
//   1. 针对 6N138 光耦非对称延时 (t_PLH > t_PHL) 定制的 31250 bps UART 接收器
//      (内置 2级同步 + 1us数字滤波 + 62.5%眼图中心偏移补偿 + 3点多数表决)
//   2. 集成 MIDI 状态机解析 (支持 0x90/0x80 及 Running Status 连弹低延迟模式)
//   3. 集成 4 声部硬件复音分配器 (满足赛题“复音数 >= 4”要求，支持和弦并发与自动抢占)
// ============================================================================
module midi_optocoupler_rx_poly4 #(
    parameter SYS_CLK_FREQ    = 50_000_000, // 50MHz 系统主时钟
    parameter MIDI_BAUD       = 31_250,     // 标准 MIDI 波特率 31.25 kbps (1 bit = 32.0 us = 1600 clks)
    // 6N138 光耦上升沿比下降沿慢约 4us (200个50MHz时钟)，将采样中心从 50%(800) 后移至 62.5%(1000)
    parameter SKEW_COMP_CLKS  = 200         
)(
    input  wire        clk,                 // 50MHz 主时钟
    input  wire        rst_n,               // 复位信号 (低有效)

    // 1. 物理层输入：来自 P11 Pin 1 (C18) 的 6N138 Pin 6 (Vout) 信号
    input  wire        midi_rx_in,          // 空闲为高电平 1，起始位为低电平 0

    // 2. 调试与状态监控输出 (可接入数码管或 LED 观察收包状态)
    output wire [7:0]  last_rx_byte,        // 最新收到的原始字节
    output wire        rx_byte_vld,         // 收到单字节脉冲
    output reg         rx_frame_err,        // 停止位帧错误指示 (用于排查光耦上拉/泄放电阻是否匹配)
    output reg  [6:0]  latest_note_num,     // 最近一次按下的音符编号 (0~127)
    output reg  [6:0]  latest_velocity,     // 最近一次按下的力度值 (0~127)

    // 3. 四声部并发控制输出 (直接驱动 4 路 ADSR 包络与 4 路 FM/DDS 算子，满足复音数 >= 4!)
    output reg  [3:0]  voice_note_on,       // 4个声部各自的按键保持信号 (送给 4 个 adsr_envelope 的 note_on)
    output wire [31:0] voice0_ftw,          // 声部 0 的 32-bit 频率控制字 (送给算子 0)
    output wire [31:0] voice1_ftw,          // 声部 1 的 32-bit 频率控制字 (送给算子 1)
    output wire [31:0] voice2_ftw,          // 声部 2 的 32-bit 频率控制字 (送给算子 2)
    output wire [31:0] voice3_ftw,          // 声部 3 的 32-bit 频率控制字 (送给算子 3)
    output reg  [6:0]  voice0_vel,          // 声部 0 力度 (0~127)
    output reg  [6:0]  voice1_vel,          // 声部 1 力度
    output reg  [6:0]  voice2_vel,          // 声部 2 力度
    output reg  [6:0]  voice3_vel           // 声部 3 力度
);

    localparam integer CLKS_PER_BIT = SYS_CLK_FREQ / MIDI_BAUD;               // 1600
    localparam integer SAMPLE_POS_1 = (CLKS_PER_BIT / 2) + SKEW_COMP_CLKS - 32; // 第 968 拍 (19.36 us)
    localparam integer SAMPLE_POS_2 = (CLKS_PER_BIT / 2) + SKEW_COMP_CLKS;      // 第 1000 拍 (20.00 us 最佳中心)
    localparam integer SAMPLE_POS_3 = (CLKS_PER_BIT / 2) + SKEW_COMP_CLKS + 32; // 第 1032 拍 (20.64 us)

    // =========================================================================
    // 第一级：跨时钟域三拍同步 + 1us (50 clocks) 数字消抖滤波 (滤除 6N138 边沿毛刺)
    // =========================================================================
    reg [2:0] rx_sync = 3'b111;
    reg [5:0] flt_cnt = 6'd50;
    reg       rx_clean = 1'b1;
    reg       rx_clean_d1 = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync     <= 3'b111;
            flt_cnt     <= 6'd50;
            rx_clean    <= 1'b1;
            rx_clean_d1 <= 1'b1;
        end else begin
            rx_sync     <= {rx_sync[1:0], midi_rx_in};
            rx_clean_d1 <= rx_clean;

            if (rx_sync[2] == 1'b1) begin
                if (flt_cnt < 6'd50) flt_cnt <= flt_cnt + 6'd1;
                else                 rx_clean <= 1'b1;
            end else begin
                if (flt_cnt > 6'd0)  flt_cnt <= flt_cnt - 6'd1;
                else                 rx_clean <= 1'b0;
            end
        end
    end

    wire start_falling_edge = rx_clean_d1 & (~rx_clean);

    // =========================================================================
    // 第二级：带 6N138 延迟偏置补偿与 3 点多数表决的 UART 接收状态机
    // =========================================================================
    localparam [1:0] U_IDLE  = 2'd0,
                     U_START = 2'd1,
                     U_DATA  = 2'd2,
                     U_STOP  = 2'd3;

    reg [1:0]  u_state   = U_IDLE;
    reg [10:0] clk_cnt   = 11'd0;
    reg [2:0]  bit_idx   = 3'd0;
    reg [2:0]  vote_buf  = 3'b111;
    reg [7:0]  rx_shift  = 8'd0;
    reg [7:0]  rx_byte_r = 8'd0;
    reg        rx_vld_r  = 1'b0;

    // 3点采样多数表决逻辑 (只要3次采样中有2次及以上为1则判为1，抗噪能力极强)
    wire majority_bit = (vote_buf[0] & vote_buf[1]) |
                        (vote_buf[1] & vote_buf[2]) |
                        (vote_buf[0] & vote_buf[2]);

    assign last_rx_byte = rx_byte_r;
    assign rx_byte_vld  = rx_vld_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u_state      <= U_IDLE;
            clk_cnt      <= 11'd0;
            bit_idx      <= 3'd0;
            vote_buf     <= 3'b111;
            rx_shift     <= 8'd0;
            rx_byte_r    <= 8'd0;
            rx_vld_r     <= 1'b0;
            rx_frame_err <= 1'b0;
        end else begin
            rx_vld_r <= 1'b0;

            case (u_state)
                U_IDLE: begin
                    clk_cnt <= 11'd0;
                    bit_idx <= 3'd0;
                    if (start_falling_edge)
                        u_state <= U_START;
                end

                U_START: begin
                    // 在补偿后的中心位置采样验证起始位是否真的稳定为 0 (防止假起始位)
                    if (clk_cnt == SAMPLE_POS_1) vote_buf[0] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_2) vote_buf[1] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_3) vote_buf[2] <= rx_clean;

                    if (clk_cnt >= CLKS_PER_BIT - 1) begin
                        clk_cnt <= 11'd0;
                        if (majority_bit == 1'b0) begin
                            u_state <= U_DATA;
                            bit_idx <= 3'd0;
                        end else begin
                            u_state <= U_IDLE; // 假起始毛刺，丢弃
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 11'd1;
                    end
                end

                U_DATA: begin
                    if (clk_cnt == SAMPLE_POS_1) vote_buf[0] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_2) vote_buf[1] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_3) vote_buf[2] <= rx_clean;

                    if (clk_cnt >= CLKS_PER_BIT - 1) begin
                        clk_cnt            <= 11'd0;
                        rx_shift[bit_idx]  <= majority_bit; // LSB First 依次装入 Bit 0 ~ Bit 7
                        if (bit_idx == 3'd7)
                            u_state <= U_STOP;
                        else
                            bit_idx <= bit_idx + 3'd1;
                    end else begin
                        clk_cnt <= clk_cnt + 11'd1;
                    end
                end

                U_STOP: begin
                    if (clk_cnt == SAMPLE_POS_1) vote_buf[0] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_2) vote_buf[1] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_3) vote_buf[2] <= rx_clean;

                    // 提前半个周期 (在停止位采样完成后，即第 1100 拍) 立即输出字节并回到 IDLE!
                    // 这样即使发送端波特率偏快或光耦拉宽了前一位，也绝不会漏掉下一字节的起始位下降沿！
                    if (clk_cnt >= SAMPLE_POS_3 + 16) begin
                        rx_byte_r    <= rx_shift;
                        rx_vld_r     <= 1'b1;
                        rx_frame_err <= ~majority_bit; // 若停止位不是高电平，拉高帧错误标志
                        u_state      <= U_IDLE;
                        clk_cnt      <= 11'd0;
                    end else begin
                        clk_cnt <= clk_cnt + 11'd1;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // 第三级：调用你工程中的 midi_decoder 解析按键事件 (MDI_jiexi.v)
    // =========================================================================
    wire       note_event_vld;
    wire       is_note_on;
    wire [6:0] note_num;
    wire [6:0] velocity;

    midi_decoder u_midi_dec (
        .clk            (clk),
        .rst_n          (rst_n),
        .rx_byte        (rx_byte_r),
        .rx_vld         (rx_vld_r),
        .note_event_vld (note_event_vld),
        .is_note_on     (is_note_on),
        .note_num       (note_num),
        .velocity       (velocity)
    );

    // =========================================================================
    // 第四级：4 声部硬件复音分配器 (4-Voice Polyphonic Note Allocator)
    //   - 琴键按下 (is_note_on == 1)：
    //       1. 若该音符已在某个声部发声，直接重触发该声部；
    //       2. 否则优先分配给当前空闲 (voice_note_on[i] == 0) 的声部；
    //       3. 若 4 个声部全满，采用轮询指针 (rr_ptr) 抢占最早的声部。
    //   - 琴键抬起 (is_note_on == 0)：
    //       精准匹配正在播放该 note_num 的声部并将其 voice_note_on[i] 拉低进入 Release!
    // =========================================================================
    reg [6:0] voice_note [0:3];
    reg [1:0] rr_ptr = 2'd0; // 轮询抢占指针

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            voice_note_on   <= 4'b0000;
            voice_note[0]   <= 7'd69; // 默认 A4 (440Hz)
            voice_note[1]   <= 7'd69;
            voice_note[2]   <= 7'd69;
            voice_note[3]   <= 7'd69;
            voice0_vel      <= 7'd0;
            voice1_vel      <= 7'd0;
            voice2_vel      <= 7'd0;
            voice3_vel      <= 7'd0;
            rr_ptr          <= 2'd0;
            latest_note_num <= 7'd69;
            latest_velocity <= 7'd0;
        end else if (note_event_vld) begin
            latest_note_num <= note_num;
            latest_velocity <= velocity;

            if (is_note_on) begin
                // 1. 检查是否同音重按
                if (voice_note_on[0] && (voice_note[0] == note_num)) begin
                    voice0_vel <= velocity;
                end else if (voice_note_on[1] && (voice_note[1] == note_num)) begin
                    voice1_vel <= velocity;
                end else if (voice_note_on[2] && (voice_note[2] == note_num)) begin
                    voice2_vel <= velocity;
                end else if (voice_note_on[3] && (voice_note[3] == note_num)) begin
                    voice3_vel <= velocity;
                end
                // 2. 寻找空闲声部 (0 -> 1 -> 2 -> 3)
                else if (!voice_note_on[0]) begin
                    voice_note_on[0] <= 1'b1;
                    voice_note[0]    <= note_num;
                    voice0_vel       <= velocity;
                end else if (!voice_note_on[1]) begin
                    voice_note_on[1] <= 1'b1;
                    voice_note[1]    <= note_num;
                    voice1_vel       <= velocity;
                end else if (!voice_note_on[2]) begin
                    voice_note_on[2] <= 1'b1;
                    voice_note[2]    <= note_num;
                    voice2_vel       <= velocity;
                end else if (!voice_note_on[3]) begin
                    voice_note_on[3] <= 1'b1;
                    voice_note[3]    <= note_num;
                    voice3_vel       <= velocity;
                end
                // 3. 四声部全满，轮询抢占 (Voice Stealing)
                else begin
                    voice_note[rr_ptr] <= note_num;
                    case (rr_ptr)
                        2'd0: voice0_vel <= velocity;
                        2'd1: voice1_vel <= velocity;
                        2'd2: voice2_vel <= velocity;
                        2'd3: voice3_vel <= velocity;
                    endcase
                    rr_ptr <= rr_ptr + 2'd1;
                end
            end else begin
                // 琴键松开：精准释放对应音高的声部，不影响其余正在按住的和弦声部！
                if (voice_note_on[0] && (voice_note[0] == note_num)) voice_note_on[0] <= 1'b0;
                if (voice_note_on[1] && (voice_note[1] == note_num)) voice_note_on[1] <= 1'b0;
                if (voice_note_on[2] && (voice_note[2] == note_num)) voice_note_on[2] <= 1'b0;
                if (voice_note_on[3] && (voice_note[3] == note_num)) voice_note_on[3] <= 1'b0;
            end
        end
    end

    // =========================================================================
    // 第五级：例化 4 路 midi_to_ftw_lut 查表器，零延迟并行输出 4 路频率控制字
    // =========================================================================
    midi_to_ftw_lut u_lut0 (.midi_note(voice_note[0]), .ftw(voice0_ftw));
    midi_to_ftw_lut u_lut1 (.midi_note(voice_note[1]), .ftw(voice1_ftw));
    midi_to_ftw_lut u_lut2 (.midi_note(voice_note[2]), .ftw(voice2_ftw));
    midi_to_ftw_lut u_lut3 (.midi_note(voice_note[3]), .ftw(voice3_ftw));

endmodule