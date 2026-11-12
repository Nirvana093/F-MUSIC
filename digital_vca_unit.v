`timescale 1ns / 1ps
// ============================================================================
// 模块名称: digital_vca_unit (24-bit Hi-Fi 无损精度升级版独立数字压控放大器模块)
// 功能描述: 
//   接收单个声部的 16-bit FM 合成波形、8-bit ADSR 包络以及 7-bit MIDI 击键力度，
//   完成一阶 IIR 包络去拉链平滑 (Anti-Zipper Filter) 与 31-bit 全精度有符号乘法：
//   - 同时提供 22-bit 高精度无损输出 (vca_audio_22b_out，保留全部微弱包络尾音与泛音细节)
//     以及向后兼容的 16-bit 输出 (vca_Audio_out)。
// ============================================================================
module digital_vca_unit (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_tick,       // 48kHz 采样率同步脉冲
    // 1. 音频信号输入 (来自 fm_synth_core 或算法路由矩阵)
    input  wire signed [15:0] wave_in,           // 16-bit 有符号等幅合成波
    // 2. 幅度控制输入 (来自独立 adsr_envelope 与 midi_optocoupler_rx_poly8)
    input  wire [7:0]         adsr_env_in,       // 8-bit ADSR 包络值 (0 ~ 255)
    input  wire [6:0]         midi_vel_in,       // 7-bit MIDI 按键力度 (0 ~ 127，若为0默认按100试音)
    // 3. VCA 调制后音频输出
    output reg  signed [15:0] vca_Audio_out,     // 兼容 16-bit 输出
    output reg  signed [21:0] vca_audio_22b_out, // 22-bit 全精度无损音频输出 (供 8 声部 24-bit 混音器直接累加!)
    output reg                vca_audio_vld
);
    wire [6:0] eff_velocity = (midi_vel_in == 7'd0) ? 7'd100 : midi_vel_in;

    // ------------------------------------------------------------------------
    // 第一级：计算目标总增益 = ADSR包络(8-bit) × 击键力度(7-bit) -> 15-bit (0 ~ 32385)
    //         并通过一阶低通去拉链平滑滤波器消除包络跳变台阶噪声 (Zipper Noise)
    // ------------------------------------------------------------------------
    reg [14:0] target_gain_15b;
    reg [14:0] smooth_gain_15b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_gain_15b <= 15'd0;
            smooth_gain_15b <= 15'd0;
        end else begin
            target_gain_15b <= adsr_env_in * eff_velocity;
            if (target_gain_15b > smooth_gain_15b)
                smooth_gain_15b <= smooth_gain_15b + ((target_gain_15b - smooth_gain_15b) >> 4) + 15'd1;
            else if (target_gain_15b < smooth_gain_15b)
                smooth_gain_15b <= smooth_gain_15b - ((smooth_gain_15b - target_gain_15b) >> 4) - 15'd1;
        end
    end

    // ------------------------------------------------------------------------
    // 第二级：核心 VCA 有符号乘法 (16-bit 波形 × 15-bit 平滑总增益 = 31-bit 全精度乘积)
    //   - 22-bit 输出仅右移 9 位 (加 256 四舍五入)，比 16-bit 整整多保留 6-bit (64倍) 微弱泛音与余音细节!
    //   - 8 个 22-bit 声部直接相加正好构成 25-bit -> 24-bit Hi-Fi 主输出，单音零右移损失!
    // ------------------------------------------------------------------------
    wire signed [15:0] gain_signed   = {1'b0, smooth_gain_15b};
    wire signed [31:0] raw_mult_prod = wave_in * gain_signed;
    wire signed [31:0] prod_rnd_22b  = raw_mult_prod + 32'sd256;   // + 2^8 四舍五入提取 [30:9]
    wire signed [31:0] prod_rnd_16b  = raw_mult_prod + 32'sd16384; // + 2^14 四舍五入提取 [30:15]

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vca_Audio_out     <= 16'sd0;
            vca_audio_22b_out <= 22'sd0;
            vca_audio_vld     <= 1'b0;
        end else begin
            vca_Audio_out     <= prod_rnd_16b[30:15];
            vca_audio_22b_out <= prod_rnd_22b[30:9];
            vca_audio_vld     <= 1'b1;
        end
    end
endmodule
