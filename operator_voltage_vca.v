`timescale 1ns / 1ps
// ============================================================================
// 模块名称: operator_voltage_vca (四要素算子 Y 轴电压与动态 VCA 控制中心)
// 架构特点: 采用 parameter + wire 架构，支持通过电容屏动态控制或通过 parameter 静态设定：
//           1. 要素一 (car_level_pct) : 发声算子设置的发声音量 (0 ~ 100%)
//           2. 要素二 (midi_vel_in)   : 琴键按下的力度值 (0 ~ 127)
//           3. 要素三 (mod_level_pct) : 调制算子的“发声音量” (0 ~ 100%，乘在该调制算子地址查表值上)
//           4. 要素四 (adsr_env_in)   : ADSR 包络实时幅度值 (0 ~ 255)
//           同时输出乘包络前的双通道等幅合成波 (dbg_alg_mixed_wave 与 ch2_alg_wave_out) 供 8 声部独立包络引擎使用!
// ============================================================================
module operator_voltage_vca #(
    parameter [2:0] MODE_FM_STD        = 3'd0,
    parameter [2:0] MODE_FM_DEEP       = 3'd1,
    parameter [2:0] MODE_AM_MOD        = 3'd2,
    parameter [2:0] MODE_RING_MOD      = 3'd3,
    parameter [2:0] MODE_ADD_MIX       = 3'd4,
    parameter [2:0] MODE_SUB_DIFF      = 3'd5,
    parameter       USE_DYN_UI         = 1,
    parameter       DIRECT_ALG_TO_MODE = 1,
    parameter [2:0] DEFAULT_MOD_MODE   = MODE_FM_STD,
    parameter [3:0] DEFAULT_ALG_SEL    = 4'd4,
    parameter [6:0] DEFAULT_CAR_LEVEL  = 7'd85,
    parameter [6:0] DEFAULT_MIDI_VEL   = 7'd100,
    parameter [6:0] DEFAULT_MOD_LEVEL  = 7'd75,
    parameter [7:0] DEFAULT_ADSR_ENV   = 8'd200,
    parameter [6:0] DEFAULT_FEEDBACK   = 7'd30
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_tick,
    // 接口 A: 算子当前地址查表原始波形输入
    input  wire signed [15:0] ch1_car_raw_in,
    input  wire signed [15:0] ch1_mod_raw_in,
    input  wire signed [15:0] ch2_car_raw_in,
    input  wire signed [15:0] ch2_mod_raw_in,
    input  wire signed [15:0] op5_raw_in,
    input  wire signed [15:0] op6_raw_in,
    // 接口 B: 来自电容屏的调制方式、算法拓扑与自反馈控制线
    input  wire [2:0]  mod_mode_sel,
    input  wire [3:0]  fm_alg_sel,
    input  wire [6:0]  feedback_pct,
    // 接口 C: 决定发声算子最终 Y 轴电压的 4 大核心要素输入
    input  wire [6:0]  car_level_pct,
    input  wire [6:0]  midi_vel_in,
    input  wire [6:0]  mod_level_pct,
    input  wire [7:0]  adsr_env_in,
    // 接口 D: 回传给 fm_synth_core 的 6 算子查找表掩码与相位调制偏移
    output wire [5:0]  carrier_mask_out,
    output wire [9:0]  op1_phase_mod_out,
    output wire [9:0]  op2_phase_mod_out,
    output wire [9:0]  op3_phase_mod_out,
    output wire [9:0]  op4_phase_mod_out,
    output wire [9:0]  op5_phase_mod_out,
    output wire [9:0]  op6_phase_mod_out,
    // 接口 E: 过程电压观测探针与最终 VCA 音频输出
    output reg  signed [15:0] dbg_car1_vol_wave,
    output reg  signed [15:0] dbg_mod1_vol_wave,
    output wire signed [15:0] dbg_alg_mixed_wave, // Voice 0 乘包络前的算子合成等幅波
    output wire signed [15:0] ch2_alg_wave_out,   // Voice 1 乘包络前的副声部算子合成等幅波
    output reg  signed [15:0] vca_Audio_out,
    output reg                vca_audio_vld
);
    wire [2:0] w_eff_mod_mode  = USE_DYN_UI ? mod_mode_sel : DEFAULT_MOD_MODE;
    wire [3:0] w_eff_alg_sel   = USE_DYN_UI ? ((fm_alg_sel == 4'd0) ? DEFAULT_ALG_SEL : fm_alg_sel) : DEFAULT_ALG_SEL;
    wire [6:0] w_eff_car_level = USE_DYN_UI ? ((car_level_pct > 7'd100) ? 7'd100 : car_level_pct)  : DEFAULT_CAR_LEVEL;
    wire [6:0] w_eff_mod_level = USE_DYN_UI ? ((mod_level_pct > 7'd100) ? 7'd100 : mod_level_pct)  : DEFAULT_MOD_LEVEL;
    wire [6:0] w_eff_velocity  = USE_DYN_UI ? ((midi_vel_in   == 7'd0)  ? 7'd100 : midi_vel_in)    : DEFAULT_MIDI_VEL;
    wire [7:0] w_eff_adsr_env  = USE_DYN_UI ? adsr_env_in : DEFAULT_ADSR_ENV;
    wire [6:0] w_eff_feedback  = USE_DYN_UI ? ((feedback_pct  > 7'd100) ? 7'd100 : feedback_pct)   : DEFAULT_FEEDBACK;

    wire [13:0]        w_car_gain_u14 = (w_eff_car_level == 7'd100) ? 14'd8192 : (w_eff_car_level * 7'd82);
    wire [13:0]        w_mod_gain_u14 = (w_eff_mod_level == 7'd100) ? 14'd8192 : (w_eff_mod_level * 7'd82);
    reg  signed [14:0] w_car_gain_q13, w_mod_gain_q13;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_car_gain_q13 <= 15'sd6970;
            w_mod_gain_q13 <= 15'sd6150;
        end else begin
            w_car_gain_q13 <= $signed({1'b0, w_car_gain_u14});
            w_mod_gain_q13 <= $signed({1'b0, w_mod_gain_u14});
        end
    end

    wire signed [30:0] c1_prod = ch1_car_raw_in * w_car_gain_q13 + 31'sd4096;
    wire signed [30:0] m1_prod = ch1_mod_raw_in * w_mod_gain_q13 + 31'sd4096;
    wire signed [30:0] c2_prod = ch2_car_raw_in * w_car_gain_q13 + 31'sd4096;
    wire signed [30:0] m2_prod = ch2_mod_raw_in * w_mod_gain_q13 + 31'sd4096;

    wire signed [14:0] op1_g = carrier_mask_out[0] ? w_car_gain_q13 : w_mod_gain_q13;
    wire signed [14:0] op2_g = carrier_mask_out[1] ? w_car_gain_q13 : w_mod_gain_q13;
    wire signed [14:0] op3_g = carrier_mask_out[2] ? w_car_gain_q13 : w_mod_gain_q13;
    wire signed [14:0] op4_g = carrier_mask_out[3] ? w_car_gain_q13 : w_mod_gain_q13;
    wire signed [14:0] op5_g = carrier_mask_out[4] ? w_car_gain_q13 : w_mod_gain_q13;
    wire signed [14:0] op6_g = carrier_mask_out[5] ? w_car_gain_q13 : w_mod_gain_q13;

    wire signed [30:0] op1_p = ch1_car_raw_in * op1_g + 31'sd4096;
    wire signed [30:0] op2_p = ch1_mod_raw_in * op2_g + 31'sd4096;
    wire signed [30:0] op3_p = ch2_car_raw_in * op3_g + 31'sd4096;
    wire signed [30:0] op4_p = ch2_mod_raw_in * op4_g + 31'sd4096;
    wire signed [30:0] op5_p = op5_raw_in     * op5_g + 31'sd4096;
    wire signed [30:0] op6_p = op6_raw_in     * op6_g + 31'sd4096;

    reg signed [15:0] u_car2, u_mod2;
    reg signed [15:0] u_op1, u_op2, u_op3, u_op4, u_op5, u_op6;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_car1_vol_wave <= 16'sd0;
            dbg_mod1_vol_wave <= 16'sd0;
            u_car2            <= 16'sd0;
            u_mod2            <= 16'sd0;
            u_op1             <= 16'sd0;
            u_op2             <= 16'sd0;
            u_op3             <= 16'sd0;
            u_op4             <= 16'sd0;
            u_op5             <= 16'sd0;
            u_op6             <= 16'sd0;
        end else begin
            dbg_car1_vol_wave <= c1_prod[28:13];
            dbg_mod1_vol_wave <= m1_prod[28:13];
            u_car2            <= c2_prod[28:13];
            u_mod2            <= m2_prod[28:13];
            u_op1             <= op1_p[28:13];
            u_op2             <= op2_p[28:13];
            u_op3             <= op3_p[28:13];
            u_op4             <= op4_p[28:13];
            u_op5             <= op5_p[28:13];
            u_op6             <= op6_p[28:13];
        end
    end

    wire signed [31:0] ch1_ring_prod = dbg_car1_vol_wave * dbg_mod1_vol_wave + 32'sd16384;
    wire signed [15:0] ch1_ring_s16  = ch1_ring_prod[30:15];
    wire signed [15:0] ch1_am_s16    = (dbg_car1_vol_wave >>> 1) + (ch1_ring_s16 >>> 1);
    wire signed [15:0] ch1_add_s16   = (dbg_car1_vol_wave >>> 1) + (dbg_mod1_vol_wave >>> 1);
    wire signed [15:0] ch1_sub_s16   = (dbg_car1_vol_wave >>> 1) - (dbg_mod1_vol_wave >>> 1);

    wire signed [31:0] ch2_ring_prod = u_car2 * u_mod2 + 32'sd16384;
    wire signed [15:0] ch2_ring_s16  = ch2_ring_prod[30:15];
    wire signed [15:0] ch2_am_s16    = (u_car2 >>> 1) + (ch2_ring_s16 >>> 1);
    wire signed [15:0] ch2_add_s16   = (u_car2 >>> 1) + (u_mod2 >>> 1);
    wire signed [15:0] ch2_sub_s16   = (u_car2 >>> 1) - (u_mod2 >>> 1);

    reg signed [15:0] ch1_op_out;
    reg signed [15:0] ch2_op_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch1_op_out <= 16'sd0;
            ch2_op_out <= 16'sd0;
        end else begin
            case (w_eff_mod_mode)
                MODE_AM_MOD: begin
                    ch1_op_out <= ch1_am_s16;
                    ch2_op_out <= ch2_am_s16;
                end
                MODE_RING_MOD: begin
                    ch1_op_out <= ch1_ring_s16;
                    ch2_op_out <= ch2_ring_s16;
                end
                MODE_ADD_MIX: begin
                    ch1_op_out <= ch1_add_s16;
                    ch2_op_out <= ch2_add_s16;
                end
                MODE_SUB_DIFF: begin
                    ch1_op_out <= ch1_sub_s16;
                    ch2_op_out <= ch2_sub_s16;
                end
                default: begin
                    ch1_op_out <= dbg_car1_vol_wave;
                    ch2_op_out <= u_car2;
                end
            endcase
        end
    end

    wire signed [15:0] lut_routed_wave;
    fm_alg_lut_router u_alg_lut_router (
        .clk               (clk),
        .rst_n             (rst_n),
        .alg_sel_in        (w_eff_alg_sel),
        .mod_depth_in      (w_eff_mod_mode == MODE_FM_DEEP ? 7'd100 : 7'd75),
        .feedback_in       (w_eff_feedback),
        .op1_raw_wave      (u_op1),
        .op2_raw_wave      (u_op2),
        .op3_raw_wave      (u_op3),
        .op4_raw_wave      (u_op4),
        .op5_raw_wave      (u_op5),
        .op6_raw_wave      (u_op6),
        .w_carrier_mask    (carrier_mask_out),
        .w_fb_type         (),
        .w_car_gain_weight (),
        .op1_phase_mod_out (op1_phase_mod_out),
        .op2_phase_mod_out (op2_phase_mod_out),
        .op3_phase_mod_out (op3_phase_mod_out),
        .op4_phase_mod_out (op4_phase_mod_out),
        .op5_phase_mod_out (op5_phase_mod_out),
        .op6_phase_mod_out (op6_phase_mod_out),
        .routed_audio_out  (lut_routed_wave)
    );

    wire is_fm_mode = (w_eff_mod_mode == MODE_FM_STD) || (w_eff_mod_mode == MODE_FM_DEEP);
    reg signed [16:0] alg_mixed;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alg_mixed <= 17'sd0;
        end else if (DIRECT_ALG_TO_MODE || !is_fm_mode) begin
            case (w_eff_alg_sel)
                4'd7:    alg_mixed <= ((ch1_op_out + ch2_op_out + 17'sd1) >>> 1);
                4'd8:    alg_mixed <= ((ch1_op_out + lut_routed_wave + 17'sd1) >>> 1);
                default: alg_mixed <= ch1_op_out;
            endcase
        end else begin
            alg_mixed <= ((ch1_op_out + lut_routed_wave + 17'sd1) >>> 1);
        end
    end

    wire signed [15:0] alg_wave_16b = (alg_mixed > 17'sd32767)  ? 16'sd32767 :
                                      (alg_mixed < -17'sd32768) ? -16'sd32768 : alg_mixed[15:0];
    assign dbg_alg_mixed_wave = alg_wave_16b;
    assign ch2_alg_wave_out   = ch2_op_out;

    reg [14:0] target_gain_15b;
    reg [14:0] smooth_gain_15b;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_gain_15b <= 15'd0;
            smooth_gain_15b <= 15'd0;
        end else begin
            target_gain_15b <= w_eff_adsr_env * w_eff_velocity;
            if (target_gain_15b > smooth_gain_15b)
                smooth_gain_15b <= smooth_gain_15b + ((target_gain_15b - smooth_gain_15b) >> 3) + 15'd1;
            else if (target_gain_15b < smooth_gain_15b)
                smooth_gain_15b <= smooth_gain_15b - ((smooth_gain_15b - target_gain_15b) >> 3) - 15'd1;
        end
    end

    wire signed [15:0] gain_signed = {1'b0, smooth_gain_15b};
    reg  signed [31:0] mult_prod;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mult_prod     <= 32'sd0;
            vca_Audio_out <= 16'sd0;
            vca_audio_vld <= 1'b0;
        end else begin
            mult_prod     <= alg_wave_16b * gain_signed + 32'sd16384;
            vca_Audio_out <= mult_prod[30:15];
            vca_audio_vld <= 1'b1;
        end
    end
endmodule
