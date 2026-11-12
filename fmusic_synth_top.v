`timescale 1ns / 1ps
// ============================================================================
// 模块名称: fmusic_synth_top (24-bit Hi-Fi 无损 8声部独立 ADSR 真复音升级版)
// 功能描述: 集成 touch_ui_param_decoder、touch_fm_param_mapper、midi_optocoupler_rx_poly8、
//           poly8_adsr_vca_engine (24-bit 无损混音) 与 WM8978 24-bit I2S 驱动的合成器顶层模块
// ============================================================================
module fmusic_synth_top #(
    parameter        USE_TOUCH_UI_REGS  = 1,
    parameter        DIRECT_ALG_TO_MODE = 1,
    parameter        WM_USE_INT_PLL     = 0,
    parameter        AUDIO_WORD_LEN     = 24,        // 默认启用 24-bit Hi-Fi 音频位宽
    parameter [21:0] PWR_WAIT_CLKS      = 22'd2_500_000,
    parameter [3:0]  PARAM_ALG_MODE     = 4'd4,
    parameter [2:0]  PARAM_CAR_OP_ID    = 3'd1,
    parameter [1:0]  PARAM_CAR_WAVE_SEL = 2'd0,
    parameter [2:0]  PARAM_MOD_OP_ID    = 3'd2,
    parameter [1:0]  PARAM_MOD_WAVE_SEL = 2'd3,
    parameter [4:0]  PARAM_CAR_RATIO    = 5'd2,
    parameter [6:0]  PARAM_CAR_LEVEL    = 7'd85,
    parameter [6:0]  PARAM_MOD_DEPTH    = 7'd75,
    parameter [4:0]  PARAM_MOD_RATIO    = 5'd4,
    parameter [6:0]  PARAM_MOD_FEEDBACK = 7'd30,
    parameter [6:0]  PARAM_ENV_ATTACK   = 7'd15,
    parameter [6:0]  PARAM_ENV_DECAY    = 7'd40,
    parameter [6:0]  PARAM_ENV_SUSTAIN  = 7'd80,
    parameter [6:0]  PARAM_ENV_RELEASE  = 7'd25
)(
    input  wire         clk_50m,
    input  wire         rst_n,
    // 1. 电容屏 UI 桥接器 512-bit 调制寄存器总线
    input  wire [511:0] mod_regs_flat,
    // 2. 外部 MIDI 光耦串口输入 & 板载按键试音触发
    input  wire         midi_rx_in,
    input  wire         test_key_note,
    // 3. WM8978 音频扩展板物理引脚 (I2C + I2S)
    input  wire         ext_pwr_ok,
    output wire         wm_mode,
    output wire         wm_csb_gpio1,
    output wire         wm_i2c_sclk,
    inout  wire         wm_i2c_sdin,
    output wire         wm_mclk,
    inout  wire         wm_bclk,
    inout  wire         wm_lrc,
    output wire         wm_dacdat,
    input  wire         wm_adcdat,
    // 4. 合成音频输出与状态指示
    output wire signed [15:0] scope_pcm_out,
    output wire               wm_init_done,
    output wire               wm_i2c_err,
    // 5. ModelSim 全链路信号观测探针 (含 24-bit Hi-Fi 全精度混音与抽取探针)
    output wire [7:0]         dbg_uart_rx_byte,    // 探针1: 6N138 UART 原始接收字节
    output wire               dbg_uart_rx_vld,     // 探针1: UART 字节有效脉冲
    output wire               dbg_uart_frame_err,  // 探针1: 停止位帧错误标志
    output wire [4:0]         dbg_midi_fifo_cnt,   // 探针2: MIDI LUT FIFO 当前水位 (0~16)
    output wire [7:0]         dbg_voice_note_on,   // 探针3: 8声部复音激活掩码 (Bit 0~7)
    output wire [7:0]         dbg_voice_retrig,    // 探针3: 8声部独立起音重触发脉冲 (Bit 0~7)
    output wire [63:0]        dbg_voice_env_flat,  // 探针3: 8声部各自独立 ADSR 包络实时值 {env7..env0}
    output wire [6:0]         dbg_latest_note,     // 探针3: 最新解析音符编号
    output wire [6:0]         dbg_latest_vel,      // 探针3: 最新解析力度值
    output wire signed [15:0] dbg_fm_ch1_wave,     // 探针4: FM 核心通道1输出波形
    output wire signed [15:0] dbg_routed_wave,     // 探针4: 算法路由混音后等幅波形
    output wire [7:0]         dbg_adsr_env,        // 探针5: Voice 0 ADSR 包络当前值 (0~255)
    output wire signed [23:0] dbg_vca_out_24b,     // 探针5A: 8声部 24-bit Hi-Fi 无损混音音频 (-8388608 ~ +8388607)
    output wire signed [15:0] dbg_vca_out,         // 探针5B: 16-bit 兼容观测音频
    output wire signed [23:0] dbg_decim_pcm_24b,   // 探针6A: 1024点抗混叠抽取后 24-bit Hi-Fi 音频
    output wire signed [15:0] dbg_decim_pcm,       // 探针6B: 1024点抗混叠抽取后 16-bit 音频
    output wire               dbg_decim_vld,       // 探针6: 1024点抽取完成写FIFO脉冲
    output wire [4:0]         dbg_audio_fifo_cnt,  // 探针7: 音频 LUT FIFO 当前水位 (0~16)
    output wire               dbg_i2s_sample_req,  // 探针7: WM8978 48kHz 读FIFO采样脉冲
    output wire [4:0]         dbg_i2c_fifo_cnt     // 探针8: I2C 指令 LUT FIFO 当前水位 (0~16)
);
    wire [3:0]  w_fm_alg_sel;
    wire [2:0]  w_core_mod_mode;
    wire [2:0]  w_car_op_id;
    wire [1:0]  w_car_wave_sel;
    wire [2:0]  w_mod_op_id;
    wire [1:0]  w_mod_wave_sel;
    wire [4:0]  w_car_ratio;
    wire [4:0]  w_mod_ratio;
    wire [6:0]  w_car_level;
    wire [6:0]  w_mod_depth;
    wire [6:0]  w_mod_feedback;
    wire [6:0]  w_car_detune;
    wire [6:0]  w_car_init_phase;
    wire [6:0]  w_mod_init_phase;
    wire [7:0]  w_master_vol;
    wire [23:0] step_a;
    wire [23:0] step_d;
    wire [7:0]  sus_lv;
    wire [23:0] step_r;

    touch_ui_param_decoder #(
        .USE_TOUCH_UI_REGS  (USE_TOUCH_UI_REGS),
        .DIRECT_ALG_TO_MODE (DIRECT_ALG_TO_MODE),
        .PARAM_ALG_MODE     (PARAM_ALG_MODE),
        .PARAM_CAR_OP_ID    (PARAM_CAR_OP_ID),
        .PARAM_CAR_WAVE_SEL (PARAM_CAR_WAVE_SEL),
        .PARAM_MOD_OP_ID    (PARAM_MOD_OP_ID),
        .PARAM_MOD_WAVE_SEL (PARAM_MOD_WAVE_SEL),
        .PARAM_CAR_RATIO    (PARAM_CAR_RATIO),
        .PARAM_CAR_LEVEL    (PARAM_CAR_LEVEL),
        .PARAM_MOD_DEPTH    (PARAM_MOD_DEPTH),
        .PARAM_MOD_RATIO    (PARAM_MOD_RATIO),
        .PARAM_MOD_FEEDBACK (PARAM_MOD_FEEDBACK),
        .PARAM_MASTER_VOL   (8'd85),
        .PARAM_ENV_ATTACK   (PARAM_ENV_ATTACK),
        .PARAM_ENV_DECAY    (PARAM_ENV_DECAY),
        .PARAM_ENV_SUSTAIN  (PARAM_ENV_SUSTAIN),
        .PARAM_ENV_RELEASE  (PARAM_ENV_RELEASE)
    ) u_ui_dec (
        .mod_regs_flat    (mod_regs_flat),
        .w_ui_page_id     (),
        .w_fm_alg_sel     (w_fm_alg_sel),
        .w_core_mod_mode  (w_core_mod_mode),
        .w_car_op_id      (w_car_op_id),
        .w_car_wave_sel   (w_car_wave_sel),
        .w_mod_op_id      (w_mod_op_id),
        .w_mod_wave_sel   (w_mod_wave_sel),
        .w_car_ratio      (w_car_ratio),
        .w_mod_ratio      (w_mod_ratio),
        .w_car_level      (w_car_level),
        .w_mod_depth      (w_mod_depth),
        .w_mod_feedback   (w_mod_feedback),
        .w_car_detune     (w_car_detune),
        .w_car_init_phase (w_car_init_phase),
        .w_mod_init_phase (w_mod_init_phase),
        .w_ui_event_cnt   (),
        .w_master_vol     (w_master_vol),
        .w_env_atk        (),
        .w_env_dec        (),
        .w_env_sus        (),
        .w_env_rel        (),
        .w_adsr_step_a    (step_a),
        .w_adsr_step_d    (step_d),
        .w_adsr_sus_lv    (sus_lv),
        .w_adsr_step_r    (step_r)
    );

    wire [7:0]   voice_note_on;
    wire [7:0]   voice_retrig;
    wire [7:0]   voice_env_active;
    wire [255:0] voice_ftw_flat;
    wire [55:0]  voice_vel_flat;
    wire [31:0]  voice0_ftw, voice1_ftw;
    wire [6:0]   voice0_vel;

    midi_optocoupler_rx_poly8 u_midi_rx (
        .clk              (clk_50m),
        .rst_n            (rst_n),
        .midi_rx_in       (midi_rx_in),
        .voice_env_active (voice_env_active),
        .last_rx_byte     (dbg_uart_rx_byte),
        .rx_byte_vld      (dbg_uart_rx_vld),
        .rx_frame_err     (dbg_uart_frame_err),
        .fifo_data_cnt    (dbg_midi_fifo_cnt),
        .latest_note_num  (dbg_latest_note),
        .latest_velocity  (dbg_latest_vel),
        .voice_note_on    (voice_note_on),
        .voice_retrig     (voice_retrig),
        .voice_ftw_flat   (voice_ftw_flat),
        .voice_vel_flat   (voice_vel_flat),
        .voice0_ftw       (voice0_ftw),
        .voice1_ftw       (voice1_ftw),
        .voice2_ftw       (),
        .voice3_ftw       (),
        .voice4_ftw       (),
        .voice5_ftw       (),
        .voice6_ftw       (),
        .voice7_ftw       (),
        .voice0_vel       (voice0_vel),
        .voice1_vel       (),
        .voice2_vel       (),
        .voice3_vel       (),
        .voice4_vel       (),
        .voice5_vel       (),
        .voice6_vel       (),
        .voice7_vel       ()
    );
    assign dbg_voice_note_on = voice_note_on;
    assign dbg_voice_retrig  = voice_retrig;

    wire [5:0]  w_carrier_mask;
    wire [31:0] ch1_car_ftw, ch1_mod_ftw, ch2_car_ftw, ch2_mod_ftw;
    wire [9:0]  car_phase_offset, mod_phase_offset;
    wire [11:0] op_waves_flat;

    touch_fm_param_mapper u_param_mapper (
        .clk                  (clk_50m),
        .rst_n                (rst_n),
        .car_op_id_in         (w_car_op_id),
        .car_wave_sel_in      (w_car_wave_sel),
        .mod_op_id_in         (w_mod_op_id),
        .mod_wave_sel_in      (w_mod_wave_sel),
        .carrier_mask_in      (w_carrier_mask),
        .car_ratio_in         (w_car_ratio),
        .mod_ratio_in         (w_mod_ratio),
        .car_detune_in        (w_car_detune),
        .car_init_phase_in    (w_car_init_phase),
        .mod_init_phase_in    (w_mod_init_phase),
        .base_v0_ftw_in       (voice0_ftw),
        .base_v1_ftw_in       (voice1_ftw),
        .ch1_car_ftw_out      (ch1_car_ftw),
        .ch1_mod_ftw_out      (ch1_mod_ftw),
        .ch2_car_ftw_out      (ch2_car_ftw),
        .ch2_mod_ftw_out      (ch2_mod_ftw),
        .car_phase_offset_out (car_phase_offset),
        .mod_phase_offset_out (mod_phase_offset),
        .op_waves_flat_out    (op_waves_flat)
    );

    wire [9:0]         op1_pmod, op2_pmod, op3_pmod, op4_pmod, op5_pmod, op6_pmod;
    wire signed [15:0] ch1_car_raw, ch1_mod_raw, ch2_car_raw, ch2_mod_raw, op5_raw, op6_raw;
    wire signed [15:0] ch1_fm_out,  ch2_fm_out;

    fm_synth_core #(
        .USE_DYN_UI        (1),
        .USE_EXT_WAVE_MAP  (1),
        .DEFAULT_CAR_WAVE  (PARAM_CAR_WAVE_SEL),
        .DEFAULT_MOD_WAVE  (PARAM_MOD_WAVE_SEL),
        .DEFAULT_MOD_DEPTH (PARAM_MOD_DEPTH),
        .DEFAULT_FEEDBACK  (PARAM_MOD_FEEDBACK),
        .DEFAULT_CAR_LEVEL (PARAM_CAR_LEVEL)
    ) u_fm_core (
        .clk                 (clk_50m),
        .rst_n               (rst_n),
        .ch1_mod_freq_ctrl   (ch1_mod_ftw),
        .ch1_car_freq_ctrl   (ch1_car_ftw),
        .ch2_mod_freq_ctrl   (ch2_mod_ftw),
        .ch2_car_freq_ctrl   (ch2_car_ftw),
        .car_phase_offset_in (car_phase_offset),
        .mod_phase_offset_in (mod_phase_offset),
        .op_waves_flat_in    (op_waves_flat),
        .car_op_id           (w_car_op_id),
        .mod_op_id           (w_mod_op_id),
        .carrier_mask_in     (w_carrier_mask),
        .mod_mode_sel        (w_core_mod_mode),
        .car_wave_sel        (w_car_wave_sel),
        .mod_wave_sel        (w_mod_wave_sel),
        .mod_depth_pct       (w_mod_depth),
        .feedback_pct        (w_mod_feedback),
        .car_level_pct       (w_car_level),
        .op1_phase_mod_in    (op1_pmod),
        .op2_phase_mod_in    (op2_pmod),
        .op3_phase_mod_in    (op3_pmod),
        .op4_phase_mod_in    (op4_pmod),
        .op5_phase_mod_in    (op5_pmod),
        .op6_phase_mod_in    (op6_pmod),
        .ch1_car_raw_out     (ch1_car_raw),
        .ch1_mod_raw_out     (ch1_mod_raw),
        .ch2_car_raw_out     (ch2_car_raw),
        .ch2_mod_raw_out     (ch2_mod_raw),
        .op5_raw_out         (op5_raw),
        .op6_raw_out         (op6_raw),
        .ch1_fm_out          (ch1_fm_out),
        .ch2_fm_out          (ch2_fm_out)
    );
    assign dbg_fm_ch1_wave = ch1_fm_out;

    wire               sample_req_pulse;
    wire signed [15:0] primary_v0_wave;
    wire signed [15:0] primary_v1_wave;
    wire signed [23:0] vca_modulated_wave_24b;
    wire signed [15:0] vca_modulated_wave_16b;
    wire               vca_wave_vld;
    wire [7:0]         v0_env_val;

    operator_voltage_vca #(
        .USE_DYN_UI         (1),
        .DIRECT_ALG_TO_MODE (DIRECT_ALG_TO_MODE),
        .DEFAULT_CAR_LEVEL  (PARAM_CAR_LEVEL),
        .DEFAULT_MOD_LEVEL  (PARAM_MOD_DEPTH),
        .DEFAULT_FEEDBACK   (PARAM_MOD_FEEDBACK)
    ) u_op_vca (
        .clk                (clk_50m),
        .rst_n              (rst_n),
        .sample_tick        (sample_req_pulse),
        .ch1_car_raw_in     (ch1_car_raw),
        .ch1_mod_raw_in     (ch1_mod_raw),
        .ch2_car_raw_in     (ch2_car_raw),
        .ch2_mod_raw_in     (ch2_mod_raw),
        .op5_raw_in         (op5_raw),
        .op6_raw_in         (op6_raw),
        .mod_mode_sel       (w_core_mod_mode),
        .fm_alg_sel         (w_fm_alg_sel),
        .feedback_pct       (w_mod_feedback),
        .car_level_pct      (w_car_level),
        .midi_vel_in        (voice0_vel),
        .mod_level_pct      (w_mod_depth),
        .adsr_env_in        (v0_env_val),
        .carrier_mask_out   (w_carrier_mask),
        .op1_phase_mod_out  (op1_pmod),
        .op2_phase_mod_out  (op2_pmod),
        .op3_phase_mod_out  (op3_pmod),
        .op4_phase_mod_out  (op4_pmod),
        .op5_phase_mod_out  (op5_pmod),
        .op6_phase_mod_out  (op6_pmod),
        .dbg_car1_vol_wave  (),
        .dbg_mod1_vol_wave  (),
        .dbg_alg_mixed_wave (primary_v0_wave),
        .ch2_alg_wave_out   (primary_v1_wave),
        .vca_Audio_out      (),
        .vca_audio_vld      ()
    );
    assign dbg_routed_wave = primary_v0_wave;

    poly8_adsr_vca_engine u_poly8_engine (
        .clk                   (clk_50m),
        .rst_n                 (rst_n),
        .sample_tick           (sample_req_pulse),
        .voice_note_on         (voice_note_on),
        .voice_retrig          (voice_retrig),
        .test_key_note         (test_key_note),
        .voice_ftw_flat        (voice_ftw_flat),
        .voice_vel_flat        (voice_vel_flat),
        .mod_mode_sel          (w_core_mod_mode),
        .car_wave_sel          (op_waves_flat[1:0]),
        .mod_wave_sel          (op_waves_flat[3:2]),
        .car_ratio             (w_car_ratio),
        .mod_ratio             (w_mod_ratio),
        .car_detune            (w_car_detune),
        .car_phase_offset      (car_phase_offset),
        .mod_phase_offset      (mod_phase_offset),
        .car_level_pct         (w_car_level),
        .mod_level_pct         (w_mod_depth),
        .feedback_pct          (w_mod_feedback),
        .step_a                (step_a),
        .step_d                (step_d),
        .sustain_level         (sus_lv),
        .step_r                (step_r),
        .primary_v0_wave_in    (primary_v0_wave),
        .primary_v1_wave_in    (primary_v1_wave),
        .voice_env_active      (voice_env_active),
        .voice_env_flat        (dbg_voice_env_flat),
        .dbg_v0_env_out        (v0_env_val),
        .poly8_mixed_pcm24_out (vca_modulated_wave_24b),
        .poly8_mixed_pcm_out   (vca_modulated_wave_16b),
        .poly8_mixed_pcm_vld   (vca_wave_vld)
    );
    assign dbg_adsr_env    = v0_env_val;
    assign dbg_vca_out_24b = vca_modulated_wave_24b;
    assign dbg_vca_out     = vca_modulated_wave_16b;

    wire signed [23:0] anti_aliased_pcm_24b;
    wire               anti_aliased_vld;
    oversample_decimator_1024_fifo #(
        .DATA_WIDTH (24)
    ) u_anti_alias_flt (
        .clk_50m       (clk_50m),
        .rst_n         (rst_n),
        .pcm_in        (vca_modulated_wave_24b),
        .clean_pcm_out (anti_aliased_pcm_24b),
        .clean_pcm_vld (anti_aliased_vld)
    );
    assign dbg_decim_pcm_24b = anti_aliased_pcm_24b;
    assign dbg_decim_pcm     = anti_aliased_pcm_24b[23:8];
    assign dbg_decim_vld     = anti_aliased_vld;

    wire [23:0] final_i2s_pcm_24b;
    audio_staging_buffer_fifo #(
        .DATA_WIDTH  (24),
        .STAGE_WIDTH (28)
    ) u_staging (
        .clk              (clk_50m),
        .rst_n            (rst_n),
        .local_synth_data (anti_aliased_pcm_24b),
        .local_synth_en   (anti_aliased_vld),
        .cascade_data_in  (28'd0),
        .cascade_en_in    (1'b0),
        .group_addr_in    (2'b00),
        .i2s_sample_req   (sample_req_pulse),
        .fifo_data_cnt    (dbg_audio_fifo_cnt),
        .stage_data_out   (),
        .stage_en_out     (),
        .final_i2s_data   (final_i2s_pcm_24b),
        .final_i2s_en     ()
    );
    assign scope_pcm_out      = final_i2s_pcm_24b[23:8];
    assign dbg_i2s_sample_req = sample_req_pulse;

    wm8978_driver_fifo #(
        .SYS_CLK_FREQ   (50_000_000),
        .I2C_FREQ       (100_000),
        .USE_INT_PLL    (WM_USE_INT_PLL),
        .AUDIO_WORD_LEN (24),
        .PWR_WAIT_CLKS  (PWR_WAIT_CLKS)
    ) u_wm8978 (
        .clk_50m          (clk_50m),
        .rst_n            (rst_n),
        .ext_pwr_ok       (ext_pwr_ok),
        .reinit_trig      (1'b0),
        .master_vol_pct   (w_master_vol),
        .pcm_left_in      (final_i2s_pcm_24b),
        .pcm_right_in     (final_i2s_pcm_24b),
        .sample_req_pulse (sample_req_pulse),
        .i2c_fifo_cnt     (dbg_i2c_fifo_cnt),
        .init_done        (wm_init_done),
        .i2c_ack_err      (wm_i2c_err),
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
