`timescale 1ns / 1ps

// ============================================================================
// 模块名称: synth_chain_diag_fsm (全链路 8 级自检与故障时刻锁存诊断状态机)
// 输出 24-bit 诊断字 diag_chain_word_24b = {stage_id[3:0], err_flags[3:0], detail_byte[7:0], latest_note[7:0]}
// 直接显示在数码管 CC 通道上，实时锁定哪一级、哪一刻出现异常!
// ============================================================================
module synth_chain_diag_fsm (
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire        wm_init_done,
    input  wire        wm_i2c_err,
    input  wire [4:0]  wm_i2c_fifo_cnt,
    input  wire        sample_req_pulse,
    input  wire        midi_rx_in,
    input  wire        midi_frame_err,
    input  wire [6:0]  latest_note_num,
    input  wire [31:0] voice_note_on,
    input  wire        test_key_note,
    input  wire [31:0] voice_env_active,
    input  wire signed [24:0] core0_sum25,
    input  wire        anti_aliased_vld,
    input  wire [23:0] final_i2s_pcm_24b,
    input  wire        wm_dacdat,
    output wire [23:0] diag_chain_word_24b
);
    // 1. 监测 I2S 采样脉冲是否停振 (超过 4096 拍无脉冲则判定 I2S 时钟异常)
    reg [11:0] i2s_wdog_cnt  = 12'd0;
    reg        i2s_clk_stall = 1'b0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            i2s_wdog_cnt  <= 12'd0;
            i2s_clk_stall <= 1'b0;
        end else if (sample_req_pulse) begin
            i2s_wdog_cnt  <= 12'd0;
            i2s_clk_stall <= 1'b0;
        end else if (i2s_wdog_cnt == 12'hFFF) begin
            i2s_clk_stall <= 1'b1;
        end else begin
            i2s_wdog_cnt <= i2s_wdog_cnt + 12'd1;
        end
    end

    // 2. 监测 MIDI 光耦输入引脚是否卡死在低电平 (>10ms 低电平代表线接反或 6N138 断电)
    reg [19:0] midi_low_cnt   = 20'd0;
    reg        midi_stuck_low = 1'b0;
    reg        latched_f_err  = 1'b0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            midi_low_cnt   <= 20'd0;
            midi_stuck_low <= 1'b0;
            latched_f_err  <= 1'b0;
        end else begin
            if (midi_frame_err)
                latched_f_err <= 1'b1;
            if (midi_rx_in == 1'b0) begin
                if (midi_low_cnt >= 20'd500_000)
                    midi_stuck_low <= 1'b1;
                else
                    midi_low_cnt <= midi_low_cnt + 20'd1;
            end else begin
                midi_low_cnt   <= 20'd0;
                midi_stuck_low <= 1'b0;
            end
        end
    end

    // 3. 锁存 WM8978 I2C 报错瞬间剩余的指令数 (精准反推哪一步寄存器 NACK)
    reg       i2c_err_d1      = 1'b0;
    reg [7:0] latched_err_idx = 8'd0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            i2c_err_d1      <= 1'b0;
            latched_err_idx <= 8'd0;
        end else begin
            i2c_err_d1 <= wm_i2c_err;
            if (wm_i2c_err && !i2c_err_d1)
                latched_err_idx <= {3'd0, 5'd23 - wm_i2c_fifo_cnt};
        end
    end

    // 4. 窗口活动性检测：监测 2ms 内算子核、抽取器、FIFO 与物理引脚 wm_dacdat 是否有真实翻转
    reg [16:0] win_cnt        = 17'd0;
    reg        core_act_tmp   = 1'b0, core_act_ok   = 1'b0;
    reg        decim_act_tmp  = 1'b0, decim_act_ok  = 1'b0;
    reg        fifo_act_tmp   = 1'b0, fifo_act_ok   = 1'b0;
    reg        dacdat_act_tmp = 1'b0, dacdat_act_ok = 1'b0;
    reg        dacdat_d1      = 1'b0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            win_cnt        <= 17'd0;
            core_act_tmp   <= 1'b0; core_act_ok   <= 1'b0;
            decim_act_tmp  <= 1'b0; decim_act_ok  <= 1'b0;
            fifo_act_tmp   <= 1'b0; fifo_act_ok   <= 1'b0;
            dacdat_act_tmp <= 1'b0; dacdat_act_ok <= 1'b0;
            dacdat_d1      <= 1'b0;
        end else begin
            dacdat_d1 <= wm_dacdat;
            if (core0_sum25 != 25'sd0)          core_act_tmp   <= 1'b1;
            if (anti_aliased_vld)               decim_act_tmp  <= 1'b1;
            if (final_i2s_pcm_24b != 24'd0)     fifo_act_tmp   <= 1'b1;
            if (wm_dacdat != dacdat_d1)         dacdat_act_tmp <= 1'b1;

            if (win_cnt >= 17'd99_999) begin
                win_cnt        <= 17'd0;
                core_act_ok    <= core_act_tmp;   core_act_tmp   <= 1'b0;
                decim_act_ok   <= decim_act_tmp;  decim_act_tmp  <= 1'b0;
                fifo_act_ok    <= fifo_act_tmp;   fifo_act_tmp   <= 1'b0;
                dacdat_act_ok  <= dacdat_act_tmp; dacdat_act_tmp <= 1'b0;
            end else begin
                win_cnt <= win_cnt + 17'd1;
            end
        end
    end

    // 5. 8 级全链路状态机实时裁决
    wire any_key_pressed = (|voice_note_on) | test_key_note;
    wire any_env_active  = (|voice_env_active);

    reg [3:0] stage_id;
    always @(*) begin
        if (!rst_n)                  stage_id = 4'h0; // 0: 复位未释放
        else if (!wm_init_done)      stage_id = 4'h1; // 1: 正在配置 WM8978 I2C
        else if (wm_i2c_err)         stage_id = 4'h2; // 2: WM8978 I2C 应答错误 (NACK)
        else if (!any_key_pressed && !any_env_active)
                                     stage_id = 4'h3; // 3: 芯片配置成功、待机等待按键
        else if (!any_env_active)    stage_id = 4'h4; // 4: 按键已触发但 ADSR 包络未激活
        else if (!core_act_ok)       stage_id = 4'h5; // 5: 包络已激活但算子核输出为 0
        else if (!decim_act_ok || !fifo_act_ok)
                                     stage_id = 4'h6; // 6: 抽取器或音频 FIFO 输出为 0
        else if (!dacdat_act_ok)     stage_id = 4'h7; // 7: FIFO 有数据但 wm_dacdat 引脚未翻转
        else                         stage_id = 4'h8; // 8: 全链路 8 级直达物理引脚 100% 正常!
    end

    reg [5:0] active_v_cnt;
    integer k;
    always @(*) begin
        active_v_cnt = 6'd0;
        for (k = 0; k < 32; k = k + 1)
            active_v_cnt = active_v_cnt + {5'd0, voice_env_active[k]};
    end

    wire [3:0] err_flags   = {midi_stuck_low, latched_f_err, i2s_clk_stall, wm_i2c_err};
    wire [7:0] detail_byte = wm_i2c_err ? latched_err_idx : {2'd0, active_v_cnt};

    assign diag_chain_word_24b = {stage_id, err_flags, detail_byte, 1'b0, latest_note_num};

endmodule

(* syn_hier = "hard" *)
module fmusic_synth_subsystem #(
    parameter        USE_TOUCH_UI_REGS  = 1,
    parameter        DIRECT_ALG_TO_MODE = 0,
    parameter        WM_USE_INT_PLL     = 0,
    parameter [21:0] PWR_WAIT_CLKS      = 22'd2_500_000,
    parameter [4:0]  PARAM_ALG_MODE     = 5'd1,
    parameter [2:0]  PARAM_CAR_OP_ID    = 3'd1,
    parameter [1:0]  PARAM_CAR_WAVE_SEL = 2'd0,
    parameter [2:0]  PARAM_MOD_OP_ID    = 3'd2,
    parameter [1:0]  PARAM_MOD_WAVE_SEL = 2'd3,
    parameter [4:0]  PARAM_CAR_RATIO    = 5'd1,
    parameter [6:0]  PARAM_CAR_LEVEL    = 7'd85,
    parameter [6:0]  PARAM_MOD_DEPTH    = 7'd0,
    parameter [4:0]  PARAM_MOD_RATIO    = 5'd1,
    parameter [6:0]  PARAM_MOD_FEEDBACK = 7'd0,
    parameter [6:0]  PARAM_ENV_ATTACK   = 7'd15,
    parameter [6:0]  PARAM_ENV_DECAY    = 7'd40,
    parameter [6:0]  PARAM_ENV_SUSTAIN  = 7'd80,
    parameter [6:0]  PARAM_ENV_RELEASE  = 7'd25
)(
    input  wire         clk_50m,
    input  wire         rst_n,
    input  wire [511:0] mod_regs_flat,
    input  wire         midi_rx_in,
    input  wire         test_key_note,
    input  wire         pure_sine_byp, // 同时按住 S3+S4 触发硬件纯 440Hz 正弦波直通自检
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
    output wire signed [15:0] scope_pcm_out,
    output wire               wm_init_done,
    output wire               wm_i2c_err,
    output wire [23:0]        diag_chain_word_24b
);
    (* syn_preserve = 1 *) reg [2:0] rst_pipe = 3'b000;
    (* syn_preserve = 1, syn_maxfan = 128 *) reg synth_rst_n = 1'b0;
    always @(posedge clk_50m) begin
        rst_pipe    <= {rst_pipe[1:0], rst_n};
        synth_rst_n <= rst_pipe[2];
    end

    (* syn_preserve = 1 *) reg [1:0] test_key_sync = 2'b00;
    (* syn_preserve = 1 *) reg [1:0] byp_sine_sync = 2'b00;
    (* syn_preserve = 1 *) reg [1:0] pwr_ok_sync_r = 2'b00;
    always @(posedge clk_50m) begin
        if (!synth_rst_n) begin
            test_key_sync <= 2'b00;
            byp_sine_sync <= 2'b00;
            pwr_ok_sync_r <= 2'b00;
        end else begin
            test_key_sync <= {test_key_sync[0], test_key_note};
            byp_sine_sync <= {byp_sine_sync[0], pure_sine_byp};
            pwr_ok_sync_r <= {pwr_ok_sync_r[0], ext_pwr_ok};
        end
    end

    (* syn_preserve = 1 *) reg [351:0] mod_regs_sync1;
    (* syn_preserve = 1 *) reg [351:0] mod_regs_sync2;
    always @(posedge clk_50m) begin
        if (!synth_rst_n) begin
            mod_regs_sync1 <= 352'd0;
            mod_regs_sync2 <= 352'd0;
        end else begin
            mod_regs_sync1 <= mod_regs_flat[351:0];
            mod_regs_sync2 <= mod_regs_sync1;
        end
    end

    wire [511:0] isolated_mod_regs_flat = {160'd0, mod_regs_sync2};
    wire [4:0]  w_fm_alg_sel;
    wire [2:0]  w_core_mod_mode;
    wire [2:0]  w_car_op_id;
    wire [1:0]  w_car_wave_sel;
    wire [2:0]  w_mod_op_id;
    wire [1:0]  w_mod_wave_sel;
    wire [4:0]  w_car_ratio;
    wire [4:0]  w_mod_ratio;
    wire [6:0]  w_car_level;
    wire [6:0]  w_mod_level;
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
        .PARAM_MASTER_VOL   (8'd90),
        .PARAM_ENV_ATTACK   (PARAM_ENV_ATTACK),
        .PARAM_ENV_DECAY    (PARAM_ENV_DECAY),
        .PARAM_ENV_SUSTAIN  (PARAM_ENV_SUSTAIN),
        .PARAM_ENV_RELEASE  (PARAM_ENV_RELEASE)
    ) u_ui_dec (
        .mod_regs_flat    (isolated_mod_regs_flat),
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
        .w_mod_depth      (w_mod_level),
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

    wire [31:0]   voice_note_on;
    wire [31:0]   voice_retrig;
    wire [31:0]   voice_env_active;
    wire [1023:0] voice_ftw_flat;
    wire [223:0]  voice_vel_flat;
    wire [31:0]   voice0_ftw;
    wire [6:0]    voice0_vel;
    wire          midi_frame_err;
    wire [6:0]    latest_note_num;

    midi_optocoupler_rx_poly32 u_midi_rx_poly32 (
        .clk              (clk_50m),
        .rst_n            (synth_rst_n),
        .midi_rx_in       (midi_rx_in),
        .voice_env_active (voice_env_active),
        .last_rx_byte     (),
        .rx_byte_vld      (),
        .rx_frame_err     (midi_frame_err),
        .fifo_data_cnt    (),
        .latest_note_num  (latest_note_num),
        .latest_velocity  (),
        .voice_note_on    (voice_note_on),
        .voice_retrig     (voice_retrig),
        .voice_ftw_flat   (voice_ftw_flat),
        .voice_vel_flat   (voice_vel_flat),
        .voice0_ftw       (voice0_ftw),
        .voice0_vel       (voice0_vel)
    );

    wire [5:0]  w_carrier_mask = 6'b000101;
    wire [31:0] ch1_car_ftw, ch1_mod_ftw, ch2_car_ftw, ch2_mod_ftw;
    wire [9:0]  car_phase_offset, mod_phase_offset;
    wire [11:0] op_waves_flat;
    wire [41:0] op_levels_flat;
    wire [59:0] op_phases_flat;
    wire [29:0] op_ratios_flat;
    wire [41:0] op_det_fb_flat;

    touch_fm_param_mapper u_param_mapper (
        .clk                  (clk_50m),
        .rst_n                (synth_rst_n),
        .car_op_id_in         (w_car_op_id),
        .car_wave_sel_in      (w_car_wave_sel),
        .mod_op_id_in         (w_mod_op_id),
        .mod_wave_sel_in      (w_mod_wave_sel),
        .carrier_mask_in      (w_carrier_mask),
        .car_ratio_in         (w_car_ratio),
        .mod_ratio_in         (w_mod_ratio),
        .car_level_in         (w_car_level),
        .mod_depth_in         (w_mod_level),
        .car_detune_in        (w_car_detune),
        .mod_feedback_in      (w_mod_feedback),
        .car_init_phase_in    (w_car_init_phase),
        .mod_init_phase_in    (w_mod_init_phase),
        .base_v0_ftw_in       (voice0_ftw),
        .base_v1_ftw_in       (voice0_ftw),
        .ch1_car_ftw_out      (ch1_car_ftw),
        .ch1_mod_ftw_out      (ch1_mod_ftw),
        .ch2_car_ftw_out      (ch2_car_ftw),
        .ch2_mod_ftw_out      (ch2_mod_ftw),
        .car_phase_offset_out (car_phase_offset),
        .mod_phase_offset_out (mod_phase_offset),
        .op_waves_flat_out    (op_waves_flat),
        .op_levels_flat_out   (op_levels_flat),
        .op_phases_flat_out   (op_phases_flat),
        .op_ratios_flat_out   (op_ratios_flat),
        .op_det_fb_flat_out   (op_det_fb_flat)
    );

    wire               sample_req_pulse;
    wire signed [24:0] core_raw_sum25 [0:3];
    wire signed [23:0] vca_modulated_wave_24b;
    wire signed [15:0] vca_modulated_wave_16b;
    wire               vca_wave_vld;

    genvar c;
    generate
        for (c = 0; c < 4; c = c + 1) begin : GEN_POLY32_CORES
            poly8_adsr_vca_engine u_poly8_core (
                .clk                   (clk_50m),
                .rst_n                 (synth_rst_n),
                .sample_tick           (sample_req_pulse),
                .voice_note_on         (voice_note_on[c*8   +: 8]),
                .voice_retrig          (voice_retrig[c*8    +: 8]),
                .test_key_note         ((c == 0) ? test_key_sync[1] : 1'b0),
                .voice_ftw_flat        (voice_ftw_flat[c*256 +: 256]),
                .voice_vel_flat        (voice_vel_flat[c*56  +: 56]),
                .fm_alg_sel            (w_fm_alg_sel),
                .mod_mode_sel          (w_core_mod_mode),
                .op_waves_flat         (op_waves_flat),
                .op_levels_flat        (op_levels_flat),
                .op_phases_flat        (op_phases_flat),
                .op_ratios_flat        (op_ratios_flat),
                .op_det_fb_flat        (op_det_fb_flat),
                .car_wave_sel          (op_waves_flat[1:0]),
                .mod_wave_sel          (op_waves_flat[3:2]),
                .car_ratio             (w_car_ratio),
                .mod_ratio             (w_mod_ratio),
                .car_detune            (w_car_detune),
                .car_phase_offset      (car_phase_offset),
                .mod_phase_offset      (mod_phase_offset),
                .car_level_pct         (w_car_level),
                .mod_level_pct         (w_mod_level),
                .feedback_pct          (w_mod_feedback),
                .step_a                (step_a),
                .step_d                (step_d),
                .sustain_level         (sus_lv),
                .step_r                (step_r),
                .primary_v0_wave_in    (16'sd0),
                .primary_v1_wave_in    (16'sd0),
                .voice_env_active      (voice_env_active[c*8 +: 8]),
                .voice_env_flat        (),
                .dbg_v0_env_out        (),
                .poly8_mixed_pcm24_out (),
                .poly8_mixed_pcm_out   (),
                .poly8_mixed_pcm_vld   (),
                .poly8_raw_sum25_out   (core_raw_sum25[c])
            );
        end
    endgenerate

    poly32_lossless_mixer_24b u_poly32_mixer (
        .clk              (clk_50m),
        .rst_n            (synth_rst_n),
        .voice_env_active (voice_env_active),
        .core0_sum25_in   (core_raw_sum25[0]),
        .core1_sum25_in   (core_raw_sum25[1]),
        .core2_sum25_in   (core_raw_sum25[2]),
        .core3_sum25_in   (core_raw_sum25[3]),
        .mixed_pcm24_out  (vca_modulated_wave_24b),
        .mixed_pcm16_out  (vca_modulated_wave_16b),
        .mixed_pcm_vld    (vca_wave_vld)
    );

    // 硬件 440Hz 纯正弦波直通自检源 (按住 S3+S4 时直接旁路 UI 参数，输出教科书级正弦波)
    reg [31:0] byp_dds_acc = 32'd0;
    always @(posedge clk_50m) begin
        byp_dds_acc <= byp_dds_acc + 32'h000093A4; // 440Hz A4
    end
    wire [15:0] byp_raw_u16;
    Gowin_pROM_sine u_byp_sine_rom (
        .dout  (byp_raw_u16),
        .clk   (clk_50m),
        .oce   (1'b1),
        .ce    (1'b1),
        .reset (~synth_rst_n),
        .ad    (byp_dds_acc[31:22])
    );
    wire signed [15:0] byp_sine_s16 = {~byp_raw_u16[15], byp_raw_u16[14:0]};
    wire signed [23:0] byp_sine_24b = {{2{byp_sine_s16[15]}}, byp_sine_s16, 6'd0};
    wire signed [23:0] muxed_pcm_24b = byp_sine_sync[1] ? byp_sine_24b : vca_modulated_wave_24b;

    wire signed [23:0] anti_aliased_pcm_24b;
    wire               anti_aliased_vld;
    oversample_decimator_1024 #(
        .DATA_WIDTH (24)
    ) u_anti_alias_flt (
        .clk_50m          (clk_50m),
        .rst_n            (synth_rst_n),
        .sample_req_pulse (sample_req_pulse),
        .pcm_in           (muxed_pcm_24b),
        .clean_pcm_out    (anti_aliased_pcm_24b),
        .clean_pcm_vld    (anti_aliased_vld)
    );

    wire [23:0] final_i2s_pcm_24b;
    audio_staging_buffer #(
        .DATA_WIDTH  (24),
        .STAGE_WIDTH (28)
    ) u_staging (
        .clk              (clk_50m),
        .rst_n            (synth_rst_n),
        .local_synth_data (anti_aliased_pcm_24b),
        .local_synth_en   (anti_aliased_vld),
        .cascade_data_in  (28'd0),
        .cascade_en_in    (1'b0),
        .group_addr_in    (2'b00),
        .i2s_sample_req   (sample_req_pulse),
        .stage_data_out   (),
        .stage_en_out     (),
        .final_i2s_data   (final_i2s_pcm_24b),
        .final_i2s_en     ()
    );

    assign scope_pcm_out = final_i2s_pcm_24b[23:8];

    wire [4:0] wm_i2c_fifo_cnt;
    wm8978_driver #(
        .SYS_CLK_FREQ   (50_000_000),
        .I2C_FREQ       (100_000),
        .USE_INT_PLL    (WM_USE_INT_PLL),
        .AUDIO_WORD_LEN (24),
        .PWR_WAIT_CLKS  (PWR_WAIT_CLKS)
    ) u_wm8978 (
        .clk_50m          (clk_50m),
        .rst_n            (synth_rst_n),
        .ext_pwr_ok       (pwr_ok_sync_r[1]),
        .reinit_trig      (1'b0),
        .master_vol_pct   (w_master_vol),
        .pcm_left_in      (final_i2s_pcm_24b),
        .pcm_right_in     (final_i2s_pcm_24b),
        .sample_req_pulse (sample_req_pulse),
        .i2c_fifo_cnt     (wm_i2c_fifo_cnt),
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

    // 例化全链路 8 级自检与故障时刻锁存状态机
    synth_chain_diag_fsm u_chain_diag (
        .clk_50m             (clk_50m),
        .rst_n               (synth_rst_n),
        .wm_init_done        (wm_init_done),
        .wm_i2c_err          (wm_i2c_err),
        .wm_i2c_fifo_cnt     (wm_i2c_fifo_cnt),
        .sample_req_pulse    (sample_req_pulse),
        .midi_rx_in          (midi_rx_in),
        .midi_frame_err      (midi_frame_err),
        .latest_note_num     (latest_note_num),
        .voice_note_on       (voice_note_on),
        .test_key_note       (test_key_sync[1] | byp_sine_sync[1]),
        .voice_env_active    (voice_env_active),
        .core0_sum25         (core_raw_sum25[0]),
        .anti_aliased_vld    (anti_aliased_vld),
        .final_i2s_pcm_24b   (final_i2s_pcm_24b),
        .wm_dacdat           (wm_dacdat),
        .diag_chain_word_24b (diag_chain_word_24b)
    );
endmodule