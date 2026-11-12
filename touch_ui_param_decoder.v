`timescale 1ns / 1ps

module touch_ui_param_decoder #(
    parameter        USE_TOUCH_UI_REGS  = 1,
    parameter        DIRECT_ALG_TO_MODE = 0,
    parameter [1:0]  WAVE_SINE          = 2'd0,
    parameter [1:0]  WAVE_SQUARE        = 2'd1,
    parameter [1:0]  WAVE_SAWTOOTH      = 2'd2,
    parameter [1:0]  WAVE_TRIANGLE      = 2'd3,
    parameter [2:0]  MODE_FM_STD        = 3'd0,
    parameter [2:0]  MODE_FM_DEEP       = 3'd1,
    parameter [2:0]  MODE_AM_MOD        = 3'd2,
    parameter [2:0]  MODE_RING_MOD      = 3'd3,
    parameter [2:0]  MODE_ADD_MIX       = 3'd4,
    parameter [2:0]  MODE_SUB_DIFF      = 3'd5,
    parameter [4:0]  PARAM_ALG_MODE     = 5'd1,
    parameter [2:0]  PARAM_CAR_OP_ID    = 3'd1,
    parameter [1:0]  PARAM_CAR_WAVE_SEL = WAVE_SINE,
    parameter [2:0]  PARAM_MOD_OP_ID    = 3'd2,
    parameter [1:0]  PARAM_MOD_WAVE_SEL = WAVE_TRIANGLE,
    parameter [4:0]  PARAM_CAR_RATIO    = 5'd1,
    parameter [6:0]  PARAM_CAR_LEVEL    = 7'd85,
    parameter [6:0]  PARAM_MOD_DEPTH    = 7'd0,
    parameter [4:0]  PARAM_MOD_RATIO    = 5'd1,
    parameter [6:0]  PARAM_MOD_FEEDBACK = 7'd0,
    parameter [6:0]  PARAM_CAR_DETUNE   = 7'd50,
    parameter [6:0]  PARAM_CAR_PHASE    = 7'd0,
    parameter [6:0]  PARAM_MOD_PHASE    = 7'd0,
    parameter [7:0]  PARAM_MASTER_VOL   = 8'd90,
    parameter [6:0]  PARAM_ENV_ATTACK   = 7'd25,
    parameter [6:0]  PARAM_ENV_DECAY    = 7'd45,
    parameter [6:0]  PARAM_ENV_SUSTAIN  = 7'd75,
    parameter [6:0]  PARAM_ENV_RELEASE  = 7'd35
)(
    input  wire [511:0] mod_regs_flat,
    output wire [3:0]   w_ui_page_id,
    output wire [4:0]   w_fm_alg_sel,
    output wire [2:0]   w_core_mod_mode,
    output wire [2:0]   w_car_op_id,
    output wire [1:0]   w_car_wave_sel,
    output wire [2:0]   w_mod_op_id,
    output wire [1:0]   w_mod_wave_sel,
    output wire [4:0]   w_car_ratio,
    output wire [4:0]   w_mod_ratio,
    output wire [6:0]   w_car_level,
    output wire [6:0]   w_mod_depth,
    output wire [6:0]   w_mod_feedback,
    output wire [6:0]   w_car_detune,
    output wire [6:0]   w_car_init_phase,
    output wire [6:0]   w_mod_init_phase,
    output wire [7:0]   w_ui_event_cnt,
    output wire [7:0]   w_master_vol,
    output wire [6:0]   w_env_atk,
    output wire [6:0]   w_env_dec,
    output wire [6:0]   w_env_sus,
    output wire [6:0]   w_env_rel,
    output wire [23:0]  w_adsr_step_a,
    output wire [23:0]  w_adsr_step_d,
    output wire [7:0]   w_adsr_sus_lv,
    output wire [23:0]  w_adsr_step_r
);

    wire [31:0] reg_c0 = mod_regs_flat[0*32  +: 32];
    wire [31:0] reg_c1 = mod_regs_flat[1*32  +: 32];
    wire [31:0] reg_c2 = mod_regs_flat[2*32  +: 32];
    wire [31:0] reg_c3 = mod_regs_flat[3*32  +: 32];
    wire [31:0] reg_c4 = mod_regs_flat[4*32  +: 32];
    wire [31:0] reg_c5 = mod_regs_flat[5*32  +: 32];
    wire [31:0] reg_c6 = mod_regs_flat[6*32  +: 32];
    wire [31:0] reg_c7 = mod_regs_flat[7*32  +: 32];
    wire [31:0] reg_c8 = mod_regs_flat[8*32  +: 32];
    wire [31:0] reg_c9 = mod_regs_flat[9*32  +: 32];
    wire [31:0] reg_ca = mod_regs_flat[10*32 +: 32];

    wire [7:0] raw_car_det = reg_c2[31:24];
    wire [7:0] raw_car_pha = reg_c1[31:24];
    wire [7:0] raw_mod_pha = (reg_c3[31:24] != 8'd0) ? reg_c3[31:24] : reg_c5[31:24];
    wire       cpu_synced  = (reg_ca[31:24] != 8'd0) || (raw_car_det != 8'd0);

    assign w_ui_page_id = USE_TOUCH_UI_REGS ? reg_c0[19:16] : 4'd1;
    assign w_fm_alg_sel = USE_TOUCH_UI_REGS ? ((reg_c0[4:0] == 5'd0 || reg_c0[4:0] > 5'd16) ? PARAM_ALG_MODE : reg_c0[4:0]) : PARAM_ALG_MODE;

    assign w_core_mod_mode = DIRECT_ALG_TO_MODE ?
                             ((w_fm_alg_sel == 5'd1) ? MODE_FM_STD   :
                              (w_fm_alg_sel == 5'd2) ? MODE_FM_DEEP  :
                              (w_fm_alg_sel == 5'd3) ? MODE_AM_MOD   :
                              (w_fm_alg_sel == 5'd4) ? MODE_RING_MOD :
                              (w_fm_alg_sel == 5'd5) ? MODE_ADD_MIX  :
                              (w_fm_alg_sel == 5'd6) ? MODE_SUB_DIFF :
                              (w_fm_alg_sel == 5'd7) ? MODE_FM_DEEP  : MODE_FM_STD) :
                             ((w_fm_alg_sel == 5'd5) ? MODE_FM_DEEP  :
                              (w_fm_alg_sel == 5'd6) ? MODE_RING_MOD :
                              (w_fm_alg_sel == 5'd7) ? MODE_SUB_DIFF :
                              (w_fm_alg_sel == 5'd8) ? MODE_ADD_MIX  : MODE_FM_STD);

    // 仅当 CPU 显式写入 op_id (1~6) 时才更新算子，op_id == 0 保护锁期间绝不误写 OP1/OP2!
    assign w_car_op_id    = USE_TOUCH_UI_REGS ? reg_c1[18:16] : PARAM_CAR_OP_ID;
    assign w_car_wave_sel = USE_TOUCH_UI_REGS ? reg_c1[1:0]   : PARAM_CAR_WAVE_SEL;
    assign w_mod_op_id    = USE_TOUCH_UI_REGS ? reg_c3[18:16] : PARAM_MOD_OP_ID;
    assign w_mod_wave_sel = USE_TOUCH_UI_REGS ? reg_c3[1:0]   : PARAM_MOD_WAVE_SEL;
    assign w_car_ratio    = USE_TOUCH_UI_REGS ? ((reg_c2[20:16] == 5'd0) ? PARAM_CAR_RATIO : reg_c2[20:16]) : PARAM_CAR_RATIO;
    assign w_mod_ratio    = USE_TOUCH_UI_REGS ? ((reg_c5[20:16] == 5'd0) ? PARAM_MOD_RATIO : reg_c5[20:16]) : PARAM_MOD_RATIO;

    // 若高 8 位 raw_car_det 为 0 (固件未显式设置微调)，始终保持 50 (零频偏中心)，杜绝 -50 跑调!
    wire [6:0] clp_car_det = (raw_car_det == 8'd0)  ? PARAM_CAR_DETUNE :
                             (raw_car_det > 8'd100) ? 7'd100           : raw_car_det[6:0];
    wire [6:0] clp_car_pha = (raw_car_pha > 8'd100) ? 7'd100           : raw_car_pha[6:0];
    wire [6:0] clp_mod_pha = (raw_mod_pha > 8'd100) ? 7'd100           : raw_mod_pha[6:0];

    assign w_car_detune     = USE_TOUCH_UI_REGS ? clp_car_det : PARAM_CAR_DETUNE;
    assign w_car_init_phase = USE_TOUCH_UI_REGS ? clp_car_pha : PARAM_CAR_PHASE;
    assign w_mod_init_phase = USE_TOUCH_UI_REGS ? clp_mod_pha : PARAM_MOD_PHASE;
    assign w_ui_event_cnt   = reg_ca[31:24];

    wire [7:0] raw_car_lvl = reg_c2[7:0];
    wire [7:0] raw_mod_dep = reg_c4[7:0];
    wire [7:0] raw_mod_fb  = reg_c5[7:0];
    wire [7:0] raw_mst_vol = (reg_ca[7:0] != 8'd0) ? reg_ca[7:0] : PARAM_MASTER_VOL;

    // 允许用户将任意发声算子音量调为 0 静音!
    wire [6:0] clamped_car_lvl = (raw_car_lvl > 8'd100) ? 7'd100 : raw_car_lvl[6:0];
    wire [6:0] clamped_mod_dep = (raw_mod_dep > 8'd100) ? 7'd100 : raw_mod_dep[6:0];
    wire [6:0] clamped_mod_fb  = (raw_mod_fb  > 8'd100) ? 7'd100 : raw_mod_fb[6:0];
    wire [7:0] clamped_mst_vol = (raw_mst_vol > 8'd100) ? 8'd100 : raw_mst_vol;

    assign w_car_level    = USE_TOUCH_UI_REGS ? clamped_car_lvl : PARAM_CAR_LEVEL;
    assign w_mod_depth    = USE_TOUCH_UI_REGS ? clamped_mod_dep : PARAM_MOD_DEPTH;
    assign w_mod_feedback = USE_TOUCH_UI_REGS ? clamped_mod_fb  : PARAM_MOD_FEEDBACK;
    assign w_master_vol   = USE_TOUCH_UI_REGS ? clamped_mst_vol : PARAM_MASTER_VOL;

    assign w_env_atk = USE_TOUCH_UI_REGS ? ((reg_c6[7:0] > 8'd100) ? 7'd100 : reg_c6[6:0]) : PARAM_ENV_ATTACK;
    assign w_env_dec = USE_TOUCH_UI_REGS ? ((reg_c7[7:0] > 8'd100) ? 7'd100 : reg_c7[6:0]) : PARAM_ENV_DECAY;
    assign w_env_sus = USE_TOUCH_UI_REGS ? ((reg_c8[7:0] > 8'd100) ? 7'd100 : reg_c8[6:0]) : PARAM_ENV_SUSTAIN;
    assign w_env_rel = USE_TOUCH_UI_REGS ? ((reg_c9[7:0] > 8'd100) ? 7'd100 : reg_c9[6:0]) : PARAM_ENV_RELEASE;

    function [23:0] calc_adsr_step(input [6:0] val);
        reg [6:0] inv;
        begin
            inv = (val >= 7'd100) ? 7'd0 : (7'd100 - val);
            if (inv <= 7'd25)
                calc_adsr_step = 24'd6 + {17'd0, inv};
            else if (inv <= 7'd50)
                calc_adsr_step = 24'd31 + ({17'd0, inv - 7'd25} << 2);
            else if (inv <= 7'd80)
                calc_adsr_step = 24'd131 + ({17'd0, inv - 7'd50} << 4);
            else
                calc_adsr_step = 24'd611 + ({17'd0, inv - 7'd80} << 8);
        end
    endfunction

    assign w_adsr_step_a = calc_adsr_step(w_env_atk);
    assign w_adsr_step_d = calc_adsr_step(w_env_dec);
    assign w_adsr_step_r = calc_adsr_step(w_env_rel);

    wire [18:0] sus_mult_q10 = w_env_sus * 12'd2611;
    assign w_adsr_sus_lv     = (w_env_sus >= 7'd100) ? 8'd255 : sus_mult_q10[17:10];

endmodule