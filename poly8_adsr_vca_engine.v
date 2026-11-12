`timescale 1ns / 1ps
`ifndef POLY8_ADSR_VCA_ENGINE_V
`define POLY8_ADSR_VCA_ENGINE_V

// ============================================================================
// 模块名称: poly8_adsr_vca_engine (消除 prom_inst_0/DO 25ns 违例 + 零符号位抖动稳定版)
// 核心修复:
//   1. 严格保持 6 步 TDM 状态机 (tdm_step = 0..5, tdm_sub = 0..3) 节拍不变；
//   2. 在 tdm_sub==2 用触发器 rom_y0_latch 直接截断 BSRAM DO 输出 (0 算术逻辑!)，
//      在 tdm_sub==3 仅从触发器 rom_y0_latch 出发做 1 次算子音量乘法，彻底消除符号位时序抖动；
//   3. 保留 vca_p1 / vca_p2 两级后置流水线寄存器，将载波归一化与声部包络乘法完全解耦。
// ============================================================================
module poly8_adsr_vca_engine #(
    parameter [1:0] WAVE_SINE     = 2'd0,
    parameter [1:0] WAVE_SQUARE   = 2'd1,
    parameter [1:0] WAVE_SAWTOOTH = 2'd2,
    parameter [1:0] WAVE_TRIANGLE = 2'd3,
    parameter [2:0] MODE_FM_STD   = 3'd0,
    parameter [2:0] MODE_FM_DEEP  = 3'd1,
    parameter [2:0] MODE_AM_MOD   = 3'd2,
    parameter [2:0] MODE_RING_MOD = 3'd3,
    parameter [2:0] MODE_ADD_MIX  = 3'd4,
    parameter [2:0] MODE_SUB_DIFF = 3'd5
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         sample_tick,
    input  wire [7:0]   voice_note_on,
    input  wire [7:0]   voice_retrig,
    input  wire         test_key_note,
    input  wire [255:0] voice_ftw_flat,
    input  wire [55:0]  voice_vel_flat,
    input  wire [4:0]   fm_alg_sel,
    input  wire [2:0]   mod_mode_sel,
    input  wire [11:0]  op_waves_flat,
    input  wire [41:0]  op_levels_flat,
    input  wire [59:0]  op_phases_flat,
    input  wire [29:0]  op_ratios_flat,
    input  wire [41:0]  op_det_fb_flat,
    input  wire [1:0]   car_wave_sel,
    input  wire [1:0]   mod_wave_sel,
    input  wire [4:0]   car_ratio,
    input  wire [4:0]   mod_ratio,
    input  wire [6:0]   car_detune,
    input  wire [9:0]   car_phase_offset,
    input  wire [9:0]   mod_phase_offset,
    input  wire [6:0]   car_level_pct,
    input  wire [6:0]   mod_level_pct,
    input  wire [6:0]   feedback_pct,
    input  wire [23:0]  step_a,
    input  wire [23:0]  step_d,
    input  wire [7:0]   sustain_level,
    input  wire [23:0]  step_r,
    input  wire signed [15:0] primary_v0_wave_in,
    input  wire signed [15:0] primary_v1_wave_in,
    output wire [7:0]         voice_env_active,
    output wire [63:0]        voice_env_flat,
    output wire [7:0]         dbg_v0_env_out,
    output reg  signed [23:0] poly8_mixed_pcm24_out,
    output reg  signed [15:0] poly8_mixed_pcm_out,
    output reg                poly8_mixed_pcm_vld,
    output wire signed [24:0] poly8_raw_sum25_out
);

    wire gowin_reset = ~rst_n;

    reg [5:0] car_mask;
    reg [3:0] car_gain_weight;

    always @(*) begin
        case (fm_alg_sel)
            5'd1 : begin car_mask = 6'b000101; car_gain_weight = 4'd4; end
            5'd2 : begin car_mask = 6'b010101; car_gain_weight = 4'd3; end
            5'd3 : begin car_mask = 6'b000101; car_gain_weight = 4'd4; end
            5'd4 : begin car_mask = 6'b000001; car_gain_weight = 4'd7; end
            5'd5 : begin car_mask = 6'b011001; car_gain_weight = 4'd3; end
            5'd6 : begin car_mask = 6'b001001; car_gain_weight = 4'd4; end
            5'd7 : begin car_mask = 6'b011111; car_gain_weight = 4'd2; end
            5'd8 : begin car_mask = 6'b111111; car_gain_weight = 4'd2; end
            5'd9 : begin car_mask = 6'b001001; car_gain_weight = 4'd4; end
            5'd10: begin car_mask = 6'b001001; car_gain_weight = 4'd4; end
            5'd11: begin car_mask = 6'b000101; car_gain_weight = 4'd4; end
            5'd12: begin car_mask = 6'b000001; car_gain_weight = 4'd7; end
            5'd13: begin car_mask = 6'b011101; car_gain_weight = 4'd2; end
            5'd14: begin car_mask = 6'b001011; car_gain_weight = 4'd3; end
            5'd15: begin car_mask = 6'b000011; car_gain_weight = 4'd4; end
            5'd16: begin car_mask = 6'b001111; car_gain_weight = 4'd2; end
            default: begin car_mask = 6'b000101; car_gain_weight = 4'd4; end
        endcase
    end

    wire [7:0] eff_note_on = {voice_note_on[7:1], voice_note_on[0] | test_key_note};
    wire [7:0] voice_env_val [0:7];
    assign dbg_v0_env_out = voice_env_val[0];

    genvar v;
    generate
        for (v = 0; v < 8; v = v + 1) begin : GEN_POLY8_ADSR
            wire [7:0] v_rom_addr;
            wire [7:0] v_rom_data;

            Gowin_pROM_quxian_baoluo u_voice_env_rom (
                .dout  (v_rom_data),
                .clk   (clk),
                .oce   (1'b1),
                .ce    (1'b1),
                .reset (gowin_reset),
                .ad    (v_rom_addr)
            );

            adsr_envelope #(
                .ACC_WIDTH (24),
                .ENV_WIDTH (8)
            ) u_voice_adsr (
                .clk           (clk),
                .rst_n         (rst_n),
                .note_on       (eff_note_on[v]),
                .retrig_pulse  (voice_retrig[v]),
                .step_a        (step_a),
                .step_d        (step_d),
                .step_r        (step_r),
                .sustain_level (sustain_level),
                .rom_addr      (v_rom_addr),
                .rom_data      (v_rom_data),
                .envelope_out  (voice_env_val[v]),
                .env_active    (voice_env_active[v])
            );

            assign voice_env_flat[v*8 +: 8] = voice_env_val[v];
        end
    endgenerate

    (* syn_ramstyle = "registers" *) reg [31:0]        v_base_acc      [0:7];
    (* syn_ramstyle = "registers" *) reg [31:0]        v_det_acc       [0:7];
    (* syn_ramstyle = "registers" *) reg signed [15:0] v_fb_prev       [0:7];
    (* syn_ramstyle = "registers" *) reg [14:0]        v_smooth_gain   [0:7];
    (* syn_ramstyle = "registers" *) reg signed [21:0] voice_vca_out22 [0:7];
    (* syn_ramstyle = "registers" *) reg signed [15:0] v0_op_wave      [0:5];
    reg signed [15:0] v0_fm_raw_wave = 16'sd0;
    reg signed [15:0] v0_fm_env_wave = 16'sd0;

    integer idx;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (idx = 0; idx < 8; idx = idx + 1) begin
                v_base_acc[idx] <= 32'd0;
                v_det_acc[idx]  <= 32'd0;
            end
        end else begin
            for (idx = 0; idx < 8; idx = idx + 1) begin
                if (voice_retrig[idx] && (voice_env_val[idx] == 8'd0)) begin
                    v_base_acc[idx] <= 32'd0;
                    v_det_acc[idx]  <= 32'd0;
                end else begin
                    v_base_acc[idx] <= v_base_acc[idx] + voice_ftw_flat[idx*32 +: 32];
                    v_det_acc[idx]  <= v_det_acc[idx]  + {11'd0, voice_ftw_flat[idx*32 + 11 +: 21]};
                end
            end
        end
    end

    reg [2:0] tdm_vid  = 3'd0;
    reg [2:0] tdm_step = 3'd0;
    reg [1:0] tdm_sub  = 2'd0;

    wire [2:0] cur_op = 3'd5 - tdm_step;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tdm_vid  <= 3'd0;
            tdm_step <= 3'd0;
            tdm_sub  <= 2'd0;
        end else begin
            tdm_sub <= tdm_sub + 2'd1;
            if (tdm_sub == 2'd3) begin
                if (tdm_step == 3'd5) begin
                    tdm_step <= 3'd0;
                    tdm_vid  <= tdm_vid + 3'd1;
                end else begin
                    tdm_step <= tdm_step + 3'd1;
                end
            end
        end
    end

    wire [1:0] cur_op_wave   = op_waves_flat[cur_op*2  +: 2];
    wire [6:0] cur_op_level  = op_levels_flat[cur_op*7 +: 7];
    wire [9:0] cur_op_phase0 = op_phases_flat[cur_op*10 +: 10];
    wire [4:0] cur_op_ratio  = op_ratios_flat[cur_op*5 +: 5];
    wire [6:0] cur_op_det_fb = op_det_fb_flat[cur_op*7 +: 7];
    wire       cur_is_car    = car_mask[cur_op];

    reg signed [15:0] op_out_buf [0:5];
    (* syn_preserve = 1 *) reg signed [19:0] car_bus_acc = 20'sd0;

    wire signed [17:0] s_op2 = $signed({{2{op_out_buf[1][15]}}, op_out_buf[1]});
    wire signed [17:0] s_op3 = $signed({{2{op_out_buf[2][15]}}, op_out_buf[2]});
    wire signed [17:0] s_op4 = $signed({{2{op_out_buf[3][15]}}, op_out_buf[3]});
    wire signed [17:0] s_op5 = $signed({{2{op_out_buf[4][15]}}, op_out_buf[4]});
    wire signed [17:0] s_op6 = $signed({{2{op_out_buf[5][15]}}, op_out_buf[5]});

    reg signed [17:0] mod_in_wave;
    always @(*) begin
        mod_in_wave = 18'sd0;
        case (cur_op)
            3'd5: mod_in_wave = 18'sd0;
            3'd4: begin
                if (fm_alg_sel == 5'd1  || fm_alg_sel == 5'd2  || fm_alg_sel == 5'd3  ||
                    fm_alg_sel == 5'd4  || fm_alg_sel == 5'd6  || fm_alg_sel == 5'd7  ||
                    fm_alg_sel == 5'd9  || fm_alg_sel == 5'd12 || fm_alg_sel == 5'd13 ||
                    fm_alg_sel == 5'd15 || fm_alg_sel == 5'd16)
                    mod_in_wave = s_op6;
            end
            3'd3: begin
                if (fm_alg_sel == 5'd1  || fm_alg_sel == 5'd6  || fm_alg_sel == 5'd9  ||
                    fm_alg_sel == 5'd12 || fm_alg_sel == 5'd16)
                    mod_in_wave = s_op5;
                else if (fm_alg_sel == 5'd5 || fm_alg_sel == 5'd13)
                    mod_in_wave = s_op6;
                else if (fm_alg_sel == 5'd10 || fm_alg_sel == 5'd11 || fm_alg_sel == 5'd14)
                    mod_in_wave = (s_op5 + s_op6) >>> 1;
            end
            3'd2: begin
                if (fm_alg_sel == 5'd1  || fm_alg_sel == 5'd2  || fm_alg_sel == 5'd4  ||
                    fm_alg_sel == 5'd11 || fm_alg_sel == 5'd15)
                    mod_in_wave = s_op4;
                else if (fm_alg_sel == 5'd3)
                    mod_in_wave = (s_op4 + s_op5) >>> 1;
                else if (fm_alg_sel == 5'd13)
                    mod_in_wave = s_op2;
                else if (fm_alg_sel == 5'd16)
                    mod_in_wave = s_op5;
            end
            3'd1: begin
                if (fm_alg_sel == 5'd5  || fm_alg_sel == 5'd6  || fm_alg_sel == 5'd9  ||
                    fm_alg_sel == 5'd10 || fm_alg_sel == 5'd14)
                    mod_in_wave = s_op3;
                else if (fm_alg_sel == 5'd15 || fm_alg_sel == 5'd16)
                    mod_in_wave = s_op5;
            end
            3'd0: begin
                if (fm_alg_sel == 5'd4)
                    mod_in_wave = (s_op2 + s_op3 + s_op5) >>> 1;
                else if (fm_alg_sel == 5'd12)
                    mod_in_wave = (s_op2 + s_op3 + s_op4) >>> 1;
                else if (fm_alg_sel == 5'd15)
                    mod_in_wave = s_op3;
                else if (fm_alg_sel == 5'd16)
                    mod_in_wave = s_op5;
                else if (fm_alg_sel != 5'd7  && fm_alg_sel != 5'd8 &&
                         fm_alg_sel != 5'd13 && fm_alg_sel != 5'd14)
                    mod_in_wave = s_op2;
            end
            default: mod_in_wave = 18'sd0;
        endcase
    end

    // 仅取高 20 位做谐波倍频与微调乘法 (减少 12 位进位链延迟，且与 32 位取 [31:14] 精度一致)
    wire [19:0] base_harmonic_ph = v_base_acc[tdm_vid][31:12] * ((cur_op_ratio == 5'd0) ? 5'd1 : cur_op_ratio);
    wire signed [7:0]  det_signed = cur_is_car ? ($signed({1'b0, cur_op_det_fb}) - 8'sd50) : 8'sd0;
    wire signed [19:0] det_ph_off = $signed(v_det_acc[tdm_vid][31:12]) * det_signed;
    wire [19:0] total_osc_ph      = base_harmonic_ph + (cur_is_car ? det_ph_off : 20'd0);

    wire [17:0] base_ph_18b = total_osc_ph[19:2] + {cur_op_phase0, 8'd0};
    wire [6:0]  fb_amt      = (cur_op == 3'd5 && !cur_is_car) ? cur_op_det_fb : 7'd0;
    wire signed [23:0] fb_prod = v_fb_prev[tdm_vid] * $signed({1'b0, fb_amt});
    wire [17:0] fb_ph_18b   = (cur_op_wave == WAVE_SQUARE || cur_op_wave == WAVE_SAWTOOTH) ?
                              18'd0 : {{4{fb_prod[23]}}, fb_prod[23:10]};

    wire signed [15:0] clamped_mod_s16 = (mod_in_wave > 18'sd32767)  ? 16'sd32767  :
                                         (mod_in_wave < -18'sd32768) ? -16'sd32768 :
                                         mod_in_wave[15:0];
    wire [17:0] pmod_ph_18b = {{3{clamped_mod_s16[15]}}, clamped_mod_s16[15:1]};
    wire [17:0] full_ph_18b = base_ph_18b + fb_ph_18b + pmod_ph_18b;

    reg  [9:0]         rom_phase_ad;
    reg  [1:0]         rom_wave_sel;
    (* syn_preserve = 1 *) reg signed [15:0] rom_y0_latch = 16'sd0;
    wire signed [15:0] rom_wave_s16;

    fmusic_op_quad_wave_rom u_shared_op_rom (
        .clk          (clk),
        .rst_n        (rst_n),
        .phase_ad     (rom_phase_ad),
        .wave_sel     (rom_wave_sel),
        .wave_s16_out (rom_wave_s16)
    );

    wire [7:0] cur_v_env = voice_env_val[tdm_vid];
    wire [6:0] v_raw_vel = voice_vel_flat[tdm_vid*7 +: 7];
    reg  [6:0] v_mapped_vel;

    always @(*) begin
        if (v_raw_vel == 7'd0)
            v_mapped_vel = 7'd82;
        else if (v_raw_vel <= 7'd20)
            v_mapped_vel = 7'd37 + v_raw_vel;
        else if (v_raw_vel <= 7'd76)
            v_mapped_vel = 7'd57 + ((v_raw_vel - 7'd20) >> 1);
        else
            v_mapped_vel = 7'd85 + ((v_raw_vel - 7'd76) >> 2);
    end

    wire [6:0] clamped_op_lv = (cur_op_level > 7'd100) ? 7'd100 : cur_op_level;
    (* syn_preserve = 1 *) reg [13:0] eff_op_gain_r   = 14'd8192;
    (* syn_preserve = 1 *) reg [14:0] v_target_gain_r = 15'd0;

    // 在 tdm_sub == 3 仅从触发器 rom_y0_latch 出发执行 1 级乘法 (完全脱离 BSRAM DO 长路径!)
    wire signed [30:0] op_sc_prod    = rom_y0_latch * $signed({1'b0, eff_op_gain_r}) + 31'sd4096;
    wire signed [16:0] op_sc_s17     = $signed(op_sc_prod[29:13]);
    wire signed [15:0] op_scaled_s16 = (op_sc_s17 > 17'sd32767)  ? 16'sd32767  :
                                       (op_sc_s17 < -17'sd32768) ? -16'sd32768 :
                                       op_sc_s17[15:0];

    wire [14:0] v_prev_gain   = v_smooth_gain[tdm_vid];
    wire [14:0] gain_diff_up  = v_target_gain_r - v_prev_gain;
    wire [14:0] gain_diff_dn  = v_prev_gain     - v_target_gain_r;
    wire [14:0] step_up       = (gain_diff_up > 15'd256) ? 15'd256 : ((gain_diff_up >> 3) + 15'd1);
    wire [14:0] step_dn       = (gain_diff_dn > 15'd256) ? 15'd256 : ((gain_diff_dn >> 3) + 15'd1);

    wire [14:0] v_next_gain   = (v_target_gain_r > v_prev_gain) ? (v_prev_gain + step_up) :
                                (v_target_gain_r < v_prev_gain) ? (v_prev_gain - step_dn) :
                                v_prev_gain;

    wire signed [19:0] final_car_sum = cur_is_car ?
                                       (car_bus_acc + $signed({{4{op_scaled_s16[15]}}, op_scaled_s16})) :
                                       car_bus_acc;

    (* syn_preserve = 1 *) reg               vca_p1_vld  = 1'b0;
    (* syn_preserve = 1 *) reg [2:0]         vca_p1_vid  = 3'd0;
    (* syn_preserve = 1 *) reg [14:0]        vca_p1_gain = 15'd0;
    (* syn_preserve = 1 *) reg signed [19:0] vca_p1_sum  = 20'sd0;

    wire signed [24:0] p1_norm_prod  = vca_p1_sum * $signed({1'b0, car_gain_weight}) + 25'sd8;
    wire signed [20:0] p1_norm_s21   = p1_norm_prod[24:4];
    wire signed [15:0] p1_wave_s16   = (p1_norm_s21 > 21'sd32767)  ? 16'sd32767  :
                                       (p1_norm_s21 < -21'sd32768) ? -16'sd32768 :
                                       p1_norm_s21[15:0];

    (* syn_preserve = 1 *) reg               vca_p2_vld  = 1'b0;
    (* syn_preserve = 1 *) reg [2:0]         vca_p2_vid  = 3'd0;
    (* syn_preserve = 1 *) reg [14:0]        vca_p2_gain = 15'd0;
    (* syn_preserve = 1 *) reg signed [15:0] vca_p2_wave = 16'sd0;

    wire signed [31:0] p2_vca_prod = vca_p2_wave * $signed({1'b0, vca_p2_gain}) + 32'sd256;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (idx = 0; idx < 8; idx = idx + 1) begin
                v_fb_prev[idx]       <= 16'sd0;
                v_smooth_gain[idx]   <= 15'd0;
                voice_vca_out22[idx] <= 22'sd0;
            end
            for (idx = 0; idx < 6; idx = idx + 1) begin
                op_out_buf[idx] <= 16'sd0;
                v0_op_wave[idx] <= 16'sd0;
            end
            v0_fm_raw_wave  <= 16'sd0;
            v0_fm_env_wave  <= 16'sd0;
            car_bus_acc     <= 20'sd0;
            rom_phase_ad    <= 10'd0;
            rom_wave_sel    <= 2'd0;
            rom_y0_latch    <= 16'sd0;
            eff_op_gain_r   <= 14'd8192;
            v_target_gain_r <= 15'd0;
            vca_p1_vld      <= 1'b0;
            vca_p1_vid      <= 3'd0;
            vca_p1_gain     <= 15'd0;
            vca_p1_sum      <= 20'sd0;
            vca_p2_vld      <= 1'b0;
            vca_p2_vid      <= 3'd0;
            vca_p2_gain     <= 15'd0;
            vca_p2_wave     <= 16'sd0;
        end else begin
            vca_p1_vld <= 1'b0;

            case (tdm_sub)
                2'd0: begin
                    // 子拍 0：送出 10-bit 四舍五入相位地址 (与原版 BSRAM 读取节拍 100% 一致!)
                    rom_phase_ad    <= full_ph_18b[17:8] + {9'd0, full_ph_18b[7]};
                    rom_wave_sel    <= cur_op_wave;
                    eff_op_gain_r   <= (clamped_op_lv == 7'd100) ? 14'd8192 : (clamped_op_lv * 7'd82);
                    v_target_gain_r <= cur_v_env * v_mapped_vel;
                    if (tdm_step == 3'd0) begin
                        car_bus_acc <= 20'sd0;
                    end
                end
                2'd1: begin
                    // 子拍 1：BSRAM 内部锁存地址，同时平滑更新当前声部包络增益
                    if (tdm_step == 3'd0) begin
                        v_smooth_gain[tdm_vid] <= v_next_gain;
                    end
                end
                2'd2: begin
                    // 子拍 2：BSRAM 输出有效，直接打入独立触发器 rom_y0_latch (0 算术逻辑，彻底切断 25ns 路径!)
                    rom_y0_latch <= rom_wave_s16;
                end
                2'd3: begin
                    // 子拍 3：从触发器 rom_y0_latch 出发仅做 1 次算子音量乘法并累加至载波母线
                    op_out_buf[cur_op] <= op_scaled_s16;

                    if (tdm_vid == 3'd0) begin
                        v0_op_wave[cur_op] <= op_scaled_s16;
                    end

                    if ((fm_alg_sel == 5'd6 && cur_op == 3'd3) || (fm_alg_sel != 5'd6 && cur_op == 3'd5)) begin
                        v_fb_prev[tdm_vid] <= (v_fb_prev[tdm_vid] + rom_y0_latch) >>> 1;
                    end

                    if (cur_is_car) begin
                        car_bus_acc <= final_car_sum;
                    end

                    if (tdm_step == 3'd5) begin
                        vca_p1_vld  <= 1'b1;
                        vca_p1_vid  <= tdm_vid;
                        vca_p1_gain <= v_prev_gain;
                        vca_p1_sum  <= final_car_sum;
                    end
                end
            endcase

            vca_p2_vld <= vca_p1_vld;
            if (vca_p1_vld) begin
                vca_p2_vid  <= vca_p1_vid;
                vca_p2_gain <= vca_p1_gain;
                vca_p2_wave <= p1_wave_s16;
            end

            if (vca_p2_vld) begin
                voice_vca_out22[vca_p2_vid] <= p2_vca_prod[30:9];
                if (vca_p2_vid == 3'd0) begin
                    v0_fm_raw_wave <= vca_p2_wave;
                    v0_fm_env_wave <= p2_vca_prod[30:15];
                end
            end
        end
    end

    reg signed [24:0] sum22_0123, sum22_4567, raw_sum_25b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum22_0123  <= 25'sd0;
            sum22_4567  <= 25'sd0;
            raw_sum_25b <= 25'sd0;
        end else begin
            sum22_0123  <= $signed({{3{voice_vca_out22[0][21]}}, voice_vca_out22[0]}) +
                           $signed({{3{voice_vca_out22[1][21]}}, voice_vca_out22[1]}) +
                           $signed({{3{voice_vca_out22[2][21]}}, voice_vca_out22[2]}) +
                           $signed({{3{voice_vca_out22[3][21]}}, voice_vca_out22[3]});
            sum22_4567  <= $signed({{3{voice_vca_out22[4][21]}}, voice_vca_out22[4]}) +
                           $signed({{3{voice_vca_out22[5][21]}}, voice_vca_out22[5]}) +
                           $signed({{3{voice_vca_out22[6][21]}}, voice_vca_out22[6]}) +
                           $signed({{3{voice_vca_out22[7][21]}}, voice_vca_out22[7]});
            raw_sum_25b <= sum22_0123 + sum22_4567;
        end
    end

    reg signed [34:0] scaled_24b_prod;
    wire signed [27:0] shifted_24b    = scaled_24b_prod[34:7];
    wire signed [23:0] clamped_24b    = (shifted_24b > 28'sd8388607)  ? 24'sd8388607  :
                                        (shifted_24b < -28'sd8388608) ? -24'sd8388608 :
                                        shifted_24b[23:0];
    wire signed [24:0] rnd_to_16b     = $signed({clamped_24b[23], clamped_24b}) + 25'sd128;
    wire signed [16:0] rnd_to_16b_s17 = $signed(rnd_to_16b[24:8]);
    wire signed [15:0] clamped_16b    = (rnd_to_16b_s17 > 17'sd32767)  ? 16'sd32767  :
                                        (rnd_to_16b_s17 < -17'sd32768) ? -16'sd32768 :
                                        rnd_to_16b_s17[15:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scaled_24b_prod       <= 35'sd0;
            poly8_mixed_pcm24_out <= 24'sd0;
            poly8_mixed_pcm_out   <= 16'sd0;
            poly8_mixed_pcm_vld   <= 1'b0;
        end else begin
            scaled_24b_prod       <= raw_sum_25b * 10'sd128 + 35'sd64;
            poly8_mixed_pcm24_out <= clamped_24b;
            poly8_mixed_pcm_out   <= clamped_16b;
            poly8_mixed_pcm_vld   <= 1'b1;
        end
    end

    assign poly8_raw_sum25_out = raw_sum_25b;

endmodule
`endif