`timescale 1ns / 1ps
`ifndef FM_SYNTH_CORE_V
`define FM_SYNTH_CORE_V

module fmusic_op_quad_wave_rom (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [9:0]         phase_ad,
    input  wire [1:0]         wave_sel,
    output wire signed [15:0] wave_s16_out
);
    wire gowin_reset = ~rst_n;
    wire [15:0] raw_sin_u16;
    wire [15:0] raw_tri_u16;
    wire [15:0] raw_saw_u16;
    wire [15:0] raw_sqr_u16;

    Gowin_pROM_sine u_rom_sine (
        .dout  (raw_sin_u16),
        .clk   (clk),
        .oce   (1'b1),
        .ce    (1'b1),
        .reset (gowin_reset),
        .ad    (phase_ad)
    );
    Gowin_pROM_triangular_wave u_rom_tri (
        .dout  (raw_tri_u16),
        .clk   (clk),
        .oce   (1'b1),
        .ce    (1'b1),
        .reset (gowin_reset),
        .ad    (phase_ad)
    );
    Gowin_pROM_sawtooth_wave u_rom_saw (
        .dout  (raw_saw_u16),
        .clk   (clk),
        .oce   (1'b1),
        .ce    (1'b1),
        .reset (gowin_reset),
        .ad    (phase_ad)
    );
    Gowin_pROM_square_wave u_rom_sqr (
        .dout  (raw_sqr_u16),
        .clk   (clk),
        .oce   (1'b1),
        .ce    (1'b1),
        .reset (gowin_reset),
        .ad    (phase_ad)
    );

    wire signed [15:0] w_sin_s16 = {~raw_sin_u16[15], raw_sin_u16[14:0]};
    wire signed [15:0] w_tri_s16 = {~raw_tri_u16[15], raw_tri_u16[14:0]};
    // 兼容 10-bit (0x0000~0x03FF) 与 16-bit 锯齿波 ROM 数据，消除负直流偏置
    wire signed [15:0] w_saw_s16 = (|raw_saw_u16[15:10]) ?
                                   {~raw_saw_u16[15], raw_saw_u16[14:0]} :
                                   {~raw_saw_u16[9],  raw_saw_u16[8:0], 6'b000000};
    wire signed [15:0] w_sqr_s16 = (raw_sqr_u16 == 16'h0000 || raw_sqr_u16 == 16'h8000) ?
                                   -16'sd32767 : {~raw_sqr_u16[15], raw_sqr_u16[14:0]};

    assign wave_s16_out = (wave_sel == 2'd0) ? w_sin_s16 :
                          (wave_sel == 2'd1) ? w_sqr_s16 :
                          (wave_sel == 2'd2) ? w_saw_s16 :
                                               w_tri_s16;
endmodule

module fm_synth_core #(
    parameter [1:0] WAVE_SINE          = 2'd0,
    parameter [1:0] WAVE_SQUARE        = 2'd1,
    parameter [1:0] WAVE_SAWTOOTH      = 2'd2,
    parameter [1:0] WAVE_TRIANGLE      = 2'd3,
    parameter [2:0] MODE_FM_STD        = 3'd0,
    parameter [2:0] MODE_FM_DEEP       = 3'd1,
    parameter [2:0] MODE_AM_MOD        = 3'd2,
    parameter [2:0] MODE_RING_MOD      = 3'd3,
    parameter [2:0] MODE_ADD_MIX       = 3'd4,
    parameter [2:0] MODE_SUB_DIFF      = 3'd5,
    parameter       USE_DYN_UI         = 1,
    parameter       USE_EXT_WAVE_MAP   = 0,
    parameter [2:0] DEFAULT_MOD_MODE   = MODE_FM_STD,
    parameter [1:0] DEFAULT_CAR_WAVE   = WAVE_SINE,
    parameter [1:0] DEFAULT_MOD_WAVE   = WAVE_TRIANGLE,
    parameter [6:0] DEFAULT_MOD_DEPTH  = 7'd75,
    parameter [6:0] DEFAULT_FEEDBACK   = 7'd30,
    parameter [6:0] DEFAULT_CAR_LEVEL  = 7'd85
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] ch1_mod_freq_ctrl,
    input  wire [31:0] ch1_car_freq_ctrl,
    input  wire [31:0] ch2_mod_freq_ctrl,
    input  wire [31:0] ch2_car_freq_ctrl,
    input  wire [9:0]  car_phase_offset_in,
    input  wire [9:0]  mod_phase_offset_in,
    input  wire [11:0] op_waves_flat_in,
    input  wire [2:0]  car_op_id,
    input  wire [2:0]  mod_op_id,
    input  wire [5:0]  carrier_mask_in,
    input  wire [2:0]  mod_mode_sel,
    input  wire [1:0]  car_wave_sel,
    input  wire [1:0]  mod_wave_sel,
    input  wire [6:0]  mod_depth_pct,
    input  wire [6:0]  feedback_pct,
    input  wire [6:0]  car_level_pct,
    input  wire [9:0]  op1_phase_mod_in,
    input  wire [9:0]  op2_phase_mod_in,
    input  wire [9:0]  op3_phase_mod_in,
    input  wire [9:0]  op4_phase_mod_in,
    input  wire [9:0]  op5_phase_mod_in,
    input  wire [9:0]  op6_phase_mod_in,
    output wire signed [15:0] ch1_car_raw_out,
    output wire signed [15:0] ch1_mod_raw_out,
    output wire signed [15:0] ch2_car_raw_out,
    output wire signed [15:0] ch2_mod_raw_out,
    output wire signed [15:0] op5_raw_out,
    output wire signed [15:0] op6_raw_out,
    output wire signed [15:0] ch1_fm_out,
    output wire signed [15:0] ch2_fm_out
);
    wire gowin_reset = ~rst_n;
    wire [2:0] w_eff_mod_mode  = USE_DYN_UI ? mod_mode_sel  : DEFAULT_MOD_MODE;
    wire [1:0] w_eff_car_wave  = USE_DYN_UI ? car_wave_sel  : DEFAULT_CAR_WAVE;
    wire [1:0] w_eff_mod_wave  = USE_DYN_UI ? mod_wave_sel  : DEFAULT_MOD_WAVE;
    wire [6:0] w_eff_mod_depth = USE_DYN_UI ? ((mod_depth_pct > 7'd100) ? 7'd100 : mod_depth_pct) : DEFAULT_MOD_DEPTH;
    wire [6:0] w_eff_feedback  = USE_DYN_UI ? ((feedback_pct  > 7'd100) ? 7'd100 : feedback_pct)  : DEFAULT_FEEDBACK;
    wire [6:0] w_eff_car_level = USE_DYN_UI ? ((car_level_pct > 7'd100) ? 7'd100 : car_level_pct) : DEFAULT_CAR_LEVEL;
    wire [5:0] w_eff_mask      = (carrier_mask_in == 6'd0) ? 6'b000101 : carrier_mask_in;

    wire [13:0]        w_mod_gain_u14 = (w_eff_mod_depth == 7'd100) ? 14'd8192 : (w_eff_mod_depth * 7'd82);
    wire signed [14:0] w_mod_gain_q13 = $signed({1'b0, w_mod_gain_u14});
    wire [13:0]        w_car_gain_u14 = (w_eff_car_level == 7'd100) ? 14'd8192 : (w_eff_car_level * 7'd82);
    wire signed [14:0] w_car_gain_q13 = $signed({1'b0, w_car_gain_u14});

    (* syn_ramstyle = "registers" *)
    reg [1:0] op_wave_sel [0:5];
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            op_wave_sel[0] <= WAVE_SINE;
            op_wave_sel[1] <= WAVE_TRIANGLE;
            op_wave_sel[2] <= WAVE_SINE;
            op_wave_sel[3] <= WAVE_SAWTOOTH;
            op_wave_sel[4] <= WAVE_TRIANGLE;
            op_wave_sel[5] <= WAVE_SAWTOOTH;
        end else if (USE_EXT_WAVE_MAP) begin
            for (i = 0; i < 6; i = i + 1) begin
                op_wave_sel[i] <= op_waves_flat_in[i*2 +: 2];
            end
        end else begin
            for (i = 0; i < 6; i = i + 1) begin
                if (car_op_id == (i + 1))      op_wave_sel[i] <= w_eff_car_wave;
                else if (mod_op_id == (i + 1)) op_wave_sel[i] <= w_eff_mod_wave;
                else if (w_eff_mask[i])        op_wave_sel[i] <= w_eff_car_wave;
                else                           op_wave_sel[i] <= w_eff_mod_wave;
            end
        end
    end

    reg [31:0] ch1_mod_acc;
    reg [31:0] ch2_mod_acc;
    reg [31:0] op5_acc;
    reg [31:0] op6_acc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch1_mod_acc <= 32'd0;
            ch2_mod_acc <= 32'd0;
            op5_acc     <= 32'd0;
            op6_acc     <= 32'd0;
        end else begin
            ch1_mod_acc <= ch1_mod_acc + ch1_mod_freq_ctrl;
            ch2_mod_acc <= ch2_mod_acc + ch2_mod_freq_ctrl;
            op5_acc     <= op5_acc     + (ch1_car_freq_ctrl << 1);
            op6_acc     <= op6_acc     + (ch1_mod_freq_ctrl << 1);
        end
    end

    wire [9:0]  ch1_mod_base_ad = ch1_mod_acc[31:22] + {9'd0, ch1_mod_acc[21]} + mod_phase_offset_in;
    wire [15:0] raw_fb_sin_u16;
    Gowin_pROM_sine u_mod1_fb_rom (
        .dout  (raw_fb_sin_u16),
        .clk   (clk),
        .oce   (1'b1),
        .ce    (1'b1),
        .reset (gowin_reset),
        .ad    (ch1_mod_base_ad)
    );
    wire signed [15:0] fb_base_wave     = {~raw_fb_sin_u16[15], raw_fb_sin_u16[14:0]};
    wire signed [23:0] fb_mult_prod     = fb_base_wave * $signed({1'b0, w_eff_feedback});
    wire [9:0]         ch1_mod_phase_fb = (op_wave_sel[1] == WAVE_SQUARE || op_wave_sel[1] == WAVE_SAWTOOTH) ?
                                          10'd0 : fb_mult_prod[23:14];
    wire               is_fm_phase_mode = (w_eff_mod_mode == MODE_FM_STD) || (w_eff_mod_mode == MODE_FM_DEEP);

    wire [9:0] ch1_mod_addr = ch1_mod_base_ad + ch1_mod_phase_fb + (is_fm_phase_mode ? op2_phase_mod_in : 10'd0);
    wire [9:0] ch2_mod_addr = ch2_mod_acc[31:22] + {9'd0, ch2_mod_acc[21]} + mod_phase_offset_in + (is_fm_phase_mode ? op4_phase_mod_in : 10'd0);
    wire [9:0] op5_addr     = op5_acc[31:22]     + {9'd0, op5_acc[21]}     + mod_phase_offset_in + (is_fm_phase_mode ? op5_phase_mod_in : 10'd0);
    wire [9:0] op6_addr     = op6_acc[31:22]     + {9'd0, op6_acc[21]}     + mod_phase_offset_in + (is_fm_phase_mode ? op6_phase_mod_in : 10'd0);

    wire signed [15:0] w_selected_ch1_mod;
    wire signed [15:0] w_selected_ch2_mod;
    wire signed [15:0] w_selected_op5;
    wire signed [15:0] w_selected_op6;

    fmusic_op_quad_wave_rom u_mod1_wave_rom (
        .clk          (clk),
        .rst_n        (rst_n),
        .phase_ad     (ch1_mod_addr),
        .wave_sel     (op_wave_sel[1]),
        .wave_s16_out (w_selected_ch1_mod)
    );
    fmusic_op_quad_wave_rom u_mod2_wave_rom (
        .clk          (clk),
        .rst_n        (rst_n),
        .phase_ad     (ch2_mod_addr),
        .wave_sel     (op_wave_sel[3]),
        .wave_s16_out (w_selected_ch2_mod)
    );
    fmusic_op_quad_wave_rom u_op5_wave_rom (
        .clk          (clk),
        .rst_n        (rst_n),
        .phase_ad     (op5_addr),
        .wave_sel     (op_wave_sel[4]),
        .wave_s16_out (w_selected_op5)
    );
    fmusic_op_quad_wave_rom u_op6_wave_rom (
        .clk          (clk),
        .rst_n        (rst_n),
        .phase_ad     (op6_addr),
        .wave_sel     (op_wave_sel[5]),
        .wave_s16_out (w_selected_op6)
    );

    wire signed [30:0] w_mod_dep_prod_s0  = w_selected_ch1_mod * w_mod_gain_q13 + 31'sd4096;
    wire signed [30:0] w_mod2_dep_prod_s0 = w_selected_ch2_mod * w_mod_gain_q13 + 31'sd4096;
    reg signed [15:0] mod_dep_s0_r, mod2_dep_s0_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mod_dep_s0_r  <= 16'sd0;
            mod2_dep_s0_r <= 16'sd0;
        end else begin
            mod_dep_s0_r  <= w_mod_dep_prod_s0[28:13];
            mod2_dep_s0_r <= w_mod2_dep_prod_s0[28:13];
        end
    end

    wire signed [32:0] w_car1_ftw_s  = $signed({1'b0, ch1_car_freq_ctrl});
    wire signed [48:0] w_fm1_prod    = w_car1_ftw_s * mod_dep_s0_r;
    wire signed [32:0] w_car2_ftw_s  = $signed({1'b0, ch2_car_freq_ctrl});
    wire signed [48:0] w_fm2_prod    = w_car2_ftw_s * mod2_dep_s0_r;
    reg signed [33:0] w_fm1_dev_r, w_fm2_dev_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_fm1_dev_r <= 34'sd0;
            w_fm2_dev_r <= 34'sd0;
        end else begin
            w_fm1_dev_r <= $signed({w_fm1_prod[48], w_fm1_prod[47:15]}) +
                           $signed({{2{w_fm1_prod[48]}}, w_fm1_prod[47:16]});
            w_fm2_dev_r <= $signed({w_fm2_prod[48], w_fm2_prod[47:15]}) +
                           $signed({{2{w_fm2_prod[48]}}, w_fm2_prod[47:16]});
        end
    end

    wire signed [33:0] w_fm1_delta    = (w_eff_mod_mode == MODE_FM_DEEP) ? (w_fm1_dev_r <<< 1) : w_fm1_dev_r;
    wire signed [33:0] w_inst_ftw1_s  = $signed({1'b0, w_car1_ftw_s}) + w_fm1_delta;
    wire [31:0]        w_eff_car1_ftw = !is_fm_phase_mode ? ch1_car_freq_ctrl : w_inst_ftw1_s[31:0];

    wire signed [33:0] w_fm2_delta    = (w_eff_mod_mode == MODE_FM_DEEP) ? (w_fm2_dev_r <<< 1) : w_fm2_dev_r;
    wire signed [33:0] w_inst_ftw2_s  = $signed({1'b0, w_car2_ftw_s}) + w_fm2_delta;
    wire [31:0]        w_eff_car2_ftw = !is_fm_phase_mode ? ch2_car_freq_ctrl : w_inst_ftw2_s[31:0];

    reg [31:0] ch1_car_acc,    ch2_car_acc;
    reg [31:0] ch1_car_fm_acc, ch2_car_fm_acc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch1_car_acc    <= 32'd0;
            ch2_car_acc    <= 32'd0;
            ch1_car_fm_acc <= 32'd0;
            ch2_car_fm_acc <= 32'd0;
        end else begin
            ch1_car_acc    <= ch1_car_acc    + ch1_car_freq_ctrl;
            ch2_car_acc    <= ch2_car_acc    + ch2_car_freq_ctrl;
            ch1_car_fm_acc <= ch1_car_fm_acc + w_eff_car1_ftw;
            ch2_car_fm_acc <= ch2_car_fm_acc + w_eff_car2_ftw;
        end
    end

    wire [9:0] ch1_car_fm_addr = ch1_car_fm_acc[31:22] + {9'd0, ch1_car_fm_acc[21]} + car_phase_offset_in + (is_fm_phase_mode ? op1_phase_mod_in : 10'd0);
    wire [9:0] ch2_car_fm_addr = ch2_car_fm_acc[31:22] + {9'd0, ch2_car_fm_acc[21]} + car_phase_offset_in + (is_fm_phase_mode ? op3_phase_mod_in : 10'd0);

    wire signed [15:0] w_car1_wave;
    wire signed [15:0] w_car2_wave;
    fmusic_op_quad_wave_rom u_car1_wave_rom (
        .clk          (clk),
        .rst_n        (rst_n),
        .phase_ad     (ch1_car_fm_addr),
        .wave_sel     (op_wave_sel[0]),
        .wave_s16_out (w_car1_wave)
    );
    fmusic_op_quad_wave_rom u_car2_wave_rom (
        .clk          (clk),
        .rst_n        (rst_n),
        .phase_ad     (ch2_car_fm_addr),
        .wave_sel     (op_wave_sel[2]),
        .wave_s16_out (w_car2_wave)
    );

    assign ch1_car_raw_out = w_car1_wave;
    assign ch1_mod_raw_out = w_selected_ch1_mod;
    assign ch2_car_raw_out = w_car2_wave;
    assign ch2_mod_raw_out = w_selected_ch2_mod;
    assign op5_raw_out     = w_selected_op5;
    assign op6_raw_out     = w_selected_op6;

    wire signed [15:0] w_mod1_scaled   = w_mod_dep_prod_s0[28:13];
    wire signed [31:0] w_ring_prod     = w_car1_wave * w_mod1_scaled + 32'sd16384;
    wire signed [15:0] w_ring_s16      = w_ring_prod[30:15];
    wire signed [15:0] w_am_s16        = (w_car1_wave >>> 1) + (w_ring_s16 >>> 1);
    wire signed [15:0] w_add_s16       = (w_car1_wave >>> 1) + (w_mod1_scaled >>> 1);
    wire signed [15:0] w_sub_s16       = (w_car1_wave >>> 1) - (w_mod1_scaled >>> 1);
    wire signed [15:0] w_mode_out1     = (w_eff_mod_mode == MODE_AM_MOD)   ? w_am_s16   :
                                         (w_eff_mod_mode == MODE_RING_MOD) ? w_ring_s16 :
                                         (w_eff_mod_mode == MODE_ADD_MIX)  ? w_add_s16  :
                                         (w_eff_mod_mode == MODE_SUB_DIFF) ? w_sub_s16  :
                                                                             w_car1_wave;

    wire signed [15:0] w_mod2_scaled   = w_mod2_dep_prod_s0[28:13];
    wire signed [31:0] w_ring2_prod    = w_car2_wave * w_mod2_scaled + 32'sd16384;
    wire signed [15:0] w_ring2_s16     = w_ring2_prod[30:15];
    wire signed [15:0] w_am2_s16       = (w_car2_wave >>> 1) + (w_ring2_s16 >>> 1);
    wire signed [15:0] w_add2_s16      = (w_car2_wave >>> 1) + (w_mod2_scaled >>> 1);
    wire signed [15:0] w_sub2_s16      = (w_car2_wave >>> 1) - (w_mod2_scaled >>> 1);
    wire signed [15:0] w_mode_out2     = (w_eff_mod_mode == MODE_AM_MOD)   ? w_am2_s16   :
                                         (w_eff_mod_mode == MODE_RING_MOD) ? w_ring2_s16 :
                                         (w_eff_mod_mode == MODE_ADD_MIX)  ? w_add2_s16  :
                                         (w_eff_mod_mode == MODE_SUB_DIFF) ? w_sub2_s16  :
                                                                             w_car2_wave;

    wire signed [30:0] w_car1_vol_prod = w_mode_out1 * w_car_gain_q13 + 31'sd4096;
    wire signed [30:0] w_car2_vol_prod = w_mode_out2 * w_car_gain_q13 + 31'sd4096;
    assign ch1_fm_out = w_car1_vol_prod[28:13];
    assign ch2_fm_out = w_car2_vol_prod[28:13];
endmodule
`endif