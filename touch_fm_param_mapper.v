`timescale 1ns / 1ps

module touch_fm_param_mapper (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [2:0]   car_op_id_in,
    input  wire [1:0]   car_wave_sel_in,
    input  wire [2:0]   mod_op_id_in,
    input  wire [1:0]   mod_wave_sel_in,
    input  wire [5:0]   carrier_mask_in,
    input  wire [4:0]   car_ratio_in,
    input  wire [4:0]   mod_ratio_in,
    input  wire [6:0]   car_level_in,
    input  wire [6:0]   mod_depth_in,
    input  wire [6:0]   car_detune_in,
    input  wire [6:0]   mod_feedback_in,
    input  wire [6:0]   car_init_phase_in,
    input  wire [6:0]   mod_init_phase_in,
    input  wire [31:0]  base_v0_ftw_in,
    input  wire [31:0]  base_v1_ftw_in,
    output reg  [31:0]  ch1_car_ftw_out,
    output reg  [31:0]  ch1_mod_ftw_out,
    output reg  [31:0]  ch2_car_ftw_out,
    output reg  [31:0]  ch2_mod_ftw_out,
    output reg  [9:0]   car_phase_offset_out,
    output reg  [9:0]   mod_phase_offset_out,
    output wire [11:0]  op_waves_flat_out,
    output wire [41:0]  op_levels_flat_out,
    output wire [59:0]  op_phases_flat_out,
    output wire [29:0]  op_ratios_flat_out,
    output wire [41:0]  op_det_fb_flat_out
);

    (* syn_ramstyle = "registers" *) reg [1:0] op_wave_mem   [0:5];
    (* syn_ramstyle = "registers" *) reg [6:0] op_level_mem  [0:5];
    (* syn_ramstyle = "registers" *) reg [9:0] op_phase_mem  [0:5];
    (* syn_ramstyle = "registers" *) reg [4:0] op_ratio_mem  [0:5];
    (* syn_ramstyle = "registers" *) reg [6:0] op_det_fb_mem [0:5];

    wire [16:0] car_pha_prod = car_init_phase_in * 10'd655;
    wire [16:0] mod_pha_prod = mod_init_phase_in * 10'd655;
    wire [9:0]  car_pha_10b  = car_pha_prod[15:6];
    wire [9:0]  mod_pha_10b  = mod_pha_prod[15:6];

    wire [2:0] c_idx = car_op_id_in - 3'd1;
    wire [2:0] m_idx = mod_op_id_in - 3'd1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 上电默认全部算子为纯净正弦波 (0)，OP1 音量为 85 (1倍频)，OP2~OP6 初始音量为 0，
            // 杜绝未调参前 OP3(4x锯齿波)+OP4/5/6 强行混入导致波形畸变!
            op_wave_mem[0]   <= 2'd0; op_wave_mem[1]   <= 2'd0; op_wave_mem[2]   <= 2'd0;
            op_wave_mem[3]   <= 2'd0; op_wave_mem[4]   <= 2'd0; op_wave_mem[5]   <= 2'd0;

            op_level_mem[0]  <= 7'd85; op_level_mem[1]  <= 7'd0;  op_level_mem[2]  <= 7'd0;
            op_level_mem[3]  <= 7'd0;  op_level_mem[4]  <= 7'd0;  op_level_mem[5]  <= 7'd0;

            op_phase_mem[0]  <= 10'd0; op_phase_mem[1]  <= 10'd0; op_phase_mem[2]  <= 10'd0;
            op_phase_mem[3]  <= 10'd0; op_phase_mem[4]  <= 10'd0; op_phase_mem[5]  <= 10'd0;

            op_ratio_mem[0]  <= 5'd1;  op_ratio_mem[1]  <= 5'd1;  op_ratio_mem[2]  <= 5'd1;
            op_ratio_mem[3]  <= 5'd1;  op_ratio_mem[4]  <= 5'd1;  op_ratio_mem[5]  <= 5'd1;

            op_det_fb_mem[0] <= 7'd50; op_det_fb_mem[1] <= 7'd0;  op_det_fb_mem[2] <= 7'd50;
            op_det_fb_mem[3] <= 7'd0;  op_det_fb_mem[4] <= 7'd0;  op_det_fb_mem[5] <= 7'd0;

            car_phase_offset_out <= 10'd0;
            mod_phase_offset_out <= 10'd0;
        end else begin
            if (car_op_id_in >= 3'd1 && car_op_id_in <= 3'd6) begin
                op_wave_mem[c_idx]   <= car_wave_sel_in;
                op_level_mem[c_idx]  <= car_level_in;
                op_phase_mem[c_idx]  <= car_pha_10b;
                op_ratio_mem[c_idx]  <= (car_ratio_in == 5'd0) ? 5'd1 : car_ratio_in;
                op_det_fb_mem[c_idx] <= car_detune_in;
            end
            if (mod_op_id_in >= 3'd1 && mod_op_id_in <= 3'd6) begin
                op_wave_mem[m_idx]   <= mod_wave_sel_in;
                op_level_mem[m_idx]  <= mod_depth_in;
                op_phase_mem[m_idx]  <= mod_pha_10b;
                op_ratio_mem[m_idx]  <= (mod_ratio_in == 5'd0) ? 5'd1 : mod_ratio_in;
                op_det_fb_mem[m_idx] <= mod_feedback_in;
            end

            car_phase_offset_out <= op_phase_mem[0];
            mod_phase_offset_out <= op_phase_mem[1];
        end
    end

    genvar i;
    generate
        for (i = 0; i < 6; i = i + 1) begin : GEN_FLAT_OP_PARAMS
            assign op_waves_flat_out[i*2  +: 2]   = op_wave_mem[i];
            assign op_levels_flat_out[i*7 +: 7]   = op_level_mem[i];
            assign op_phases_flat_out[i*10 +: 10] = op_phase_mem[i];
            assign op_ratios_flat_out[i*5 +: 5]   = op_ratio_mem[i];
            assign op_det_fb_flat_out[i*7 +: 7]   = op_det_fb_mem[i];
        end
    endgenerate

    reg [31:0] c1_rat_ftw, m1_rat_ftw, c2_rat_ftw, m2_rat_ftw;
    reg signed [7:0] det1_r, det3_r;

    wire signed [32:0] c1_det_sum = $signed({1'b0, c1_rat_ftw}) + ($signed({1'b0, c1_rat_ftw[31:12]}) * det1_r);
    wire signed [32:0] c2_det_sum = $signed({1'b0, c2_rat_ftw}) + ($signed({1'b0, c2_rat_ftw[31:12]}) * det3_r);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c1_rat_ftw      <= 32'd0;
            m1_rat_ftw      <= 32'd0;
            c2_rat_ftw      <= 32'd0;
            m2_rat_ftw      <= 32'd0;
            det1_r          <= 8'sd0;
            det3_r          <= 8'sd0;
            ch1_car_ftw_out <= 32'd0;
            ch1_mod_ftw_out <= 32'd0;
            ch2_car_ftw_out <= 32'd0;
            ch2_mod_ftw_out <= 32'd0;
        end else begin
            c1_rat_ftw  <= base_v0_ftw_in * op_ratio_mem[0];
            m1_rat_ftw  <= base_v0_ftw_in * op_ratio_mem[1];
            c2_rat_ftw  <= base_v0_ftw_in * op_ratio_mem[2];
            m2_rat_ftw  <= base_v0_ftw_in * op_ratio_mem[3];
            det1_r      <= $signed({1'b0, op_det_fb_mem[0]}) - 8'sd50;
            det3_r      <= $signed({1'b0, op_det_fb_mem[2]}) - 8'sd50;

            ch1_car_ftw_out <= c1_det_sum[31:0];
            ch1_mod_ftw_out <= m1_rat_ftw;
            ch2_car_ftw_out <= c2_det_sum[31:0];
            ch2_mod_ftw_out <= m2_rat_ftw;
        end
    end

endmodule