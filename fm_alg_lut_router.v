`timescale 1ns / 1ps

// ============================================================================
// 模块名称: fm_alg_lut_router (16种 opsix 调制拓扑扩展 + 全算子独立路由 + 无偏偶舍入版)
// 功能描述: 
//   1. 内置 16 种经典 opsix/DX7 FM 调制拓扑组合 (对应电容屏第 1 页的 2 页 x 8 种算法: Alg 1 ~ Alg 16)
//   2. 采用 36-bit 全独立算子路由指令字：为 OP1~OP5 每个算子提供完全独立的调制源选择字段
//   3. 支持单路串联、Y型双源汇聚 (4+5 / 5+6 / 2+3)、三路树状全汇聚 (2+3+5 / 2+3+4)、一对多广播等全部拓扑
//   4. 采用 IEEE 754 无偏偶舍入 (Round-Half-to-Even)，彻底消除直流偏置 (DC Offset)
// ============================================================================
module fm_alg_lut_router #(
    // ------------------------------------------------------------------------
    // 【1. 16 种调制组合编号 Parameter 字典 (Page 1: Alg 1~8, Page 2: Alg 9~16)】
    // ------------------------------------------------------------------------
    parameter [4:0] ALG_1_DUAL_TOWER    = 5'd1,  // opsix #1 : [2->1] + [6(FB)->5->4->3]
    parameter [4:0] ALG_2_TRIPLE_PAIR   = 5'd2,  // opsix #5 : [2->1] + [4->3] + [6(FB)->5]
    parameter [4:0] ALG_3_Y_CONVERGE    = 5'd3,  // opsix #7 : [2->1] + [(4 + 6(FB)->5)->3]
    parameter [4:0] ALG_4_TREE_BELL     = 5'd4,  // opsix #16: [(2 + 4->3 + 6(FB)->5)->1]
    parameter [4:0] ALG_5_BROADCAST     = 5'd5,  // opsix #19: [3->2->1] + [6(FB)->(4,5)]
    parameter [4:0] ALG_6_LOOP_LEAD     = 5'd6,  // opsix #4 : [3->2->1] + [6->5->4(跨级反馈回6)]
    parameter [4:0] ALG_7_HYBRID_ORGAN  = 5'd7,  // opsix #31: 1 + 2 + 3 + 4 + [6(FB)->5]
    parameter [4:0] ALG_8_PURE_ADD      = 5'd8,  // opsix #32: 1 + 2 + 3 + 4 + 5 + 6(FB)
    parameter [4:0] ALG_9_SYM_TOWERS    = 5'd9,  // opsix #3 : [3->2->1] + [6(FB)->5->4]
    parameter [4:0] ALG_10_TOWER_Y      = 5'd10, // opsix #11: [3->2->1] + [(5 + 6(FB))->4]
    parameter [4:0] ALG_11_TOP_Y_CHAIN  = 5'd11, // opsix #14: [2->1] + [(5 + 6(FB))->4->3]
    parameter [4:0] ALG_12_DEEP_TREE    = 5'd12, // opsix #18: [(2 + 3 + 6(FB)->5->4)->1]
    parameter [4:0] ALG_13_PAD_BCAST    = 5'd13, // opsix #22: 1 + [2->3] + [6(FB)->(4,5)]
    parameter [4:0] ALG_14_TRIPLE_HYB   = 5'd14, // opsix #26: 1 + [3->2] + [(5 + 6(FB))->4]
    parameter [4:0] ALG_15_PARALLEL_TWR = 5'd15, // opsix #37: [4->3->1] + [6(FB)->5->2]
    parameter [4:0] ALG_16_MASTER_CHOIR = 5'd16, // opsix #40: [6(FB)->5->(1, 2, 3, 4)]

    // ------------------------------------------------------------------------
    // 【2. 路由源选择编码 Parameter (定义每个算子顶部接收谁的调制信号)】
    // ------------------------------------------------------------------------
    parameter [3:0] SRC_NONE            = 4'd0,  // 无外部调制 (纯载波或顶层调制器)
    parameter [3:0] SRC_OP2             = 4'd1,  // 接收 Op2 调制
    parameter [3:0] SRC_OP3             = 4'd2,  // 接收 Op3 调制
    parameter [3:0] SRC_OP4             = 4'd3,  // 接收 Op4 调制
    parameter [3:0] SRC_OP5             = 4'd4,  // 接收 Op5 调制
    parameter [3:0] SRC_OP6             = 4'd5,  // 接收 Op6 调制
    parameter [3:0] SRC_OP4_ADD_OP5     = 4'd6,  // Y型双源汇聚: 同时接收 (Op4 + Op5) 叠加调制
    parameter [3:0] SRC_OP2_OP3_OP5     = 4'd7,  // 树状三路全汇聚: 同时接收 (Op2 + Op3 + Op5) 叠加调制
    parameter [3:0] SRC_OP2_ADD_OP3     = 4'd8,  // 双源汇聚: 同时接收 (Op2 + Op3) 叠加调制
    parameter [3:0] SRC_OP5_ADD_OP6     = 4'd9,  // 顶层Y型汇聚: 同时接收 (Op5 + Op6) 叠加调制
    parameter [3:0] SRC_OP2_OP3_OP4     = 4'd10, // 树状三路全汇聚: 同时接收 (Op2 + Op3 + Op4) 叠加调制

    // ------------------------------------------------------------------------
    // 【3. 反馈类型编码 Parameter】
    // ------------------------------------------------------------------------
    parameter [1:0] FB_NONE             = 2'b00, // 无反馈
    parameter [1:0] FB_SELF_OP6         = 2'b01, // Op6 自身右上角回环反馈
    parameter [1:0] FB_LOOP_OP4_TO_OP6  = 2'b10, // Op4 跨三级回环反馈至 Op6 (opsix #4)
    parameter [1:0] FB_LOOP_OP5_TO_OP6  = 2'b11, // Op5 跨二级回环反馈至 Op6

    // ------------------------------------------------------------------------
    // 【4. 36-bit 全独立算子查找表编码常量定义】
    // 格式: {car_gain[3:0], mod_scale[3:0], fb_type[1:0],
    //        r_op1[3:0], r_op2[3:0], r_op3[3:0], r_op4[3:0], r_op5[3:0], car_mask[5:0]}
    // ------------------------------------------------------------------------
    // --- 第 1 页 (Alg 1 ~ Alg 8) ---
    parameter [35:0] LUT_WORD_ALG1  = {4'd5, 4'd8, FB_SELF_OP6,        SRC_OP2,         SRC_NONE, SRC_OP4,         SRC_OP5,         SRC_OP6,  6'b000101},
    parameter [35:0] LUT_WORD_ALG2  = {4'd3, 4'd8, FB_SELF_OP6,        SRC_OP2,         SRC_NONE, SRC_OP4,         SRC_NONE,        SRC_OP6,  6'b010101},
    parameter [35:0] LUT_WORD_ALG3  = {4'd5, 4'd6, FB_SELF_OP6,        SRC_OP2,         SRC_NONE, SRC_OP4_ADD_OP5, SRC_NONE,        SRC_OP6,  6'b000101},
    parameter [35:0] LUT_WORD_ALG4  = {4'd8, 4'd5, FB_SELF_OP6,        SRC_OP2_OP3_OP5, SRC_NONE, SRC_OP4,         SRC_NONE,        SRC_OP6,  6'b000001},
    parameter [35:0] LUT_WORD_ALG5  = {4'd3, 4'd7, FB_SELF_OP6,        SRC_OP2,         SRC_OP3,  SRC_NONE,        SRC_OP6,         SRC_OP6,  6'b011001},
    parameter [35:0] LUT_WORD_ALG6  = {4'd5, 4'd7, FB_LOOP_OP4_TO_OP6, SRC_OP2,         SRC_OP3,  SRC_NONE,        SRC_OP5,         SRC_OP6,  6'b001001},
    parameter [35:0] LUT_WORD_ALG7  = {4'd2, 4'd8, FB_SELF_OP6,        SRC_NONE,        SRC_NONE, SRC_NONE,        SRC_NONE,        SRC_OP6,  6'b011111},
    parameter [35:0] LUT_WORD_ALG8  = {4'd2, 4'd8, FB_SELF_OP6,        SRC_NONE,        SRC_NONE, SRC_NONE,        SRC_NONE,        SRC_NONE, 6'b111111},
    // --- 第 2 页 (Alg 9 ~ Alg 16) ---
    parameter [35:0] LUT_WORD_ALG9  = {4'd5, 4'd8, FB_SELF_OP6,        SRC_OP2,         SRC_OP3,  SRC_NONE,        SRC_OP5,         SRC_OP6,  6'b001001},
    parameter [35:0] LUT_WORD_ALG10 = {4'd5, 4'd7, FB_SELF_OP6,        SRC_OP2,         SRC_OP3,  SRC_NONE,        SRC_OP5_ADD_OP6, SRC_NONE, 6'b001001},
    parameter [35:0] LUT_WORD_ALG11 = {4'd5, 4'd7, FB_SELF_OP6,        SRC_OP2,         SRC_NONE, SRC_OP4,         SRC_OP5_ADD_OP6, SRC_NONE, 6'b000101},
    parameter [35:0] LUT_WORD_ALG12 = {4'd8, 4'd5, FB_SELF_OP6,        SRC_OP2_OP3_OP4, SRC_NONE, SRC_NONE,        SRC_OP5,         SRC_OP6,  6'b000001},
    parameter [35:0] LUT_WORD_ALG13 = {4'd3, 4'd7, FB_SELF_OP6,        SRC_NONE,        SRC_NONE, SRC_OP2,         SRC_OP6,         SRC_OP6,  6'b011101},
    parameter [35:0] LUT_WORD_ALG14 = {4'd3, 4'd7, FB_SELF_OP6,        SRC_NONE,        SRC_OP3,  SRC_NONE,        SRC_OP5_ADD_OP6, SRC_NONE, 6'b001011},
    parameter [35:0] LUT_WORD_ALG15 = {4'd5, 4'd8, FB_SELF_OP6,        SRC_OP3,         SRC_OP5,  SRC_OP4,         SRC_NONE,        SRC_OP6,  6'b000011},
    parameter [35:0] LUT_WORD_ALG16 = {4'd3, 4'd7, FB_SELF_OP6,        SRC_OP5,         SRC_OP5,  SRC_OP5,         SRC_OP5,         SRC_OP6,  6'b001111}
)(
    input  wire        clk,
    input  wire        rst_n,

    // 接口 1: 来自电容屏 UI 的控制信号输入 (支持 5-bit 1~16，同时向下兼容 4-bit 连接)
    input  wire [4:0]  alg_sel_in,        // 来自 C0[4:0]: 调制组合选择 (1 ~ 16)
    input  wire [6:0]  mod_depth_in,      // 来自 C4[6:0]: 全局调制深度百分比 (0 ~ 100)
    input  wire [6:0]  feedback_in,       // 来自 C5[6:0]: 顶层算子反馈量百分比 (0 ~ 100)

    // 接口 2: 6 个算子当前拍经各自独立音量/深度缩放后的 16-bit 有符号波形输入
    input  wire signed [15:0] op1_raw_wave,
    input  wire signed [15:0] op2_raw_wave,
    input  wire signed [15:0] op3_raw_wave,
    input  wire signed [15:0] op4_raw_wave,
    input  wire signed [15:0] op5_raw_wave,
    input  wire signed [15:0] op6_raw_wave,

    // 接口 3: 查找表译码出的 wire 状态信号
    output wire [5:0]  w_carrier_mask,    // 哪几个算子是品红色发声算子 (Bit0~5 对应 Op1~Op6)
    output wire [1:0]  w_fb_type,         // 当前反馈类型 (01:Op6自反馈, 10:Op4->Op6跨级环路反馈)
    output wire [3:0]  w_car_gain_weight, // 当前多载波防爆音归一化权重

    // 接口 4: 分发给 6 个算子 DDS 读表地址端的 10-bit 动态相位调制偏移量输出
    output wire [9:0]  op1_phase_mod_out,
    output wire [9:0]  op2_phase_mod_out,
    output wire [9:0]  op3_phase_mod_out,
    output wire [9:0]  op4_phase_mod_out,
    output wire [9:0]  op5_phase_mod_out,
    output wire [9:0]  op6_phase_mod_out,

    // 接口 5: 经发声算子掩码汇总、无偏偶舍入并防爆音饱和钳位后的 16-bit 合成波形输出
    output reg  signed [15:0] routed_audio_out
);

    // =========================================================================
    // 第一部分：16 种调制拓扑组合核心查找表 (Algorithm Lookup Table)
    // =========================================================================
    reg [35:0] lut_raw_word;

    always @(*) begin
        case (alg_sel_in)
            ALG_1_DUAL_TOWER   : lut_raw_word = LUT_WORD_ALG1;
            ALG_2_TRIPLE_PAIR  : lut_raw_word = LUT_WORD_ALG2;
            ALG_3_Y_CONVERGE   : lut_raw_word = LUT_WORD_ALG3;
            ALG_4_TREE_BELL    : lut_raw_word = LUT_WORD_ALG4;
            ALG_5_BROADCAST    : lut_raw_word = LUT_WORD_ALG5;
            ALG_6_LOOP_LEAD    : lut_raw_word = LUT_WORD_ALG6;
            ALG_7_HYBRID_ORGAN : lut_raw_word = LUT_WORD_ALG7;
            ALG_8_PURE_ADD     : lut_raw_word = LUT_WORD_ALG8;
            ALG_9_SYM_TOWERS   : lut_raw_word = LUT_WORD_ALG9;
            ALG_10_TOWER_Y     : lut_raw_word = LUT_WORD_ALG10;
            ALG_11_TOP_Y_CHAIN : lut_raw_word = LUT_WORD_ALG11;
            ALG_12_DEEP_TREE   : lut_raw_word = LUT_WORD_ALG12;
            ALG_13_PAD_BCAST   : lut_raw_word = LUT_WORD_ALG13;
            ALG_14_TRIPLE_HYB  : lut_raw_word = LUT_WORD_ALG14;
            ALG_15_PARALLEL_TWR: lut_raw_word = LUT_WORD_ALG15;
            ALG_16_MASTER_CHOIR: lut_raw_word = LUT_WORD_ALG16;
            default            : lut_raw_word = LUT_WORD_ALG1;
        endcase
    end

    // =========================================================================
    // 第二部分：使用 wire 实时解包 36-bit 拓扑指令字
    // =========================================================================
    assign w_car_gain_weight   = lut_raw_word[35:32];
    wire [3:0] w_mod_idx_scale = lut_raw_word[31:28];
    assign w_fb_type           = lut_raw_word[27:26];
    wire [3:0] w_route_op1     = lut_raw_word[25:22];
    wire [3:0] w_route_op2     = lut_raw_word[21:18];
    wire [3:0] w_route_op3     = lut_raw_word[17:14];
    wire [3:0] w_route_op4     = lut_raw_word[13:10];
    wire [3:0] w_route_op5     = lut_raw_word[9:6];
    assign w_carrier_mask      = lut_raw_word[5:0];

    // =========================================================================
    // 第三部分：通用算子调制源路由多路复用函数
    // =========================================================================
    wire signed [17:0] s_op1 = $signed({{2{op1_raw_wave[15]}}, op1_raw_wave});
    wire signed [17:0] s_op2 = $signed({{2{op2_raw_wave[15]}}, op2_raw_wave});
    wire signed [17:0] s_op3 = $signed({{2{op3_raw_wave[15]}}, op3_raw_wave});
    wire signed [17:0] s_op4 = $signed({{2{op4_raw_wave[15]}}, op4_raw_wave});
    wire signed [17:0] s_op5 = $signed({{2{op5_raw_wave[15]}}, op5_raw_wave});
    wire signed [17:0] s_op6 = $signed({{2{op6_raw_wave[15]}}, op6_raw_wave});

    function signed [17:0] select_mod_source(
        input [3:0]         route_code,
        input signed [17:0] w2,
        input signed [17:0] w3,
        input signed [17:0] w4,
        input signed [17:0] w5,
        input signed [17:0] w6
    );
        begin
            case (route_code)
                SRC_OP2        : select_mod_source = w2;
                SRC_OP3        : select_mod_source = w3;
                SRC_OP4        : select_mod_source = w4;
                SRC_OP5        : select_mod_source = w5;
                SRC_OP6        : select_mod_source = w6;
                SRC_OP4_ADD_OP5: select_mod_source = w4 + w5;
                SRC_OP2_OP3_OP5: select_mod_source = w2 + w3 + w5;
                SRC_OP2_ADD_OP3: select_mod_source = w2 + w3;
                SRC_OP5_ADD_OP6: select_mod_source = w5 + w6;
                SRC_OP2_OP3_OP4: select_mod_source = w2 + w3 + w4;
                default        : select_mod_source = 18'sd0;
            endcase
        end
    endfunction

    wire signed [17:0] w_mod_in_op1 = select_mod_source(w_route_op1, s_op2, s_op3, s_op4, s_op5, s_op6);
    wire signed [17:0] w_mod_in_op2 = select_mod_source(w_route_op2, s_op2, s_op3, s_op4, s_op5, s_op6);
    wire signed [17:0] w_mod_in_op3 = select_mod_source(w_route_op3, s_op2, s_op3, s_op4, s_op5, s_op6);
    wire signed [17:0] w_mod_in_op4 = select_mod_source(w_route_op4, s_op2, s_op3, s_op4, s_op5, s_op6);
    wire signed [17:0] w_mod_in_op5 = select_mod_source(w_route_op5, s_op2, s_op3, s_op4, s_op5, s_op6);

    reg signed [15:0] fb_raw_d1;
    reg signed [15:0] fb_src_d1;
    wire signed [16:0] fb_sum_op4 = $signed({fb_raw_d1[15], fb_raw_d1}) + $signed({op4_raw_wave[15], op4_raw_wave});
    wire signed [16:0] fb_sum_op5 = $signed({fb_raw_d1[15], fb_raw_d1}) + $signed({op5_raw_wave[15], op5_raw_wave});
    wire signed [16:0] fb_sum_op6 = $signed({fb_raw_d1[15], fb_raw_d1}) + $signed({op6_raw_wave[15], op6_raw_wave});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fb_raw_d1 <= 16'sd0;
            fb_src_d1 <= 16'sd0;
        end else if (w_fb_type == FB_LOOP_OP4_TO_OP6) begin
            fb_raw_d1 <= op4_raw_wave;
            fb_src_d1 <= fb_sum_op4[16:1];
        end else if (w_fb_type == FB_LOOP_OP5_TO_OP6) begin
            fb_raw_d1 <= op5_raw_wave;
            fb_src_d1 <= fb_sum_op5[16:1];
        end else if (w_fb_type == FB_SELF_OP6) begin
            fb_raw_d1 <= op6_raw_wave;
            fb_src_d1 <= fb_sum_op6[16:1];
        end else begin
            fb_raw_d1 <= 16'sd0;
            fb_src_d1 <= 16'sd0;
        end
    end

    // =========================================================================
    // 第四部分：结合调制深度与反馈量换算 10-bit 相位偏移 (含无偏偶舍入)
    // =========================================================================
    wire [6:0]         w_clamped_depth  = (mod_depth_in > 7'd100) ? 7'd100 : mod_depth_in;
    wire [6:0]         w_clamped_fb     = (feedback_in  > 7'd100) ? 7'd100 : feedback_in;
    wire signed [11:0] w_total_mod_gain = $signed({1'b0, w_clamped_depth}) * $signed({1'b0, w_mod_idx_scale});

    wire signed [29:0] p_mod1 = w_mod_in_op1 * w_total_mod_gain;
    wire signed [29:0] p_mod2 = w_mod_in_op2 * w_total_mod_gain;
    wire signed [29:0] p_mod3 = w_mod_in_op3 * w_total_mod_gain;
    wire signed [29:0] p_mod4 = w_mod_in_op4 * w_total_mod_gain;
    wire signed [29:0] p_mod5 = w_mod_in_op5 * w_total_mod_gain;
    wire signed [23:0] p_fb6  = fb_src_d1    * $signed({1'b0, w_clamped_fb});

    wire r_c1 = p_mod1[16] & ((|p_mod1[15:0]) | p_mod1[17]);
    wire r_c2 = p_mod2[16] & ((|p_mod2[15:0]) | p_mod2[17]);
    wire r_c3 = p_mod3[16] & ((|p_mod3[15:0]) | p_mod3[17]);
    wire r_c4 = p_mod4[16] & ((|p_mod4[15:0]) | p_mod4[17]);
    wire r_c5 = p_mod5[16] & ((|p_mod5[15:0]) | p_mod5[17]);
    wire r_c6 = p_fb6[13]  & ((|p_fb6[12:0])  | p_fb6[14]);

    assign op1_phase_mod_out = p_mod1[26:17] + {9'd0, r_c1};
    assign op2_phase_mod_out = p_mod2[26:17] + {9'd0, r_c2};
    assign op3_phase_mod_out = p_mod3[26:17] + {9'd0, r_c3};
    assign op4_phase_mod_out = p_mod4[26:17] + {9'd0, r_c4};
    assign op5_phase_mod_out = p_mod5[26:17] + {9'd0, r_c5};
    assign op6_phase_mod_out = p_fb6[23:14]  + {9'd0, r_c6};

    // =========================================================================
    // 第五部分：底部音频母线汇总 + IEEE 754 无偏偶舍入
    // =========================================================================
    wire signed [18:0] w_car1 = w_carrier_mask[0] ? $signed({{3{op1_raw_wave[15]}}, op1_raw_wave}) : 19'sd0;
    wire signed [18:0] w_car2 = w_carrier_mask[1] ? $signed({{3{op2_raw_wave[15]}}, op2_raw_wave}) : 19'sd0;
    wire signed [18:0] w_car3 = w_carrier_mask[2] ? $signed({{3{op3_raw_wave[15]}}, op3_raw_wave}) : 19'sd0;
    wire signed [18:0] w_car4 = w_carrier_mask[3] ? $signed({{3{op4_raw_wave[15]}}, op4_raw_wave}) : 19'sd0;
    wire signed [18:0] w_car5 = w_carrier_mask[4] ? $signed({{3{op5_raw_wave[15]}}, op5_raw_wave}) : 19'sd0;
    wire signed [18:0] w_car6 = w_carrier_mask[5] ? $signed({{3{op6_raw_wave[15]}}, op6_raw_wave}) : 19'sd0;

    wire signed [18:0] w_carrier_sum   = (w_car1 + w_car2 + w_car3) + (w_car4 + w_car5 + w_car6);
    wire signed [23:0] w_norm_raw_prod = w_carrier_sum * $signed({1'b0, w_car_gain_weight});
    wire               norm_guard      = w_norm_raw_prod[2];
    wire               norm_sticky     = |w_norm_raw_prod[1:0];
    wire               norm_lsb        = w_norm_raw_prod[3];
    wire               norm_rnd_carry  = norm_guard & (norm_sticky | norm_lsb);
    wire signed [20:0] w_norm_shifted  = $signed(w_norm_raw_prod[23:3]) + $signed({20'd0, norm_rnd_carry});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            routed_audio_out <= 16'sd0;
        end else begin
            if (w_norm_shifted > 21'sd32767)
                routed_audio_out <= 16'sd32767;
            else if (w_norm_shifted < -21'sd32768)
                routed_audio_out <= -16'sd32768;
            else
                routed_audio_out <= w_norm_shifted[15:0];
        end
    end

endmodule