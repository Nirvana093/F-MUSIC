`timescale 1ns / 1ps

module seg7_key_probe (
    input  wire         clk_50m,
    input  wire         rst_n,
    input  wire [4:0]   s_keys,        // S[4:0] 按键输入，按下为 0
    input  wire [511:0] mod_regs_flat, // 16 x 32-bit 调制寄存器扁平输入
    output reg          seg7_dio,      // U20
    output reg          seg7_rclk,     // V20
    output reg          seg7_sclk      // T21
);

    // 1. 20ms 硬件按键消抖与单脉冲提取 (支持 S0, S1, S2, S3 四个按键)
    reg [19:0] db_cnt        = 20'd0;
    reg [3:0]  key_sync1     = 4'b1111, key_sync2     = 4'b1111;
    reg [3:0]  key_stable    = 4'b1111, key_stable_d1 = 4'b1111;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            db_cnt        <= 20'd0;
            key_sync1     <= 4'b1111;
            key_sync2     <= 4'b1111;
            key_stable    <= 4'b1111;
            key_stable_d1 <= 4'b1111;
        end else begin
            key_sync1     <= s_keys[3:0];
            key_sync2     <= key_sync1;
            key_stable_d1 <= key_stable;

            if (key_sync2 != key_stable) begin
                if (db_cnt == 20'd999_999) begin
                    key_stable <= key_sync2;
                    db_cnt     <= 20'd0;
                end else begin
                    db_cnt <= db_cnt + 20'd1;
                end
            end else begin
                db_cnt <= 20'd0;
            end
        end
    end

    wire [3:0] key_pressed = key_stable_d1 & (~key_stable);

    // 2. 通道切换状态机
    reg [3:0] ch_sel   = 4'd0;
    reg       dec_mode = 1'b1;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            ch_sel   <= 4'd0;
            dec_mode <= 1'b1;
        end else begin
            if (key_pressed[0]) begin
                ch_sel <= ch_sel + 4'd1;
            end else if (key_pressed[1]) begin
                ch_sel <= ch_sel - 4'd1;
            end else if (key_pressed[3]) begin
                ch_sel <= 4'd0;
            end

            if (key_pressed[2]) begin
                dec_mode <= ~dec_mode;
            end
        end
    end

    // 3. 多格式硬件 BCD 编码器
    wire [31:0] ch_val = mod_regs_flat[ch_sel*32 +: 32];

    wire is_dual_field_ch = (ch_sel == 4'd0) || (ch_sel == 4'd1) ||
                            (ch_sel == 4'd2) || (ch_sel == 4'd3) || (ch_sel == 4'd5);
    wire is_hex_diag_ch   = (ch_sel == 4'd12) || (ch_sel == 4'd14) || (ch_sel == 4'd15);

    function [11:0] bin10_to_bcd3(input [9:0] bin);
        integer k;
        reg [21:0] s;
        begin
            s = {12'd0, bin};
            for (k = 0; k < 10; k = k + 1) begin
                if (s[13:10] >= 4'd5) s[13:10] = s[13:10] + 4'd3;
                if (s[17:14] >= 4'd5) s[17:14] = s[17:14] + 4'd3;
                if (s[21:18] >= 4'd5) s[21:18] = s[21:18] + 4'd3;
                s = s << 1;
            end
            bin10_to_bcd3 = s[21:10];
        end
    endfunction

    wire [23:0] fmt_touch_bcd = {bin10_to_bcd3(ch_val[25:16]), bin10_to_bcd3(ch_val[9:0])};
    wire [11:0] dual_hi_bcd   = bin10_to_bcd3(ch_val[25:16]);
    wire [11:0] dual_lo_bcd   = bin10_to_bcd3(ch_val[9:0]);
    wire [23:0] fmt_dual_bcd  = {4'h0, dual_hi_bcd[7:0], dual_lo_bcd};

    reg [23:0] fmt_single_bcd;
    integer b;
    reg [43:0] shift_bcd;
    always @(*) begin
        shift_bcd = {24'd0, ch_val[19:0]};
        for (b = 0; b < 20; b = b + 1) begin
            if (shift_bcd[23:20] >= 4'd5) shift_bcd[23:20] = shift_bcd[23:20] + 4'd3;
            if (shift_bcd[27:24] >= 4'd5) shift_bcd[27:24] = shift_bcd[27:24] + 4'd3;
            if (shift_bcd[31:28] >= 4'd5) shift_bcd[31:28] = shift_bcd[31:28] + 4'd3;
            if (shift_bcd[35:32] >= 4'd5) shift_bcd[35:32] = shift_bcd[35:32] + 4'd3;
            if (shift_bcd[39:36] >= 4'd5) shift_bcd[39:36] = shift_bcd[39:36] + 4'd3;
            if (shift_bcd[43:40] >= 4'd5) shift_bcd[43:40] = shift_bcd[43:40] + 4'd3;
            shift_bcd = shift_bcd << 1;
        end
        fmt_single_bcd = shift_bcd[43:20];
    end

    wire [23:0] dec_bus  = (ch_sel == 4'd11) ? fmt_touch_bcd :
                           is_dual_field_ch  ? fmt_dual_bcd  : fmt_single_bcd;
    wire [23:0] hex_bus  = (ch_sel == 4'd11) ? {2'b00, ch_val[25:16], 2'b00, ch_val[9:0]} : ch_val[23:0];
    wire        use_dec  = dec_mode && !is_hex_diag_ch;
    wire [23:0] disp_bus = use_dec ? dec_bus : hex_bus;

    // 4. 共阳极数码管段码查找表与分隔小数点控制
    function [6:0] hex_to_seg(input [3:0] hex);
        begin
            case (hex)
                4'h0: hex_to_seg = ~7'b0111111;
                4'h1: hex_to_seg = ~7'b0000110;
                4'h2: hex_to_seg = ~7'b1011011;
                4'h3: hex_to_seg = ~7'b1001111;
                4'h4: hex_to_seg = ~7'b1100110;
                4'h5: hex_to_seg = ~7'b1101101;
                4'h6: hex_to_seg = ~7'b1111101;
                4'h7: hex_to_seg = ~7'b0000111;
                4'h8: hex_to_seg = ~7'b1111111;
                4'h9: hex_to_seg = ~7'b1101111;
                4'hA: hex_to_seg = ~7'b1110111;
                4'hB: hex_to_seg = ~7'b1111100;
                4'hC: hex_to_seg = ~7'b0111001;
                4'hD: hex_to_seg = ~7'b1011110;
                4'hE: hex_to_seg = ~7'b1111001;
                4'hF: hex_to_seg = ~7'b1110001;
            endcase
        end
    endfunction

    reg [15:0] scan_cnt  = 16'd0;
    reg [2:0]  digit_idx = 3'd0;
    reg        send_trig = 1'b0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            scan_cnt  <= 16'd0;
            digit_idx <= 3'd0;
            send_trig <= 1'b0;
        end else if (scan_cnt == 16'd49_999) begin
            scan_cnt  <= 16'd0;
            digit_idx <= digit_idx + 3'd1;
            send_trig <= 1'b1;
        end else begin
            scan_cnt  <= scan_cnt + 16'd1;
            send_trig <= 1'b0;
        end
    end

    wire [7:0] sel_byte  = (8'b0000_0001 << digit_idx);
    wire       dp_mid_on = (use_dec && ((ch_sel == 4'd11) || is_dual_field_ch)) || (ch_sel == 4'd12);
    reg  [7:0] seg_byte;

    always @(*) begin
        case (digit_idx)
            3'd7: seg_byte = {1'b1, hex_to_seg(4'hC)};
            3'd6: seg_byte = {~use_dec, hex_to_seg(ch_sel)};
            3'd5: seg_byte = (use_dec && is_dual_field_ch) ? 8'hFF : {1'b1, hex_to_seg(disp_bus[23:20])};
            3'd4: seg_byte = {1'b1, hex_to_seg(disp_bus[19:16])};
            3'd3: seg_byte = {~dp_mid_on, hex_to_seg(disp_bus[15:12])};
            3'd2: seg_byte = {1'b1, hex_to_seg(disp_bus[11:8])};
            3'd1: seg_byte = {1'b1, hex_to_seg(disp_bus[7:4])};
            3'd0: seg_byte = {1'b1, hex_to_seg(disp_bus[3:0])};
        endcase
    end

    // 5. 双 74HC595 串行移位状态机
    reg [5:0]  div_cnt   = 6'd0;
    reg [5:0]  bit_step  = 6'd0;
    reg [15:0] shift_buf = 16'd0;
    reg        shifting  = 1'b0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt   <= 6'd0;
            bit_step  <= 6'd0;
            shift_buf <= 16'd0;
            shifting  <= 1'b0;
            seg7_dio  <= 1'b0;
            seg7_sclk <= 1'b0;
            seg7_rclk <= 1'b0;
        end else if (send_trig) begin
            shift_buf <= {seg_byte, sel_byte};
            shifting  <= 1'b1;
            bit_step  <= 6'd0;
            div_cnt   <= 6'd0;
            seg7_sclk <= 1'b0;
            seg7_rclk <= 1'b0;
        end else if (shifting) begin
            if (div_cnt == 6'd24) begin
                div_cnt <= 6'd0;
                if (bit_step < 6'd32) begin
                    if (bit_step[0] == 1'b0) begin
                        seg7_dio  <= shift_buf[15];
                        shift_buf <= {shift_buf[14:0], 1'b0};
                        seg7_sclk <= 1'b0;
                    end else begin
                        seg7_sclk <= 1'b1;
                    end
                    bit_step <= bit_step + 6'd1;
                end else if (bit_step == 6'd32) begin
                    seg7_sclk <= 1'b0;
                    seg7_rclk <= 1'b1;
                    bit_step  <= bit_step + 6'd1;
                end else begin
                    seg7_rclk <= 1'b0;
                    shifting  <= 1'b0;
                end
            end else begin
                div_cnt <= div_cnt + 6'd1;
            end
        end
    end

endmodule