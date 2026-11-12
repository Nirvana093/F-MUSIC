`timescale 1ns / 1ps

module poly32_lossless_mixer_24b (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [31:0]        voice_env_active,
    input  wire signed [24:0] core0_sum25_in,
    input  wire signed [24:0] core1_sum25_in,
    input  wire signed [24:0] core2_sum25_in,
    input  wire signed [24:0] core3_sum25_in,
    output reg  signed [23:0] mixed_pcm24_out,
    output reg  signed [15:0] mixed_pcm16_out,
    output reg                mixed_pcm_vld
);

    reg signed [25:0] sum_c01, sum_c23;
    reg signed [26:0] raw_sum_27b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_c01     <= 26'sd0;
            sum_c23     <= 26'sd0;
            raw_sum_27b <= 27'sd0;
        end else begin
            sum_c01     <= $signed({core0_sum25_in[24], core0_sum25_in}) +
                           $signed({core1_sum25_in[24], core1_sum25_in});
            sum_c23     <= $signed({core2_sum25_in[24], core2_sum25_in}) +
                           $signed({core3_sum25_in[24], core3_sum25_in});
            raw_sum_27b <= $signed({sum_c01[25], sum_c01}) +
                           $signed({sum_c23[25], sum_c23});
        end
    end

    reg [5:0] active_cnt;
    integer c;
    always @(*) begin
        active_cnt = 6'd0;
        for (c = 0; c < 32; c = c + 1) begin
            active_cnt = active_cnt + {5'd0, voice_env_active[c]};
        end
    end

    reg [14:0] prop_scale_q12;
    always @(*) begin
        case (active_cnt)
            6'd0 : prop_scale_q12 = 15'd8192;
            6'd1 : prop_scale_q12 = 15'd8192;
            6'd2 : prop_scale_q12 = 15'd7168;
            6'd3 : prop_scale_q12 = 15'd6144;
            6'd4 : prop_scale_q12 = 15'd5120;
            6'd5 : prop_scale_q12 = 15'd4352;
            6'd6 : prop_scale_q12 = 15'd3840;
            6'd7 : prop_scale_q12 = 15'd3328;
            6'd8 : prop_scale_q12 = 15'd3072;
            default: begin
                if (active_cnt <= 6'd16) prop_scale_q12 = 15'd2048;
                else                     prop_scale_q12 = 15'd1280;
            end
        endcase
    end

    reg [14:0] smooth_scale_q12 = 15'd8192;
    reg [3:0]  slew_cnt         = 4'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            smooth_scale_q12 <= 15'd8192;
            slew_cnt         <= 4'd0;
        end else begin
            slew_cnt <= slew_cnt + 4'd1;
            if (&slew_cnt) begin
                if (smooth_scale_q12 > prop_scale_q12)
                    smooth_scale_q12 <= smooth_scale_q12 - ((smooth_scale_q12 - prop_scale_q12) >> 4) - 15'd1;
                else if (smooth_scale_q12 < prop_scale_q12)
                    smooth_scale_q12 <= smooth_scale_q12 + ((prop_scale_q12 - smooth_scale_q12) >> 5) + 15'd1;
            end
        end
    end

    reg signed [42:0] mult_prod_43b;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mult_prod_43b <= 43'sd0;
        else
            mult_prod_43b <= raw_sum_27b * $signed({1'b0, smooth_scale_q12});
    end

    wire        guard_bit   = mult_prod_43b[11];
    wire        sticky_bit  = |mult_prod_43b[10:0];
    wire        lsb_bit     = mult_prod_43b[12];
    wire        round_carry = guard_bit & (sticky_bit | lsb_bit);

    wire signed [30:0] s31 = $signed(mult_prod_43b[42:12]) + $signed({30'd0, round_carry});

    localparam signed [30:0] CEIL_POS = 31'sd7340032;
    localparam signed [30:0] CEIL_NEG = -31'sd7340032;
    wire signed [30:0] s31_safe = (s31 > CEIL_POS) ? (CEIL_POS + ((s31 - CEIL_POS) >>> 3)) :
                                  (s31 < CEIL_NEG) ? (CEIL_NEG + ((s31 - CEIL_NEG) >>> 3)) : s31;

    wire signed [23:0] clamped_24b = (s31_safe > 31'sd8388607)  ? 24'sd8388607  :
                                     (s31_safe < -31'sd8388608) ? -24'sd8388608 :
                                     s31_safe[23:0];

    wire        r16_carry   = clamped_24b[7] & ((|clamped_24b[6:0]) | clamped_24b[8]);
    wire signed [16:0] s17  = $signed(clamped_24b[23:8]) + $signed({16'd0, r16_carry});
    wire signed [15:0] clamped_16b = (s17 > 17'sd32767)  ? 16'sd32767  :
                                     (s17 < -17'sd32768) ? -16'sd32768 :
                                     s17[15:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mixed_pcm24_out <= 24'sd0;
            mixed_pcm16_out <= 16'sd0;
            mixed_pcm_vld   <= 1'b0;
        end else begin
            mixed_pcm24_out <= clamped_24b;
            mixed_pcm16_out <= clamped_16b;
            mixed_pcm_vld   <= 1'b1;
        end
    end

endmodule