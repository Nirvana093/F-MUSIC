`timescale 1ns / 1ps

module oversample_decimator_1024_fifo #(
    parameter integer DATA_WIDTH = 24
)(
    input  wire                         clk_50m,
    input  wire                         rst_n,
    input  wire signed [DATA_WIDTH-1:0] pcm_in,
    output reg  signed [DATA_WIDTH-1:0] clean_pcm_out,
    output reg                          clean_pcm_vld
);
    reg signed [DATA_WIDTH+9:0] acc_sum = {(DATA_WIDTH+10){1'b0}};
    reg [9:0]                   acc_cnt = 10'd0;

    wire signed [DATA_WIDTH+9:0] next_sum = acc_sum + $signed({{10{pcm_in[DATA_WIDTH-1]}}, pcm_in});
    wire guard_b   = next_sum[9];
    wire sticky_b  = |next_sum[8:0];
    wire lsb_b     = next_sum[10];
    wire rnd_carry = guard_b & (sticky_b | lsb_b);

    wire signed [DATA_WIDTH:0] avg_rounded = $signed(next_sum[DATA_WIDTH+9:10]) +
                                             $signed({{DATA_WIDTH{1'b0}}, rnd_carry});

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            acc_sum       <= {(DATA_WIDTH+10){1'b0}};
            acc_cnt       <= 10'd0;
            clean_pcm_out <= {DATA_WIDTH{1'b0}};
            clean_pcm_vld <= 1'b0;
        end else begin
            if (acc_cnt == 10'd1023) begin
                clean_pcm_out <= avg_rounded[DATA_WIDTH-1:0];
                clean_pcm_vld <= 1'b1;
                acc_sum       <= {(DATA_WIDTH+10){1'b0}};
                acc_cnt       <= 10'd0;
            end else begin
                clean_pcm_vld <= 1'b0;
                acc_sum       <= next_sum;
                acc_cnt       <= acc_cnt + 10'd1;
            end
        end
    end
endmodule

module oversample_decimator_1024 #(
    parameter integer DATA_WIDTH = 24
)(
    input  wire                         clk_50m,
    input  wire                         rst_n,
    input  wire                         sample_req_pulse,
    input  wire signed [DATA_WIDTH-1:0] pcm_in,
    output reg  signed [DATA_WIDTH-1:0] clean_pcm_out,
    output reg                          clean_pcm_vld
);
    // 24-bit + log2(1024) = 34-bit 全周期无损累加器
    reg signed [DATA_WIDTH+9:0] acc_sum    = {(DATA_WIDTH+10){1'b0}};
    reg [9:0]                   acc_cnt    = 10'd0;
    reg signed [DATA_WIDTH-1:0] latest_avg = {DATA_WIDTH{1'b0}};
    reg [10:0]                  safety_cnt = 11'd0;

    wire signed [DATA_WIDTH+9:0] next_sum = acc_sum + $signed({{10{pcm_in[DATA_WIDTH-1]}}, pcm_in});

    // 右移 10 位 (除以 1024) 的无偏偶舍入 (Round-Half-to-Even)
    wire guard_b   = next_sum[9];
    wire sticky_b  = |next_sum[8:0];
    wire lsb_b     = next_sum[10];
    wire rnd_carry = guard_b & (sticky_b | lsb_b);

    wire signed [DATA_WIDTH:0] avg_rounded = $signed(next_sum[DATA_WIDTH+9:10]) +
                                             $signed({{DATA_WIDTH{1'b0}}, rnd_carry});

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            acc_sum       <= {(DATA_WIDTH+10){1'b0}};
            acc_cnt       <= 10'd0;
            latest_avg    <= {DATA_WIDTH{1'b0}};
            safety_cnt    <= 11'd0;
            clean_pcm_out <= {DATA_WIDTH{1'b0}};
            clean_pcm_vld <= 1'b0;
        end else begin
            if (acc_cnt == 10'd1023) begin
                latest_avg <= avg_rounded[DATA_WIDTH-1:0];
                acc_sum    <= {(DATA_WIDTH+10){1'b0}};
                acc_cnt    <= 10'd0;
            end else begin
                acc_sum <= next_sum;
                acc_cnt <= acc_cnt + 10'd1;
            end

            if (sample_req_pulse || (safety_cnt >= 11'd1200)) begin
                clean_pcm_out <= latest_avg;
                clean_pcm_vld <= 1'b1;
                safety_cnt    <= 11'd0;
            end else begin
                clean_pcm_vld <= 1'b0;
                safety_cnt    <= safety_cnt + 11'd1;
            end
        end
    end
endmodule