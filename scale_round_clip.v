`timescale 1ns / 1ps

module scale_round_clip #(
    parameter IN_WIDTH   = 16, // 输入位宽 (例如：三角波ROM的 16-bit 数据)
    parameter SHIFT_BITS = 6,  // 缩放比例 (右移 6 位，相当于按比例缩小 2^6 = 64 倍)
    parameter IS_SIGNED  = 1   // 1: 有符号数 (音频默认); 0: 无符号数
)(
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           din_vld,  // 数据输入有效信号
    input  wire [IN_WIDTH-1:0]            din,      // 原始调制数据
    output reg                            dout_vld, // 数据输出有效信号
    output reg  [IN_WIDTH-SHIFT_BITS-1:0] dout      // 缩放后的输出数据
);

    // 计算输出位宽 (例如：16 - 6 = 10-bit)
    localparam OUT_WIDTH = IN_WIDTH - SHIFT_BITS;

    // 1. 符号扩展，增加1个最高位防止加法过程中的中间溢出
    wire [IN_WIDTH:0] din_ext;
    generate
        if (IS_SIGNED) begin
            assign din_ext = {din[IN_WIDTH-1], din}; // 符号扩展
        end else begin
            assign din_ext = {1'b0, din};            // 零扩展
        end
    endgenerate

    // 2. 计算四舍五入补偿值 (Half-Up)
    // 逻辑：右移 N 位，四舍五入需要先加上 2^(N-1)
    wire [IN_WIDTH:0] round_offset = (SHIFT_BITS > 0) ? (1 << (SHIFT_BITS - 1)) : 0;

    // 3. 执行加法
    wire [IN_WIDTH:0] sum = din_ext + round_offset;

    // 4. 右移截断 (提取所需的高位部分)
    // 注意：这里提取了 OUT_WIDTH + 1 位，特意保留了1位额外的符号/溢出位用于下一步判断
    wire [OUT_WIDTH:0] shifted_sum = sum[IN_WIDTH : SHIFT_BITS];

    // 5. 饱和钳位 (Saturation) 与流水线输出
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout     <= 0;
            dout_vld <= 1'b0;
        end else begin
            dout_vld <= din_vld;
            if (din_vld) begin
                if (SHIFT_BITS == 0) begin
                    dout <= din; // 如果不需要缩放，直接输出
                end else if (IS_SIGNED) begin
                    // 有符号数溢出判断：最高两位不一致即发生溢出
                    if (shifted_sum[OUT_WIDTH] != shifted_sum[OUT_WIDTH-1]) begin
                        if (shifted_sum[OUT_WIDTH] == 1'b0) begin
                            // 正向溢出 (例如四舍五入导致的进位)，钳位至最大正数: 0111...1
                            dout <= {1'b0, {(OUT_WIDTH-1){1'b1}}}; 
                        end else begin
                            // 负向溢出，钳位至最大负数: 1000...0
                            dout <= {1'b1, {(OUT_WIDTH-1){1'b0}}}; 
                        end
                    end else begin
                        // 未溢出，正常截取
                        dout <= shifted_sum[OUT_WIDTH-1:0];
                    end
                end else begin
                    // 无符号数溢出判断
                    if (shifted_sum[OUT_WIDTH] == 1'b1) begin
                        dout <= {(OUT_WIDTH){1'b1}}; // 溢出，钳位至全 1
                    end else begin
                        dout <= shifted_sum[OUT_WIDTH-1:0];
                    end
                end
            end
        end
    end

endmodule
