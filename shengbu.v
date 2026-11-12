`timescale 1ns / 1ps
// ============================================================================
// 文件名称: shengbu.v (24-bit Hi-Fi 升级版音频弹性缓冲器)
// 模块名称: audio_staging_buffer / audio_staging_buffer_fifo
// 功能描述: 
//   支持 DATA_WIDTH = 24 (默认) 或 16 的板载分布式 LUT 同步 FIFO 缓冲，
//   将 24-bit 抗混叠合成音频无损跨速率交接给 WM8978 24-bit I2S 发送器!
// ============================================================================
module audio_staging_buffer_fifo #(
    parameter integer DATA_WIDTH = 24,
    parameter integer STAGE_WIDTH = 28
)(
    input  wire                          clk,
    input  wire                          rst_n,
    // 1. 本地声部合成输入 (默认 24-bit Hi-Fi PCM)
    input  wire signed [DATA_WIDTH-1:0]  local_synth_data,
    input  wire                          local_synth_en,
    // 2. 多声部级联总线输入与分组地址
    input  wire signed [STAGE_WIDTH-1:0] cascade_data_in,
    input  wire                          cascade_en_in,
    input  wire [1:0]                    group_addr_in,
    // 3. WM8978 I2S 48kHz 采样请求脉冲 (触发 FIFO 弹出新样本)
    input  wire                          i2s_sample_req,
    output wire [4:0]                    fifo_data_cnt,
    // 4. 级联输出与最终送往 I2S 发送器的 24-bit PCM 数据
    output reg  signed [STAGE_WIDTH-1:0] stage_data_out,
    output reg                           stage_en_out,
    output reg  [DATA_WIDTH-1:0]         final_i2s_data,
    output reg                           final_i2s_en
);
    localparam signed [STAGE_WIDTH-1:0] MAX_VAL = {1'b0, {(STAGE_WIDTH-DATA_WIDTH){1'b0}}, {(DATA_WIDTH-1){1'b1}}};
    localparam signed [STAGE_WIDTH-1:0] MIN_VAL = {1'b1, {(STAGE_WIDTH-DATA_WIDTH){1'b1}}, {(DATA_WIDTH-1){1'b0}}};

    wire signed [STAGE_WIDTH-1:0] local_ext    = {{(STAGE_WIDTH-DATA_WIDTH){local_synth_data[DATA_WIDTH-1]}}, local_synth_data};
    wire signed [STAGE_WIDTH-1:0] combined_sum = cascade_en_in ? (local_ext + cascade_data_in) : local_ext;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage_data_out <= {STAGE_WIDTH{1'b0}};
            stage_en_out   <= 1'b0;
        end else if (local_synth_en) begin
            stage_data_out <= combined_sum;
            stage_en_out   <= 1'b1;
        end else begin
            stage_en_out   <= 1'b0;
        end
    end

    wire [DATA_WIDTH-1:0] clamped_pcm = (combined_sum > MAX_VAL) ? MAX_VAL[DATA_WIDTH-1:0] :
                                        (combined_sum < MIN_VAL) ? MIN_VAL[DATA_WIDTH-1:0] :
                                        combined_sum[DATA_WIDTH-1:0];

    wire [DATA_WIDTH-1:0] fifo_dout;
    wire                  fifo_full;
    wire                  fifo_empty;

    wire fifo_wr_en = local_synth_en;
    wire fifo_rd_en = (i2s_sample_req & (~fifo_empty)) | (fifo_full & local_synth_en);

    lut_sync_fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (4),
        .FWFT_MODE  (1)
    ) u_audio_lut_fifo (
        .clk        (clk),
        .rst_n      (rst_n),
        .wr_en      (fifo_wr_en),
        .din        (clamped_pcm),
        .full       (fifo_full),
        .rd_en      (fifo_rd_en),
        .dout       (fifo_dout),
        .empty      (fifo_empty),
        .data_count (fifo_data_cnt)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            final_i2s_data <= {DATA_WIDTH{1'b0}};
            final_i2s_en   <= 1'b0;
        end else if (i2s_sample_req) begin
            final_i2s_data <= fifo_empty ? clamped_pcm : fifo_dout;
            final_i2s_en   <= 1'b1;
        end else begin
            final_i2s_en   <= 1'b0;
        end
    end
endmodule

module audio_staging_buffer #(
    parameter integer DATA_WIDTH  = 24,
    parameter integer STAGE_WIDTH = 28
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire signed [DATA_WIDTH-1:0]  local_synth_data,
    input  wire                          local_synth_en,
    input  wire signed [STAGE_WIDTH-1:0] cascade_data_in,
    input  wire                          cascade_en_in,
    input  wire [1:0]                    group_addr_in,
    input  wire                          i2s_sample_req,
    output wire signed [STAGE_WIDTH-1:0] stage_data_out,
    output wire                          stage_en_out,
    output wire [DATA_WIDTH-1:0]         final_i2s_data,
    output wire                          final_i2s_en
);
    audio_staging_buffer_fifo #(
        .DATA_WIDTH  (DATA_WIDTH),
        .STAGE_WIDTH (STAGE_WIDTH)
    ) u_staging_core (
        .clk              (clk),
        .rst_n            (rst_n),
        .local_synth_data (local_synth_data),
        .local_synth_en   (local_synth_en),
        .cascade_data_in  (cascade_data_in),
        .cascade_en_in    (cascade_en_in),
        .group_addr_in    (group_addr_in),
        .i2s_sample_req   (i2s_sample_req),
        .fifo_data_cnt    (),
        .stage_data_out   (stage_data_out),
        .stage_en_out     (stage_en_out),
        .final_i2s_data   (final_i2s_data),
        .final_i2s_en     (final_i2s_en)
    );
endmodule
