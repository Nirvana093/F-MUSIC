`timescale 1ns / 1ps

// ============================================================================
// 模块名称: lut_sync_fifo (纯寄存器级真 0 延迟 FWFT 同步 FIFO)
// 修复说明:
//   1. 采用 (* syn_ramstyle = "registers" *) 触发器阵列实现，彻底消除高云 IF0005 警告；
//   2. 实现真正的 0 周期组合逻辑首字直通 (assign dout = lut_mem[rd_ptr])，
//      确保 midi_decoder 在 fifo_rx_rd_en 拉高当拍 100% 读到当前最新字节，
//      彻底解决因 BSRAM 1拍读延迟导致 MIDI 第三字节(力度)卡在队列中引起的“分批延后发声”问题!
// ============================================================================
module lut_sync_fifo #(
    parameter integer DATA_WIDTH = 16,
    parameter integer ADDR_WIDTH = 4,
    parameter integer FWFT_MODE  = 1
)(
    input  wire                  clk,
    input  wire                  rst_n,
    // 写端口
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] din,
    output wire                  full,
    // 读端口
    input  wire                  rd_en,
    output wire [DATA_WIDTH-1:0] dout,
    output wire                  empty,
    // 水位状态指示
    output reg  [ADDR_WIDTH:0]   data_count
);

    localparam integer DEPTH = (1 << ADDR_WIDTH);

    (* syn_ramstyle = "registers" *)
    reg [DATA_WIDTH-1:0] lut_mem [0:DEPTH-1];

    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;

    wire valid_wr = wr_en & (~full);
    wire valid_rd = rd_en & (~empty);

    assign full  = (data_count == DEPTH[ADDR_WIDTH:0]);
    assign empty = (data_count == {(ADDR_WIDTH+1){1'b0}});

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {ADDR_WIDTH{1'b0}};
            for (i = 0; i < DEPTH; i = i + 1) begin
                lut_mem[i] <= {DATA_WIDTH{1'b0}};
            end
        end else if (valid_wr) begin
            lut_mem[wr_ptr] <= din;
            wr_ptr          <= wr_ptr + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= {ADDR_WIDTH{1'b0}};
        end else if (valid_rd) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_count <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            case ({valid_wr, valid_rd})
                2'b10:   data_count <= data_count + 1'b1;
                2'b01:   data_count <= data_count - 1'b1;
                default: data_count <= data_count;
            endcase
        end
    end

    // 真 0 延迟首字直通输出：一旦 empty 为 0，dout 立即呈现当前队首字节
    assign dout = lut_mem[rd_ptr];

endmodule