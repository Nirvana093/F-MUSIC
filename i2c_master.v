`timescale 1ns / 1ps

module i2c_master #(
    parameter SYS_CLK_FREQ = 50_000_000,
    parameter I2C_FREQ     = 100_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i2c_start,
    input  wire [6:0]  dev_addr,
    input  wire [15:0] reg_data,
    output reg         busy,
    output reg         ack_error,
    output reg         i2c_scl,
    inout  wire        i2c_sda
);

    localparam integer CLK_DIV_MAX = 250; // 50kHz 稳健速率
    reg [15:0] clk_cnt;
    reg        i2c_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt  <= 16'd0;
            i2c_tick <= 1'b0;
        end else if (busy) begin
            if (clk_cnt >= CLK_DIV_MAX - 1) begin
                clk_cnt  <= 16'd0;
                i2c_tick <= 1'b1;
            end else begin
                clk_cnt  <= clk_cnt + 16'd1;
                i2c_tick <= 1'b0;
            end
        end else begin
            clk_cnt  <= 16'd0;
            i2c_tick <= 1'b0;
        end
    end

    reg  sda_dir;
    reg  sda_out;
    wire sda_in;

    // 因为拓展板 PCB 上 SCLK/SDIN 没有焊外部 4.7k 上拉电阻，
    // 在发送数据位期间 (sda_dir == 1) 由 FPGA 主动推挽驱动 sda_out (确保高电平上升沿 < 5ns)；
    // 在第 8 bit 下降沿 (bit_cnt == 0, tick_state == 3) 当拍立即将 sda_dir 清零释放为 1'bz，
    // 让 WM8978 零阻力拉低 SDA 回复 ACK!
    assign i2c_sda = sda_dir ? sda_out : 1'bz;
    assign sda_in  = i2c_sda;

    localparam [5:0] IDLE      = 6'd0,
                     START     = 6'd1,
                     SEND_BYTE = 6'd2,
                     WAIT_ACK  = 6'd3,
                     STOP      = 6'd4,
                     DONE      = 6'd5;

    reg [5:0]  state;
    reg [7:0]  tx_shift_reg;
    reg [2:0]  bit_cnt;
    reg [1:0]  byte_cnt;
    reg [23:0] full_tx_data;
    reg [1:0]  tick_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            busy         <= 1'b0;
            ack_error    <= 1'b0;
            i2c_scl      <= 1'b1;
            sda_dir      <= 1'b1;
            sda_out      <= 1'b1;
            tick_state   <= 2'd0;
            bit_cnt      <= 3'd0;
            byte_cnt     <= 2'd0;
            full_tx_data <= 24'd0;
        end else begin
            case (state)
                IDLE: begin
                    busy       <= 1'b0;
                    i2c_scl    <= 1'b1;
                    sda_dir    <= 1'b1;
                    sda_out    <= 1'b1;
                    tick_state <= 2'd0;
                    if (i2c_start) begin
                        busy         <= 1'b1;
                        ack_error    <= 1'b0;
                        full_tx_data <= {dev_addr, 1'b0, reg_data[15:8], reg_data[7:0]};
                        state        <= START;
                    end
                end

                START: begin
                    if (i2c_tick) begin
                        tick_state <= tick_state + 2'd1;
                        case (tick_state)
                            2'd0: begin sda_dir <= 1'b1; sda_out <= 1'b1; i2c_scl <= 1'b1; end
                            2'd1: begin sda_out <= 1'b0; i2c_scl <= 1'b1; end
                            2'd2: begin sda_out <= 1'b0; i2c_scl <= 1'b0; end
                            2'd3: begin
                                state        <= SEND_BYTE;
                                byte_cnt     <= 2'd0;
                                bit_cnt      <= 3'd7;
                                tx_shift_reg <= full_tx_data[23:16];
                                tick_state   <= 2'd0;
                            end
                        endcase
                    end
                end

                SEND_BYTE: begin
                    if (i2c_tick) begin
                        tick_state <= tick_state + 2'd1;
                        case (tick_state)
                            2'd0: begin
                                sda_dir <= 1'b1;
                                sda_out <= tx_shift_reg[bit_cnt];
                            end
                            2'd1: begin i2c_scl <= 1'b1; end
                            2'd2: begin i2c_scl <= 1'b1; end
                            2'd3: begin
                                i2c_scl <= 1'b0;
                                if (bit_cnt == 3'd0) begin
                                    // 第 8 位 SCL 下降沿当拍立即释放 SDA 为高阻态 1'bz，绝不与 WM8978 的 ACK 顶牛!
                                    sda_dir <= 1'b0;
                                    sda_out <= 1'b1;
                                    state   <= WAIT_ACK;
                                end else begin
                                    bit_cnt <= bit_cnt - 3'd1;
                                end
                            end
                        endcase
                    end
                end

                WAIT_ACK: begin
                    if (i2c_tick) begin
                        tick_state <= tick_state + 2'd1;
                        case (tick_state)
                            2'd0: begin sda_dir <= 1'b0; sda_out <= 1'b1; end
                            2'd1: begin i2c_scl <= 1'b1; end
                            2'd2: begin
                                if (sda_in == 1'b1) ack_error <= 1'b1;
                            end
                            2'd3: begin
                                i2c_scl <= 1'b0;
                                if (byte_cnt == 2'd2) begin
                                    sda_dir <= 1'b1;
                                    sda_out <= 1'b0;
                                    state   <= STOP;
                                end else begin
                                    byte_cnt     <= byte_cnt + 2'd1;
                                    state        <= SEND_BYTE;
                                    bit_cnt      <= 3'd7;
                                    tx_shift_reg <= (byte_cnt == 2'd0) ? full_tx_data[15:8] : full_tx_data[7:0];
                                end
                                tick_state <= 2'd0;
                            end
                        endcase
                    end
                end

                STOP: begin
                    if (i2c_tick) begin
                        tick_state <= tick_state + 2'd1;
                        case (tick_state)
                            2'd0: begin sda_dir <= 1'b1; sda_out <= 1'b0; i2c_scl <= 1'b0; end
                            2'd1: begin sda_out <= 1'b0; i2c_scl <= 1'b1; end
                            2'd2: begin sda_out <= 1'b1; i2c_scl <= 1'b1; end
                            2'd3: begin state <= DONE; tick_state <= 2'd0; end
                        endcase
                    end
                end

                DONE: begin
                    busy  <= 1'b0;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule