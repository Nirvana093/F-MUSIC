`timescale 1ns / 1ps

module gt9147_touch_ctrl (
    input  wire        clk_50m,
    input  wire        rst_n,
    output reg         tp_scl,         // L13
    inout  wire        tp_sda,         // L14
    output reg         tp_rst,         // L15
    inout  wire        tp_int,         // M13
    output reg  [31:0] touch_xy_coord  // {touch_valid, 5'd0, touch_x[9:0], 6'd0, touch_y[9:0]}
);

    // =========================================================================
    // 1. 严格遵循 GT9147 编程指南 4.1 & 4.2 节的上电复位与 0x14 (0x28/0x29) 地址设定时序
    // =========================================================================
    reg [22:0] por_cnt = 23'd0;
    reg        int_oe  = 1'b1;
    reg        int_out = 1'b0;
    reg        init_ok = 1'b0;

    assign tp_int = int_oe ? int_out : 1'bz;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            por_cnt <= 23'd0;
            tp_rst  <= 1'b0;
            int_oe  <= 1'b1;
            int_out <= 1'b0;
            init_ok <= 1'b0;
        end else if (!init_ok) begin
            por_cnt <= por_cnt + 23'd1;
            if (por_cnt < 23'd500_000) begin          // 0~10ms: RST=0, INT=0 (T2 > 10ms)
                tp_rst  <= 1'b0;
                int_oe  <= 1'b1;
                int_out <= 1'b0;
            end else if (por_cnt < 23'd600_000) begin // 10~12ms: RST=0, INT=1 (T7 > 100us)
                tp_rst  <= 1'b0;
                int_oe  <= 1'b1;
                int_out <= 1'b1;
            end else if (por_cnt < 23'd1_000_000) begin // 12~20ms: RST=1, INT=1 (T8 > 5ms, 锁定 0x28/0x29)
                tp_rst  <= 1'b1;
                int_oe  <= 1'b1;
                int_out <= 1'b1;
            end else if (por_cnt < 23'd1_100_000) begin // 20~22ms: Host Output low (按手册4.1时序图先拉低INT)
                tp_rst  <= 1'b1;
                int_oe  <= 1'b1;
                int_out <= 1'b0;
            end else if (por_cnt < 23'd4_000_000) begin // 22~80ms: INT转为悬浮输入态，延时 58ms (>50ms)
                tp_rst  <= 1'b1;
                int_oe  <= 1'b0;
                int_out <= 1'b0;
            end else begin
                init_ok <= 1'b1;
            end
        end
    end

    // =========================================================================
    // 2. 100kHz I2C 四相节拍发生器 (每相 2.5us, SCL 周期 = 10us)
    // =========================================================================
    reg  sda_oe  = 1'b0;
    reg  sda_out = 1'b1;
    assign tp_sda = sda_oe ? sda_out : 1'bz;

    reg [7:0] clk_div = 8'd0;
    wire      i2c_tick = (clk_div == 8'd124);

    always @(posedge clk_50m) begin
        if (!init_ok || clk_div == 8'd124)
            clk_div <= 8'd0;
        else
            clk_div <= clk_div + 8'd1;
    end

    // =========================================================================
    // 3. GT9147_800X480_Config_20190304.cfg (0x8047 ~ 0x8100, 共 186 字节)
    // =========================================================================
    function [7:0] cfg_rom(input [7:0] idx);
        begin
            case (idx)
                8'd0:   cfg_rom = 8'h42; 8'd1:   cfg_rom = 8'h20; 8'd2:   cfg_rom = 8'h03; 8'd3:   cfg_rom = 8'hE0;
                8'd4:   cfg_rom = 8'h01; 8'd5:   cfg_rom = 8'h05; 8'd6:   cfg_rom = 8'h0F; 8'd7:   cfg_rom = 8'h00;
                8'd8:   cfg_rom = 8'h01; 8'd9:   cfg_rom = 8'h08; 8'd10:  cfg_rom = 8'h28; 8'd11:  cfg_rom = 8'h05;
                8'd12:  cfg_rom = 8'h50; 8'd13:  cfg_rom = 8'h32; 8'd14:  cfg_rom = 8'h03; 8'd15:  cfg_rom = 8'h05;
                8'd16:  cfg_rom = 8'h00; 8'd17:  cfg_rom = 8'h00; 8'd18:  cfg_rom = 8'hFF; 8'd19:  cfg_rom = 8'hFF;
                8'd20:  cfg_rom = 8'h00; 8'd21:  cfg_rom = 8'h00; 8'd22:  cfg_rom = 8'h00; 8'd23:  cfg_rom = 8'h00;
                8'd24:  cfg_rom = 8'h00; 8'd25:  cfg_rom = 8'h00; 8'd26:  cfg_rom = 8'h00; 8'd27:  cfg_rom = 8'h89;
                8'd28:  cfg_rom = 8'h28; 8'd29:  cfg_rom = 8'h0A; 8'd30:  cfg_rom = 8'h17; 8'd31:  cfg_rom = 8'h15;
                8'd32:  cfg_rom = 8'h31; 8'd33:  cfg_rom = 8'h0D; 8'd34:  cfg_rom = 8'h00; 8'd35:  cfg_rom = 8'h00;
                8'd36:  cfg_rom = 8'h02; 8'd37:  cfg_rom = 8'h9B; 8'd38:  cfg_rom = 8'h03; 8'd39:  cfg_rom = 8'h25;
                8'd40:  cfg_rom = 8'h00; 8'd41:  cfg_rom = 8'h00; 8'd42:  cfg_rom = 8'h00; 8'd43:  cfg_rom = 8'h00;
                8'd44:  cfg_rom = 8'h00; 8'd45:  cfg_rom = 8'h00; 8'd46:  cfg_rom = 8'h00; 8'd47:  cfg_rom = 8'h32;
                8'd48:  cfg_rom = 8'h00; 8'd49:  cfg_rom = 8'h00; 8'd50:  cfg_rom = 8'h00; 8'd51:  cfg_rom = 8'h0F;
                8'd52:  cfg_rom = 8'h94; 8'd53:  cfg_rom = 8'h94; 8'd54:  cfg_rom = 8'hC5; 8'd55:  cfg_rom = 8'h02;
                8'd56:  cfg_rom = 8'h07; 8'd57:  cfg_rom = 8'h00; 8'd58:  cfg_rom = 8'h00; 8'd59:  cfg_rom = 8'h04;
                8'd60:  cfg_rom = 8'h8D; 8'd61:  cfg_rom = 8'h13; 8'd62:  cfg_rom = 8'h00; 8'd63:  cfg_rom = 8'h5C;
                8'd64:  cfg_rom = 8'h1E; 8'd65:  cfg_rom = 8'h00; 8'd66:  cfg_rom = 8'h3C; 8'd67:  cfg_rom = 8'h30;
                8'd68:  cfg_rom = 8'h00; 8'd69:  cfg_rom = 8'h29; 8'd70:  cfg_rom = 8'h4C; 8'd71:  cfg_rom = 8'h00;
                8'd72:  cfg_rom = 8'h1E; 8'd73:  cfg_rom = 8'h78; 8'd74:  cfg_rom = 8'h00; 8'd75:  cfg_rom = 8'h1E;
                8'd76:  cfg_rom = 8'h00; 8'd77:  cfg_rom = 8'h00; 8'd78:  cfg_rom = 8'h00; 8'd79:  cfg_rom = 8'h00;
                8'd80:  cfg_rom = 8'h00; 8'd81:  cfg_rom = 8'h00; 8'd82:  cfg_rom = 8'h00; 8'd83:  cfg_rom = 8'h00;
                8'd84:  cfg_rom = 8'h00; 8'd85:  cfg_rom = 8'h00; 8'd86:  cfg_rom = 8'h00; 8'd87:  cfg_rom = 8'h00;
                8'd88:  cfg_rom = 8'h00; 8'd89:  cfg_rom = 8'h00; 8'd90:  cfg_rom = 8'h00; 8'd91:  cfg_rom = 8'h00;
                8'd92:  cfg_rom = 8'h00; 8'd93:  cfg_rom = 8'h00; 8'd94:  cfg_rom = 8'h00; 8'd95:  cfg_rom = 8'h00;
                8'd96:  cfg_rom = 8'h00; 8'd97:  cfg_rom = 8'h00; 8'd98:  cfg_rom = 8'h00; 8'd99:  cfg_rom = 8'h00;
                8'd100: cfg_rom = 8'h00; 8'd101: cfg_rom = 8'h00; 8'd102: cfg_rom = 8'h00; 8'd103: cfg_rom = 8'h00;
                8'd104: cfg_rom = 8'h00; 8'd105: cfg_rom = 8'h00; 8'd106: cfg_rom = 8'h00; 8'd107: cfg_rom = 8'h00;
                8'd108: cfg_rom = 8'h00; 8'd109: cfg_rom = 8'h00; 8'd110: cfg_rom = 8'h00; 8'd111: cfg_rom = 8'h00;
                8'd112: cfg_rom = 8'h08; 8'd113: cfg_rom = 8'h0A; 8'd114: cfg_rom = 8'h0C; 8'd115: cfg_rom = 8'h0E;
                8'd116: cfg_rom = 8'h10; 8'd117: cfg_rom = 8'h12; 8'd118: cfg_rom = 8'h14; 8'd119: cfg_rom = 8'h16;
                8'd120: cfg_rom = 8'h18; 8'd121: cfg_rom = 8'h1A; 8'd122: cfg_rom = 8'h00; 8'd123: cfg_rom = 8'h00;
                8'd124: cfg_rom = 8'h00; 8'd125: cfg_rom = 8'h00; 8'd126: cfg_rom = 8'h1F; 8'd127: cfg_rom = 8'hFF;
                8'd128: cfg_rom = 8'hFF; 8'd129: cfg_rom = 8'hFF; 8'd130: cfg_rom = 8'hFF; 8'd131: cfg_rom = 8'hFF;
                8'd132: cfg_rom = 8'hFF; 8'd133: cfg_rom = 8'hFF; 8'd134: cfg_rom = 8'hFF; 8'd135: cfg_rom = 8'hFF;
                8'd136: cfg_rom = 8'hFF; 8'd137: cfg_rom = 8'hFF; 8'd138: cfg_rom = 8'hFF; 8'd139: cfg_rom = 8'hFF;
                8'd140: cfg_rom = 8'hFF; 8'd141: cfg_rom = 8'hFF; 8'd142: cfg_rom = 8'h00; 8'd143: cfg_rom = 8'h02;
                8'd144: cfg_rom = 8'h04; 8'd145: cfg_rom = 8'h05; 8'd146: cfg_rom = 8'h06; 8'd147: cfg_rom = 8'h08;
                8'd148: cfg_rom = 8'h0A; 8'd149: cfg_rom = 8'h0C; 8'd150: cfg_rom = 8'h0E; 8'd151: cfg_rom = 8'h1D;
                8'd152: cfg_rom = 8'h1E; 8'd153: cfg_rom = 8'h1F; 8'd154: cfg_rom = 8'h20; 8'd155: cfg_rom = 8'h22;
                8'd156: cfg_rom = 8'h24; 8'd157: cfg_rom = 8'h28; 8'd158: cfg_rom = 8'h29; 8'd159: cfg_rom = 8'hFF;
                8'd160: cfg_rom = 8'h00; 8'd161: cfg_rom = 8'h00; 8'd162: cfg_rom = 8'h00; 8'd163: cfg_rom = 8'h00;
                8'd164: cfg_rom = 8'h00; 8'd165: cfg_rom = 8'h00; 8'd166: cfg_rom = 8'h00; 8'd167: cfg_rom = 8'h00;
                8'd168: cfg_rom = 8'hFF; 8'd169: cfg_rom = 8'hFF; 8'd170: cfg_rom = 8'hFF; 8'd171: cfg_rom = 8'hFF;
                8'd172: cfg_rom = 8'hFF; 8'd173: cfg_rom = 8'hFF; 8'd174: cfg_rom = 8'hFF; 8'd175: cfg_rom = 8'hFF;
                8'd176: cfg_rom = 8'hFF; 8'd177: cfg_rom = 8'hFF; 8'd178: cfg_rom = 8'hFF; 8'd179: cfg_rom = 8'hFF;
                8'd180: cfg_rom = 8'hFF; 8'd181: cfg_rom = 8'hFF; 8'd182: cfg_rom = 8'hFF; 8'd183: cfg_rom = 8'hFF;
                8'd184: cfg_rom = 8'h1D; // 0x80FF Config_Chksum
                8'd185: cfg_rom = 8'h01; // 0x8100 Config_Fresh
                default: cfg_rom = 8'h00;
            endcase
        end
    endfunction

    // =========================================================================
    // 4. I2C 动作指令译码 (包含上电配置下发 + 1ms 高速坐标轮询)
    // =========================================================================
    localparam [2:0] ACT_IDLE  = 3'd0,
                     ACT_START = 3'd1,
                     ACT_WRITE = 3'd2,
                     ACT_READ  = 3'd3,
                     ACT_STOP  = 3'd4;

    reg [5:0]  step      = 6'd0;
    reg        cfg_done  = 1'b0;
    reg [7:0]  cfg_idx   = 8'd0;
    reg        addr_sel  = 1'b0; // 0: 0x14 (0x28/0x29), 1: 0x5D (0xBA/0xBB)
    wire [7:0] dev_wr_addr = addr_sel ? 8'hBA : 8'h28;
    wire [7:0] dev_rd_addr = addr_sel ? 8'hBB : 8'h29;

    reg [2:0] act_type;
    reg [7:0] tx_byte;
    reg       rd_nack;

    always @(*) begin
        act_type = ACT_IDLE;
        tx_byte  = 8'h00;
        rd_nack  = 1'b0;
        case (step)
            // --- 阶段 A: 写入状态寄存器首地址 0x814E 并发送 STOP 确保建立时间 ---
            6'd1:  begin act_type = ACT_START; end
            6'd2:  begin act_type = ACT_WRITE; tx_byte = dev_wr_addr; end
            6'd3:  begin act_type = ACT_WRITE; tx_byte = 8'h81; end
            6'd4:  begin act_type = ACT_WRITE; tx_byte = 8'h4E; end
            6'd5:  begin act_type = ACT_STOP;  end
            // --- 阶段 B: 重新发送 START 并连读 0x814E ~ 0x8153 ---
            6'd6:  begin act_type = ACT_START; end
            6'd7:  begin act_type = ACT_WRITE; tx_byte = dev_rd_addr; end
            6'd8:  begin act_type = ACT_READ;  rd_nack = 1'b0; end // 0x814E status
            6'd9:  begin act_type = ACT_READ;  rd_nack = 1'b0; end // 0x814F track_id
            6'd10: begin act_type = ACT_READ;  rd_nack = 1'b0; end // 0x8150 X_L
            6'd11: begin act_type = ACT_READ;  rd_nack = 1'b0; end // 0x8151 X_H
            6'd12: begin act_type = ACT_READ;  rd_nack = 1'b0; end // 0x8152 Y_L
            6'd13: begin act_type = ACT_READ;  rd_nack = 1'b1; end // 0x8153 Y_H (NACK)
            6'd14: begin act_type = ACT_STOP;  end
            // --- 阶段 C: 向 0x814E 写 0x00 清除 buffer_status ---
            6'd15: begin act_type = ACT_START; end
            6'd16: begin act_type = ACT_WRITE; tx_byte = dev_wr_addr; end
            6'd17: begin act_type = ACT_WRITE; tx_byte = 8'h81; end
            6'd18: begin act_type = ACT_WRITE; tx_byte = 8'h4E; end
            6'd19: begin act_type = ACT_WRITE; tx_byte = 8'h00; end
            6'd20: begin act_type = ACT_STOP;  end
            // --- 阶段 D: 上电下发 186 字节配置表 (0x8047 ~ 0x8100) ---
            6'd21: begin act_type = ACT_START; end
            6'd22: begin act_type = ACT_WRITE; tx_byte = dev_wr_addr; end
            6'd23: begin act_type = ACT_WRITE; tx_byte = 8'h80; end
            6'd24: begin act_type = ACT_WRITE; tx_byte = 8'h47; end
            6'd25: begin act_type = ACT_WRITE; tx_byte = cfg_rom(cfg_idx); end
            6'd26: begin act_type = ACT_STOP;  end
            default: act_type = ACT_IDLE;
        endcase
    end

    // =========================================================================
    // 5. I2C 状态机与合法坐标锁存 (1ms 轮询间隔 + 800x480 边界过滤)
    // =========================================================================
    reg [19:0] poll_cnt   = 20'd0;
    reg [1:0]  phase      = 2'd0;
    reg [3:0]  bit_idx    = 4'd0;
    reg [7:0]  rx_byte    = 8'd0;
    reg        ack_err    = 1'b0;
    reg [7:0]  status_reg = 8'd0;
    reg [7:0]  track_id   = 8'd0;
    reg [15:0] cur_x      = 16'd0;
    reg [15:0] cur_y      = 16'd0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tp_scl         <= 1'b1;
            sda_oe         <= 1'b0;
            sda_out        <= 1'b1;
            step           <= 6'd0;
            cfg_done       <= 1'b0;
            cfg_idx        <= 8'd0;
            phase          <= 2'd0;
            bit_idx        <= 4'd0;
            poll_cnt       <= 20'd0;
            addr_sel       <= 1'b0;
            ack_err        <= 1'b0;
            status_reg     <= 8'd0;
            track_id       <= 8'd0;
            cur_x          <= 16'd0;
            cur_y          <= 16'd0;
            touch_xy_coord <= 32'd0;
        end else if (init_ok) begin
            if (step == 6'd0) begin
                tp_scl <= 1'b1;
                sda_oe <= 1'b0;
                // 按编程指南第 5 章建议，轮询等待时间设为 1ms (50,000 个 50MHz 周期)
                if (poll_cnt >= 20'd50_000) begin
                    poll_cnt <= 20'd0;
                    phase    <= 2'd0;
                    bit_idx  <= 4'd0;
                    ack_err  <= 1'b0;
                    if (!cfg_done) begin
                        cfg_idx <= 8'd0;
                        step    <= 6'd21; // 先下发 800x480 配置文件
                    end else begin
                        step    <= 6'd1;  // 进入常规坐标轮询
                    end
                end else begin
                    poll_cnt <= poll_cnt + 20'd1;
                end
            end else if (i2c_tick) begin
                case (act_type)
                    ACT_START: begin
                        case (phase)
                            2'd0: begin sda_oe <= 1'b1; sda_out <= 1'b1; tp_scl <= 1'b1; phase <= 2'd1; end
                            2'd1: begin sda_out <= 1'b0; phase <= 2'd2; end
                            2'd2: begin tp_scl  <= 1'b0; phase <= 2'd3; end
                            2'd3: begin phase   <= 2'd0; bit_idx <= 4'd0; step <= step + 6'd1; end
                        endcase
                    end

                    ACT_WRITE: begin
                        case (phase)
                            2'd0: begin
                                tp_scl <= 1'b0;
                                if (bit_idx < 4'd8) begin
                                    sda_oe  <= 1'b1;
                                    sda_out <= tx_byte[7 - bit_idx];
                                end else begin
                                    sda_oe  <= 1'b0;
                                end
                                phase <= 2'd1;
                            end
                            2'd1: begin tp_scl <= 1'b1; phase <= 2'd2; end
                            2'd2: begin
                                tp_scl <= 1'b1;
                                if (bit_idx == 4'd8 && tp_sda == 1'b1)
                                    ack_err <= 1'b1;
                                phase <= 2'd3;
                            end
                            2'd3: begin
                                tp_scl <= 1'b0;
                                phase  <= 2'd0;
                                if (bit_idx == 4'd8) begin
                                    bit_idx <= 4'd0;
                                    if (ack_err && (step == 6'd2 || step == 6'd22)) begin
                                        step <= (step == 6'd2) ? 6'd14 : 6'd26;
                                    end else if (step == 6'd25) begin
                                        if (cfg_idx == 8'd185)
                                            step <= 6'd26;
                                        else
                                            cfg_idx <= cfg_idx + 8'd1;
                                    end else begin
                                        step <= step + 6'd1;
                                    end
                                end else begin
                                    bit_idx <= bit_idx + 4'd1;
                                end
                            end
                        endcase
                    end

                    ACT_READ: begin
                        case (phase)
                            2'd0: begin
                                tp_scl <= 1'b0;
                                if (bit_idx < 4'd8) begin
                                    sda_oe <= 1'b0;
                                end else begin
                                    sda_oe  <= 1'b1;
                                    sda_out <= rd_nack;
                                end
                                phase <= 2'd1;
                            end
                            2'd1: begin tp_scl <= 1'b1; phase <= 2'd2; end
                            2'd2: begin
                                tp_scl <= 1'b1;
                                if (bit_idx < 4'd8)
                                    rx_byte[7 - bit_idx] <= tp_sda;
                                phase <= 2'd3;
                            end
                            2'd3: begin
                                tp_scl <= 1'b0;
                                phase  <= 2'd0;
                                if (bit_idx == 4'd8) begin
                                    bit_idx <= 4'd0;
                                    if (step == 6'd8)  status_reg  <= rx_byte;
                                    if (step == 6'd9)  track_id    <= rx_byte;
                                    if (step == 6'd10) cur_x[7:0]  <= rx_byte;
                                    if (step == 6'd11) cur_x[15:8] <= rx_byte;
                                    if (step == 6'd12) cur_y[7:0]  <= rx_byte;
                                    if (step == 6'd13) cur_y[15:8] <= rx_byte;
                                    step <= step + 6'd1;
                                end else begin
                                    bit_idx <= bit_idx + 4'd1;
                                end
                            end
                        endcase
                    end

                    ACT_STOP: begin
                        case (phase)
                            2'd0: begin tp_scl <= 1'b0; sda_oe <= 1'b1; sda_out <= 1'b0; phase <= 2'd1; end
                            2'd1: begin tp_scl <= 1'b1; phase <= 2'd2; end
                            2'd2: begin sda_out <= 1'b1; phase <= 2'd3; end
                            2'd3: begin
                                phase <= 2'd0;
                                if (step == 6'd5) begin
                                    step <= 6'd6;
                                end else if (step == 6'd26) begin
                                    if (ack_err)
                                        addr_sel <= ~addr_sel;
                                    else
                                        cfg_done <= 1'b1;
                                    step <= 6'd0;
                                end else if (step == 6'd14) begin
                                    if (ack_err) begin
                                        addr_sel <= ~addr_sel;
                                        step     <= 6'd0;
                                    end else if (status_reg[7]) begin
                                        // 排除 HotKnot 接近检测信号 (track_id == 32) 并校验 800x480 边界
                                        if ((status_reg[3:0] != 4'd0) && (track_id != 8'd32)) begin
                                            if (cur_x < 16'd800 && cur_y < 16'd480)
                                                touch_xy_coord <= {1'b1, 5'd0, cur_x[9:0], 6'd0, cur_y[9:0]};
                                        end else begin
                                            touch_xy_coord[31] <= 1'b0;
                                        end
                                        step <= 6'd15; // 写 0x00 清零 0x814E
                                    end else begin
                                        step <= 6'd0;
                                    end
                                end else begin
                                    step <= 6'd0;
                                end
                            end
                        endcase
                    end

                    default: step <= 6'd0;
                endcase
            end
        end
    end

endmodule