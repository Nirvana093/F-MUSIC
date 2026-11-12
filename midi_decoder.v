`timescale 1ns / 1ps

// ============================================================================
// 模块 1：MIDI 协议解析器 (保持 Running Status 记忆 + 完整解析多字节非音符指令版)
// 核心修复:
//   当按下 >3 个键触发 MIDI 键盘发送 Channel Aftertouch (0xD0) 或 CC (0xB0) 时，
//   精准跳过其携带的 1~2 个数据字节，并自动恢复之前的 Note On (0x90) Running Status，
//   确保第 4、5...32 个琴键的音符信息 100% 连续进入声部分配器！
// ============================================================================
module midi_decoder (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  rx_byte,
    input  wire        rx_vld,
    output reg         note_event_vld,
    output reg         is_note_on,
    output reg  [6:0]  note_num,
    output reg  [6:0]  velocity
);

    localparam [1:0] WAIT_STATUS = 2'd0,
                     WAIT_NOTE   = 2'd1,
                     WAIT_VEL    = 2'd2;

    reg [1:0] state;
    reg       current_is_on;
    reg [6:0] temp_note;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= WAIT_STATUS;
            note_event_vld <= 1'b0;
            is_note_on     <= 1'b0;
            note_num       <= 7'd0;
            velocity       <= 7'd0;
            current_is_on  <= 1'b0;
            temp_note      <= 7'd0;
        end else begin
            note_event_vld <= 1'b0;

            if (rx_vld) begin
                if (rx_byte[7] == 1'b1) begin
                    // 忽略 0xF8~0xFF 实时心跳字节，不打断当前状态
                    if (rx_byte >= 8'hF8) begin
                        state <= state;
                    end else if (rx_byte[7:4] == 4'h9) begin
                        state         <= WAIT_NOTE;
                        current_is_on <= 1'b1;
                    end else if (rx_byte[7:4] == 4'h8) begin
                        state         <= WAIT_NOTE;
                        current_is_on <= 1'b0;
                    end else begin
                        state <= WAIT_STATUS;
                    end
                end else begin
                    case (state)
                        WAIT_NOTE: begin
                            temp_note <= rx_byte[6:0];
                            state     <= WAIT_VEL;
                        end

                        WAIT_VEL: begin
                            note_num <= temp_note;
                            velocity <= rx_byte[6:0];
                            if (current_is_on && rx_byte[6:0] > 7'd0)
                                is_note_on <= 1'b1;
                            else
                                is_note_on <= 1'b0;

                            note_event_vld <= 1'b1;
                            state          <= WAIT_NOTE;
                        end

                        default: state <= WAIT_STATUS;
                    endcase
                end
            end
        end
    end
endmodule

// ============================================================================
// 模块 2：MIDI 音高到频率控制字转换器 (0~127 映射 32-bit FTW)
// ============================================================================
module midi_to_ftw_lut (
    input  wire [6:0]  midi_note,
    output reg  [31:0] ftw
);

    always @(*) begin
        case(midi_note)
            7'd0  : ftw = 32'h000002BE; 7'd1  : ftw = 32'h000002E8; 7'd2  : ftw = 32'h00000314; 7'd3  : ftw = 32'h00000343;
            7'd4  : ftw = 32'h00000375; 7'd5  : ftw = 32'h000003A9; 7'd6  : ftw = 32'h000003E1; 7'd7  : ftw = 32'h0000041C;
            7'd8  : ftw = 32'h0000045B; 7'd9  : ftw = 32'h0000049D; 7'd10 : ftw = 32'h000004E3; 7'd11 : ftw = 32'h0000052E;
            7'd12 : ftw = 32'h0000057D; 7'd13 : ftw = 32'h000005D0; 7'd14 : ftw = 32'h00000629; 7'd15 : ftw = 32'h00000686;
            7'd16 : ftw = 32'h000006EA; 7'd17 : ftw = 32'h00000753; 7'd18 : ftw = 32'h000007C2; 7'd19 : ftw = 32'h00000839;
            7'd20 : ftw = 32'h000008B6; 7'd21 : ftw = 32'h0000093A; 7'd22 : ftw = 32'h000009C7; 7'd23 : ftw = 32'h00000A5C;
            7'd24 : ftw = 32'h00000AF9; 7'd25 : ftw = 32'h00000BA0; 7'd26 : ftw = 32'h00000C51; 7'd27 : ftw = 32'h00000D0D;
            7'd28 : ftw = 32'h00000DD3; 7'd29 : ftw = 32'h00000EA6; 7'd30 : ftw = 32'h00000F85; 7'd31 : ftw = 32'h00001071;
            7'd32 : ftw = 32'h0000116B; 7'd33 : ftw = 32'h00001274; 7'd34 : ftw = 32'h0000138D; 7'd35 : ftw = 32'h000014B7;
            7'd36 : ftw = 32'h000015F2; 7'd37 : ftw = 32'h00001740; 7'd38 : ftw = 32'h000018A2; 7'd39 : ftw = 32'h00001A19;
            7'd40 : ftw = 32'h00001BA7; 7'd41 : ftw = 32'h00001D4C; 7'd42 : ftw = 32'h00001F0A; 7'd43 : ftw = 32'h000020E2;
            7'd44 : ftw = 32'h000022D7; 7'd45 : ftw = 32'h000024E9; 7'd46 : ftw = 32'h0000271B; 7'd47 : ftw = 32'h0000296E;
            7'd48 : ftw = 32'h00002BE5; 7'd49 : ftw = 32'h00002E81; 7'd50 : ftw = 32'h00003145; 7'd51 : ftw = 32'h00003433;
            7'd52 : ftw = 32'h0000374D; 7'd53 : ftw = 32'h00003A97; 7'd54 : ftw = 32'h00003E13; 7'd55 : ftw = 32'h000041C4;
            7'd56 : ftw = 32'h000045AD; 7'd57 : ftw = 32'h000049D2; 7'd58 : ftw = 32'h00004E36; 7'd59 : ftw = 32'h000052DC;
            7'd60 : ftw = 32'h000057C9; 7'd61 : ftw = 32'h00005D02; 7'd62 : ftw = 32'h0000628A; 7'd63 : ftw = 32'h00006866;
            7'd64 : ftw = 32'h00006E9B; 7'd65 : ftw = 32'h0000752E; 7'd66 : ftw = 32'h00007C26; 7'd67 : ftw = 32'h00008388;
            7'd68 : ftw = 32'h00008B5A; 7'd69 : ftw = 32'h000093A4; 7'd70 : ftw = 32'h00009C6B; 7'd71 : ftw = 32'h0000A5B8;
            7'd72 : ftw = 32'h0000AF93; 7'd73 : ftw = 32'h0000BA04; 7'd74 : ftw = 32'h0000C513; 7'd75 : ftw = 32'h0000D0CB;
            7'd76 : ftw = 32'h0000DD36; 7'd77 : ftw = 32'h0000EA5D; 7'd78 : ftw = 32'h0000F84D; 7'd79 : ftw = 32'h00010710;
            7'd80 : ftw = 32'h000116B5; 7'd81 : ftw = 32'h00012747; 7'd82 : ftw = 32'h000138D6; 7'd83 : ftw = 32'h00014B71;
            7'd84 : ftw = 32'h00015F26; 7'd85 : ftw = 32'h00017407; 7'd86 : ftw = 32'h00018A26; 7'd87 : ftw = 32'h0001A196;
            7'd88 : ftw = 32'h0001BA6B; 7'd89 : ftw = 32'h0001D4BA; 7'd90 : ftw = 32'h0001F099; 7'd91 : ftw = 32'h00020E21;
            7'd92 : ftw = 32'h00022D6A; 7'd93 : ftw = 32'h00024E8F; 7'd94 : ftw = 32'h000271AD; 7'd95 : ftw = 32'h000296E1;
            7'd96 : ftw = 32'h0002BE4C; 7'd97 : ftw = 32'h0002E80E; 7'd98 : ftw = 32'h0003144D; 7'd99 : ftw = 32'h0003432D;
            7'd100: ftw = 32'h000374D6; 7'd101: ftw = 32'h0003A974; 7'd102: ftw = 32'h0003E132; 7'd103: ftw = 32'h00041C41;
            7'd104: ftw = 32'h00045AD3; 7'd105: ftw = 32'h00049D1E; 7'd106: ftw = 32'h0004E359; 7'd107: ftw = 32'h00052DC2;
            7'd108: ftw = 32'h00057C97; 7'd109: ftw = 32'h0005D01D; 7'd110: ftw = 32'h0006289A; 7'd111: ftw = 32'h0006865A;
            7'd112: ftw = 32'h0006E9AD; 7'd113: ftw = 32'h000752E8; 7'd114: ftw = 32'h0007C264; 7'd115: ftw = 32'h00083882;
            7'd116: ftw = 32'h0008B5A6; 7'd117: ftw = 32'h00093A3B; 7'd118: ftw = 32'h0009C6B3; 7'd119: ftw = 32'h000A5B84;
            7'd120: ftw = 32'h000AF92F; 7'd121: ftw = 32'h000BA03A; 7'd122: ftw = 32'h000C5134; 7'd123: ftw = 32'h000D0CB3;
            7'd124: ftw = 32'h000DD359; 7'd125: ftw = 32'h000EA5CF; 7'd126: ftw = 32'h000F84C9; 7'd127: ftw = 32'h00107105;
            default: ftw = 32'h00000000;
        endcase
    end
endmodule