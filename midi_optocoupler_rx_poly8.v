`timescale 1ns / 1ps
`ifndef MIDI_OPTOCOUPLER_RX_POLY8_V
`define MIDI_OPTOCOUPLER_RX_POLY8_V

// ============================================================================
// 模块名称: midi_optocoupler_rx_poly8 (严格 8 键上限 + 零延迟和弦并发分配版)
// 修复说明:
//   1. 当 8 个声部全部处于按下状态 (voice_note_on == 8'hFF) 时，严格锁定并忽略第 9 个及以上的新按键，
//      绝不抢占正在发声的 8 个琴键，确保最多只有 8 个琴键同时发声；
//   2. 配合 0 延迟 lut_sync_fifo，收齐 3 字节当拍立即分配声部并拉高 voice_retrig!
// ============================================================================
module midi_optocoupler_rx_poly8 #(
    parameter SYS_CLK_FREQ   = 50_000_000,
    parameter MIDI_BAUD      = 31_250,
    parameter SKEW_COMP_CLKS = 200
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         midi_rx_in,
    input  wire [7:0]   voice_env_active,
    output wire [7:0]   last_rx_byte,
    output wire         rx_byte_vld,
    output reg          rx_frame_err,
    output wire [4:0]   fifo_data_cnt,
    output reg  [6:0]   latest_note_num,
    output reg  [6:0]   latest_velocity,
    output reg  [7:0]   voice_note_on,
    output reg  [7:0]   voice_retrig,
    output wire [255:0] voice_ftw_flat,
    output wire [55:0]  voice_vel_flat,
    output reg  [31:0]  voice0_ftw,
    output reg  [31:0]  voice1_ftw,
    output reg  [31:0]  voice2_ftw,
    output reg  [31:0]  voice3_ftw,
    output reg  [31:0]  voice4_ftw,
    output reg  [31:0]  voice5_ftw,
    output reg  [31:0]  voice6_ftw,
    output reg  [31:0]  voice7_ftw,
    output reg  [6:0]   voice0_vel,
    output reg  [6:0]   voice1_vel,
    output reg  [6:0]   voice2_vel,
    output reg  [6:0]   voice3_vel,
    output reg  [6:0]   voice4_vel,
    output reg  [6:0]   voice5_vel,
    output reg  [6:0]   voice6_vel,
    output reg  [6:0]   voice7_vel
);

    localparam integer CLKS_PER_BIT = SYS_CLK_FREQ / MIDI_BAUD;
    localparam integer FLT_MAX      = (CLKS_PER_BIT >= 1600) ? 50 : ((CLKS_PER_BIT / 16 > 2) ? (CLKS_PER_BIT / 16) : 2);
    localparam integer VOTE_STEP    = (CLKS_PER_BIT >= 256)  ? 32 : ((CLKS_PER_BIT / 16 > 1) ? (CLKS_PER_BIT / 16) : 2);
    localparam integer STOP_END_OFF = (CLKS_PER_BIT >= 256)  ? 16 : 2;
    localparam integer SAMPLE_POS_1 = (CLKS_PER_BIT / 2) + SKEW_COMP_CLKS - VOTE_STEP;
    localparam integer SAMPLE_POS_2 = (CLKS_PER_BIT / 2) + SKEW_COMP_CLKS;
    localparam integer SAMPLE_POS_3 = (CLKS_PER_BIT / 2) + SKEW_COMP_CLKS + VOTE_STEP;

    assign voice_ftw_flat = {voice7_ftw, voice6_ftw, voice5_ftw, voice4_ftw,
                             voice3_ftw, voice2_ftw, voice1_ftw, voice0_ftw};
    assign voice_vel_flat = {voice7_vel, voice6_vel, voice5_vel, voice4_vel,
                             voice3_vel, voice2_vel, voice1_vel, voice0_vel};

    // 第一级：跨时钟域同步 + 数字消抖
    reg [2:0] rx_sync     = 3'b111;
    reg [5:0] flt_cnt     = FLT_MAX[5:0];
    reg       rx_clean    = 1'b1;
    reg       rx_clean_d1 = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync     <= 3'b111;
            flt_cnt     <= FLT_MAX[5:0];
            rx_clean    <= 1'b1;
            rx_clean_d1 <= 1'b1;
        end else begin
            rx_sync     <= {rx_sync[1:0], midi_rx_in};
            rx_clean_d1 <= rx_clean;
            if (rx_sync[2] == 1'b1) begin
                if (flt_cnt < FLT_MAX[5:0]) flt_cnt <= flt_cnt + 6'd1;
                else                        rx_clean <= 1'b1;
            end else begin
                if (flt_cnt > 6'd0)         flt_cnt <= flt_cnt - 6'd1;
                else                        rx_clean <= 1'b0;
            end
        end
    end

    wire start_falling_edge = rx_clean_d1 & (~rx_clean);

    // 第二级：6N138 光耦偏置补偿 UART 接收状态机
    localparam [1:0] U_IDLE  = 2'd0,
                     U_START = 2'd1,
                     U_DATA  = 2'd2,
                     U_STOP  = 2'd3;

    reg [1:0]  u_state   = U_IDLE;
    reg [10:0] clk_cnt   = 11'd0;
    reg [2:0]  bit_idx   = 3'd0;
    reg [2:0]  vote_buf  = 3'b111;
    reg [7:0]  rx_shift  = 8'd0;
    reg [7:0]  rx_byte_r = 8'd0;
    reg        rx_vld_r  = 1'b0;

    wire majority_bit = (vote_buf[0] & vote_buf[1]) |
                        (vote_buf[1] & vote_buf[2]) |
                        (vote_buf[0] & vote_buf[2]);

    assign last_rx_byte = rx_byte_r;
    assign rx_byte_vld  = rx_vld_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u_state      <= U_IDLE;
            clk_cnt      <= 11'd0;
            bit_idx      <= 3'd0;
            vote_buf     <= 3'b111;
            rx_shift     <= 8'd0;
            rx_byte_r    <= 8'd0;
            rx_vld_r     <= 1'b0;
            rx_frame_err <= 1'b0;
        end else begin
            rx_vld_r <= 1'b0;
            case (u_state)
                U_IDLE: begin
                    clk_cnt <= 11'd0;
                    bit_idx <= 3'd0;
                    if (start_falling_edge)
                        u_state <= U_START;
                end
                U_START: begin
                    if (clk_cnt == SAMPLE_POS_1) vote_buf[0] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_2) vote_buf[1] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_3) vote_buf[2] <= rx_clean;
                    if (clk_cnt >= CLKS_PER_BIT - 1) begin
                        clk_cnt <= 11'd0;
                        if (majority_bit == 1'b0) begin
                            u_state <= U_DATA;
                            bit_idx <= 3'd0;
                        end else begin
                            u_state <= U_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 11'd1;
                    end
                end
                U_DATA: begin
                    if (clk_cnt == SAMPLE_POS_1) vote_buf[0] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_2) vote_buf[1] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_3) vote_buf[2] <= rx_clean;
                    if (clk_cnt >= CLKS_PER_BIT - 1) begin
                        clk_cnt           <= 11'd0;
                        rx_shift[bit_idx] <= majority_bit;
                        if (bit_idx == 3'd7)
                            u_state <= U_STOP;
                        else
                            bit_idx <= bit_idx + 3'd1;
                    end else begin
                        clk_cnt <= clk_cnt + 11'd1;
                    end
                end
                U_STOP: begin
                    if (clk_cnt == SAMPLE_POS_1) vote_buf[0] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_2) vote_buf[1] <= rx_clean;
                    if (clk_cnt == SAMPLE_POS_3) vote_buf[2] <= rx_clean;
                    if (clk_cnt >= SAMPLE_POS_3 + STOP_END_OFF) begin
                        rx_byte_r    <= rx_shift;
                        rx_vld_r     <= 1'b1;
                        rx_frame_err <= ~majority_bit;
                        u_state      <= U_IDLE;
                        clk_cnt      <= 11'd0;
                    end else begin
                        clk_cnt <= clk_cnt + 11'd1;
                    end
                end
            endcase
        end
    end

    // 0 延迟 FWFT 字节流缓冲 FIFO
    wire [7:0] fifo_rx_byte;
    wire       fifo_rx_empty;
    wire       fifo_rx_rd_en = ~fifo_rx_empty; // 只要非空当拍立即弹出送给 midi_decoder!

    lut_sync_fifo #(
        .DATA_WIDTH (8),
        .ADDR_WIDTH (4),
        .FWFT_MODE  (1)
    ) u_midi_rx_lut_fifo (
        .clk        (clk),
        .rst_n      (rst_n),
        .wr_en      (rx_vld_r & (~rx_frame_err)),
        .din        (rx_byte_r),
        .full       (),
        .rd_en      (fifo_rx_rd_en),
        .dout       (fifo_rx_byte),
        .empty      (fifo_rx_empty),
        .data_count (fifo_data_cnt)
    );

    // 第三级：MIDI 协议解析 + 单路共享 FTW 查表器
    wire        note_event_vld;
    wire        is_note_on;
    wire [6:0]  note_num;
    wire [6:0]  velocity;
    wire [31:0] decoded_note_ftw;

    midi_decoder u_midi_dec (
        .clk            (clk),
        .rst_n          (rst_n),
        .rx_byte        (fifo_rx_byte),
        .rx_vld         (fifo_rx_rd_en),
        .note_event_vld (note_event_vld),
        .is_note_on     (is_note_on),
        .note_num       (note_num),
        .velocity       (velocity)
    );

    midi_to_ftw_lut u_shared_lut (
        .midi_note (note_num),
        .ftw       (decoded_note_ftw)
    );

    // =========================================================================
    // 第四级：严格上限 8 声部硬件复音分配器 (禁止超过 8 键抢占!)
    //   - 优先级 1: 已按下或正在余音中的相同音符重按 -> 直接原位重触发
    //   - 优先级 2: 完全静音的空闲声部 (~voice_note_on & ~voice_env_active)
    //   - 优先级 3: 已松开琴键正处于 Release 余音的声部 (~voice_note_on)
    //   - 严格限制: 若 8 个声部全部处于按下状态 (voice_note_on == 8'hFF)，alloc_valid = 0，忽略第 9+ 个键!
    // =========================================================================
    localparam [31:0] FTW_A4_440HZ = 32'h000093A4;
    (* syn_ramstyle = "registers" *) reg [6:0] voice_note [0:7];

    wire [7:0] w_completely_idle = (~voice_note_on) & (~voice_env_active);
    wire [7:0] w_releasing_idle  = (~voice_note_on);

    reg [7:0] same_held_hit;
    reg [7:0] same_rel_hit;
    integer k;
    always @(*) begin
        for (k = 0; k < 8; k = k + 1) begin
            same_held_hit[k] = voice_note_on[k] && (voice_note[k] == note_num);
            same_rel_hit[k]  = (~voice_note_on[k]) && voice_env_active[k] && (voice_note[k] == note_num);
        end
    end

    reg [2:0] alloc_vid;
    reg       alloc_valid;

    always @(*) begin
        alloc_valid = 1'b1;
        // 1. 同音重按 (已按住的同音符)
        if      (same_held_hit[0]) alloc_vid = 3'd0;
        else if (same_held_hit[1]) alloc_vid = 3'd1;
        else if (same_held_hit[2]) alloc_vid = 3'd2;
        else if (same_held_hit[3]) alloc_vid = 3'd3;
        else if (same_held_hit[4]) alloc_vid = 3'd4;
        else if (same_held_hit[5]) alloc_vid = 3'd5;
        else if (same_held_hit[6]) alloc_vid = 3'd6;
        else if (same_held_hit[7]) alloc_vid = 3'd7;
        // 2. 同音重按 (刚松手尚在余音中的同音符)
        else if (same_rel_hit[0])  alloc_vid = 3'd0;
        else if (same_rel_hit[1])  alloc_vid = 3'd1;
        else if (same_rel_hit[2])  alloc_vid = 3'd2;
        else if (same_rel_hit[3])  alloc_vid = 3'd3;
        else if (same_rel_hit[4])  alloc_vid = 3'd4;
        else if (same_rel_hit[5])  alloc_vid = 3'd5;
        else if (same_rel_hit[6])  alloc_vid = 3'd6;
        else if (same_rel_hit[7])  alloc_vid = 3'd7;
        // 3. 优先寻找完全静音空闲的声部
        else if (w_completely_idle[0]) alloc_vid = 3'd0;
        else if (w_completely_idle[1]) alloc_vid = 3'd1;
        else if (w_completely_idle[2]) alloc_vid = 3'd2;
        else if (w_completely_idle[3]) alloc_vid = 3'd3;
        else if (w_completely_idle[4]) alloc_vid = 3'd4;
        else if (w_completely_idle[5]) alloc_vid = 3'd5;
        else if (w_completely_idle[6]) alloc_vid = 3'd6;
        else if (w_completely_idle[7]) alloc_vid = 3'd7;
        // 4. 次优寻找已松键处于 Release 余音的声部
        else if (w_releasing_idle[0])  alloc_vid = 3'd0;
        else if (w_releasing_idle[1])  alloc_vid = 3'd1;
        else if (w_releasing_idle[2])  alloc_vid = 3'd2;
        else if (w_releasing_idle[3])  alloc_vid = 3'd3;
        else if (w_releasing_idle[4])  alloc_vid = 3'd4;
        else if (w_releasing_idle[5])  alloc_vid = 3'd5;
        else if (w_releasing_idle[6])  alloc_vid = 3'd6;
        else if (w_releasing_idle[7])  alloc_vid = 3'd7;
        // 5. 8 个声部全部处于按下状态 (voice_note_on == 8'hFF)：严格忽略第 9+ 个琴键!
        else begin
            alloc_vid   = 3'd0;
            alloc_valid = 1'b0;
        end
    end

    integer v;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            voice_note_on   <= 8'h00;
            voice_retrig    <= 8'h00;
            for (v = 0; v < 8; v = v + 1) begin
                voice_note[v] <= 7'd127; // 初始化为无效音符编号，杜绝上电误匹配
            end
            voice0_ftw      <= FTW_A4_440HZ;
            voice1_ftw      <= FTW_A4_440HZ;
            voice2_ftw      <= FTW_A4_440HZ;
            voice3_ftw      <= FTW_A4_440HZ;
            voice4_ftw      <= FTW_A4_440HZ;
            voice5_ftw      <= FTW_A4_440HZ;
            voice6_ftw      <= FTW_A4_440HZ;
            voice7_ftw      <= FTW_A4_440HZ;
            voice0_vel      <= 7'd0;
            voice1_vel      <= 7'd0;
            voice2_vel      <= 7'd0;
            voice3_vel      <= 7'd0;
            voice4_vel      <= 7'd0;
            voice5_vel      <= 7'd0;
            voice6_vel      <= 7'd0;
            voice7_vel      <= 7'd0;
            latest_note_num <= 7'd69;
            latest_velocity <= 7'd0;
        end else begin
            voice_retrig <= 8'h00;
            if (note_event_vld) begin
                latest_note_num <= note_num;
                latest_velocity <= velocity;
                if (is_note_on) begin
                    // 仅当未超出 8 键上限 (alloc_valid == 1) 时才分配声部并发声!
                    if (alloc_valid) begin
                        voice_note_on[alloc_vid] <= 1'b1;
                        voice_retrig[alloc_vid]  <= 1'b1;
                        voice_note[alloc_vid]    <= note_num;
                        case (alloc_vid)
                            3'd0: begin voice0_ftw <= decoded_note_ftw; voice0_vel <= velocity; end
                            3'd1: begin voice1_ftw <= decoded_note_ftw; voice1_vel <= velocity; end
                            3'd2: begin voice2_ftw <= decoded_note_ftw; voice2_vel <= velocity; end
                            3'd3: begin voice3_ftw <= decoded_note_ftw; voice3_vel <= velocity; end
                            3'd4: begin voice4_ftw <= decoded_note_ftw; voice4_vel <= velocity; end
                            3'd5: begin voice5_ftw <= decoded_note_ftw; voice5_vel <= velocity; end
                            3'd6: begin voice6_ftw <= decoded_note_ftw; voice6_vel <= velocity; end
                            3'd7: begin voice7_ftw <= decoded_note_ftw; voice7_vel <= velocity; end
                        endcase
                    end
                end else begin
                    for (v = 0; v < 8; v = v + 1) begin
                        if (voice_note_on[v] && (voice_note[v] == note_num))
                            voice_note_on[v] <= 1'b0;
                    end
                end
            end
        end
    end

endmodule
`endif