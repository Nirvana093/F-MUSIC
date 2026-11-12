`timescale 1ns / 1ps
`ifndef MIDI_OPTOCOUPLER_RX_POLY32_V
`define MIDI_OPTOCOUPLER_RX_POLY32_V

// ============================================================================
// 模块名称: midi_optocoupler_rx_poly32 (恢复原版 6N138 稳定采样时序 + 严格帧错误过滤版)
// ============================================================================
module midi_optocoupler_rx_poly32 #(
    parameter SYS_CLK_FREQ   = 50_000_000,
    parameter MIDI_BAUD      = 31_250,
    parameter SKEW_COMP_CLKS = 200
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          midi_rx_in,
    input  wire [31:0]   voice_env_active,
    output wire [7:0]    last_rx_byte,
    output wire          rx_byte_vld,
    output reg           rx_frame_err,
    output wire [7:0]    fifo_data_cnt,
    output reg  [6:0]    latest_note_num,
    output reg  [6:0]    latest_velocity,
    output reg  [31:0]   voice_note_on,
    output reg  [31:0]   voice_retrig,
    output wire [1023:0] voice_ftw_flat,
    output wire [223:0]  voice_vel_flat,
    output wire [31:0]   voice0_ftw,
    output wire [6:0]    voice0_vel
);

    localparam integer CLKS_PER_BIT = SYS_CLK_FREQ / MIDI_BAUD;
    localparam integer FLT_MAX      = (CLKS_PER_BIT >= 1600) ? 50 : ((CLKS_PER_BIT / 16 > 2) ? (CLKS_PER_BIT / 16) : 2);
    localparam integer VOTE_STEP    = (CLKS_PER_BIT >= 256)  ? 32 : ((CLKS_PER_BIT / 16 > 1) ? (CLKS_PER_BIT / 16) : 2);
    localparam integer STOP_END_OFF = (CLKS_PER_BIT >= 256)  ? 16 : 2;
    localparam integer SAMPLE_POS_1 = (CLKS_PER_BIT / 2) + SKEW_COMP_CLKS - VOTE_STEP;
    localparam integer SAMPLE_POS_2 = (CLKS_PER_BIT / 2) + SKEW_COMP_CLKS;
    localparam integer SAMPLE_POS_3 = (CLKS_PER_BIT / 2) + SKEW_COMP_CLKS + VOTE_STEP;

    // 1. 跨时钟域同步 + 50拍数字消抖
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

    // 2. 6N138 光耦偏置补偿 UART 接收状态机 (与 midi_optocoupler_rx_poly8 完全一致)
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

    // 3. 16 深度 (ADDR_WIDTH = 4) 0 延迟 FWFT 字节流缓冲 FIFO (严格拦截 rx_frame_err!)
    wire [7:0] fifo_rx_byte;
    wire       fifo_rx_empty;
    wire       fifo_rx_rd_en = ~fifo_rx_empty;
    wire [4:0] fifo_cnt_5b;
    assign fifo_data_cnt = {3'd0, fifo_cnt_5b};

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
        .data_count (fifo_cnt_5b)
    );

    // 4. MIDI 协议解析 + 单路共享 FTW 查表器
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

    // 5. 32 声部四级优先级硬件分配器
    localparam [31:0] FTW_A4_440HZ = 32'h000093A4;

    (* syn_ramstyle = "registers" *) reg [6:0]  voice_note [0:31];
    (* syn_ramstyle = "registers" *) reg [31:0] voice_ftw  [0:31];
    (* syn_ramstyle = "registers" *) reg [6:0]  voice_vel  [0:31];

    genvar g;
    generate
        for (g = 0; g < 32; g = g + 1) begin : GEN_FLAT_32
            assign voice_ftw_flat[g*32 +: 32] = voice_ftw[g];
            assign voice_vel_flat[g*7  +: 7]  = voice_vel[g];
        end
    endgenerate

    assign voice0_ftw = voice_ftw[0];
    assign voice0_vel = voice_vel[0];

    wire [31:0] w_completely_idle = (~voice_note_on) & (~voice_env_active);
    wire [31:0] w_releasing_idle  = (~voice_note_on);

    reg [31:0] same_held_hit;
    reg [31:0] same_rel_hit;
    integer k;
    always @(*) begin
        for (k = 0; k < 32; k = k + 1) begin
            same_held_hit[k] = voice_note_on[k] && (voice_note[k] == note_num);
            same_rel_hit[k]  = (~voice_note_on[k]) && voice_env_active[k] && (voice_note[k] == note_num);
        end
    end

    wire [31:0] target_mask = (|same_held_hit)     ? same_held_hit     :
                              (|same_rel_hit)      ? same_rel_hit      :
                              (|w_completely_idle) ? w_completely_idle :
                                                     w_releasing_idle;

    wire alloc_valid = (|same_held_hit) | (|same_rel_hit) | (|w_completely_idle) | (|w_releasing_idle);

    reg [4:0] alloc_vid;
    always @(*) begin
        if      (target_mask[0])  alloc_vid = 5'd0;
        else if (target_mask[1])  alloc_vid = 5'd1;
        else if (target_mask[2])  alloc_vid = 5'd2;
        else if (target_mask[3])  alloc_vid = 5'd3;
        else if (target_mask[4])  alloc_vid = 5'd4;
        else if (target_mask[5])  alloc_vid = 5'd5;
        else if (target_mask[6])  alloc_vid = 5'd6;
        else if (target_mask[7])  alloc_vid = 5'd7;
        else if (target_mask[8])  alloc_vid = 5'd8;
        else if (target_mask[9])  alloc_vid = 5'd9;
        else if (target_mask[10]) alloc_vid = 5'd10;
        else if (target_mask[11]) alloc_vid = 5'd11;
        else if (target_mask[12]) alloc_vid = 5'd12;
        else if (target_mask[13]) alloc_vid = 5'd13;
        else if (target_mask[14]) alloc_vid = 5'd14;
        else if (target_mask[15]) alloc_vid = 5'd15;
        else if (target_mask[16]) alloc_vid = 5'd16;
        else if (target_mask[17]) alloc_vid = 5'd17;
        else if (target_mask[18]) alloc_vid = 5'd18;
        else if (target_mask[19]) alloc_vid = 5'd19;
        else if (target_mask[20]) alloc_vid = 5'd20;
        else if (target_mask[21]) alloc_vid = 5'd21;
        else if (target_mask[22]) alloc_vid = 5'd22;
        else if (target_mask[23]) alloc_vid = 5'd23;
        else if (target_mask[24]) alloc_vid = 5'd24;
        else if (target_mask[25]) alloc_vid = 5'd25;
        else if (target_mask[26]) alloc_vid = 5'd26;
        else if (target_mask[27]) alloc_vid = 5'd27;
        else if (target_mask[28]) alloc_vid = 5'd28;
        else if (target_mask[29]) alloc_vid = 5'd29;
        else if (target_mask[30]) alloc_vid = 5'd30;
        else                      alloc_vid = 5'd31;
    end

    integer v;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            voice_note_on   <= 32'd0;
            voice_retrig    <= 32'd0;
            latest_note_num <= 7'd69;
            latest_velocity <= 7'd0;
            for (v = 0; v < 32; v = v + 1) begin
                voice_note[v] <= 7'd127;
                voice_ftw[v]  <= FTW_A4_440HZ;
                voice_vel[v]  <= 7'd0;
            end
        end else begin
            voice_retrig <= 32'd0;
            if (note_event_vld) begin
                latest_note_num <= note_num;
                latest_velocity <= velocity;
                if (is_note_on) begin
                    if (alloc_valid) begin
                        voice_note_on[alloc_vid] <= 1'b1;
                        voice_retrig[alloc_vid]  <= 1'b1;
                        voice_note[alloc_vid]    <= note_num;
                        voice_ftw[alloc_vid]     <= decoded_note_ftw;
                        voice_vel[alloc_vid]     <= velocity;
                    end
                end else begin
                    for (v = 0; v < 32; v = v + 1) begin
                        if (voice_note_on[v] && (voice_note[v] == note_num))
                            voice_note_on[v] <= 1'b0;
                    end
                end
            end
        end
    end

endmodule
`endif