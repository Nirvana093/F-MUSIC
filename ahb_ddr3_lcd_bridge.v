`timescale 1ns / 1ps

module ahb_ddr3_lcd_bridge (
    // 1. AHB-Lite 从机接口 (对接 GoRV32 EXTS_H* 总线, 50MHz)
    input  wire         ahb_hclk,
    input  wire         ahb_hrstn,
    input  wire         ahb_hsel,
    input  wire [31:0]  ahb_haddr,
    input  wire [1:0]   ahb_htrans,
    input  wire         ahb_hwrite,
    input  wire [2:0]   ahb_hsize,
    input  wire [31:0]  ahb_hwdata,
    input  wire         ahb_hreadyin,
    output reg  [31:0]  ahb_hrdata,
    output wire         ahb_hreadyout,
    output wire [1:0]   ahb_hresp,

    // 2. 调制变量与硬件诊断探针接口
    input  wire [31:0]  touch_xy_coord_in,
    input  wire [31:0]  diag_cc_in,
    output wire [511:0] mod_regs_flat,

    // 3. 诊断状态机触发脉冲输出 (送给 top.v 点亮 LED[5]~LED[8])
    output reg          diag_ahb_hsel_hit,
    output reg          diag_ahb_reg_wr,
    output reg          diag_ahb_vram_wr,
    output reg          diag_ahb_tp_rd_tog,

    // 4. DDR3 Native 用户接口 (100MHz, 128-bit = 8 像素/拍)
    input  wire         ddr_clk,
    input  wire         ddr_rst,
    input  wire         init_calib_complete,
    input  wire         cmd_ready,
    output reg  [2:0]   cmd,
    output reg          cmd_en,
    output reg  [27:0]  addr,
    input  wire         wr_data_rdy,
    output reg  [127:0] wr_data,
    output reg          wr_data_en,
    output reg          wr_data_end,
    output reg  [15:0]  wr_data_mask,
    input  wire [127:0] rd_data,
    input  wire         rd_data_valid,
    input  wire         rd_data_end,
    output wire         sr_req,
    output wire         ref_req,
    output wire         burst,

    // 5. LCD 视频流接口 (33.3MHz)
    input  wire         lcd_clk,
    input  wire         lcd_vsync,
    input  wire         lcd_de,
    output wire [15:0]  lcd_rgb565,
    output reg          init_done
);

    assign sr_req    = 1'b0;
    assign ref_req   = 1'b0;
    assign burst     = 1'b1; // BL8 模式 (128-bit = 8 个 RGB565 像素)
    assign ahb_hresp = 2'b00;

    // =========================================================================
    // 第 1 部分：AHB-Lite 从机接口 + 1KB 硬件栈 SRAM + 2D 硬件图形加速队列 (50MHz 域)
    // =========================================================================
    reg [31:0] mod_regs [0:15];

    genvar g;
    generate
        for (g = 0; g < 16; g = g + 1) begin : GEN_FLAT_REGS
            assign mod_regs_flat[g*32 +: 32] = mod_regs[g];
        end
    endgenerate

    wire ahb_trans_valid = ahb_hsel && ahb_hreadyin && ahb_htrans[1];
    wire is_csr_addr     = ahb_haddr[20]; // bit[20]==1 访问 0xE810_0000 寄存器区与 0xE818_0000 硬件栈区

    reg        ahb_wr_data_phase;
    reg        ahb_is_csr_latch;
    reg [31:0] ahb_addr_latch;
    reg [2:0]  ahb_size_latch;

    // -------------------------------------------------------------------------
    // 新增：256 x 32-bit (1KB) 零等待分布式硬件栈与数据 SRAM (地址: 0xE818_0000 ~ 0xE818_03FF)
    // 支持 8-bit (sb)、16-bit (sh)、32-bit (sw) 写入与组合逻辑零延迟读取
    // -------------------------------------------------------------------------
    (* syn_ramstyle = "distributed_ram", ram_style = "distributed" *)
    reg [31:0] stack_ram [0:255];

    wire [31:0] stack_old_word = stack_ram[ahb_addr_latch[9:2]];
    reg  [31:0] stack_new_word;

    always @(*) begin
        stack_new_word = ahb_hwdata;
        if (ahb_size_latch == 3'b000) begin
            case (ahb_addr_latch[1:0])
                2'b00: stack_new_word = {stack_old_word[31:8],  ahb_hwdata[7:0]};
                2'b01: stack_new_word = {stack_old_word[31:16], ahb_hwdata[15:8],  stack_old_word[7:0]};
                2'b10: stack_new_word = {stack_old_word[31:24], ahb_hwdata[23:16], stack_old_word[15:0]};
                2'b11: stack_new_word = {ahb_hwdata[31:24],     stack_old_word[23:0]};
            endcase
        end else if (ahb_size_latch == 3'b001) begin
            if (ahb_addr_latch[1] == 1'b0)
                stack_new_word = {stack_old_word[31:16], ahb_hwdata[15:0]};
            else
                stack_new_word = {ahb_hwdata[31:16], stack_old_word[15:0]};
        end
    end

    // 传统单次显存写握手信号
    reg        vram_req_tog;
    reg        ddr_ack_tog;
    reg [2:0]  ddr_ack_sync;
    reg        vram_wr_busy;
    reg [19:0] vram_wr_addr;
    reg [15:0] tp_rd_cnt;

    // -------------------------------------------------------------------------
    // 64 深度跨时钟域硬件 2D 绘图指令队列 (Blitter Command Asynchronous FIFO)
    // 每条指令 56-bit: {2'b00, x0[9:0], y0[8:0], w[9:0], h[8:0], color[15:0]}
    // -------------------------------------------------------------------------
    reg  [9:0]  blt_x0_stag;
    reg  [8:0]  blt_y0_stag;
    reg  [9:0]  blt_w_stag;
    reg  [8:0]  blt_h_stag;

    reg  [6:0]  cmd_wr_ptr        = 7'd0;
    reg  [6:0]  cmd_wr_ptr_commit = 7'd0; // 延迟一拍在 RAM 真正完成写入时更新，供 ddr_clk 同步
    reg  [6:0]  cmd_rd_ptr_sync1  = 7'd0, cmd_rd_ptr_sync2 = 7'd0;
    wire [6:0]  cmd_rd_ptr;
    wire        blt_ddr_busy;
    reg  [2:0]  blt_busy_sync = 3'b000;

    wire [6:0]  fifo_used      = cmd_wr_ptr - cmd_rd_ptr_sync2;
    wire        fifo_full      = (fifo_used >= 7'd60);
    wire        fifo_empty_ahb = (cmd_wr_ptr == cmd_rd_ptr_sync2);

    reg         fifo_we;
    reg  [5:0]  fifo_waddr;
    reg  [55:0] fifo_wdata;
    wire        blt_all_busy = (~fifo_empty_ahb) | blt_busy_sync[2] | fifo_we;

    // 当使用硬件 2D 队列或硬件栈 RAM 时，只要 64 级队列未满，CPU 0 等待全速运行
    assign ahb_hreadyout = ~((ahb_wr_data_phase && !ahb_is_csr_latch) | vram_wr_busy | fifo_full);

    always @(posedge ahb_hclk) begin
        if (ahb_wr_data_phase && ahb_is_csr_latch && ahb_addr_latch[19]) begin
            stack_ram[ahb_addr_latch[9:2]] <= stack_new_word;
        end
    end

    always @(posedge ahb_hclk or negedge ahb_hrstn) begin
        if (!ahb_hrstn) begin
            ahb_wr_data_phase  <= 1'b0;
            ahb_is_csr_latch   <= 1'b0;
            ahb_addr_latch     <= 32'd0;
            ahb_size_latch     <= 3'd0;
            vram_req_tog       <= 1'b0;
            vram_wr_busy       <= 1'b0;
            vram_wr_addr       <= 20'd0;
            ddr_ack_sync       <= 3'b000;
            diag_ahb_hsel_hit  <= 1'b0;
            diag_ahb_reg_wr    <= 1'b0;
            diag_ahb_vram_wr   <= 1'b0;
            diag_ahb_tp_rd_tog <= 1'b0;
            tp_rd_cnt          <= 16'd0;
            blt_x0_stag        <= 10'd0;
            blt_y0_stag        <= 9'd0;
            blt_w_stag         <= 10'd0;
            blt_h_stag         <= 9'd0;
            cmd_wr_ptr         <= 7'd0;
            cmd_wr_ptr_commit  <= 7'd0;
            cmd_rd_ptr_sync1   <= 7'd0;
            cmd_rd_ptr_sync2   <= 7'd0;
            blt_busy_sync      <= 3'b000;
            fifo_we            <= 1'b0;
            fifo_waddr         <= 6'd0;
            fifo_wdata         <= 56'd0;

            mod_regs[0]  <= {16'd1, 16'd1};
            mod_regs[1]  <= {16'd1, 16'd0};
            mod_regs[2]  <= {16'd1, 16'd80};
            mod_regs[3]  <= {16'd2, 16'd0};
            mod_regs[4]  <= 32'd50;
            mod_regs[5]  <= {16'd2, 16'd20};
            mod_regs[6]  <= 32'd10;
            mod_regs[7]  <= 32'd30;
            mod_regs[8]  <= 32'd70;
            mod_regs[9]  <= 32'd25;
            mod_regs[10] <= 32'd0;
            mod_regs[11] <= 32'd0;
            mod_regs[12] <= 32'd0;
            mod_regs[13] <= 32'd0;
            mod_regs[14] <= 32'd0;
            mod_regs[15] <= 32'd0;
        end else begin
            fifo_we          <= 1'b0;
            // 在 u_blt_fifo 真正把数据写入 RAM 的这一拍提交写指针给 ddr_clk 域
            if (fifo_we)
                cmd_wr_ptr_commit <= cmd_wr_ptr;

            ddr_ack_sync     <= {ddr_ack_sync[1:0], ddr_ack_tog};
            cmd_rd_ptr_sync1 <= cmd_rd_ptr;
            cmd_rd_ptr_sync2 <= cmd_rd_ptr_sync1;
            blt_busy_sync    <= {blt_busy_sync[1:0], blt_ddr_busy};

            if (vram_wr_busy && (ddr_ack_sync[2] != ddr_ack_sync[1]))
                vram_wr_busy <= 1'b0;

            mod_regs[11] <= touch_xy_coord_in;
            mod_regs[12] <= diag_cc_in;

            // 1. AHB 地址相位锁存 (仅在 ahb_hreadyout 为高时锁存新事务，防止慢速写期间地址被覆盖)
            if (ahb_trans_valid && ahb_hreadyout) begin
                ahb_wr_data_phase <= ahb_hwrite;
                ahb_is_csr_latch  <= is_csr_addr;
                ahb_addr_latch    <= ahb_haddr;
                ahb_size_latch    <= ahb_hsize;

                diag_ahb_hsel_hit <= 1'b1;
                mod_regs[13]      <= mod_regs[13] + 32'd1;
                mod_regs[14]      <= {8'd0, ahb_haddr[31:8]};

                if (!ahb_hwrite && is_csr_addr && !ahb_haddr[19] && (ahb_haddr[5:2] == 4'd11)) begin
                    tp_rd_cnt <= tp_rd_cnt + 16'd1;
                    if (tp_rd_cnt == 16'h01FF)
                        diag_ahb_tp_rd_tog <= ~diag_ahb_tp_rd_tog;
                end
            end else begin
                ahb_wr_data_phase <= 1'b0;
            end

            // 2. AHB 数据相位执行写入
            if (ahb_wr_data_phase) begin
                mod_regs[15] <= {8'd0, ahb_hwdata[23:0]};
                if (ahb_is_csr_latch) begin
                    if (!ahb_addr_latch[19]) begin
                        diag_ahb_reg_wr <= 1'b1;
                        if (ahb_addr_latch[5:2] < 4'd11) begin
                            mod_regs[ahb_addr_latch[5:2]] <= ahb_hwdata;
                        end else if (ahb_addr_latch[5:2] == 4'd12) begin
                            // 0xE810_0030: 锁存矩形左上角坐标 {x0[15:0], y0[15:0]}
                            blt_x0_stag <= ahb_hwdata[25:16];
                            blt_y0_stag <= ahb_hwdata[8:0];
                        end else if (ahb_addr_latch[5:2] == 4'd13) begin
                            // 0xE810_0034: 锁存矩形宽高 {w[15:0], h[15:0]}
                            blt_w_stag  <= ahb_hwdata[25:16];
                            blt_h_stag  <= ahb_hwdata[8:0];
                        end else if (ahb_addr_latch[5:2] == 4'd14) begin
                            // 0xE810_0038: 写入颜色并立即将 2D 绘图指令压入硬件加速队列
                            diag_ahb_vram_wr <= 1'b1;
                            if (blt_w_stag != 10'd0 && blt_h_stag != 9'd0 &&
                                blt_x0_stag < 10'd800 && blt_y0_stag < 9'd480) begin
                                fifo_we    <= 1'b1;
                                fifo_waddr <= cmd_wr_ptr[5:0];
                                fifo_wdata <= {2'b00, blt_x0_stag, blt_y0_stag, blt_w_stag, blt_h_stag, ahb_hwdata[15:0]};
                                cmd_wr_ptr <= cmd_wr_ptr + 7'd1;
                            end
                        end
                    end
                end else begin
                    // 兼容传统 0xE800_0000 显存直接写，同时锁存目标显存块地址
                    diag_ahb_vram_wr <= 1'b1;
                    vram_wr_addr     <= ahb_addr_latch[23:4];
                    vram_req_tog     <= ~vram_req_tog;
                    vram_wr_busy     <= 1'b1;
                end
            end
        end
    end

    always @(*) begin
        if (ahb_is_csr_latch) begin
            if (ahb_addr_latch[19])
                ahb_hrdata = stack_ram[ahb_addr_latch[9:2]];
            else if (ahb_addr_latch[5:2] == 4'd15)
                ahb_hrdata = {30'd0, blt_all_busy, fifo_full};
            else
                ahb_hrdata = mod_regs[ahb_addr_latch[5:2]];
        end else begin
            ahb_hrdata = {31'd0, init_done};
        end
    end

    reg [127:0] ahb_wdata_128;
    reg [15:0]  ahb_wmask_16;

    always @(posedge ahb_hclk or negedge ahb_hrstn) begin
        if (!ahb_hrstn) begin
            ahb_wdata_128 <= 128'd0;
            ahb_wmask_16  <= 16'hFFFF;
        end else if (ahb_wr_data_phase && !ahb_is_csr_latch) begin
            if (ahb_size_latch == 3'b001) begin
                ahb_wdata_128 <= {8{ahb_hwdata[15:0]}};
                ahb_wmask_16  <= ~(16'h0003 << {ahb_addr_latch[3:1], 1'b0});
            end else begin
                ahb_wdata_128 <= {4{ahb_hwdata[31:0]}};
                ahb_wmask_16  <= ~(16'h000F << {ahb_addr_latch[3:2], 2'b00});
            end
        end
    end

    // 例化 64x56-bit 跨时钟域绘图命令缓冲 RAM
    wire [55:0] fifo_rdata;
    reg  [6:0]  ddr_rd_ptr = 7'd0;
    assign cmd_rd_ptr = ddr_rd_ptr;

    blt_cmd_fifo_64x56 u_blt_fifo (
        .wr_clk  (ahb_hclk),
        .wr_en   (fifo_we),
        .wr_addr (fifo_waddr),
        .wr_data (fifo_wdata),
        .rd_clk  (ddr_clk),
        .rd_addr (ddr_rd_ptr[5:0]),
        .rd_data (fifo_rdata)
    );

    // =========================================================================
    // 第 2 部分：LCD 行同步与乒乓半区切换 (lcd_clk 33.3MHz 域)
    // =========================================================================
    reg        lcd_de_d1;
    reg        lcd_bank_sel;
    reg        line_req_tog;

    always @(posedge lcd_clk) begin
        lcd_de_d1 <= lcd_de;
        if (!lcd_vsync) begin
            lcd_bank_sel <= 1'b0;
            line_req_tog <= 1'b0;
        end else begin
            if (!lcd_de_d1 && lcd_de)
                line_req_tog <= ~line_req_tog;
            if (lcd_de_d1 && !lcd_de)
                lcd_bank_sel <= ~lcd_bank_sel;
        end
    end

    reg [6:0] lcd_word_idx;
    reg [2:0] pixel_sub_cnt;

    always @(posedge lcd_clk) begin
        if (!lcd_vsync) begin
            lcd_word_idx  <= 7'd0;
            pixel_sub_cnt <= 3'd0;
        end else if (lcd_de) begin
            pixel_sub_cnt <= pixel_sub_cnt + 3'd1;
            if (pixel_sub_cnt == 3'd7)
                lcd_word_idx <= lcd_word_idx + 7'd1;
        end else begin
            lcd_word_idx  <= 7'd0;
            pixel_sub_cnt <= 3'd0;
        end
    end

    wire [6:0] ram_rd_idx = (!lcd_de) ? 7'd0 :
                            (pixel_sub_cnt == 3'd7) ? (lcd_word_idx + 7'd1) : lcd_word_idx;

    // =========================================================================
    // 第 3 部分：乒乓双缓冲 BSRAM (256 x 128-bit)
    // =========================================================================
    reg          ddr_wr_bank;
    reg  [6:0]   ddr_wr_idx;
    wire [127:0] ram_rd_data;

    pingpong_ram_128x256 u_pingpong_ram (
        .wr_clk  (ddr_clk),
        .wr_en   (rd_data_valid),
        .wr_addr ({ddr_wr_bank, ddr_wr_idx}),
        .wr_data (rd_data),
        .rd_clk  (lcd_clk),
        .rd_addr ({lcd_bank_sel, ram_rd_idx}),
        .rd_data (ram_rd_data)
    );

    // =========================================================================
    // 第 4 部分：100MHz 128-bit 硬件 2D 矩形加速引擎 + 行预取状态机
    // =========================================================================
    reg [1:0] ddr_rst_pipe = 2'b11;
    always @(posedge ddr_clk) begin
        ddr_rst_pipe <= {ddr_rst_pipe[0], ddr_rst};
    end
    wire ddr_rst_local = ddr_rst_pipe[1];

    reg [2:0] vsync_ddr_sync;
    reg [2:0] line_req_sync;
    reg [6:0] wr_ptr_ddr_sync1 = 7'd0, wr_ptr_ddr_sync2 = 7'd0;
    always @(posedge ddr_clk) begin
        vsync_ddr_sync   <= {vsync_ddr_sync[1:0], lcd_vsync};
        line_req_sync    <= {line_req_sync[1:0],  line_req_tog};
        wr_ptr_ddr_sync1 <= cmd_wr_ptr_commit;
        wr_ptr_ddr_sync2 <= wr_ptr_ddr_sync1;
    end

    wire vsync_is_low   = ~vsync_ddr_sync[2];
    wire vsync_pos_edge = (vsync_ddr_sync[2:1] == 2'b01);
    wire line_req_edge  = (line_req_sync[2] != line_req_sync[1]);
    wire fifo_has_cmd   = (ddr_rd_ptr != wr_ptr_ddr_sync2);

    localparam [1:0] S_WAIT_CALIB = 2'd0,
                     S_INIT_VRAM  = 2'd1,
                     S_IDLE       = 2'd2,
                     S_FETCH_LINE = 2'd3;

    reg [1:0]  state;
    reg [27:0] init_addr;
    reg [27:0] lcd_rd_addr;
    reg [6:0]  rd_cmd_cnt;
    reg [2:0]  cmd_gap;
    reg [2:0]  vram_req_sync;
    reg        ahb_wr_pending;
    reg        fetch_line_pending;

    // 2D 硬件 Blitter 内部状态机寄存器
    localparam [1:0] BLT_IDLE  = 2'd0,
                     BLT_LOAD  = 2'd1,
                     BLT_RUN   = 2'd2;

    reg [1:0]  blt_state = BLT_IDLE;
    reg [9:0]  cur_x0, cur_x1;
    reg [6:0]  blk_x_start, blk_x_end, blk_x_cur;
    reg [8:0]  rem_rows;
    reg [27:0] row_base_addr;
    reg [15:0] cur_color;

    assign blt_ddr_busy = (blt_state != BLT_IDLE) | fifo_has_cmd;

    wire [15:0] mask_left_keep  = ~(16'hFFFF << {cur_x0[2:0], 1'b0});
    wire [15:0] mask_right_keep =  (16'hFFFE << {cur_x1[2:0], 1'b1});
    wire [15:0] first_blk_mask  = (blk_x_cur == blk_x_start) ? mask_left_keep  : 16'h0000;
    wire [15:0] last_blk_mask   = (blk_x_cur == blk_x_end)   ? mask_right_keep : 16'h0000;
    wire [15:0] blt_calc_mask   = first_blk_mask | last_blk_mask;

    // 使用 11-bit 与 10-bit 扩展加法防止 (x0 + w - 1) 发生 10-bit 溢出回卷
    wire [9:0]  f_x0       = fifo_rdata[53:44];
    wire [8:0]  f_y0       = fifo_rdata[43:35];
    wire [9:0]  f_w        = fifo_rdata[34:25];
    wire [8:0]  f_h        = fifo_rdata[24:16];
    wire [15:0] f_col      = fifo_rdata[15:0];
    wire [10:0] f_x1_sum   = {1'b0, f_x0} + {1'b0, f_w} - 11'd1;
    wire [9:0]  f_x1_clamp = (f_x1_sum >= 11'd800) ? 10'd799 : f_x1_sum[9:0];
    wire [9:0]  f_y1_sum   = {1'b0, f_y0} + {1'b0, f_h};
    wire [8:0]  f_h_clamp  = (f_y1_sum > 10'd480) ? (9'd480 - f_y0) : f_h;

    always @(posedge ddr_clk) begin
        if (ddr_rst_local || vsync_is_low) begin
            ddr_wr_idx <= 7'd0;
        end else if (state == S_IDLE && (vsync_pos_edge || (fetch_line_pending && (lcd_rd_addr < 28'd384000)))) begin
            ddr_wr_idx <= 7'd0;
        end else if (rd_data_valid) begin
            ddr_wr_idx <= ddr_wr_idx + 7'd1;
        end
    end

    always @(posedge ddr_clk) begin
        if (ddr_rst_local) begin
            vram_req_sync      <= 3'b000;
            ahb_wr_pending     <= 1'b0;
            ddr_ack_tog        <= 1'b0;
            state              <= S_WAIT_CALIB;
            init_addr          <= 28'd0;
            lcd_rd_addr        <= 28'd0;
            ddr_wr_bank        <= 1'b0;
            rd_cmd_cnt         <= 7'd0;
            cmd_gap            <= 3'd0;
            fetch_line_pending <= 1'b0;
            init_done          <= 1'b0;
            cmd                <= 3'b000;
            cmd_en             <= 1'b0;
            addr               <= 28'd0;
            wr_data            <= 128'd0;
            wr_data_en         <= 1'b0;
            wr_data_end        <= 1'b0;
            wr_data_mask       <= 16'h0000;
            ddr_rd_ptr         <= 7'd0;
            blt_state          <= BLT_IDLE;
            cur_x0             <= 10'd0;
            cur_x1             <= 10'd0;
            blk_x_start        <= 7'd0;
            blk_x_end          <= 7'd0;
            blk_x_cur          <= 7'd0;
            rem_rows           <= 9'd0;
            row_base_addr      <= 28'd0;
            cur_color          <= 16'd0;
        end else begin
            cmd_en      <= 1'b0;
            wr_data_en  <= 1'b0;
            wr_data_end <= 1'b0;

            if (cmd_gap != 3'd0)
                cmd_gap <= cmd_gap - 3'd1;

            vram_req_sync <= {vram_req_sync[1:0], vram_req_tog};
            if (vram_req_sync[2] != vram_req_sync[1])
                ahb_wr_pending <= 1'b1;

            if (line_req_edge)
                fetch_line_pending <= 1'b1;

            // 2D Blitter 指令装载状态机
            if (blt_state == BLT_IDLE && fifo_has_cmd) begin
                blt_state <= BLT_LOAD;
            end else if (blt_state == BLT_LOAD) begin
                cur_x0        <= f_x0;
                cur_x1        <= f_x1_clamp;
                blk_x_start   <= f_x0[9:3];
                blk_x_cur     <= f_x0[9:3];
                blk_x_end     <= f_x1_clamp[9:3];
                rem_rows      <= f_h_clamp;
                row_base_addr <= f_y0 * 28'd800;
                cur_color     <= f_col;
                ddr_rd_ptr    <= ddr_rd_ptr + 7'd1;
                blt_state     <= BLT_RUN;
            end

            case (state)
                S_WAIT_CALIB: begin
                    if (init_calib_complete) begin
                        state     <= S_INIT_VRAM;
                        init_addr <= 28'd0;
                        cmd_gap   <= 3'd4;
                    end
                end

                S_INIT_VRAM: begin
                    if (cmd_ready && wr_data_rdy && (cmd_gap == 3'd0)) begin
                        cmd          <= 3'b000;
                        cmd_en       <= 1'b1;
                        addr         <= init_addr;
                        wr_data_en   <= 1'b1;
                        wr_data_end  <= 1'b1;
                        wr_data_mask <= 16'h0000;
                        cmd_gap      <= 3'd1;
                        wr_data      <= {8{16'h08A5}};

                        if (init_addr >= 28'd383992) begin
                            init_done <= 1'b1;
                            state     <= S_IDLE;
                        end else begin
                            init_addr <= init_addr + 28'd8;
                        end
                    end
                end

                S_IDLE: begin
                    if (vsync_is_low) begin
                        lcd_rd_addr        <= 28'd0;
                        ddr_wr_bank        <= 1'b0;
                        fetch_line_pending <= 1'b0;
                    end

                    // 优先级 1：LCD 行同步预取 (确保屏幕显示 100% 稳定无撕裂)
                    if (!vsync_is_low && vsync_pos_edge) begin
                        ddr_wr_bank <= 1'b0;
                        rd_cmd_cnt  <= 7'd0;
                        cmd_gap     <= 3'd1;
                        state       <= S_FETCH_LINE;
                    end else if (!vsync_is_low && fetch_line_pending) begin
                        fetch_line_pending <= 1'b0;
                        if (lcd_rd_addr < 28'd384000) begin
                            ddr_wr_bank <= ~ddr_wr_bank;
                            rd_cmd_cnt  <= 7'd0;
                            cmd_gap     <= 3'd1;
                            state       <= S_FETCH_LINE;
                        end
                    end
                    // 优先级 2：100MHz 硬件 2D 矩形加速引擎
                    else if ((blt_state == BLT_RUN) && cmd_ready && wr_data_rdy && (cmd_gap == 3'd0)) begin
                        cmd          <= 3'b000;
                        cmd_en       <= 1'b1;
                        addr         <= row_base_addr + {18'd0, blk_x_cur, 3'b000};
                        wr_data      <= {8{cur_color}};
                        wr_data_en   <= 1'b1;
                        wr_data_end  <= 1'b1;
                        wr_data_mask <= blt_calc_mask;
                        cmd_gap      <= 3'd1;

                        if (blk_x_cur >= blk_x_end) begin
                            if (rem_rows <= 9'd1) begin
                                blt_state <= BLT_IDLE;
                            end else begin
                                rem_rows      <= rem_rows - 9'd1;
                                row_base_addr <= row_base_addr + 28'd800;
                                blk_x_cur     <= blk_x_start;
                            end
                        end else begin
                            blk_x_cur <= blk_x_cur + 7'd1;
                        end
                    end
                    // 优先级 3：传统 AHB 单次直接写显存 (仅当 2D 队列完全空闲时执行，杜绝乱序覆盖!)
                    else if (!blt_ddr_busy && ahb_wr_pending && cmd_ready && wr_data_rdy && (cmd_gap == 3'd0)) begin
                        cmd            <= 3'b000;
                        cmd_en         <= 1'b1;
                        addr           <= {5'd0, vram_wr_addr, 3'b000};
                        wr_data        <= ahb_wdata_128;
                        wr_data_en     <= 1'b1;
                        wr_data_end    <= 1'b1;
                        wr_data_mask   <= ahb_wmask_16;
                        cmd_gap        <= 3'd1;
                        ahb_wr_pending <= 1'b0;
                        ddr_ack_tog    <= ~ddr_ack_tog;
                    end
                end

                S_FETCH_LINE: begin
                    if (vsync_is_low) begin
                        state <= S_IDLE;
                    end else if (cmd_ready && (cmd_gap == 3'd0)) begin
                        cmd         <= 3'b001;
                        cmd_en      <= 1'b1;
                        addr        <= lcd_rd_addr;
                        lcd_rd_addr <= lcd_rd_addr + 28'd8;
                        cmd_gap     <= 3'd1;
                        if (rd_cmd_cnt == 7'd99)
                            state <= S_IDLE;
                        else
                            rd_cmd_cnt <= rd_cmd_cnt + 7'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // 第 5 部分：LCD 像素拆包输出
    // =========================================================================
    reg [15:0] current_pixel;
    always @(*) begin
        case (pixel_sub_cnt)
            3'd0: current_pixel = ram_rd_data[15:0];
            3'd1: current_pixel = ram_rd_data[31:16];
            3'd2: current_pixel = ram_rd_data[47:32];
            3'd3: current_pixel = ram_rd_data[63:48];
            3'd4: current_pixel = ram_rd_data[79:64];
            3'd5: current_pixel = ram_rd_data[95:80];
            3'd6: current_pixel = ram_rd_data[111:96];
            3'd7: current_pixel = ram_rd_data[127:112];
        endcase
    end

    assign lcd_rgb565 = lcd_de ? current_pixel : 16'h0000;

endmodule

// 64 x 56-bit 硬件 2D 绘图队列双端口 RAM
module blt_cmd_fifo_64x56 (
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [5:0]  wr_addr,
    input  wire [55:0] wr_data,
    input  wire        rd_clk,
    input  wire [5:0]  rd_addr,
    output reg  [55:0] rd_data
);
    reg [55:0] mem [0:63];
    always @(posedge wr_clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end
    always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
    end
endmodule

module pingpong_ram_128x256 (
    input  wire         wr_clk,
    input  wire         wr_en,
    input  wire [7:0]   wr_addr,
    input  wire [127:0] wr_data,
    input  wire         rd_clk,
    input  wire [7:0]   rd_addr,
    output reg  [127:0] rd_data
);
    reg [127:0] mem [0:255];
    always @(posedge wr_clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end
    always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
    end
endmodule