`timescale 1ns / 1ps

module adsr_envelope #(
    parameter ACC_WIDTH = 24,       // 相位累加器位宽（决定时间调节的平滑度）
    parameter ENV_WIDTH = 8         // 包络位宽 (8-bit: 0~255)
)(
    input  wire                   clk,
    input  wire                   rst_n,
    
    // 控制接口
    input  wire                   note_on,       // 按键触发信号（长按保持为1，松开为0）
    input  wire [ACC_WIDTH-1:0]   step_a,        // 起音步进值 (越大时间越短)
    input  wire [ACC_WIDTH-1:0]   step_d,        // 衰减步进值
    input  wire [ACC_WIDTH-1:0]   step_r,        // 释放步进值
    input  wire [ENV_WIDTH-1:0]   sustain_level, // 保持电平目标值
    
    // 曲线 ROM 接口 (可连接至高云 BRAM IP)
    output wire [ENV_WIDTH-1:0]   rom_addr,      // 输出给ROM的读地址
    input  wire [ENV_WIDTH-1:0]   rom_data,      // 从ROM读取的曲线幅值
    
    // 包络输出
    output reg  [ENV_WIDTH-1:0]   envelope_out   // 最终送给乘法器的包络值
);

    // FSM 状态编码 (独热码或二进制码皆可，此处用二进制)
    localparam IDLE    = 3'd0;
    localparam ATTACK  = 3'd1;
    localparam DECAY   = 3'd2;
    localparam SUSTAIN = 3'd3;
    localparam RELEASE = 3'd4;

    reg [2:0] current_state, next_state;
    reg [ACC_WIDTH-1:0] phase_acc;        // 24位相位累加器
    reg [ACC_WIDTH-1:0] next_phase_acc;

    // 将累加器的高8位作为ROM地址
    assign rom_addr = phase_acc[ACC_WIDTH-1 : ACC_WIDTH-ENV_WIDTH];

    // 1. 状态机：状态转移同步时序逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            phase_acc <= 0;
        end else begin
            current_state <= next_state;
            phase_acc <= next_phase_acc;
        end
    end

    // 2. 状态机：次态与累加器步进组合逻辑
    always @(*) begin
        // 默认保持当前值，防止锁存器
        next_state = current_state;
        next_phase_acc = phase_acc;

        case (current_state)
            IDLE: begin
                next_phase_acc = 0;
                if (note_on) 
                    next_state = ATTACK;
            end

            ATTACK: begin
                if (!note_on) begin // 中途松开按键，直接进入释放
                    next_state = RELEASE;
                end else begin
                    // 检查累加器是否即将溢出（高8位达到全1即255）
                    if (phase_acc[ACC_WIDTH-1 : ACC_WIDTH-ENV_WIDTH] >= {ENV_WIDTH{1'b1}}) begin
                        next_state = DECAY;
                    end else begin
                        next_phase_acc = phase_acc + step_a; // 正向累加
                    end
                end
            end

            DECAY: begin
                if (!note_on) begin
                    next_state = RELEASE;
                end else begin
                    // 检查当前ROM输出值是否降至设定的保持电平
                    if (rom_data <= sustain_level) begin
                        next_state = SUSTAIN;
                    end else begin
                        // 注意这里是减法！复用同一条上升曲线，倒着读实现衰减
                        if (phase_acc >= step_d) 
                            next_phase_acc = phase_acc - step_d;
                    end
                end
            end

            SUSTAIN: begin
                // 保持电平状态下停止累加，停留在当前地址
                next_phase_acc = phase_acc; 
                if (!note_on) 
                    next_state = RELEASE;
            end

            RELEASE: begin
                if (note_on) begin // 没释放完又按下了，重新触发
                    next_state = ATTACK;
                end else begin
                    // 减到接近0时回到空闲
                    if (phase_acc < step_r || rom_data == 0) begin
                        next_state = IDLE;
                        next_phase_acc = 0;
                    end else begin
                        next_phase_acc = phase_acc - step_r; // 继续倒着读直到归零
                    end
                end
            end

            default: next_state = IDLE;
        endcase
    end

    // 3. 包络输出逻辑
    // 为了防止ROM读取的1个Clock延迟导致状态切换瞬间出现毛刺，通过寄存器打一拍输出
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            envelope_out <= 0;
        end else if (current_state == SUSTAIN) begin
            envelope_out <= sustain_level; // 强制钳位，消除ROM曲线微小误差
        end else begin
            envelope_out <= rom_data;      // 其他状态跟随ROM输出
        end
    end

endmodule