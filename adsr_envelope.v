      `timescale 1ns / 1ps

// ============================================================================
// 模块名称: adsr_envelope (无爆音平滑重触发 Soft Re-Trigger + 64分频音乐时基版)
// ============================================================================
module adsr_envelope #(
    parameter integer ACC_WIDTH = 24,
    parameter integer ENV_WIDTH = 8
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 note_on,
    input  wire                 retrig_pulse,
    input  wire [ACC_WIDTH-1:0] step_a,
    input  wire [ACC_WIDTH-1:0] step_d,
    input  wire [ACC_WIDTH-1:0] step_r,
    input  wire [ENV_WIDTH-1:0] sustain_level,
    output wire [ENV_WIDTH-1:0] rom_addr,
    input  wire [ENV_WIDTH-1:0] rom_data,
    output reg  [ENV_WIDTH-1:0] envelope_out,
    output wire                 env_active
);

    localparam [2:0] S_IDLE    = 3'd0,
                     S_ATTACK  = 3'd1,
                     S_DECAY   = 3'd2,
                     S_SUSTAIN = 3'd3,
                     S_RELEASE = 3'd4;

    localparam [ACC_WIDTH-1:0] MAX_ACC_VAL = {ACC_WIDTH{1'b1}};

    reg [2:0]           state        = S_IDLE;
    reg [ACC_WIDTH-1:0] env_acc      = {ACC_WIDTH{1'b0}};
    reg                 note_on_d1   = 1'b0;
    reg [5:0]           env_tick_cnt = 6'd0;
    wire                env_tick     = (env_tick_cnt == 6'd63);

    wire note_rising = (note_on & (~note_on_d1)) | retrig_pulse;
    assign env_active = (state != S_IDLE) | note_on;

    wire [ACC_WIDTH-1:0] sus_acc_target = {sustain_level, {(ACC_WIDTH-ENV_WIDTH){1'b0}}};
    wire [ACC_WIDTH-1:0] eff_step_a     = (step_a == {ACC_WIDTH{1'b0}}) ? 24'd64 : step_a;
    wire [ACC_WIDTH-1:0] eff_step_d     = (step_d == {ACC_WIDTH{1'b0}}) ? 24'd64 : step_d;
    wire [ACC_WIDTH-1:0] eff_step_r     = (step_r == {ACC_WIDTH{1'b0}}) ? 24'd64 : step_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            env_acc      <= {ACC_WIDTH{1'b0}};
            note_on_d1   <= 1'b0;
            env_tick_cnt <= 6'd0;
            envelope_out <= {ENV_WIDTH{1'b0}};
        end else begin
            note_on_d1   <= note_on;
            env_tick_cnt <= env_tick_cnt + 6'd1;
            envelope_out <= rom_data;

            case (state)
                S_IDLE: begin
                    env_acc <= {ACC_WIDTH{1'b0}};
                    if (note_on || retrig_pulse) begin
                        state <= S_ATTACK;
                    end
                end

                S_ATTACK: begin
                    if (!note_on && !retrig_pulse) begin
                        state <= S_RELEASE;
                    end else if (env_tick) begin
                        if ({1'b0, env_acc} + {1'b0, eff_step_a} >= {1'b0, MAX_ACC_VAL}) begin
                            env_acc <= MAX_ACC_VAL;
                            state   <= S_DECAY;
                        end else begin
                            env_acc <= env_acc + eff_step_a;
                        end
                    end
                end

                S_DECAY: begin
                    if (!note_on && !retrig_pulse) begin
                        state <= S_RELEASE;
                    end else if (note_rising) begin
                        // 平滑重触发：直接从当前包络高度重新向上起音，消除 >>3 陡降引起的爆音!
                        state <= S_ATTACK;
                    end else if (env_tick) begin
                        if (env_acc <= sus_acc_target + eff_step_d) begin
                            env_acc <= sus_acc_target;
                            state   <= S_SUSTAIN;
                        end else begin
                            env_acc <= env_acc - eff_step_d;
                        end
                    end
                end

                S_SUSTAIN: begin
                    if (!note_on && !retrig_pulse) begin
                        state <= S_RELEASE;
                    end else if (note_rising) begin
                        state <= S_ATTACK;
                    end else begin
                        env_acc <= sus_acc_target;
                    end
                end

                S_RELEASE: begin
                    if (note_on || retrig_pulse) begin
                        state <= S_ATTACK;
                    end else if (env_tick) begin
                        if (env_acc <= eff_step_r) begin
                            env_acc <= {ACC_WIDTH{1'b0}};
                            state   <= S_IDLE;
                        end else begin
                            env_acc <= env_acc - eff_step_r;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    assign rom_addr = env_acc[ACC_WIDTH-1 : ACC_WIDTH-ENV_WIDTH];

endmodule