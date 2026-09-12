// ============================================================
// i2c_master.v — 通用 I2C 主控制器 (7位地址, 原语级)
// 位引擎采用"三相"保证 I2C 时序: sda 只在 scl 为低时改变
//   ph0: 拉低scl   ph1: 低相设置数据   ph2: 拉高scl(数据有效)
// 原语: op_start / op_write / op_read / op_stop
// ============================================================
module i2c_master #(
    parameter CLK_FREQ = 50_000_000,
    parameter SCL_FREQ = 400_000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       op_start,
    input  wire       op_write,
    input  wire       op_read,
    input  wire       op_stop,
    input  wire [7:0] wdata,
    input  wire       ack_nack,
    output reg  [7:0] rdata,
    output reg        ack,
    output reg        busy,
    inout  wire       sda,
    output reg        scl
);
    reg sda_o;
    reg sda_oe;
    assign sda = sda_oe ? sda_o : 1'bz;

    localparam HALF = CLK_FREQ / SCL_FREQ / 2;

    reg [15:0] half_cnt;
    reg        half_tick;

    reg [2:0]  state;
    localparam ST_IDLE=0, ST_START=1, ST_WRITE=2, ST_READ=3, ST_STOP=4;

    reg [3:0]  bit_idx;
    reg [7:0]  shift;
    reg [1:0]  ph;          // 0=低相开始 1=低相设数据 2=高相

    // op 上升沿检测(防止 op 保持高电平导致重复触发)
    reg op_start_d, op_write_d, op_read_d, op_stop_d;
    wire op_start_rise = op_start  && !op_start_d;
    wire op_write_rise = op_write  && !op_write_d;
    wire op_read_rise  = op_read   && !op_read_d;
    wire op_stop_rise  = op_stop   && !op_stop_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin half_cnt <= 16'd0; half_tick <= 1'b0; end
        else if (half_cnt == HALF-1) begin half_cnt <= 0; half_tick <= 1'b1; end
        else begin half_cnt <= half_cnt + 1; half_tick <= 1'b0; end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=ST_IDLE; busy<=0; scl<=1; sda_o<=1; sda_oe<=0;
            ack<=0; rdata<=0; bit_idx<=0; shift<=0; ph<=0;
            op_start_d<=0; op_write_d<=0; op_read_d<=0; op_stop_d<=0;
        end else begin
            op_start_d <= op_start;
            op_write_d <= op_write;
            op_read_d  <= op_read;
            op_stop_d  <= op_stop;
            case (state)
                ST_IDLE: begin
                    sda_oe <= 1'b0;
                    if (op_start_rise) begin busy<=1; ph<=0; sda_oe<=1; sda_o<=1; state<=ST_START; end
                    else if (op_write_rise) begin busy<=1; shift<=wdata; bit_idx<=8; ph<=0; state<=ST_WRITE; end
                    else if (op_read_rise) begin busy<=1; shift<=0; bit_idx<=8; ph<=0; sda_oe<=0; state<=ST_READ; end
                    else if (op_stop_rise) begin busy<=1; ph<=0; sda_oe<=1; sda_o<=0; state<=ST_STOP; end
                end

                ST_START: begin
                    // ph0: scl高; ph1: sda拉低(START); ph2: 结束scl拉低
                    if (ph == 0) begin
                        scl <= 1'b1;
                        if (half_tick) ph <= 1;
                    end else if (ph == 1) begin
                        sda_o <= 1'b0;          // sda 1->0 (scl 高) = START
                        if (half_tick) ph <= 2;
                    end else begin
                        scl <= 1'b0;
                        if (half_tick) begin
                            busy <= 0; ph <= 0; state <= ST_IDLE;
                        end
                    end
                end

                ST_WRITE: begin
                    if (bit_idx != 0) begin
                        if (ph == 0) begin
                            scl <= 1'b0;         // 拉低 scl
                            if (half_tick) ph <= 1;
                        end else if (ph == 1) begin
                            sda_oe <= 1'b1;
                            sda_o  <= shift[7];  // 低相设置数据
                            if (half_tick) ph <= 2;
                        end else begin
                            scl <= 1'b1;         // 拉高 scl, 数据有效
                            if (half_tick) begin
                                shift   <= {shift[6:0], 1'b0};
                                bit_idx <= bit_idx - 1;
                                ph <= 0;
                            end
                        end
                    end else begin
                        // ACK 位
                        if (ph == 0) begin
                            scl <= 1'b0;
                            sda_oe <= 1'b0;      // 释放, 等从机应答
                            if (half_tick) ph <= 1;
                        end else if (ph == 1) begin
                            if (half_tick) ph <= 2;
                        end else begin
                            scl <= 1'b1;
                            if (half_tick) begin
                                ack  <= ~sda;
                                busy <= 0;
                                scl  <= 1'b0;
                                state <= ST_IDLE;
                            end
                        end
                    end
                end

                ST_READ: begin
                    if (bit_idx != 0) begin
                        if (ph == 0) begin
                            scl <= 1'b0;
                            if (half_tick) ph <= 1;
                        end else if (ph == 1) begin
                            if (half_tick) ph <= 2;
                        end else begin
                            scl <= 1'b1;
                            if (half_tick) begin
                                shift   <= {shift[6:0], sda};
                                bit_idx <= bit_idx - 1;
                                ph <= 0;
                            end
                        end
                    end else begin
                        // ACK/NACK 位
                        if (ph == 0) begin
                            scl   <= 1'b0;
                            sda_oe <= 1'b1;
                            sda_o  <= ack_nack;
                            if (half_tick) ph <= 1;
                        end else if (ph == 1) begin
                            if (half_tick) ph <= 2;
                        end else begin
                            scl <= 1'b1;
                            if (half_tick) begin
                                rdata <= shift;
                                busy  <= 0;
                                scl   <= 1'b0;
                                sda_oe <= 1'b0;
                                state <= ST_IDLE;
                            end
                        end
                    end
                end

                ST_STOP: begin
                    if (ph == 0) begin
                        scl <= 1'b0;
                        if (half_tick) ph <= 1;
                    end else if (ph == 1) begin
                        if (half_tick) ph <= 2;
                    end else begin
                        scl   <= 1'b1;           // scl 拉高
                        if (half_tick) begin
                            sda_o <= 1'b1;       // sda 0->1 (scl 高) = STOP
                            busy  <= 0;
                            sda_oe <= 1'b0;
                            state <= ST_IDLE;
                        end
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
