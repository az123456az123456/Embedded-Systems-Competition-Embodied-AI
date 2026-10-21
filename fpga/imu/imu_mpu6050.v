// ============================================================
// imu_mpu6050.v — MPU6050 封装
// 初始化(唤醒) -> 周期读加速度计 6 字节(0x3B~0x40)
// 握手: 发起op -> 等busy变高 -> 等busy变低(op_done) -> 下一步
// ============================================================
module imu_mpu6050 #(
    parameter I2C_ADDR = 7'h68
)(
    input  wire               clk,
    input  wire               rst_n,
    inout  wire               sda,
    output wire               scl,
    output reg  signed [15:0] accel_x,
    output reg  signed [15:0] accel_y,
    output reg  signed [15:0] accel_z,
    output reg                data_valid,
    output reg                busy
);
    localparam OP_START=2'd0, OP_WRITE=2'd1, OP_READ=2'd2, OP_STOP=2'd3;

    reg         op_start, op_write, op_read, op_stop;
    reg  [7:0]  wdata;
    reg         ack_nack;
    wire [7:0]  rdata;
    wire        ack;
    wire        mbusy;

    i2c_master #(.CLK_FREQ(50_000_000), .SCL_FREQ(400_000)) u_i2c (
        .clk(clk), .rst_n(rst_n),
        .op_start(op_start), .op_write(op_write), .op_read(op_read), .op_stop(op_stop),
        .wdata(wdata), .ack_nack(ack_nack),
        .rdata(rdata), .ack(ack), .busy(mbusy),
        .sda(sda), .scl(scl)
    );

    reg [2:0] state;
    localparam ST_IDLE=0, ST_INIT=1, ST_READ=2;
    reg [3:0] step;
    reg       iss;          // 本步已发出操作(防重复)
    reg       mbusy_d;
    wire      op_done = mbusy_d && !mbusy;    // mbusy 下降沿
    reg [3:0] byte_cnt;
    reg [7:0] rd_buf [0:5];   // 读回 6 字节缓冲

    // 执行一个操作并推进到 nxt 步
    task run_op(input [1:0] op, input [7:0] d, input ackbit, input [3:0] nxt);
        begin
            if (op_done) begin
                op_start <= 1'b0; op_write <= 1'b0; op_read <= 1'b0; op_stop <= 1'b0;
                iss  <= 1'b0;
                step <= nxt;
            end else if (!mbusy && !iss) begin
                wdata  <= d;
                ack_nack <= ackbit;
                case (op)
                    OP_START: op_start <= 1'b1;
                    OP_WRITE: op_write <= 1'b1;
                    OP_READ : op_read  <= 1'b1;
                    OP_STOP : op_stop  <= 1'b1;
                endcase
                iss <= 1'b1;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=ST_IDLE; step<=0; iss<=0; byte_cnt<=0; mbusy_d<=0;
            accel_x<=0; accel_y<=0; accel_z<=0; data_valid<=0; busy<=0;
            op_start<=0; op_write<=0; op_read<=0; op_stop<=0; wdata<=0; ack_nack<=0;
        end else begin
            mbusy_d <= mbusy;
            data_valid <= 1'b0;
            case (state)
                ST_IDLE: begin
                    if (!mbusy) begin
                        busy <= 1'b1;
                        step <= 4'd0;
                        iss  <= 1'b0;
                        state <= ST_INIT;
                    end
                end

                ST_INIT: begin
                    case (step)
                        4'd0: run_op(OP_START, 8'h00, 0, 4'd1);
                        4'd1: run_op(OP_WRITE, {I2C_ADDR,1'b0}, 0, 4'd2);  // addr+W
                        4'd2: run_op(OP_WRITE, 8'h6B, 0, 4'd3);           // PWR_MGMT_1
                        4'd3: run_op(OP_WRITE, 8'h00, 0, 4'd4);           // 唤醒
                        4'd4: run_op(OP_STOP, 8'h00, 0, 4'd5);
                        4'd5: begin
                            op_stop <= 0; step <= 0; iss <= 0; state <= ST_READ;
                        end
                    endcase
                end

                ST_READ: begin
                    case (step)
                        4'd0 : run_op(OP_START, 8'h00, 0, 4'd1);
                        4'd1 : run_op(OP_WRITE, {I2C_ADDR,1'b0}, 0, 4'd2);  // addr+W
                        4'd2 : run_op(OP_WRITE, 8'h3B, 0, 4'd3);           // 起始寄存器
                        4'd3 : run_op(OP_STOP, 8'h00, 0, 4'd4);
                        4'd4 : run_op(OP_START, 8'h00, 0, 4'd5);           // repeat start
                        4'd5 : run_op(OP_WRITE, {I2C_ADDR,1'b1}, 0, 4'd6); // addr+R
                        4'd6 : begin run_op(OP_READ, 8'h00, 0, 4'd7);  if (op_done) rd_buf[0] <= rdata; end
                        4'd7 : begin run_op(OP_READ, 8'h00, 0, 4'd8);  if (op_done) rd_buf[1] <= rdata; end
                        4'd8 : begin run_op(OP_READ, 8'h00, 0, 4'd9);  if (op_done) rd_buf[2] <= rdata; end
                        4'd9 : begin run_op(OP_READ, 8'h00, 0, 4'd10); if (op_done) rd_buf[3] <= rdata; end
                        4'd10: begin run_op(OP_READ, 8'h00, 0, 4'd11); if (op_done) rd_buf[4] <= rdata; end
                        4'd11: begin run_op(OP_READ, 8'h00, 1, 4'd12); if (op_done) rd_buf[5] <= rdata; end
                        4'd12: begin
                            op_read <= 0;
                            accel_x <= {rd_buf[0], rd_buf[1]};
                            accel_y <= {rd_buf[2], rd_buf[3]};
                            accel_z <= {rd_buf[4], rd_buf[5]};
                            step <= 4'd13;
                        end
                        4'd13: run_op(OP_STOP, 8'h00, 0, 4'd14);
                        4'd14: begin
                            op_stop <= 0;
                            data_valid <= 1'b1;
                            busy <= 1'b0;
                            step <= 4'd0;
                            state <= ST_IDLE;
                        end
                    endcase
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
