// ============================================================
// tb_imu.v — MPU6050 封装测试（最终版）
// 用 Verilog 模拟 I2C 从机(MPU6050)，预置:
//   ACCEL_X=0x1234  Y=0x5678  Z=0x9ABC
// 从机按主控 ACK/NACK 判断读突发结束（真实从机行为）
// ============================================================
`timescale 1ns/1ps
module tb_imu;

    reg clk = 0;
    reg rst_n = 0;
    always #10 clk = ~clk;   // 50MHz

    wire scl;
    wire i2c_sda;
    pullup(i2c_sda);

    wire signed [15:0] accel_x, accel_y, accel_z;
    wire data_valid;

    imu_mpu6050 #(.I2C_ADDR(7'h68)) u_imu (
        .clk(clk), .rst_n(rst_n),
        .sda(i2c_sda), .scl(scl),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .data_valid(data_valid), .busy()
    );

    // ============ 模拟 MPU6050 从机 ============
    reg  s_drive = 0;
    reg  s_sda = 1;
    assign i2c_sda = s_drive ? s_sda : 1'bz;

    reg [7:0] s_mem [0:127];
    reg [3:0] s_edge = 0;       // 0..8 (每字节9个scl边沿)
    reg [7:0] s_rx = 0;
    reg [6:0] s_reg = 0;
    reg       s_addr_phase = 1;
    reg       s_rw = 0;
    reg       s_expect_reg = 0;
    reg       s_in_read = 0;
    reg       s_master_nack = 0;
    reg [7:0] s_tx = 0;

    initial begin
        s_mem[8'h3B] = 8'h12; s_mem[8'h3C] = 8'h34;
        s_mem[8'h3D] = 8'h56; s_mem[8'h3E] = 8'h78;
        s_mem[8'h3F] = 8'h9A; s_mem[8'h40] = 8'hBC;
    end

    // START / STOP 检测
    always @(negedge i2c_sda) begin
        if (scl == 1) begin
            s_addr_phase <= 1; s_edge <= 0; s_in_read <= 0; s_rw <= 0;
            s_expect_reg <= 0; s_drive <= 0; s_master_nack <= 0;
        end
    end
    always @(posedge i2c_sda) begin
        if (scl == 1) begin
            s_addr_phase <= 1; s_in_read <= 0; s_expect_reg <= 0;
            s_drive <= 0; s_master_nack <= 0;
        end
    end

    // scl 上升沿: 采样 / 第9边沿处理字节
    always @(posedge scl) begin
        if (s_edge == 4'd8) begin
            s_edge <= 0;
            if (s_addr_phase) begin
                if (s_rx[7:1] == 7'h68) begin
                    s_rw         <= s_rx[0];
                    s_addr_phase <= 0;
                    s_expect_reg <= ~s_rx[0];
                    s_in_read    <= s_rx[0];
                end
            end else if (s_rw == 1'b0) begin
                if (s_expect_reg) begin
                    s_reg <= s_rx;
                    s_expect_reg <= 0;
                end else begin
                    s_mem[s_reg] <= s_rx;
                    s_reg <= s_reg + 1;
                end
            end else begin
                // 读模式: 捕获主控 ACK/NACK (1=读突发结束)
                s_master_nack <= i2c_sda;
            end
        end else begin
            s_rx  <= {s_rx[6:0], i2c_sda};
            s_edge <= s_edge + 1;
        end
    end

    // scl 下降沿: 应答 / 读数据驱动
    always @(negedge scl) begin
        if (s_edge == 4'd8) begin
            // ACK 位: 地址/写数据 从机应答; 读数据 主控应答
            if (s_in_read && !s_addr_phase) s_drive <= 0;
            else begin s_drive <= 1; s_sda <= 0; end
        end else if (s_in_read && !s_addr_phase && !s_master_nack) begin
            if (s_edge == 4'd0) begin
                s_sda <= s_mem[s_reg][7];
                s_tx  <= {s_mem[s_reg][6:0], 1'b0};
                s_reg <= s_reg + 1;
            end else begin
                s_sda <= s_tx[7];
                s_tx  <= {s_tx[6:0], 1'b0};
            end
            s_drive <= 1;
        end else begin
            s_drive <= 0;
        end
    end

    // ============ 测试 ============
    integer err = 0;
    integer vcnt = 0;
    always @(posedge clk) if (data_valid) vcnt <= vcnt + 1;

    initial begin
        $dumpfile("imu_wave.vcd");
        $dumpvars(1, tb_imu);
        #1000 rst_n = 1;
        #1000;
        while (vcnt < 2) @(posedge clk);
        #100;
        $display("ACCEL_X=%0d (期望 4660=0x1234) %s", accel_x, (accel_x==16'sd4660)?"PASS":"FAIL");
        if (accel_x != 16'sd4660) err = err + 1;
        $display("ACCEL_Y=%0d (期望 22136=0x5678) %s", accel_y, (accel_y==16'sd22136)?"PASS":"FAIL");
        if (accel_y != 16'sd22136) err = err + 1;
        $display("ACCEL_Z=%0d (期望 -25924=0x9ABC) %s", accel_z, (accel_z==-16'sd25924)?"PASS":"FAIL");
        if (accel_z != -16'sd25924) err = err + 1;
        if (err == 0) $display("\n===== IMU 测试全部通过! =====");
        else          $display("\n===== 有 %0d 项失败 =====", err);
        $finish;
    end

    initial begin
        #80_000_000;
        $display("TIMEOUT"); $finish;
    end
endmodule
