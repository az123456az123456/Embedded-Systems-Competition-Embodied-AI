`timescale 1ns/1ps
// MPU9250 (WHO_AM_I=71) + AK8963 (WIA=48), I2C bypass mode.
// MPU data: +/-2g, +/-250dps. AK8963: 16-bit, continuous 100Hz.
// Reference: TDK RM-MPU-9250A-00 rev1.6; product spec rev1.1.
module mpu9250_reader #(
    parameter integer CLK_HZ=50000000, I2C_HZ=100000,
    parameter integer STARTUP_MS=100, RESET_MS=100, RETRY_MS=1000
)(
    input wire clk, rst_n, ms_tick,
    inout wire scl, sda,
    output reg [6:0] mpu_address,
    output reg [7:0] mpu_id, mag_id,
    output reg mpu_initialized, mag_initialized, fault, mag_overflow,
    output reg ag_valid, mag_valid,
    output reg [15:0] ax, ay, az, temperature, gx, gy, gz, mx, my, mz,
    output reg [23:0] asa,
    output reg [31:0] error_count
);
    reg transaction_start, transaction_read;
    reg [6:0] transaction_addr;
    reg [7:0] transaction_reg, transaction_write;
    reg [4:0] transaction_len;
    wire transaction_busy, transaction_done, transaction_error;
    wire [127:0] data;
    i2c_reg_master #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ)) bus (
        .clk(clk),.rst_n(rst_n),.start(transaction_start),.read(transaction_read),
        .address(transaction_addr),.register_address(transaction_reg),
        .write_data(transaction_write),.read_length(transaction_len),
        .busy(transaction_busy),.done(transaction_done),.error(transaction_error),
        .read_data(data),.scl(scl),.sda(sda)
    );
    reg [4:0] operation;
    reg issued;
    reg [31:0] wait_ms;
    reg [15:0] mag_retry_ms;
    reg [3:0] poll_ms;
    // Poll on a 10ms cadence. If delayed, take one sample rather than replaying
    // missed reads. Reading INT_STATUS prevents unchanged data being called new.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            operation<=0; issued<=0; wait_ms<=STARTUP_MS; poll_ms<=0; mag_retry_ms<=0;
            transaction_start<=0; transaction_read<=0; transaction_addr<=0;
            transaction_reg<=0; transaction_write<=0; transaction_len<=1;
            mpu_address<=7'h68; mpu_id<=0; mag_id<=0;
            mpu_initialized<=0; mag_initialized<=0; fault<=0; mag_overflow<=0;
            ag_valid<=0; mag_valid<=0; error_count<=0; asa<=24'h808080;
            ax<=0; ay<=0; az<=0; temperature<=0; gx<=0; gy<=0; gz<=0;
            mx<=0; my<=0; mz<=0;
        end else begin
            transaction_start<=0; ag_valid<=0; mag_valid<=0;
            if (ms_tick) begin
                if (wait_ms!=0) wait_ms<=wait_ms-1'b1;
                if (mag_retry_ms!=0) mag_retry_ms<=mag_retry_ms-1'b1;
                if (poll_ms!=0) poll_ms<=poll_ms-1'b1;
            end
            if (!issued && wait_ms==0 && !transaction_busy
                && (operation!=18 || poll_ms==0)) begin
                transaction_start<=1; issued<=1;
                transaction_addr<=(operation>=12 && operation<=17) || operation==19
                                  ? 7'h0c : mpu_address;
                transaction_read<=0; transaction_write<=0; transaction_len<=1;
                case (operation)
                    0: begin transaction_read<=1; transaction_reg<=8'h75; end
                    1: begin transaction_reg<=8'h6b; transaction_write<=8'h80; end
                    2: begin transaction_reg<=8'h6b; transaction_write<=8'h01; end
                    3: transaction_reg<=8'h6c;
                    4: begin transaction_reg<=8'h1a; transaction_write<=8'h03; end
                    5: transaction_reg<=8'h1b;
                    6: transaction_reg<=8'h1c;
                    7: begin transaction_reg<=8'h1d; transaction_write<=8'h03; end
                    8: begin transaction_reg<=8'h19; transaction_write<=8'h09; end
                    9: transaction_reg<=8'h6a;
                    10: begin transaction_reg<=8'h37; transaction_write<=8'h02; end
                    11: begin transaction_reg<=8'h38; transaction_write<=8'h01; end
                    12: begin transaction_read<=1; transaction_reg<=8'h00; end
                    13,16: transaction_reg<=8'h0a;
                    14: begin transaction_reg<=8'h0a; transaction_write<=8'h0f; end
                    15: begin transaction_read<=1; transaction_reg<=8'h10; transaction_len<=3; end
                    17: begin transaction_reg<=8'h0a; transaction_write<=8'h16; end
                    18: begin
                        transaction_read<=1; transaction_reg<=8'h3a; transaction_len<=15;
                        poll_ms<=10;
                    end
                    19: begin transaction_read<=1; transaction_reg<=8'h02; transaction_len<=8; end
                    default: transaction_reg<=8'h75;
                endcase
            end else if (issued && transaction_done) begin
                issued<=0;
                if (operation==0) mpu_id<=data[7:0];
                if (operation==12) mag_id<=data[7:0];
                if (transaction_error || (operation==0 && data[7:0]!=8'h71)
                    || (operation==12 && data[7:0]!=8'h48)) begin
                    error_count<=error_count+1'b1; fault<=1;
                    if ((operation>=12 && operation<=17) || operation==19) begin
                        mag_initialized<=0; mag_retry_ms<=RETRY_MS;
                        operation<=18; // Keep reporting the working six-axis sensor.
                    end else begin
                        mpu_initialized<=0; mag_initialized<=0;
                        if (operation==0) mpu_address<=mpu_address ^ 7'h01;
                        operation<=0; wait_ms<=RETRY_MS;
                    end
                end else begin
                    operation<=operation+1'b1;
                    case (operation)
                        1: wait_ms<=RESET_MS;
                        2: wait_ms<=RESET_MS; // Gyro settling after wakeup.
                        11: mpu_initialized<=1;
                        13,14,16: wait_ms<=1;
                        15: asa<=data[23:0];
                        17: begin mag_initialized<=1; mag_overflow<=0; wait_ms<=10; fault<=0; end
                        18: begin
                            if (data[112]) begin
                                ax<=data[111:96]; ay<=data[95:80]; az<=data[79:64];
                                temperature<=data[63:48]; gx<=data[47:32];
                                gy<=data[31:16]; gz<=data[15:0]; ag_valid<=1;
                            end
                            if (!mag_initialized) begin
                                if (mag_retry_ms==0) operation<=12;
                                else operation<=18;
                            end
                        end
                        19: begin
                            operation<=18;
                            if (data[56]) begin // ST1.DRDY
                                mag_overflow<=data[3]; // ST2.HOFL; always read ST2.
                                if (!data[3]) begin
                                    mx<={data[47:40],data[55:48]};
                                    my<={data[31:24],data[39:32]};
                                    mz<={data[15:8],data[23:16]};
                                    mag_valid<=1; fault<=0;
                                end
                            end
                        end
                    endcase
                end
            end
        end
    end
endmodule
