`timescale 1ns/1ps
module tb_gmii_share;
    reg cb=0,cv=0,ip=0,ib=0,iv=0,mp=0,mb=0,mv=0;
    wire ca,ia,ma,valid;wire [7:0] data;
    gmii_three_source_share dut(.camera_busy(cb),.camera_valid(cv),.camera_data(8'h11),
        .imu_pending(ip),.imu_busy(ib),.imu_valid(iv),.imu_data(8'h22),
        .motor_pending(mp),.motor_busy(mb),.motor_valid(mv),.motor_data(8'h33),
        .camera_allow(ca),.imu_allow(ia),.motor_allow(ma),.gmii_data(data),.gmii_valid(valid));
    initial begin
        #1;if(!ca || !ia || !ma || valid)$fatal(1,"idle arbitration wrong");
        cb=1;cv=1;ip=1;mp=1;
        #1;if(ia || ma || data!=8'h11 || !valid)$fatal(1,"pending preempted camera frame");
        cv=0;#1;if(ia || ma)$fatal(1,"camera IFG not protected");
        cb=0;#1;if(ca || ia || !ma)$fatal(1,"motor command ACK priority wrong");
        mb=1;mv=1;#1;if(ca || ia || data!=8'h33)$fatal(1,"ACK frame not exclusive");
        mv=0;#1;if(ca || ia)$fatal(1,"ACK IFG not protected");
        mb=0;mp=0;#1;if(ca || !ia)$fatal(1,"IMU next-frame priority wrong");
        ib=1;iv=1;#1;if(ma || ca || data!=8'h22)$fatal(1,"IMU frame not exclusive");
        iv=0;#1;if(ma || ca)$fatal(1,"IMU IFG not protected");
        ib=0;ip=0;#1;if(!ca)$fatal(1,"camera starved without metadata pending");
        $display("PASS camera/IMU/ACK frame-boundary priority and protected IFG");$finish;
    end
endmodule
