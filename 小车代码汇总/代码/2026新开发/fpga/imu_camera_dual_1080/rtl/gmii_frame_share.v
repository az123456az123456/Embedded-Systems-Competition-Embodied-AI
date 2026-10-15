// Sources must honor allow ONLY at frame start and include IFG in busy.
// Camera busy includes its configured frame guard; IMU busy includes 12-cycle IFG.
// Pending IMU wins the next frame boundary. An in-flight camera frame finishes.
module gmii_frame_share (
    input camera_busy,camera_valid,input [7:0] camera_data,
    input imu_pending,imu_busy,imu_valid,input [7:0] imu_data,
    output camera_allow,imu_allow,output [7:0] gmii_data,output gmii_valid
);
    assign camera_allow=!imu_pending && !imu_busy && !imu_valid;
    assign imu_allow=!camera_busy && !camera_valid;
    assign gmii_valid=camera_valid || imu_valid;
    assign gmii_data=imu_valid ? imu_data : camera_data;
endmodule
