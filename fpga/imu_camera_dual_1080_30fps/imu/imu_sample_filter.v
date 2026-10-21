`timescale 1ns/1ps
// Unity-DC-gain IIR. Values and output units stay signed 16-bit MPU9250 codes.
// Q16.8 internal state avoids a whole-code dead zone for small constant rates.
// At nominal 100Hz: accel alpha=1/4, gyro alpha=1/2 (30ms/10ms DC delay).
// First sample seeds the filter; no startup zero ramp, bias learning or clamping.
module imu_sample_filter #(
    parameter integer ACCEL_SHIFT=2,
    parameter integer GYRO_SHIFT=1
)(
    input wire clk, rst_n, clear, in_valid,
    input wire [15:0] ax_in,ay_in,az_in,temp_in,gx_in,gy_in,gz_in,
    output reg out_valid,
    output reg [15:0] ax_out,ay_out,az_out,temp_out,gx_out,gy_out,gz_out
);
    reg seeded;
    reg signed [23:0] state [0:5];
    wire signed [15:0] sample [0:5];
    assign sample[0]=ax_in; assign sample[1]=ay_in; assign sample[2]=az_in;
    assign sample[3]=gx_in; assign sample[4]=gy_in; assign sample[5]=gz_in;
    wire signed [24:0] target [0:5],difference [0:5],next_state [0:5];
    genvar k;
    generate for(k=0;k<6;k=k+1) begin: axis_filter
        assign target[k]={sample[k][15],sample[k],8'b0};
        assign difference[k]=target[k]-$signed({state[k][23],state[k]});
        assign next_state[k]=!seeded ? target[k] :
            $signed({state[k][23],state[k]})+(difference[k] >>> (k<3?ACCEL_SHIFT:GYRO_SHIFT));
    end endgenerate
    function [15:0] rounded;
        input signed [24:0] value;
        reg signed [24:0] magnitude;
        begin
            // Symmetric nearest rounding, including -32768 (needs 25 bits).
            if(value<0) begin magnitude=-value; rounded=-((magnitude+25'sd128) >>> 8); end
            else rounded=(value+25'sd128) >>> 8;
        end
    endfunction
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            seeded<=0;out_valid<=0;
            ax_out<=0;ay_out<=0;az_out<=0;temp_out<=0;gx_out<=0;gy_out<=0;gz_out<=0;
            for(i=0;i<6;i=i+1) state[i]<=0;
        end else if(clear) begin
            seeded<=0;out_valid<=0;
            ax_out<=0;ay_out<=0;az_out<=0;temp_out<=0;gx_out<=0;gy_out<=0;gz_out<=0;
            for(i=0;i<6;i=i+1) state[i]<=0;
        end else begin
            out_valid<=in_valid;
            if(in_valid) begin
                seeded<=1;
                for(i=0;i<6;i=i+1) state[i]<=next_state[i][23:0];
                ax_out<=rounded(next_state[0]);ay_out<=rounded(next_state[1]);az_out<=rounded(next_state[2]);
                gx_out<=rounded(next_state[3]);gy_out<=rounded(next_state[4]);gz_out<=rounded(next_state[5]);
                temp_out<=temp_in;
            end
        end
    end
endmodule
