`timescale 1ns/1ps
// Register-ROM and initialization handshake regression. I2C/AF are boundary
// stubs: this verifies the complete 259-write sequence, not electrical SCCB.
module tb_camera_40fps;
    reg clk=0,rst_n=0;
    always #10 clk=~clk;
    wire init_done;
    wire [15:0] af_status;
    wire camera_rst_n,camera_pwdn,i2c_sclk;
    tri i2c_sdat;
    camera_init #(.IMAGE_WIDTH(960),.IMAGE_HEIGHT(1080)) dut(
        .Clk(clk),.Rst_n(rst_n),.Init_Done(init_done),.af_status(af_status),
        .camera_rst_n(camera_rst_n),.camera_pwdn(camera_pwdn),
        .i2c_sclk(i2c_sclk),.i2c_sdat(i2c_sdat));
    reg [8:0] rom_addr=0;
    wire [23:0] rom_q;
    ov5640_init_table_rgb rom(.clk(clk),.addr(rom_addr),.q(rom_q));
    reg [23:0] expected[0:258];
    reg [7:0] sensor_regs[0:65535];
    integer i,hts,vts,band50,band60,max_exposure;
    real xclk,vco,sclk,pclk,fps,bit_div,scale_div;
    initial begin
        for(i=0;i<259;i=i+1) begin
            @(negedge clk); rom_addr=i;
            @(posedge clk); #1;
            if ((^rom_q)===1'bx) $fatal(1,"uninitialized ROM address %0d",i);
            expected[i]=rom_q;
            sensor_regs[rom_q[23:8]]=rom_q[7:0];
        end
        // Reconstruct clock tree from the last write to each register.
        // Board PLL produces XVCLK=1250 MHz/52; OV5640 diagram 2-3.
        xclk=1250000000.0/52.0;
        vco=xclk/(sensor_regs['h3037]&15)*sensor_regs['h3036];
        bit_div=(sensor_regs['h3034]&15)/4.0;
        sclk=vco/(sensor_regs['h3035]>>4)/
            (((sensor_regs['h3037]>>4)&1)+1)/bit_div/
            (1<<(sensor_regs['h3108]&3));
        scale_div=(sensor_regs['h3824]&31)/2;
        // MIPI is powered down; DVP P_DIV is twice 3035[3:0].
        pclk=vco/(sensor_regs['h3035]>>4)/
            (((sensor_regs['h3037]>>4)&1)+1)/bit_div/
            (1<<((sensor_regs['h3108]>>4)&3))/
            (2*(sensor_regs['h3035]&15))/scale_div;
        hts={sensor_regs['h380c],sensor_regs['h380d]};
        vts={sensor_regs['h380e],sensor_regs['h380f]};
        fps=sclk/hts/vts;
        if(vco<500000000.0 || vco>1000000000.0) $fatal(1,"PLL VCO outside rating");
        if(fps<39.9 || fps>40.1) $fatal(1,"wrong FPS %f",fps);
        if(pclk<sclk || pclk>85000000.0) $fatal(1,"RAW DVP clock invalid %f",pclk);
        if(hts!=1880 || vts!=1120) $fatal(1,"frame timing changed");
        if({sensor_regs['h3808],sensor_regs['h3809]}!=960 ||
           {sensor_regs['h380a],sensor_regs['h380b]}!=1080) $fatal(1,"ROI changed");
        if(sensor_regs['h3036]!='h69 || sensor_regs['h3108]!='h01 ||
           sensor_regs['h460c]!='h22 || sensor_regs['h3824]!=2)
            $fatal(1,"clock routing no longer matches qualified RAW path");
        if(sensor_regs['h4300]!=3 || sensor_regs['h501f]!=3)
            $fatal(1,"RAW output format changed");
        if(sensor_regs['h3a00][2] || !sensor_regs['h3a00][5] || sensor_regs['h3503]!=0)
            $fatal(1,"night mode, banding or automatic exposure configured incorrectly");
        max_exposure={sensor_regs['h3a02],sensor_regs['h3a03]};
        if(max_exposure!=vts-4 || {sensor_regs['h3a14],sensor_regs['h3a15]}!=max_exposure)
            $fatal(1,"AEC ceiling does not fit fixed frame");
        band50=$rtoi(sclk/hts/100.0);
        band60=$rtoi(sclk/hts/120.0);
        if({sensor_regs['h3a08],sensor_regs['h3a09]}!=band50 ||
           {sensor_regs['h3a0a],sensor_regs['h3a0b]}!=band60)
            $fatal(1,"50/60 Hz banding steps do not follow new SCLK");
        if(sensor_regs['h3a0e]!=(vts-4)/band50 || sensor_regs['h3a0d]!=(vts-4)/band60)
            $fatal(1,"max band count exceeds exposure budget");
        if($bits(dut.cnt)!=9 || $bits(dut.lut_size)!=9 || dut.lut_size!=259)
            $fatal(1,"initialization counter/table length cannot reach all writes");
        // Trigger the normal state machine after its power-up delay.
        @(negedge clk); rst_n=1;
        force dut.delay_cnt=21'h1007ff;
        @(posedge clk); #1; force dut.delay_cnt=21'h100800;
        @(posedge clk); #1; release dut.delay_cnt;
        wait(init_done); #1;
        if(dut.i2c_control.writes!=259) $fatal(1,"wrong write count %0d",dut.i2c_control.writes);
        for(i=0;i<259;i=i+1)
            if(dut.i2c_control.written[i]!==expected[i])
                $fatal(1,"write %0d mismatch %h vs %h",i,dut.i2c_control.written[i],expected[i]);
        repeat(20)@(posedge clk);
        if(!init_done || dut.i2c_control.writes!=259) $fatal(1,"init repeated after completion");
        $display("PASS 259 SCCB writes including addresses 256..258; 960x1080 RAW, PLL VCO=%0.3f MHz SCLK/PCLK=%0.3f/%0.3f MHz FPS=%0.5f, AEC ceiling=%0d, 50/60Hz=%0d/%0d lines",vco/1e6,sclk/1e6,pclk/1e6,fps,max_exposure,band50,band60);
        $finish;
    end
    initial begin #1000000; $fatal(1,"initialization timed out (possible 8-bit counter wrap)"); end
endmodule

module ov5640_af_loader(input clk,rst_n,start,output wr_req,rd_req,
    output [15:0] addr,output [7:0] wrdata,input [7:0] rddata,
    input rw_done,nack,output [15:0] status);
    assign wr_req=0; assign rd_req=0; assign addr=0; assign wrdata=0; assign status=0;
endmodule

module i2c_control(input Clk,Rst_n,wrreg_req,rdreg_req,input [15:0]addr,
    input addr_mode,input [7:0]wrdata,output [7:0]rddata,
    input [7:0]device_id,output reg RW_Done=0,output ack,input [31:0]dly_cnt_max,
    output i2c_sclk,inout i2c_sdat);
    integer writes=0,pending=0;
    reg [23:0] written[0:258];
    assign ack=0; assign rddata=0; assign i2c_sclk=1;
    always@(posedge Clk) begin
        RW_Done<=0;
        if(!Rst_n) begin writes<=0; pending<=0; end
        else if(wrreg_req) begin
            if(writes>=259) $fatal(1,"extra initialization write");
            if(device_id!='h78 || !addr_mode) $fatal(1,"SCCB address format changed");
            written[writes]<={addr,wrdata}; writes<=writes+1; pending<=3;
        end else if(pending>0) begin
            pending<=pending-1;
            if(pending==1) RW_Done<=1;
        end
    end
endmodule
