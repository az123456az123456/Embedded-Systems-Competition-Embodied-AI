`timescale 1ns/1ps
// Functional ODDR model only; physical RGMII timing still needs Gowin STA and
// board testing. D0/D1 are both sampled on the rising clock edge.
module ODDR(input D0,D1,TX,CLK,output reg Q0,output wire Q1);
    reg falling=0;
    assign Q1=TX;
    always @(posedge CLK) begin Q0<=D0; falling<=D1; end
    always @(negedge CLK) Q0<=falling;
endmodule
module tb_top;
    reg clk=0,net_clk=0;
    always #500 clk=~clk;
    always #4000 net_clk=~net_clk; // Slow simulation clock; frame logic unchanged.
    tri1 scl,sda;
    wire [3:0] led,txd;
    wire ctl,txclk;
    imu_rgmii_top #(.CLK_HZ(1000000),.I2C_HZ(50000)) dut (
        .clk_50m(clk),.imu_scl(scl),.imu_sda(sda),.led(led),
        .rgmii_rx_clk(net_clk),.rgmii_txd(txd),.rgmii_tx_ctl(ctl),.rgmii_tx_clk(txclk));
    reg ag=0,mag=0;
    // Sensor bus behavior is exercised in tb_sensor. Here inject samples to
    // independently verify the top-level snapshot, LEDs and DDR nibble order.
    integer f,frames=0;
    reg [3:0] low_nibble;
    reg low_ctl,was_valid=0;
    always @(posedge net_clk) begin
        #1; low_nibble=txd; low_ctl=ctl;
        if (txclk!==1'b1) $fatal(1,"TX clock rising");
    end
    always @(negedge net_clk) begin
        #1;
        if(txclk!==1'b0) $fatal(1,"TX clock falling");
        if (low_ctl===1'b1) begin
            if (ctl!==1'b1) $fatal(1,"TX control DDR mismatch");
            $fwrite(f,"%02x\n",{txd,low_nibble}); was_valid=1;
        end else if (was_valid) begin
            $fwrite(f,"END\n"); frames=frames+1; was_valid=0;
        end
    end
    initial begin
        f=$fopen("build/top_frames.hex","w");
        force dut.ag_valid=ag; force dut.mag_valid=mag;
        force dut.mpu_init=1; force dut.mag_init=1;
        force dut.fault=0; force dut.overflow=0;
        force dut.mpu_id=8'h71; force dut.mag_id=8'h48; force dut.address=7'h69;
        force dut.ax=16'h4000; force dut.ay=16'hc000; force dut.az=16'd0;
        force dut.temp=16'd0; force dut.gx=16'h0083; force dut.gy=16'hff7d; force dut.gz=16'd0;
        force dut.mx=16'd100; force dut.my=16'hff9c; force dut.mz=16'd200;
        force dut.asa=24'h80a060; force dut.errors=32'd0;
        wait(dut.rst_n && dut.net_rst_n); #1000;
        if (led[2:1]!=0) $fatal(1,"LED claims data before any sample");
        @(negedge clk); ag=1; @(negedge clk); ag=0;
        #3000; if(!led[1] || led[2]) $fatal(1,"AG LED");
        wait(frames==1);
        @(negedge clk); mag=1; @(negedge clk); mag=0;
        wait(frames==2);
        if(led[2:1]!=3 || led[3]) $fatal(1,"Nine-axis LED");
        #510000000;
        if(led[2:1]!=0 || !led[3]) $fatal(1,"Stale LEDs");
        wait(frames==3); // 1 second heartbeat reports stale state.
        #100; $fclose(f);
        $display("PASS top: sample snapshot, sequence, freshness LEDs, heartbeat and DDR mapping"); $finish;
    end
    initial begin #1200000000; $fatal(1,"Top timeout"); end
endmodule
