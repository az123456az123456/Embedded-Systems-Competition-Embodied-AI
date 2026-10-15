`timescale 1ns/1ps
module tb_sensor;
    reg clk=0, rst_n=0;
    always #500 clk=~clk; // 1MHz, simulation only
    tri1 scl,sda;
    reg hold_scl=0;
    assign scl=hold_scl ? 1'b0 : 1'bz;
    reg slave_low=0;
    assign sda=slave_low ? 1'b0 : 1'bz;
    integer ticks=0;
    reg ms_tick=0;
    always @(posedge clk) begin
        ms_tick<=0;
        if (ticks==999) begin ticks<=0; ms_tick<=1; end else ticks<=ticks+1;
    end
    wire ag_valid,mag_valid,mpu_init,mag_init,fault,overflow;
    wire [6:0] address;
    wire [7:0] mpu_id,mag_id;
    wire [15:0] ax,ay,az,temp,gx,gy,gz,mx,my,mz;
    wire [23:0] asa;
    wire [31:0] errors;
    mpu9250_reader #(.CLK_HZ(1000000),.I2C_HZ(50000),
        .STARTUP_MS(1),.RESET_MS(1),.RETRY_MS(10)) dut (
        .clk(clk),.rst_n(rst_n),.ms_tick(ms_tick),.scl(scl),.sda(sda),
        .mpu_address(address),.mpu_id(mpu_id),.mag_id(mag_id),
        .mpu_initialized(mpu_init),.mag_initialized(mag_init),.fault(fault),
        .mag_overflow(overflow),.ag_valid(ag_valid),.mag_valid(mag_valid),
        .ax(ax),.ay(ay),.az(az),.temperature(temp),.gx(gx),.gy(gy),.gz(gz),
        .mx(mx),.my(my),.mz(mz),.asa(asa),.error_count(errors)
    );
    // Behavioral two-device I2C slave. Drives only on falling SCL, samples on
    // rising SCL; verifies repeated START, register writes and final NACK.
    reg [7:0] mpu[0:255],mag[0:255];
    reg mpu_present=1,mag_present=1;
    reg [6:0] device_address=7'h68;
    integer mode=8, bitno=0, ptr=0, bank=0;
    integer reads=0, final_nacks=0, st2_reads=0, ag_samples=0,mag_samples=0;
    reg active=0, addr_phase=1, need_register=0, accepted=0, transmit=0;
    reg [7:0] shift=0, byte_out=0;
    function [7:0] value;
        input integer b,p;
        begin value=b ? mag[p&255] : mpu[p&255]; end
    endfunction
    always @(negedge sda) if (scl===1'b1) begin
        active=1; addr_phase=1; mode=0; bitno=7; shift=0; slave_low=0;
    end
    always @(posedge sda) if (scl===1'b1) begin active=0; mode=8; slave_low=0; end
    always @(posedge scl) if (active) begin
        case (mode)
            0: begin
                shift={shift[6:0],sda};
                if (bitno==0) begin
                    transmit=0;
                    if (addr_phase) begin
                        accepted=(shift[7:1]==device_address && mpu_present) ||
                            (shift[7:1]==7'h0c && mag_present && mpu[8'h37][1]);
                        bank=(shift[7:1]==7'h0c);
                        transmit=shift[0]; need_register=!shift[0]; addr_phase=0;
                    end else if (accepted) begin
                        if (need_register) begin ptr=shift; need_register=0; end
                        else begin
                            if (bank) mag[ptr]=shift; else mpu[ptr]=shift;
                            ptr=(ptr+1)&255;
                        end
                    end
                    mode=1;
                end else bitno=bitno-1;
            end
            2: mode=3;
            4: if (bitno==0) begin
                    mode=5; reads=reads+1;
                    if (bank && ptr==9) st2_reads=st2_reads+1;
                end else bitno=bitno-1;
            6: begin
                if (sda) begin final_nacks=final_nacks+1; mode=8; end
                else begin ptr=(ptr+1)&255; byte_out=value(bank,ptr); bitno=7; mode=7; end
            end
        endcase
    end
    always @(negedge scl) if (active) begin
        case (mode)
            1: begin slave_low=accepted; mode=2; end
            3: begin
                slave_low=0; bitno=7;
                if (!accepted) mode=8;
                else if (transmit) begin byte_out=value(bank,ptr); slave_low=!byte_out[7]; mode=4; end
                else begin mode=0; shift=0; end
            end
            4: slave_low=!byte_out[bitno];
            5: begin slave_low=0; mode=6; end
            7: begin slave_low=!byte_out[7]; mode=4; end
        endcase
    end
    always @(posedge clk) begin
        if (ag_valid) ag_samples=ag_samples+1;
        if (mag_valid) mag_samples=mag_samples+1;
    end
    integer i,previous_errors,previous_samples;
    initial begin
        for(i=0;i<256;i=i+1) begin mpu[i]=0; mag[i]=0; end
        mpu['h75]='h71; mpu['h3a]=1;
        {mpu['h3b],mpu['h3c],mpu['h3d],mpu['h3e],mpu['h3f],mpu['h40],
         mpu['h41],mpu['h42],mpu['h43],mpu['h44],mpu['h45],mpu['h46],mpu['h47],mpu['h48]}=
         112'h4000c0007fff01000200fe008000;
        mag[0]='h48; mag[2]=1;
        {mag[3],mag[4],mag[5],mag[6],mag[7],mag[8],mag[9]}=56'h3412cced008000;
        mag['h10]=128; mag['h11]=160; mag['h12]=96;
        #5000; rst_n=1;
        @(posedge mag_valid); #2000;
        if ({ax,ay,az,temp,gx,gy,gz}!==112'h4000c0007fff01000200fe008000) $fatal(1,"MPU burst decode");
        if ({mx,my,mz}!==48'h1234edcc8000 || asa!==24'h80a060) $fatal(1,"AK8963 endian / ASA");
        if (!mpu_init || !mag_init || fault || mpu_id!='h71 || mag_id!='h48) $fatal(1,"Initialization");
        if (mpu['h6b]!=1 || mpu['h6c]!=0 || mpu['h1a]!=3 || mpu['h1b]!=0 ||
            mpu['h1c]!=0 || mpu['h1d]!=3 || mpu['h19]!=9 || mpu['h6a]!=0 ||
            mpu['h37]!=2 || mpu['h38]!=1 || mag['h0a]!='h16) $fatal(1,"Register setup");
        mag[9]=8; previous_samples=mag_samples;
        wait(overflow); #10000;
        if (mag_samples!=previous_samples || mx!='h1234) $fatal(1,"Overflow accepted");
        mag[9]=0; @(posedge mag_valid); #2000;
        if (overflow) $fatal(1,"Overflow did not clear");
        // Absent AK8963 must leave the six-axis stream running.
        mag_present=0; previous_samples=ag_samples;
        wait(!mag_init); #25000000;
        if (!fault || !mpu_init || ag_samples<=previous_samples) $fatal(1,"Mag NACK degraded mode");
        mag_present=1; @(posedge mag_valid);
        // No DRDY: old register contents must not emit a new sample.
        mpu['h3a]=0; #15000000; previous_samples=ag_samples; #25000000;
        if (ag_samples!=previous_samples) $fatal(1,"Stale sample accepted");
        mpu['h3a]=1;
        // SCL stuck low must time out and recover when released.
        previous_errors=errors; hold_scl=1; #25000000;
        if (errors<=previous_errors) $fatal(1,"Missing bus timeout");
        hold_scl=0; @(posedge mag_valid);
        // Reconnect at AD0=1 with invalid identity first.
        mpu_present=0; wait(!mpu_init); device_address=7'h69; mpu['h75]='h70; mpu_present=1;
        #40000000;
        if (mpu_init) $fatal(1,"Wrong WHO_AM_I accepted");
        mpu['h75]='h71; @(posedge mag_valid); #2000;
        if (address!=7'h69 || !mpu_init || fault) $fatal(1,"Address fallback / reconnect");
        if (final_nacks<10 || st2_reads<3) $fatal(1,"Burst termination / ST2");
        $display("PASS sensor: nine axes, init, final NACK, overflow, absent mag, stale, stuck SCL, WHO_AM_I, AD0 fallback");
        $finish;
    end
    initial begin #1000000000; $fatal(1,"Sensor test timed out op=%d mode=%d ptr=%h errors=%d",dut.operation,mode,ptr,errors); end
endmodule
