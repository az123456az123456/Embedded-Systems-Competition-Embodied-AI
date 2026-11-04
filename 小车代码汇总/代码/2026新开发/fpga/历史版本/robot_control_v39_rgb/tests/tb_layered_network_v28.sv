`timescale 1ns/1ps
module tb_layered_network_v28;
    reg clk=0,rst_n=0,valid=0,error=0;reg [7:0] data=0;
    wire command_valid;wire [127:0] command_data;wire [31:0] accepted,rejected;
    motor_packet_rx dut(.clk(clk),.rst_n(rst_n),.data(data),.valid(valid),.error(error),
        .command_valid(command_valid),.command_data(command_data),
        .accepted_frames(accepted),.rejected_frames(rejected));
    reg clock_run=1;
    always #4 if(clock_run)clk=~clk;
    reg dst_clk=0;always #5 dst_clk=~dst_clk;
    wire dst_valid;wire [127:0] dst_data;wire [3:0] pwm,in1,in2,flags;wire [31:0] lastseq;wire [127:0] ack;
    motor_command_cdc cdc(.src_clk(clk),.src_rst_n(rst_n),.src_valid(command_valid),.src_data(command_data),.src_ready(),
      .dst_clk(dst_clk),.dst_rst_n(rst_n),.dst_valid(dst_valid),.dst_data(dst_data));
    motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),.WATCHDOG_MS(20),.RAMP_STEP_MS(1)) motor(
      .clk(dst_clk),.rst_n(rst_n),.command_valid(dst_valid),.command_data(dst_data),
      .encoder_valid(1'b0),.encoder_deltas(128'd0),.encoder_illegal(4'd0),
      .motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.status_flags(flags),.status_valid(),.status_data(ack),.last_sequence(lastseq),.applied_left(),.applied_right(),
      .cpu_pwm({4{16'd200}}),.cpu_ready(1'b1),.cpu_trap(1'b0),.raw_sonar_mm({3{16'd1000}}),.raw_sonar_online(3'b111),
      .control_flags(),.control_epoch(),.control_sequence(),.control_motion(),.control_masks());
    reg [7:0] frame[0:63];reg [127:0] payload;
    reg [15:0] c16;reg [31:0] c32;integer i,k,commands=0,magic_version,wire_version,before_commands;
    function [15:0] crc16;
        input [15:0] c;input [7:0] b;reg [15:0] x;integer j;
        begin x=c^{b,8'b0};for(j=0;j<8;j=j+1)x=x[15]?(x<<1)^16'h1021:x<<1;crc16=x;end
    endfunction
    function [31:0] crc32;
        input [31:0] c;input [7:0] b;reg [31:0] x;integer j;
        begin x=c^b;for(j=0;j<8;j=j+1)x=x[0]?(x>>1)^32'hedb88320:x>>1;crc32=x;end
    endfunction
    task prepare;
        begin
            for(i=0;i<64;i=i+1)frame[i]=0;
            frame[0]=2;frame[1]=8'hac;frame[2]=8'h13;frame[3]=8'h80;frame[4]=0;frame[5]=1;
            frame[6]=2;frame[7]=3;frame[8]=4;frame[9]=5;frame[10]=6;frame[11]=7;
            frame[12]=8'h88;frame[13]=8'hb6;
            payload={32'h4d4f5433,8'd3,8'd1,32'h12345678,16'hfed4,16'd200,8'd10,8'd15};
            c16=16'hffff;
            for(i=0;i<16;i=i+1)begin frame[14+i]=payload[127-i*8-:8];c16=crc16(c16,frame[14+i]);end
            frame[30]=c16[15:8];frame[31]=c16[7:0];
        end
    endtask
    task payload_crc;
        begin
            c16=16'hffff;
            for(i=0;i<16;i=i+1)c16=crc16(c16,frame[14+i]);
            frame[30]=c16[15:8];frame[31]=c16[7:0];
        end
    endtask
    task fcs;
        begin

            c32=32'hffffffff;
            for(i=0;i<60;i=i+1)c32=crc32(c32,frame[i]);
            c32=~c32;
            for(i=0;i<4;i=i+1)frame[60+i]=c32[i*8+:8];
        end
    endtask
    task send;
        input integer bytes;input integer inject_error;
        begin
            for(k=0;k<8;k=k+1)begin @(negedge clk);valid=1;data=k==7?8'hd5:8'h55;end
            for(k=0;k<bytes;k=k+1)begin @(negedge clk);data=frame[k];error=(inject_error && k==25);end
            @(negedge clk);valid=0;error=0;data=0;repeat(5)@(negedge clk);
        end
    endtask
    always @(posedge clk)if(command_valid)begin commands=commands+1;if(command_data!==payload)$fatal(1,"payload mismatch");end
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        prepare;fcs;send(64,0);if(commands!=1)$fatal(1,"valid frame rejected");
        prepare;fcs;frame[63]=frame[63]^1;send(64,0);
        prepare;frame[30]=frame[30]^1;fcs;send(64,0);
        prepare;frame[5]=2;fcs;send(64,0);
        prepare;frame[40]=1;fcs;send(64,0);
        prepare;fcs;send(64,1);
        prepare;fcs;send(62,0);
        if(commands!=1 || rejected!=6)$fatal(1,"corrupt frame accepted or counters wrong");
        prepare;frame[18]=1;payload_crc;fcs;send(64,0); // MOT3 magic with MOT1 version
        prepare;frame[19]=3;payload_crc;fcs;send(64,0); // reserved flag
        prepare;frame[28]=0;payload_crc;fcs;send(64,0); // empty left mask
        prepare;frame[29]=8'hff;payload_crc;fcs;send(64,0); // reserved inversion bits
        if(commands!=1 || rejected!=10)$fatal(1,"format violations accepted");
        prepare;fcs;send(64,0);
        if(commands!=2 || accepted!=2)$fatal(1,"receiver did not resynchronize");
        // Exact magic/version pairing, never accidentally permit a new version
        // just because another supported magic is present.
        for(magic_version=1;magic_version<=4;magic_version=magic_version+1)begin
          for(wire_version=1;wire_version<=4;wire_version=wire_version+1)begin
            prepare;frame[17]=8'h30+magic_version;frame[18]=wire_version;
            for(i=0;i<16;i=i+1)payload[127-i*8-:8]=frame[14+i];
            payload_crc;fcs;before_commands=commands;send(64,0);
            if(commands-before_commands!==((magic_version==wire_version && magic_version<=4)?1:0))
              $fatal(1,"magic/version mismatch magic=%0d version=%0d",magic_version,wire_version);
          end
        end
        // Actual valid Ethernet + FCS + asynchronous mailbox + physical actuator.
        prepare;frame[17]=8'h34;frame[18]=4;frame[19]=0;
        frame[20]=0;frame[21]=0;frame[22]=0;frame[23]=1;
        frame[24]=0;frame[25]=0;frame[26]=0;frame[27]=0;
        for(i=0;i<16;i=i+1)payload[127-i*8-:8]=frame[14+i];
        payload_crc;fcs;send(64,0);repeat(20)@(negedge dst_clk);
        if(lastseq!=1 || ack[95:88]!=4 || flags[0])$fatal(1,"MOT4 configure didnotcrossCDC");
        frame[19]=1;frame[23]=2;frame[24]=1;frame[25]=44;
        for(i=0;i<16;i=i+1)payload[127-i*8-:8]=frame[14+i];payload_crc;fcs;send(64,0);repeat(200)@(negedge dst_clk);
        if(lastseq!=2 || in2!=15 || in1!=0 || !flags[0])$fatal(1,"MOT4 physical output lost throughCRC/CDC");
        // A corrupt fresh stop is rejected before CDC/sequence/watchdog.
        frame[19]=0;frame[23]=3;frame[24]=0;frame[25]=0;
        for(i=0;i<16;i=i+1)payload[127-i*8-:8]=frame[14+i];payload_crc;fcs;frame[63]^=1;send(64,0);repeat(20)@(negedge dst_clk);
        if(lastseq!=2 || !flags[0])$fatal(1,"badFCSstop changed actuator");
        clock_run=0;repeat(2100)@(negedge dst_clk);
        if(flags[0] || !flags[1] || pwm!=0 || in1!=0 || in2!=0)$fatal(1,"lostRXclock bypassed localwatchdog");
        $display("PASS v28 realEthernetCRC16/FCS/exactMOT1..4version/RXerror/padding -> asynchronousCDC -> ACK4/physicalPWM; corruptstop rejected and RXclockloss watchdog");$finish;
    end
    initial begin #200000;$fatal(1,"timeout");end
endmodule
