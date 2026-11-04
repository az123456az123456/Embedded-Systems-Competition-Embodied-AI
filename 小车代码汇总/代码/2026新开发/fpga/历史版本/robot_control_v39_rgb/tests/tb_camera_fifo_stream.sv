`timescale 1ns/1ps
// Real FIFO + row controller, unlike the old ideal combinational FIFO stub.
// Two complete RAW frames at 40FPS, unrelated84MHz sensor clocks,125MHz TX.
module tb_camera_fifo_stream;
    localparam WIDTH=960,HEIGHT=1080,FRAMES=2;
    reg txclk=0,cam1=0,cam2=0,aclr=1,rst_n=0;
    reg wr1=0,wr2=0;
    reg [7:0] in1=0,in2=0;
    wire [7:0] q1,q2,out_data;
    wire [13:0] used1,used2;
    wire rd1,rd2,empty1,empty2,full1,full2,out_valid,fault;
    integer output_byte=0,output_row=0,output_frame=0,max1=0,max2=0;
    integer f1,y1,k1,f2,y2,k2;
    always #4 txclk=~txclk;
    always #5.943 cam1=~cam1;
    initial begin #2.5;forever #5.947 cam2=~cam2;end
    camera_async_fifo fifo1(.wrclk(cam1),.rdclk(txclk),.aclr(aclr),.wrreq(wr1),.rdreq(rd1),
        .wrdata(in1),.rddata(q1),.rdusedw(used1),.empty(empty1),.full(full1));
    camera_async_fifo fifo2(.wrclk(cam2),.rdclk(txclk),.aclr(aclr),.wrreq(wr2),.rdreq(rd2),
        .wrdata(in2),.rddata(q2),.rdusedw(used2),.empty(empty2),.full(full2));
    controller dut(.clk(txclk),.rst_n(rst_n),.rdclk(),.rdreq1(rd1),.rddata1(q1),.rdusedw1(used1),
        .rdreq2(rd2),.rddata2(q2),.rdusedw2(used2),.wrclk(),.fifo_aclr(),
        .fifo_wrreq(out_valid),.fifo_wrdata(out_data),.pair_fault(fault));
    function [7:0] pixel;
        input integer eye,frame,row,column;
        begin pixel=(column+row+79*eye+17*frame)%251;end
    endfunction
    reg [7:0] expected;
    always @(posedge txclk) if(rst_n)begin
        if(used1>max1)max1=used1;if(used2>max2)max2=used2;
        if((rd1 && empty1)||(rd2 && empty2))$fatal(1,"controller FIFO underflow");
        if(fault)$fatal(1,"row pair fault");
        if(out_valid)begin
            if(output_byte==0)expected=output_row;
            else if(output_byte==1)expected=output_row>>8;
            else if(output_byte<WIDTH+2)expected=pixel(0,output_frame,output_row,output_byte-2);
            else expected=pixel(1,output_frame,output_row,output_byte-WIDTH-2);
            if(out_data!==expected)$fatal(1,"frame%0d row%0d byte%0d got%02h expected%02h",output_frame,output_row,output_byte,out_data,expected);
            output_byte=output_byte+1;
            if(output_byte==2*WIDTH+2)begin
                output_byte=0;output_row=output_row+1;
                if(output_row==HEIGHT)begin output_row=0;output_frame=output_frame+1;end
            end
        end
    end
    always @(posedge cam1)if(rst_n && wr1 && full1)$fatal(1,"camera1 overflow");
    always @(posedge cam2)if(rst_n && wr2 && full2)$fatal(1,"camera2 overflow");
    initial begin
        #57;aclr=0;#200;rst_n=1;
    end
    initial begin
        wait(rst_n);
        for(f1=0;f1<FRAMES;f1=f1+1)begin
            for(y1=0;y1<HEIGHT;y1=y1+1)begin
                for(k1=0;k1<WIDTH+2;k1=k1+1)begin
                    @(negedge cam1);wr1=1;
                    if(k1==0)in1=y1;else if(k1==1)in1=y1>>8;else in1=pixel(0,f1,y1,k1-2);
                end
                @(negedge cam1);wr1=0;
                #10870;
            end
            #892857;
        end
    end
    initial begin
        wait(rst_n);#1800;
        for(f2=0;f2<FRAMES;f2=f2+1)begin
            for(y2=0;y2<HEIGHT;y2=y2+1)begin
                for(k2=0;k2<WIDTH+2;k2=k2+1)begin
                    @(negedge cam2);wr2=1;
                    if(k2==0)in2=y2;else if(k2==1)in2=y2>>8;else in2=pixel(1,f2,y2,k2-2);
                end
                @(negedge cam2);wr2=0;
                #10862;
            end
            #892857;
        end
        wait(output_frame==FRAMES);#1000;
        if(used1!=0 || used2!=0 || output_byte!=0)$fatal(1,"stream not drained");
        $display("PASS real async FIFO + controller: %0d complete1920x1080 frames at40FPS, allrows/wraps/bytes exact, maxoccupancy=%0d/%0d",FRAMES,max1,max2);$finish;
    end
    initial begin #60000000;$fatal(1,"timeout frames%0d row%0d",output_frame,output_row);end
endmodule
