`timescale 1ns/1ps
module tb_bayer_demosaic;
    parameter WIDTH=8,HEIGHT=6,BAYER=0;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,in_valid=0,in_sof=0,in_eol=0,in_abort=0;
    reg [7:0] in_pixel=0;
    wire in_ready,out_valid,out_sof,out_eol,out_eof,out_abort;
    wire [10:0] out_x,out_y;wire [23:0] out_rgb;
    bayer_demosaic #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BAYER(BAYER)) dut(.*);
    integer fd,read_ok,v,s,e,a,p,c,cycle=0;reg [1023:0] stimulus;
    always @(posedge clk)begin
        #1;
        if(rst_n && out_abort)$display("ABORT %0d",cycle);
        if(rst_n && out_valid)$display("PIXEL %0d %0d %0d %0d %0d %0d %06x",cycle,out_x,out_y,out_sof,out_eol,out_eof,out_rgb);
        cycle=cycle+1;
    end
    initial begin
        if(!$value$plusargs("stimulus=%s",stimulus))$fatal(1,"missing stimulus");
        repeat(4)@(negedge clk);rst_n=1;
        fd=$fopen(stimulus,"r");if(fd==0)$fatal(1,"cannot open stimulus");
        while(!$feof(fd))begin
            read_ok=$fscanf(fd,"%d %d %d %d %d %d\n",c,v,s,e,a,p);
            if(read_ok==6)begin
                @(negedge clk);in_valid=v;in_sof=s;in_eol=e;in_abort=a;in_pixel=p;
            end
        end
        @(negedge clk);in_valid=0;in_sof=0;in_eol=0;in_abort=0;
        repeat(WIDTH+12)@(negedge clk);
        $display("DONE");$finish;
    end
endmodule
