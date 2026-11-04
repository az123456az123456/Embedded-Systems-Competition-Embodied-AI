`timescale 1ns/1ps
// 128 ASCII bytes per message, sent as 115200-8N1 on the existing J2 UART.
module diag_uart_logger #(
    parameter integer CLK_HZ=50_000_000, BAUD=115200
)(input wire clk,rst_n,input wire event_valid,input wire [3:0] event_id,
  output wire tx,output reg overflow);
    localparam integer DIV=(CLK_HZ+BAUD/2)/BAUD;
    reg [7:0] rom[0:2047];
    initial $readmemh("rom/messages.hex",rom);
    reg [3:0] message;
    reg [6:0] position;
    reg [9:0] shift;
    reg [3:0] bits_left;
    reg [31:0] cycles;
    reg sending,boot_pending;
    assign tx=sending ? shift[0] : 1'b1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            message<=0;position<=0;shift<=10'h3ff;bits_left<=0;
            cycles<=0;sending<=0;boot_pending<=1;overflow<=0;
        end else begin
            if(event_valid && sending) overflow<=1;
            if(!sending) begin
                if(event_valid || boot_pending) begin
                    message<=event_valid ? event_id : 0;
                    position<=0;bits_left<=10;cycles<=0;sending<=1;boot_pending<=0;
                    shift<={1'b1,rom[{(event_valid ? event_id : 4'b0),7'b0}],1'b0};
                end
            end else if(cycles==DIV-1) begin
                cycles<=0;
                if(bits_left==1) begin
                    if(position==127) begin sending<=0;bits_left<=0;end
                    else begin
                        position<=position+1'b1;bits_left<=10;
                        shift<={1'b1,rom[{message,position+7'd1}],1'b0};
                    end
                end else begin shift<={1'b1,shift[9:1]};bits_left<=bits_left-1'b1;end
            end else cycles<=cycles+1'b1;
        end
    end
    // synthesis translate_off
    initial if(DIV<2) $fatal(1,"UART divider too small");
    // synthesis translate_on
endmodule
