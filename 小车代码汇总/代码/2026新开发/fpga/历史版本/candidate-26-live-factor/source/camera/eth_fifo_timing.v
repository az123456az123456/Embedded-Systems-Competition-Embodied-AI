`timescale 1ns/1ps
// Preserve the camera transmitter's 8KiB FWFT interface. The read-enable
// path uses registered empty/full flags, like the two source camera FIFOs.
module eth_dcfifo(
    input wire [7:0] Data,
    input wire Reset,WrClk,RdClk,WrEn,RdEn,
    output wire [12:0] Wnum,Rnum,
    output wire [7:0] Q,
    output wire Empty,Full
);
    camera_async_fifo #(.ADDR_WIDTH(13)) fifo(
        .wrclk(WrClk),.rdclk(RdClk),.aclr(Reset),
        .wrreq(WrEn),.rdreq(RdEn),.wrdata(Data),
        .rddata(Q),.rdusedw(Rnum),.wrusedw(Wnum),
        .empty(Empty),.full(Full)
    );
endmodule
