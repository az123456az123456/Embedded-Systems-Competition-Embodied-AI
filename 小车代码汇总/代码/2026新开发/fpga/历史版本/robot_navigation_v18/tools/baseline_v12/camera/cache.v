module cache(
	wrclk,
	aclr,
	wrreq,
	wrdata,
	rdclk,
	rdreq,
	rddata,
	rdusedw
);
	input wrclk;
	input aclr;
	input wrreq;
	input [7:0] wrdata;
	input rdclk;
	input rdreq;
	output [7:0] rddata;
	output [13:0] rdusedw;
	

	dc_fifo dc_fifo(
		.Data(wrdata), //input [7:0] Data
		.Reset(aclr), //input Reset
		.WrClk(wrclk), //input WrClk
		.RdClk(rdclk), //input RdClk
		.WrEn(wrreq), //input WrEn
		.RdEn(rdreq), //input RdEn
		.Wnum(), //output [13:0] Wnum
		.Rnum(rdusedw), //output [13:0] Rnum
		.Q(rddata), //output [7:0] Q
		.Empty(), //output Empty
		.Full() //output Full
	);

endmodule
