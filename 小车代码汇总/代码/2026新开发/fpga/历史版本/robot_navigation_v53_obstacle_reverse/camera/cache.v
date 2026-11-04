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
	

	// Preserve the16KiB FWFT boundary, but use registered empty/full flags.
	// The previous protected FIFO put next-pointer equality logic on the
	// BSRAM read-enable path, leaving only40ps at125MHz in thev18 build.
	camera_async_fifo #(.ADDR_WIDTH(14)) fifo(
		.wrclk(wrclk),.rdclk(rdclk),.aclr(aclr),
		.wrreq(wrreq),.rdreq(rdreq),.wrdata(wrdata),
		.rddata(rddata),.rdusedw(rdusedw),.empty(),.full()
	);

endmodule
