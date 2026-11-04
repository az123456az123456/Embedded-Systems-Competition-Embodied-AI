module controller_top #(parameter META_INTERVAL=12500000)(
        profile_info,
        pair_fault,
        clk,
        rst_n,
        
        wrclk1,
        fifo1_aclr,
        wrdata1,
        wrreq1,
        
        wrclk2,
        fifo2_aclr,
        wrdata2,
        wrreq2,
        
        wrclk,
        fifo_aclr,
        fifo_wrreq,
        fifo_wrdata
    );
    
    input [1215:0] profile_info;
    output pair_fault;
    input clk;
    input rst_n;
    
    input wrclk1;
    input fifo1_aclr;
    input [7:0]wrdata1;
    input wrreq1;
        
    input wrclk2;
    input fifo2_aclr;
    input [7:0]wrdata2;
    input wrreq2;
        
    output wrclk;
    output fifo_aclr;
    output fifo_wrreq;
    output [7:0]fifo_wrdata;
    
    wire cache_rdclk;
    wire cache1_rdreq;
    wire cache2_rdreq;
    wire [7:0] cache1_rddata;
    wire [7:0] cache2_rddata;
    wire [13:0] rdusedw1;
    wire [13:0] rdusedw2;
    
    
	cache cache1(
		.wrclk				(wrclk1     	),
		.aclr				(fifo1_aclr		),
		.wrreq				(wrreq1     	),
		.wrdata				(wrdata1    	),
		.rdclk				(cache_rdclk	),
		.rdreq				(cache1_rdreq	),
		.rddata				(cache1_rddata	),
		.rdusedw			(rdusedw1		)
	);
	
	cache cache2(
		.wrclk				(wrclk2      	),
		.aclr				(fifo2_aclr		),
		.wrreq				(wrreq2     	),
		.wrdata				(wrdata2     	),
		.rdclk				(cache_rdclk	),
		.rdreq				(cache2_rdreq	),
		.rddata				(cache2_rddata	),
		.rdusedw			(rdusedw2		)
	);
	
    // Both camera FIFO resets can deassert in unrelated PCLK domains.
    // Keep their combined assertion immediate, but release the row scheduler locally.
    wire controller_async_reset_n=rst_n;
    wire controller_reset_n;
    camera_reset_release controller_reset(.clk(clk),.async_reset_n(controller_async_reset_n),.reset_n(controller_reset_n));
	controller #(.META_INTERVAL(META_INTERVAL)) controller(
        .profile_info(profile_info),
        .pair_fault(pair_fault),
		.clk				(clk  	     	),
		.rst_n				(controller_reset_n),
		.rdclk				(cache_rdclk	),
		.rdreq1				(cache1_rdreq	),
		.rddata1			(cache1_rddata	),
		.rdusedw1			(rdusedw1		),
		.rdreq2				(cache2_rdreq	),
		.rddata2			(cache2_rddata	),
		.rdusedw2			(rdusedw2		),
		.wrclk				(wrclk			),
		.fifo_aclr			(fifo_aclr		),
		.fifo_wrreq			(fifo_wrreq		),
		.fifo_wrdata		(fifo_wrdata	)
	);

endmodule
