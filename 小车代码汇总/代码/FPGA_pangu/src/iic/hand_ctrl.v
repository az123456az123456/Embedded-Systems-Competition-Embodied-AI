
module hand_ctrl#(
	parameter	SLAVE_ID	=	7'b111_0011		,
				SYS_CLK_FREQ=	26'd50_000_000	,
				SCL_FREQ	=	23'd250_000
)                                                                                                              
(
	input	wire			sys_clk		,
	input	wire			sys_rst_n	,
	
	output	wire			scl			,
	
	inout	wire			sda		
);

localparam	CNT_CLK_MAX		=	(SYS_CLK_FREQ/SCL_FREQ) >> 2'd3  ;
localparam	CNT_T1_MAX		=	'd1000  ,
			CNT_T2_MAX		=	'd1000	;
localparam	IDLE			=	'd0		,
			START			=	'd1		,
			SLAVE_ADDR		=	'd2		,
			WAIT			=	'd3		,
			STOP			=	'd4		;
			
reg		[4:0]	n_state		;
reg		[4:0]	c_state		;
reg		[4:0]	cnt_clk		;
reg				i2c_clk		;
reg				skip_en_1	;
reg		[9:0]	cnt_wait	;
reg				i2c_scl		;
reg				i2c_sda		;
reg				i2c_end		;
reg		[1:0]	cnt_i2c_clk	;
reg		[2:0]	cnt_bit		;
reg		[9:0]	cnt_delay	;
reg		[2:0]	mode		;
reg		[7:0]	slave_addr	;
reg		[7:0]	device_addr	;
reg		[7:0]	wr_addr		;
wire			sda_en		;
wire			sda_in		;

assign	scl		=	i2c_scl		;
assign	sda_in	=	sda			;
assign	sda_en	=	1'b1		;
assign	sda		=	(sda_en == 1'b1) ? i2c_sda : 1'bz  ;

always@(*)
	case(mode)
		3'd0	:	begin
						slave_addr	<=  {SLAVE_ID,1'b0}  ;	//激活
						device_addr	<=  8'd0  ;
						wr_addr		<=  8'd0  ;					
					end
		default	:	begin
						slave_addr	<=  slave_addr   ;			
						device_addr	<=	device_addr  ;	
						wr_addr		<=	wr_addr		 ; 
					end
    endcase

///////生成i2c设备驱动时钟/////
always@(posedge sys_clk or negedge sys_rst_n)
	if(sys_rst_n == 1'b0)
		cnt_clk  <=  5'd0  ;
	else  if(cnt_clk == CNT_CLK_MAX - 1'b1)
		cnt_clk  <=  5'd0  ;
	else
		cnt_clk  <=  cnt_clk + 1'b1  ;	

always@(posedge sys_clk or negedge sys_rst_n)
	if(sys_rst_n == 1'b0)
		i2c_clk  <=  1'b0  ;
	else  if(cnt_clk == CNT_CLK_MAX - 1'b1)
		i2c_clk  <=  ~i2c_clk  ;
	else
		i2c_clk  <=  i2c_clk ;		
///////////////////////////////

///////状态机第一段////////
always@(posedge i2c_clk or negedge sys_rst_n)
	if(sys_rst_n == 1'b0)
		c_state  <=  IDLE  ;
	else
		c_state  <=  n_state  ;
		
//////状态机第二段
always@(*)
	case(c_state)
		IDLE		:	if(skip_en_1 == 1'b1)
							n_state  <=  START  ;
						else
							n_state  <=  IDLE  ;
		START		:	if(skip_en_1 == 1'b1)
							n_state  <=  SLAVE_ADDR  ;
						else
							n_state  <=  START  ;
	    SLAVE_ADDR	:	if(skip_en_1 == 1'b1)
							n_state  <=  WAIT  ;
						else
							n_state  <=  SLAVE_ADDR  ;
		WAIT		:	if(skip_en_1 == 1'b1)
							n_state  <=  STOP  ;
						else
							n_state  <=  WAIT  ;
		STOP		:	if(skip_en_1 == 1'b1)
							n_state  <=  IDLE  ;
						else
							n_state  <=  STOP  ;
		default		:	n_state  <=  IDLE  ;
	endcase
		
///////状态机第三段///////
always@(posedge i2c_clk or negedge sys_rst_n)
	if(sys_rst_n == 1'b0)
		begin
			cnt_wait  	<=  10'd0  	;
			skip_en_1	<=  1'b0    ;
			cnt_i2c_clk	<=  2'd0	;
			cnt_bit		<=  3'd0	;
			i2c_end		<=  1'b0	;
			mode		<=  3'd0	;
			cnt_delay	<=  10'd0	;
		end
	else
		case(c_state)
			IDLE		:begin
							if(cnt_wait == CNT_T1_MAX - 1'b1)
								cnt_wait  <=  10'd0  ;
							else
								cnt_wait  <=  cnt_wait + 1'b1  ;
							if((cnt_wait == CNT_T1_MAX - 2'd2)&&(mode == 3'd0))
								skip_en_1  <=  1'b1  ;
							else
								skip_en_1  <=  1'b0  ;
						 end
			START		:begin
							cnt_i2c_clk		<=  cnt_i2c_clk + 1'b1  ;
							if((cnt_i2c_clk == 2'd2)&&(mode == 3'd0))
								skip_en_1  <=  1'b1  ;
						    else
								skip_en_1  <=  1'b0  ;
						 end
			SLAVE_ADDR	:begin
							cnt_i2c_clk		<=  cnt_i2c_clk + 1'b1  ;
							if((cnt_i2c_clk == 2'd3)&&(cnt_bit == 3'd7))
								cnt_bit  <=  3'd0  ;
							else  if(cnt_i2c_clk == 2'd3)
								cnt_bit  <=  cnt_bit + 1'b1  ;
							else
								cnt_bit  <=  cnt_bit  ;
							if((cnt_i2c_clk == 2'd2)&&(cnt_bit == 3'd7)&&(mode == 3'd0))
								skip_en_1  <=  1'b1  ;
							else
								skip_en_1  <=  1'b0  ;
						 end
			WAIT		:begin
							cnt_delay  <=  cnt_delay + 1'b1  ;
							if(cnt_delay == CNT_T2_MAX - 2'd2)
								skip_en_1  <=  1'b1  ;
							else
								skip_en_1  <=  1'b0  ;
						 end
		    STOP		:begin
							cnt_i2c_clk  <=  cnt_i2c_clk + 1'b1  ;
							if(cnt_i2c_clk == 2'd2)
								i2c_end  <=  1'b1  ;
							else
								i2c_end  <=  1'b0  ;
							if((cnt_i2c_clk == 2'd2)&&(mode == 3'd0))
								skip_en_1  <=  1'b1  ;
							else
								skip_en_1  <=  1'b0  ;
							if(i2c_end == 1'b1)
								mode  <=  mode + 1'b1  ;
							else
								mode  <=  mode  ;
						 end
			default		:begin
							cnt_wait 	<=  10'd0	;
							skip_en_1	<=	1'b0	;
							cnt_i2c_clk	<=  2'd0	;
							cnt_bit		<=  3'd0	;
							i2c_end		<=  1'b0	;
							mode		<=  mode	;
							cnt_delay	<=  10'd0	;
						 end
		endcase
		
always@(*)
	case(c_state)
		IDLE		:	i2c_scl  <=  1'b1  ;
		START		:	if(cnt_i2c_clk == 2'd3)
							i2c_scl  <=  1'b0  ;
						else
							i2c_scl  <=  1'b1  ;
		SLAVE_ADDR:
						if((cnt_i2c_clk == 2'd1)||(cnt_i2c_clk == 2'd2))
							i2c_scl  <=  1'b1  ;
						else
							i2c_scl  <=  1'b0  ;
		WAIT		:	if((cnt_delay == 10'd0)||(cnt_delay == CNT_T2_MAX - 1'b1))
							i2c_scl  <=  1'b0  ;
						else
							i2c_scl  <=  1'b1  ;
		STOP		:	if(cnt_i2c_clk == 2'd0)
							i2c_scl  <=  1'b0  ;
						else
							i2c_scl  <=  1'b1  ;
	    default		:	i2c_scl  <=  1'b1  ;
	endcase
	
always@(*)
	case(c_state)
		IDLE		:	i2c_sda		<=  1'b1  ;
		START		:	if(cnt_i2c_clk == 2'd0)
							i2c_sda  <=  1'b1  ;
						else
							i2c_sda  <=  1'b0  ;
		SLAVE_ADDR	:	i2c_sda  <=  slave_addr[7-cnt_bit]  ;
		WAIT		:	i2c_sda  <=  1'b1  ;
		STOP		:	if((cnt_i2c_clk == 2'd0)||(cnt_i2c_clk == 2'd1))
							i2c_sda  <=  1'b0  ;
						else
							i2c_sda  <=  1'b1  ;
		default		:	i2c_sda  <=  1'b1  ;
	endcase

endmodule
