module prj7620_top
(
    input wire clk ,
    input wire rst_n ,
    output wire scl ,
    output reg [3:0] led ,
    inout wire sda ,
    output wire [5:0] sel , // 片选
    output reg [7:0] dig , // 段选
    output wire buzzer
);
parameter   ZERO = 7'b1000_000,
            ONE = 7'b1111_001,
            TWO = 7'b0100_100,
            THR = 7'b0110_000,
            FOU = 7'b0011_001;

wire [15:0] cfg_data ;
wire i2c_start ;
wire [5:0] reg_num ;
wire cfg_start ;
wire [3:0] mode ;
wire i2c_clk ;
wire [7:0] po_data ;
reg en;
wire pwm;

assign buzzer = en || pwm;

assign sel = 6'b111_110 ;

/*从高到低，依次分别是右、左、下、上*/

always@(posedge i2c_clk or negedge rst_n)begin

if(!rst_n )begin

led <= 4'b0000 ;

dig <= {1'b1,ZERO};

en <= 1'b1 ;

end

else if(po_data[3:0] == 4'b0001)begin //上

led <= 4'b0001 ;

dig <= {1'b1,ONE} ;

en <= 1'b0 ;

end

else if(po_data[3:0] == 4'b0010)begin //下

led <= 4'b0010 ;

dig <= {1'b1,TWO} ;

en <= 1'b1 ;

end

else if(po_data[3:0] == 4'b0100)begin //左

led <= 4'b0100 ;

dig <= {1'b1,THR} ;

en <= 1'b0 ;

end

else if(po_data[3:0] == 4'b1000)begin //右

led <= 4'b1000 ;

dig <= {1'b1,FOU} ;

en <= 1'b1 ;

end

else begin

led <= led ;

end

end

i2c_ctrl i2c_ctrl_inst

(

.clk (clk ),

.rst_n (rst_n ),

.cfg_data (cfg_data ),

.i2c_start (i2c_start ),

.reg_num (reg_num ),

.scl (scl ),

.cfg_start (cfg_start ),

.i2c_clk (i2c_clk ),

.mode (mode ),

.sda (sda ),

.po_data (po_data )

);

prj7620_cfg prj7620_cfg_inst

(

.i2c_clk (i2c_clk ),

.rst_n (rst_n ),

.cfg_start (cfg_start ),

.mode (mode ),

.cfg_data (cfg_data ),

.i2c_start (i2c_start ),

.reg_num (reg_num )

);

beep beep_inst(

.clk (clk ),

.rst_n (rst_n ),

.pwm (pwm)

);

endmodule