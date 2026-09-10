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

module prj7620_cfg

(

input wire i2c_clk ,

input wire rst_n ,

input wire cfg_start ,

input wire [3:0] mode ,

output wire [15:0] cfg_data ,

output reg i2c_start ,

output reg [5:0] reg_num //寄存器读取计数个数

);

wire [15:0] cfg_data_reg[50:0];

always@(posedge i2c_clk or negedge rst_n)

if(!rst_n)

reg_num <= 6'd0 ;

else if((cfg_start == 1'b1)&&(mode == 4'd4))

reg_num <= reg_num + 1'b1 ;

else

reg_num <= reg_num ;

always@(posedge i2c_clk or negedge rst_n)

if(!rst_n)

i2c_start <= 1'b0 ;

else if((cfg_start == 1'b1)&&(mode == 4'd4))

i2c_start <= 1'b1 ;

else

i2c_start <= 1'b0 ;

assign cfg_data = (mode != 4'd4) ? 15'd0 : cfg_data_reg[reg_num - 1'b1];

assign cfg_data_reg[00] = {8'hEF,8'h00} ;

assign cfg_data_reg[01] = {8'h37,8'h07} ;

assign cfg_data_reg[02] = {8'h38,8'h17} ;

assign cfg_data_reg[03] = {8'h39,8'h06} ;

assign cfg_data_reg[04] = {8'h42,8'h01} ;

assign cfg_data_reg[05] = {8'h46,8'h2D} ;

assign cfg_data_reg[06] = {8'h47,8'h0F} ;

assign cfg_data_reg[07] = {8'h48,8'h3C} ;

assign cfg_data_reg[08] = {8'h49,8'h00} ;

assign cfg_data_reg[09] = {8'h4A,8'h1E} ;

assign cfg_data_reg[10] = {8'h4C,8'h20} ;

assign cfg_data_reg[11] = {8'h51,8'h10} ;

assign cfg_data_reg[12] = {8'h5E,8'h10} ;

assign cfg_data_reg[13] = {8'h60,8'h27} ;

assign cfg_data_reg[14] = {8'h80,8'h42} ;

assign cfg_data_reg[15] = {8'h81,8'h44} ;

assign cfg_data_reg[16] = {8'h82,8'h04} ;

assign cfg_data_reg[17] = {8'h8B,8'h01} ;

assign cfg_data_reg[18] = {8'h90,8'h06} ;

assign cfg_data_reg[19] = {8'h95,8'h0A} ;

assign cfg_data_reg[20] = {8'h96,8'h0C} ;

assign cfg_data_reg[21] = {8'h97,8'h05} ;

assign cfg_data_reg[22] = {8'h9A,8'h14} ;

assign cfg_data_reg[23] = {8'h9C,8'h3F} ;

assign cfg_data_reg[24] = {8'hA5,8'h19} ;

assign cfg_data_reg[25] = {8'hCC,8'h19} ;

assign cfg_data_reg[26] = {8'hCD,8'h0B} ;

assign cfg_data_reg[27] = {8'hCE,8'h13} ;

assign cfg_data_reg[28] = {8'hCF,8'h64} ;

assign cfg_data_reg[29] = {8'hD0,8'h21} ;

assign cfg_data_reg[30] = {8'hEF,8'h01} ;

assign cfg_data_reg[31] = {8'h02,8'h0F} ;

assign cfg_data_reg[32] = {8'h03,8'h10} ;

assign cfg_data_reg[33] = {8'h04,8'h02} ;

assign cfg_data_reg[34] = {8'h25,8'h01} ;

assign cfg_data_reg[35] = {8'h27,8'h39} ;

assign cfg_data_reg[36] = {8'h28,8'h7F} ;

assign cfg_data_reg[37] = {8'h29,8'h08} ;

assign cfg_data_reg[38] = {8'h3E,8'hFF} ;

assign cfg_data_reg[39] = {8'h5E,8'h3D} ;

assign cfg_data_reg[40] = {8'h65,8'h96} ;

assign cfg_data_reg[41] = {8'h67,8'h97} ;

assign cfg_data_reg[42] = {8'h69,8'hCD} ;

assign cfg_data_reg[43] = {8'h6A,8'h01} ;

assign cfg_data_reg[44] = {8'h6D,8'h2C} ;

assign cfg_data_reg[45] = {8'h6E,8'h01} ;

assign cfg_data_reg[46] = {8'h72,8'h01} ;

assign cfg_data_reg[47] = {8'h73,8'h35} ;

assign cfg_data_reg[48] = {8'h74,8'h00} ;

assign cfg_data_reg[49] = {8'h77,8'h01} ;

assign cfg_data_reg[50] = {8'hEF,8'h00} ;

endmodule

module i2c_ctrl (

input clk ,

input rst_n ,

input [15:0] cfg_data ,

input i2c_start ,

input [5:0] reg_num ,

output scl ,

output reg cfg_start ,

output reg i2c_clk , //分频后的时钟

output reg [3:0] mode ,

output reg [7:0] po_data ,

inout sda

);

parameter CNT_CLK_MAX = 25, //分频计数

CNT_DELAY_MAX = 1000 , //计数1000us

SLAVE_ID = 7'b111_0011,

BANK_ADDR = 8'hEF,

DARA_ADDR = 8'h43;

parameter IDLE =4'd0,

START =4'd1,

SLAVE_ADDR =4'd2,

WAIT =4'd3,

STOP =4'd4,

ACK_1 =4'd5,

ACK_2 =4'd6,

ACK_3 =4'd7,

DATA =4'd8,

DEVICE_ADDR =4'd9,

NACK =4'd10;

reg [4:0] cnt_clk ;//分频计数器

reg [3:0] state_n ;// 次态

reg [3:0] state_c ;//现态

reg skip_en_0 ;//唤醒状态跳转条件

reg skip_en_1 ;//激活bank0跳转信号

reg skip_en_2 ;//配置0x00寄存器状态跳转信号

reg skip_en_3 ;//读取0x00寄存器状态跳转信号

reg skip_en_4 ;//配置51个操作寄存器

reg skip_en_5 ;//配置0x43寄存器状态跳转信号

reg skip_en_6 ;//读取0x43寄存器状态跳转信号

reg error_en ;//读取出来的值不是0x20，错误信号

reg [9:0] cnt_wait ;//等待1000us

reg [2:0] cnt_bit ;//计数传输数据8bit

reg [9:0] cnt_delay ;// 等待1000us

reg [1:0] cnt_i2c_clk ;

reg i2c_scl ;

reg i2c_end ;

reg [7:0] po_data_reg ;//采集数据，拼接完成后赋值给po_data

reg [7:0] slave_addr ;//不同模式下7'h73+1'bx

reg ACK ;//应答信号 高电平有效 ~sda_in

reg [7:0] device_addr ;//不同模式下寄存器地址变化

reg [7:0] data ;

reg [7:0] re_data ;//唤醒操作读取0x00寄存器数据寄存

wire sda_in ;

wire sda_en ; //主机控制sda线有效

reg i2c_sda ; //在响应状态下，从机发送给主机的数据

assign scl = i2c_scl;

assign sda_in = sda ;

assign sda = sda_en ? i2c_sda : 1'bz; //高阻态，让从机控制sda

assign sda_en = (state_c == ACK_1 || state_c == ACK_2 || state_c == ACK_3 || ((state_c == DATA) && mode == 4'd3) || ((state_c == DATA) && mode == 4'd6)) ? 1'b0 : 1'b1; //主机控制sda有效

always @(posedge i2c_clk or negedge rst_n)begin

if(!rst_n)begin

cfg_start <= 0;

end

else

cfg_start <= i2c_end;

end

/**************************************************************

引入mode信号的原因是，我们需要发送不同的指令实现数据的接收及发送，因此这些操作有差异。

通过该信号，编写代码时，mode对应模式执行来相应操作，避免后续状态跳转产生影响

**************************************************************/

//利用位拼接，提高了slave_addr和device_addr复用性

always @(*)begin

case(mode)

0 : slave_addr = {SLAVE_ID,1'b0}; //唤醒操作

1 : begin

slave_addr = {SLAVE_ID,1'b0}; //激活Bank0

device_addr = BANK_ADDR;

data = 8'b0000_0000;

end

2 : begin //配置0x00寄存器

slave_addr = {SLAVE_ID,1'b0};

device_addr = 8'b0000_0000;

data = 8'b0000_0000;

end

3 : begin //读取0x00寄存器

slave_addr = {SLAVE_ID,1'b1};

data = 8'b0000_0000;

end

4 : begin //配置操作寄存器，寄存器高8位储存地址，低8位储存数据

slave_addr = {SLAVE_ID,1'b0};

device_addr = cfg_data[15:8] ;

data = cfg_data[7:0] ;

end

5 : begin //配置手势寄存器

slave_addr = {SLAVE_ID,1'b0};

device_addr = DARA_ADDR; //DARA_ADDR = 8'h43为储存手势数据地址

end

6 : begin //读取手势寄存器

slave_addr = {SLAVE_ID,1'b1};

end

default : ;

endcase

end

//分频计数器进行计数

always @(posedge clk or negedge rst_n)begin

if(!rst_n)begin

cnt_clk <= 5'b0;

end

else if(cnt_clk == CNT_CLK_MAX - 1)begin

cnt_clk <= 5'b0;

end

else begin

cnt_clk <= cnt_clk + 1'b1;

end

end

//产生i2c驱动时钟

always @(posedge clk or negedge rst_n)begin

if(!rst_n)begin

i2c_clk <= 1'b1;

end

else if(cnt_clk == CNT_CLK_MAX - 1)begin

i2c_clk <= ~i2c_clk;

end

else begin

i2c_clk <= i2c_clk;

end

end

//状态机第一段

always @(posedge i2c_clk or negedge rst_n)begin

if(!rst_n)begin

state_c <= IDLE;

end

else begin

state_c <= state_n;

end

end

//状态机第二段

always @(*)begin

case(state_c)

IDLE : begin

if(skip_en_0 || skip_en_1 || skip_en_2 || skip_en_3 || skip_en_4 || skip_en_5 || skip_en_6)

state_n = START;

else

state_n = state_c;

end

START : begin

if(skip_en_0 || skip_en_1 || skip_en_2 || skip_en_3 || skip_en_4 || skip_en_5 || skip_en_6)

state_n = SLAVE_ADDR;

else

state_n = state_c;

end

SLAVE_ADDR : begin

if(skip_en_0)

state_n = WAIT;

else if(skip_en_1 || skip_en_2 || skip_en_3 || skip_en_4 || skip_en_5 || skip_en_6)

state_n = ACK_1;

else

state_n = state_c;

end

ACK_1 : begin

if(skip_en_1 || skip_en_2 || skip_en_4 || skip_en_5)

state_n = DEVICE_ADDR;

else if(skip_en_3 || skip_en_6)

state_n = DATA ;

else

state_n = state_c;

end

DEVICE_ADDR : begin

if(skip_en_1 || skip_en_2 || skip_en_4 || skip_en_5 )

state_n = ACK_2;

else

state_n = state_c;

end

ACK_2 : begin

if(skip_en_1 || skip_en_4)

state_n = DATA;

else if(skip_en_2 || skip_en_5)

state_n = STOP;

else

state_n = state_c;

end

DATA : begin

if(skip_en_1 || skip_en_4)

state_n = ACK_3;

else if(skip_en_3 || skip_en_6)

state_n = NACK;

else if(error_en)

state_n = IDLE;

else

state_n = state_c;

end

ACK_3 : begin

if(skip_en_1 || skip_en_4)

state_n = STOP;

else

state_n = state_c;

end

NACK : begin

if(skip_en_3 || skip_en_6)

state_n = STOP;

else

state_n = state_c;

end

WAIT : begin

if(skip_en_0)

state_n = STOP;

else

state_n = state_c;

end

STOP : begin

if(skip_en_0 || skip_en_1 || skip_en_2 || skip_en_3 || skip_en_4 || skip_en_5 || skip_en_6)

state_n = IDLE;

else

state_n = state_c;

end

default : state_n = state_c;

endcase

end

//状态机第三段

always @(posedge i2c_clk or negedge rst_n)begin

if(!rst_n)begin

cnt_wait <= 10'b0;

skip_en_0 <= 1'b0 ;

skip_en_1 <= 1'b0 ;

skip_en_2 <= 1'b0 ;

skip_en_3 <= 1'b0 ;

skip_en_4 <= 1'b0 ;

skip_en_5 <= 1'b0 ;

skip_en_6 <= 1'b0 ;

error_en <= 1'b0 ;

cnt_bit <= 3'b0 ;

cnt_i2c_clk <= 2'b0 ;

cnt_delay <= 10'b0;

mode <= 4'b0 ;

i2c_end <= 1'b0 ;

end

else begin

case(state_c)

IDLE : begin

if(cnt_wait == CNT_DELAY_MAX - 1)

cnt_wait <= 10'b0;

else

cnt_wait <= cnt_wait + 1'b1;

if((cnt_wait == CNT_DELAY_MAX - 2'd2) && (mode == 4'd0))

skip_en_0 <= 1'b1;

else

skip_en_0 <= 1'b0;

if((cnt_wait == CNT_DELAY_MAX - 2'd2) && (mode == 4'd1))

skip_en_1 <= 1'b1;

else

skip_en_1 <= 1'b0;

if((cnt_wait == CNT_DELAY_MAX - 2'd2) && (mode == 4'd2))

skip_en_2 <= 1'b1;

else

skip_en_2 <= 1'b0;

if((cnt_wait == CNT_DELAY_MAX - 2'd2) && (mode == 4'd3))

skip_en_3 <= 1'b1;

else

skip_en_3 <= 1'b0;

if((i2c_start == 1'b1) && (mode == 4'd4))

skip_en_4 <= 1'b1;

else

skip_en_4 <= 1'b0;

if((cnt_wait == CNT_DELAY_MAX - 2'd2) && (mode == 4'd5))

skip_en_5 <= 1'b1;

else

skip_en_5 <= 1'b0;

if((cnt_wait == CNT_DELAY_MAX - 2'd2) && (mode == 4'd6))

skip_en_6 <= 1'b1;

else

skip_en_6 <= 1'b0;

end

START : begin

cnt_i2c_clk <= cnt_i2c_clk + 1'b1;

if((cnt_i2c_clk == 2'd2) && (mode == 4'd0))

skip_en_0 <= 1'b1;

else

skip_en_0 <= 1'b0;

if((cnt_i2c_clk == 2'd2) && (mode == 4'd1))

skip_en_1 <= 1'b1;

else

skip_en_1 <= 1'b0;

if((cnt_i2c_clk == 2'd2) && (mode == 4'd2))

skip_en_2 <= 1'b1;

else

skip_en_2 <= 1'b0;

if((cnt_i2c_clk == 2'd2) && (mode == 4'd3))

skip_en_3 <= 1'b1;

else

skip_en_3 <= 1'b0;

if((cnt_i2c_clk == 2'd2) && (mode == 4'd4))

skip_en_4 <= 1'b1;

else

skip_en_4 <= 1'b0;

if((cnt_i2c_clk == 2'd2) && (mode == 4'd5))

skip_en_5 <= 1'b1;

else

skip_en_5 <= 1'b0;

if((cnt_i2c_clk == 2'd2) && (mode == 4'd6))

skip_en_6 <= 1'b1;

else

skip_en_6 <= 1'b0;

end

SLAVE_ADDR : begin

cnt_i2c_clk <= cnt_i2c_clk + 1'b1;

if(cnt_i2c_clk == 2'd3)

cnt_bit <= cnt_bit +1'b1;

else

cnt_bit <= cnt_bit;

if((cnt_bit == 3'd7) && (mode == 4'd0) && (cnt_i2c_clk == 2'd2))

skip_en_0 <= 1'b1;

else

skip_en_0 <= 1'b0;

if((cnt_bit == 3'd7) && (mode == 4'd1) && (cnt_i2c_clk == 2'd2))

skip_en_1 <= 1'b1;

else

skip_en_1 <= 1'b0;

if((cnt_bit == 3'd7) && (mode == 4'd2) && (cnt_i2c_clk == 2'd2))

skip_en_2 <= 1'b1;

else

skip_en_2 <= 1'b0;

if((cnt_bit == 3'd7) && (mode == 4'd3) && (cnt_i2c_clk == 2'd2))

skip_en_3 <= 1'b1;

else

skip_en_3 <= 1'b0;

if((cnt_bit == 3'd7) && (mode == 4'd4) && (cnt_i2c_clk == 2'd2))

skip_en_4 <= 1'b1;

else

skip_en_4 <= 1'b0;

if((cnt_bit == 3'd7) && (mode == 4'd5) && (cnt_i2c_clk == 2'd2))

skip_en_5 <= 1'b1;

else

skip_en_5 <= 1'b0;

if((cnt_bit == 3'd7) && (mode == 4'd6) && (cnt_i2c_clk == 2'd2))

skip_en_6 <= 1'b1;

else

skip_en_6 <= 1'b0;

end

ACK_1 : begin

cnt_i2c_clk <= cnt_i2c_clk + 1'b1;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd1)

skip_en_1 <= 1'b1;

else

skip_en_1 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd2)

skip_en_2 <= 1'b1;

else

skip_en_2 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd3)

skip_en_3 <= 1'b1;

else

skip_en_3 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd4)

skip_en_4 <= 1'b1;

else

skip_en_4 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd5)

skip_en_5 <= 1'b1;

else

skip_en_5 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd6)

skip_en_6 <= 1'b1;

else

skip_en_6 <= 1'b0;

end

DEVICE_ADDR : begin

cnt_i2c_clk <= cnt_i2c_clk + 1'b1;

if(cnt_i2c_clk == 2'd3)

cnt_bit <= cnt_bit +1'b1;

else

cnt_bit <= cnt_bit;

if((cnt_bit == 3'd7) && (mode == 4'd1) && (cnt_i2c_clk == 2'd2))

skip_en_1 <= 1'b1;

else

skip_en_1 <= 1'b0;

if((cnt_bit == 3'd7) && (mode == 4'd2) && (cnt_i2c_clk == 2'd2))

skip_en_2 <= 1'b1;

else

skip_en_2 <= 1'b0;

if((cnt_bit == 3'd7) && (mode == 4'd4) && (cnt_i2c_clk == 2'd2))

skip_en_4 <= 1'b1;

else

skip_en_4 <= 1'b0;

if((cnt_bit == 3'd7) && (mode == 4'd5) && (cnt_i2c_clk == 2'd2))

skip_en_5 <= 1'b1;

else

skip_en_5 <= 1'b0;

end

ACK_2 : begin

cnt_i2c_clk <= cnt_i2c_clk + 1'b1;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd1)

skip_en_1 <= 1'b1;

else

skip_en_1 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd2)

skip_en_2 <= 1'b1;

else

skip_en_2 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd4)

skip_en_4 <= 1'b1;

else

skip_en_4 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd5)

skip_en_5 <= 1'b1;

else

skip_en_5 <= 1'b0;

end

DATA : begin

cnt_i2c_clk <= cnt_i2c_clk + 1'b1;

if(cnt_i2c_clk == 2'd3)

cnt_bit <= cnt_bit +1'b1;

else

cnt_bit <= cnt_bit;

if(cnt_bit ==3'd7 && cnt_i2c_clk == 2'd2 && mode == 4'd1)

skip_en_1 <= 1'b1;

else

skip_en_1 <= 1'b0;

if(cnt_bit ==3'd7 && cnt_i2c_clk == 2'd2 && mode == 4'd3 && re_data == 8'h20)

skip_en_3 <= 1'b1;

else

skip_en_3 <= 1'b0;

if(cnt_bit ==3'd7 && cnt_i2c_clk == 2'd2 && mode == 4'd3 && re_data != 8'h20)

begin

error_en <= 1'b1;

mode <= 4'd0;

end

else begin

error_en <= 1'b0;

mode <= mode;

end

if((cnt_i2c_clk == 2'd2)&&(cnt_bit == 3'd7)&&(mode == 4'd4))

skip_en_4 <= 1'b1 ;

else

skip_en_4 <= 1'b0 ;

if((cnt_i2c_clk == 2'd2)&&(cnt_bit == 3'd7)&&(mode == 4'd6))

skip_en_6 <= 1'b1 ;

else

skip_en_6 <= 1'b0 ;

end

NACK : begin

cnt_i2c_clk <= cnt_i2c_clk + 1'b1;

if(ACK == 1'b1 && cnt_i2c_clk == 2'd2 && mode == 4'd3)

skip_en_3 <= 1'b1;

else

skip_en_3 <= 1'b0;

if(ACK == 1'b1 && cnt_i2c_clk == 2'd2 && mode == 4'd6)

skip_en_6 <= 1'b1;

else

skip_en_6 <= 1'b0;

end

ACK_3 : begin

cnt_i2c_clk <= cnt_i2c_clk + 1'b1;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd1)

skip_en_1 <= 1'b1;

else

skip_en_1 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && ACK && mode == 4'd4)

skip_en_4 <= 1'b1;

else

skip_en_4 <= 1'b0;

end

WAIT : begin

if(cnt_delay == CNT_DELAY_MAX - 1)

cnt_delay <= 10'b0;

else

cnt_delay <= cnt_delay + 1'b1;

if((cnt_delay == CNT_DELAY_MAX -2) && (mode == 4'd0))

skip_en_0 <= 1'b1;

else

skip_en_0 <= 1'b0;

end

STOP : begin

cnt_i2c_clk <= cnt_i2c_clk + 1'b1;

if(cnt_i2c_clk == 2'd2 && mode == 4'd0)

skip_en_0 <= 1'b1;

else

skip_en_0 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && mode == 4'd1)

skip_en_1 <= 1'b1;

else

skip_en_1 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && mode == 4'd2)

skip_en_2 <= 1'b1;

else

skip_en_2 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && mode == 4'd3)

skip_en_3 <= 1'b1;

else

skip_en_3 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && mode == 4'd4)

skip_en_4 <= 1'b1;

else

skip_en_4 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && mode == 4'd5)

skip_en_5 <= 1'b1;

else

skip_en_5 <= 1'b0;

if(cnt_i2c_clk == 2'd2 && mode == 4'd6)

skip_en_6 <= 1'b1;

else

skip_en_6 <= 1'b0;

if(cnt_i2c_clk == 2'd2)

i2c_end <= 1'b1;

else

i2c_end <= 1'b0;

if(i2c_end && (mode <= 4'd3))

mode <= mode + 1'b1;

else if(mode == 4'd4 && i2c_end == 1'b1 && reg_num == 6'd51)

mode <= mode + 1'b1;

else if((mode ==4'd5)&&(i2c_end == 1'b1))

mode <= mode + 1'b1;

else

mode <= mode;

end

default : begin

cnt_wait <= 10'b0;

skip_en_0 <= 1'b0 ;

skip_en_1 <= 1'b0 ;

skip_en_2 <= 1'b0 ;

skip_en_3 <= 1'b0 ;

skip_en_4 <= 1'b0 ;

skip_en_5 <= 1'b0 ;

skip_en_6 <= 1'b0 ;

error_en <= 1'b0 ;

cnt_bit <= 3'b0 ;

cnt_i2c_clk <= 2'b0 ;

cnt_delay <= 10'b0;

mode <= mode ;

i2c_end <= 1'b0 ;

end

endcase

end

end

always @(posedge i2c_clk or negedge rst_n)begin

if(rst_n == 1'b0)

re_data <= 8'b0;

else

case(state_c)

DATA : if((mode ==4'd3) && cnt_i2c_clk == 2'd1)

re_data <= {re_data[6:0],sda_in};

else

re_data <= re_data;

default : re_data <= re_data;

endcase

end

always @(posedge i2c_clk or negedge rst_n)begin

if(rst_n == 1'b0)

po_data_reg <= 8'd0;

else

case(state_c)

DATA : if((mode ==4'd6) && cnt_i2c_clk == 2'd1)

po_data_reg <= {po_data_reg[6:0],sda_in};//MSB 串并转换

else

po_data_reg <= po_data_reg;

default : po_data_reg <= po_data_reg ;

endcase

end

always@(posedge i2c_clk or negedge rst_n)

if(!rst_n)

po_data <= 8'd0 ;

else if((cnt_i2c_clk == 2'd2)&&(cnt_bit == 3'd7)&&(mode == 4'd6))

po_data <= po_data_reg ;

else

po_data <= po_data ;

always @(*)begin

case(state_c)

IDLE , START , SLAVE_ADDR , DEVICE_ADDR , DATA , STOP : ACK =1'b0;

ACK_1 ,ACK_2 , ACK_3 : ACK = ~sda_in;

NACK : ACK = i2c_sda;

default : ACK = 1'b0;

endcase

end

//scl

always @(*)begin

case(state_c)

IDLE : i2c_scl = 1'b1;

START : if(cnt_i2c_clk == 2'd3)

i2c_scl = 1'b0;

else

i2c_scl = 1'b1;

SLAVE_ADDR , ACK_1 , DEVICE_ADDR , ACK_2 , DATA , ACK_3 , NACK :

if(cnt_i2c_clk == 2'd0 || cnt_i2c_clk == 2'd3)

i2c_scl = 1'b0;

else

i2c_scl = 1'b1;

WAIT : i2c_scl = 1'b0;

STOP : if(cnt_i2c_clk == 2'd0)

i2c_scl = 1'b0;

else

i2c_scl = 1'b1;

default : i2c_scl = 1'b1;

endcase

end

//sda

always @(*)begin

case(state_c)

IDLE : i2c_sda = 1'b1;

START : if(cnt_i2c_clk == 2'd0)

i2c_sda = 1'b1;

else

i2c_sda = 1'b0;

SLAVE_ADDR : i2c_sda = slave_addr[7-cnt_bit]; //从MSB开始传

DEVICE_ADDR : i2c_sda = device_addr[7-cnt_bit];

DATA : i2c_sda = data[7-cnt_bit];

ACK_1 , ACK_2 , ACK_3 : i2c_sda = 1'b0;

NACK : i2c_sda = 1'b1;

WAIT : i2c_sda = 1'b0;

STOP : if(cnt_i2c_clk == 2'd0 || cnt_i2c_clk == 2'd1)

i2c_sda = 1'b0;

else

i2c_sda = 1'b1;

default : i2c_sda = 1'b1;

endcase

end

endmodule

module beep#( parameter CLK_PRE = 50_000_000, TIME_300MS = 15_000_000)( //频率控制音色 占空比控制音量 占空比越小 低电平占比越大 音量就越大

input clk ,

input rst_n ,

output reg pwm

);

parameter DO = CLK_PRE / 523, // 该频率需要多少个系统时钟周期

RE = CLK_PRE / 587,

MI = CLK_PRE / 659,

FA = CLK_PRE / 698,

SO = CLK_PRE / 784,

LA = CLK_PRE / 880,

SI = CLK_PRE / 988;

reg [16:0] cnt1 ; //计数频率

wire add_cnt1 ;

wire end_cnt1 ;

reg [16:0] X ; //cnt1最大值 表示当前为哪一个频率

reg [23:0] cnt2 ; //记300ms 每一个音符放300ms

wire add_cnt2 ;

wire end_cnt2 ;

reg [5:0] cnt3 ; //计数乐谱48个

wire add_cnt3 ;

wire end_cnt3 ;

reg pwm_ctrl ; //为1时 不发声音 让后25%不发声

always @(posedge clk or negedge rst_n)begin

if(!rst_n)begin

cnt1 <= 17'b0;

end

else if(end_cnt2)begin //300ms计数器完了 重新开始记新的一个频率

cnt1 <= 17'b0;

end

else if(add_cnt1)begin

if(end_cnt1)begin

cnt1 <= 17'b0;

end

else begin

cnt1 <= cnt1 + 1'b1;

end

end

else begin

cnt1 <= cnt1;

end

end

assign add_cnt1 = 1;

assign end_cnt1 = add_cnt1 && cnt1 == X - 1;

always @(posedge clk or negedge rst_n)begin

if(!rst_n)begin

cnt2 <= 24'b0;

end

else if(add_cnt2)begin

if(end_cnt2)begin

cnt2 <= 24'b0;

end

else begin

cnt2 <= cnt2 + 1'b1;

end

end

else begin

cnt2 <= cnt2;

end

end

assign add_cnt2 = 1;

assign end_cnt2 = add_cnt2 && cnt2 == TIME_300MS - 1;

always @(posedge clk or negedge rst_n)begin

if(!rst_n)begin

cnt3 <= 6'b0;

end

else if(add_cnt3)begin

if(end_cnt3)begin

cnt3 <= 6'b0;

end

else begin

cnt3 <= cnt3 + 1'b1;

end

end

else begin

cnt3 <= cnt3;

end

end

assign add_cnt3 = end_cnt2;

assign end_cnt3 = add_cnt3 && cnt3 == 48 - 1;

always @(*)begin

case(cnt3)

0 : X = DO;

1 : X = RE;

2 : X = MI;

3 : X = DO;

4 : X = DO;

5 : X = RE;

6 : X = MI;

7 : X = DO;

8 : X = MI;

9 : X = FA;

10 : X = SO;

11 : X = SO;

12 : X = MI;

13 : X = FA;

14 : X = SO;

15 : X = SO;

16 : X = SO;

17 : X = SI;

18 : X = SO;

19 : X = FA;

20 : X = MI;

21 : X = MI;

22 : X = DO;

23 : X = DO;

24 : X = SO;

25 : X = SI;

26 : X = SO;

27 : X = FA;

28 : X = MI;

29 : X = MI;

30 : X = DO;

31 : X = DO;

32 : X = RE;

33 : X = RE;

34 : X = SO;

35 : X = SO;

36 : X = DO;

37 : X = DO;

38 : X = 1 ;

39 : X = 1 ;

40 : X = RE;

41 : X = RE;

42 : X = SO;

43 : X = SO;

44 : X = DO;

45 : X = DO;

46 : X = 1 ;

47 : X = 1 ;

default : X = 1 ;

endcase

end

always @(posedge clk or negedge rst_n)begin

if(!rst_n)begin

pwm_ctrl <= 1'b0;

end

else if(X == 1)begin

pwm_ctrl <= 1'b1;

end

else if(cnt2 > ((TIME_300MS >> 2) + (TIME_300MS >> 1)))begin // 1/2 + 1/4 表示后25%

pwm_ctrl <= 1'b1;

end

else begin

pwm_ctrl <= 1'b0;

end

end

always @(posedge clk or negedge rst_n)begin

if(!rst_n)begin

pwm <= 1'b1;

end

else if(pwm_ctrl == 1'b1)begin

pwm <= 1'b1;

end

else if(cnt1 <(X >> 5))begin

pwm <= 1'b0;

end

else begin

pwm <= 1'b1;

end

end

endmodule