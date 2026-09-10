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