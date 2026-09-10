
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