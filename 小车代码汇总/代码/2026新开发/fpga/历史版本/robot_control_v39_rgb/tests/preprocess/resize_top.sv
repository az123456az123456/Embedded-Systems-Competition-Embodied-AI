module resize_top(
    input clk,rst_n,in_valid,in_sof,in_eol,in_eof,in_abort,input bypass,
    input [10:0] in_x,in_y,input [23:0] in_rgb,
    output out_valid,out_sof,out_eol,out_eof,out_abort,
    output [9:0] out_x,out_y,output [23:0] out_rgb,
    output [6:0] fifo_peak
);
    wire v,s,l,f,a;wire [9:0] x,y;wire [23:0] rgb;
    rgb_bilinear_resize resize(.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_sof(in_sof),
        .in_eol(in_eol),.in_eof(in_eof),.in_x(in_x),.in_y(in_y),.in_rgb(in_rgb),.in_abort(in_abort),
        .out_valid(v),.out_sof(s),.out_eol(l),.out_eof(f),.out_x(x),.out_y(y),.out_rgb(rgb),.out_abort(a));
    rgb_letterbox pad(.clk(clk),.rst_n(rst_n),.in_valid(bypass?in_valid:v),.in_sof(bypass?in_sof:s),
        .in_eol(bypass?in_eol:l),.in_eof(bypass?in_eof:f),.in_x(bypass?in_x[9:0]:x),
        .in_y(bypass?in_y[9:0]:y),.in_rgb(bypass?in_rgb:rgb),.in_abort(bypass?in_abort:a),.out_valid(out_valid),.out_sof(out_sof),
        .out_eol(out_eol),.out_eof(out_eof),.out_x(out_x),.out_y(out_y),.out_rgb(out_rgb),
        .out_abort(out_abort),.fifo_peak(fifo_peak));
endmodule
