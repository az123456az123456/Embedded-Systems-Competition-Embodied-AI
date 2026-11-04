// Fixed half-pixel bilinear downscale: 960x1080 RGB -> 569x640 RGB.
// No input backpressure and no full-frame storage. Two alternating source
// row banks are read synchronously while the OTHER bank is written. Each
// selected output is calculated as its x1/y1 source pixel arrives; there is
// no later scan that could race a bank being reused by the next source row.
module rgb_bilinear_resize #(
    parameter X_COEFF_FILE="preprocess/resize_x_q11.hex"
)(
    input wire clk, input wire rst_n,
    input wire in_valid, input wire in_sof, input wire in_eol, input wire in_eof,
    input wire [10:0] in_x, input wire [10:0] in_y,
    input wire [23:0] in_rgb, input wire in_abort,
    output reg out_valid, output reg out_sof, output reg out_eol, output reg out_eof,
    output reg [9:0] out_x, output reg [9:0] out_y,
    output reg [23:0] out_rgb, output reg out_abort
);
    (* syn_ramstyle="block_ram" *) reg [23:0] row_even[0:959];
    (* syn_ramstyle="block_ram" *) reg [23:0] row_odd[0:959];
    (* syn_romstyle="block_rom" *) reg [11:0] x_coeff[0:959];
    initial $readmemh(X_COEFF_FILE,x_coeff);
    function [11:0] y_coefficient;
        input [4:0] phase;
        begin
            case(phase)
                1:y_coefficient=12'hac0; 3:y_coefficient=12'h840;
                4:y_coefficient=12'hdc0; 6:y_coefficient=12'hb40;
                8:y_coefficient=12'h8c0; 9:y_coefficient=12'he40;
                11:y_coefficient=12'hbc0;13:y_coefficient=12'h940;
                14:y_coefficient=12'hec0;16:y_coefficient=12'hc40;
                18:y_coefficient=12'h9c0;19:y_coefficient=12'hf40;
                21:y_coefficient=12'hcc0;23:y_coefficient=12'ha40;
                24:y_coefficient=12'hfc0;26:y_coefficient=12'hd40;
                default:y_coefficient=0;
            endcase
        end
    endfunction
    function signed [20:0] horizontal_product;
        input [7:0] a,b;input [10:0] weight;
        reg signed [8:0] delta;
        begin
            delta=$signed({1'b0,b})-$signed({1'b0,a});
            horizontal_product=delta*$signed({1'b0,weight});
        end
    endfunction
    function [18:0] horizontal_sum;
        input [7:0] base;input signed [20:0] product;
        reg signed [20:0] sum;
        begin
            sum=$signed({2'b0,base,11'd0})+product;
            horizontal_sum=sum[18:0];
        end
    endfunction
    // Nonnegative unsigned products are kept in their own register stage.
    // Gowin 1.9.12.03 incorrectly folded the former signed delta form into
    // a MULTALU27X18 preadder: D was sign-extended with ones, adding
    // 2^19*weight and causing 16-row brightness bands on the physical board.
    // This formulation is exactly a*(2048-w)+b*w, with Q22 rounding below.
    function [30:0] vertical_product;
        input [18:0] value; input [11:0] weight;
        begin vertical_product=value*weight; end
    endfunction
    reg input_active;
    reg [10:0] expected_x,expected_y;
    reg [4:0] row_phase;
    wire start=in_valid&&in_sof&&in_x==0&&in_y==0&&!in_eol&&!in_eof;
    wire ordered=in_valid&&input_active&&!in_sof&&in_x==expected_x&&in_y==expected_y&&
        in_eol==(in_x==959)&&(!in_eof||(in_y==1079&&in_eol));
    wire accept=start||ordered;
    wire bad=in_valid&&!start&&((in_sof)||input_active&&!ordered);
    wire restart=start&&input_active;
    reg s0_valid,s0_sof;
    reg [23:0] s0_rgb,previous_rgb,previous_top;
    reg [23:0] even_read,odd_read;reg s0_bank;
    // Register each BRAM output BEFORE the bank mux, so synthesis does not
    // mistake one shared output register for an asynchronous memory read.
    wire [23:0] s0_top=s0_bank?even_read:odd_read;
    reg [11:0] s0_cx,s0_cy;
    // Terminate BRAM bank/neighbor selection at these input registers.
    // A second stage holds the signed products, so horizontal multiply and
    // base addition no longer share either the BRAM DO path or one cycle.
    reg h0_valid,h0_sof,h0_eol,h0_eof;
    reg [9:0] h0_x,h0_y;
    reg [23:0] h0_top_left,h0_top_right,h0_bottom_left,h0_bottom_right;
    reg [10:0] h0_wx,h0_wy;
    reg hp_valid,hp_sof,hp_eol,hp_eof;
    reg [9:0] hp_x,hp_y;
    reg [23:0] hp_top_base,hp_bottom_base;
    reg [10:0] hp_wy;
    reg signed [20:0] hp_top_r,hp_top_g,hp_top_b,hp_bottom_r,hp_bottom_g,hp_bottom_b;
    reg s1_valid,s1_sof,s1_eol,s1_eof;
    reg [9:0] s1_x,s1_y,dest_x,dest_y;
    reg [18:0] htop_r,htop_g,htop_b,hbottom_r,hbottom_g,hbottom_b;
    reg [10:0] s1_wy;
    reg vp_valid,vp_sof,vp_eol,vp_eof;
    reg [9:0] vp_x,vp_y;
    (* syn_keep=1 *) reg [30:0] vp_ar,vp_ag,vp_ab,vp_br,vp_bg,vp_bb;
    wire [11:0] vertical_weight_b={1'b0,s1_wy};
    wire [11:0] vertical_weight_a=12'd2048-vertical_weight_b;
    reg s2_valid,s2_sof,s2_eol,s2_eof;
    reg [9:0] s2_x,s2_y;reg [29:0] vr,vg,vb;
    always @(posedge clk) begin
        if(!rst_n) begin
            input_active<=0;expected_x<=0;expected_y<=0;row_phase<=0;
            s0_valid<=0;h0_valid<=0;hp_valid<=0;s1_valid<=0;vp_valid<=0;s2_valid<=0;dest_x<=0;dest_y<=0;
            out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_abort<=0;
            out_x<=0;out_y<=0;out_rgb<=0;
            previous_rgb<=0;previous_top<=0;
        end else begin
            out_valid<=s2_valid;out_sof<=s2_valid&&s2_sof;
            out_eol<=s2_valid&&s2_eol;out_eof<=s2_valid&&s2_eof;out_abort<=0;
            out_x<=s2_x;out_y<=s2_y;out_rgb<={vr[29:22],vg[29:22],vb[29:22]};
            s2_valid<=vp_valid;s2_sof<=vp_sof;s2_eol<=vp_eol;s2_eof<=vp_eof;s2_x<=vp_x;s2_y<=vp_y;
            if(vp_valid)begin
                vr<={1'b0,vp_ar}+{1'b0,vp_br}+32'd2097152;
                vg<={1'b0,vp_ag}+{1'b0,vp_bg}+32'd2097152;
                vb<={1'b0,vp_ab}+{1'b0,vp_bb}+32'd2097152;
            end
            vp_valid<=s1_valid;vp_sof<=s1_sof;vp_eol<=s1_eol;vp_eof<=s1_eof;vp_x<=s1_x;vp_y<=s1_y;
            if(s1_valid)begin
                vp_ar<=vertical_product(htop_r,vertical_weight_a);
                vp_ag<=vertical_product(htop_g,vertical_weight_a);
                vp_ab<=vertical_product(htop_b,vertical_weight_a);
                vp_br<=vertical_product(hbottom_r,vertical_weight_b);
                vp_bg<=vertical_product(hbottom_g,vertical_weight_b);
                vp_bb<=vertical_product(hbottom_b,vertical_weight_b);
            end
            s1_valid<=hp_valid;s1_sof<=hp_sof;s1_eol<=hp_eol;s1_eof<=hp_eof;s1_x<=hp_x;s1_y<=hp_y;
            if(hp_valid)begin
                s1_wy<=hp_wy;
                htop_r<=horizontal_sum(hp_top_base[23:16],hp_top_r);
                htop_g<=horizontal_sum(hp_top_base[15:8],hp_top_g);
                htop_b<=horizontal_sum(hp_top_base[7:0],hp_top_b);
                hbottom_r<=horizontal_sum(hp_bottom_base[23:16],hp_bottom_r);
                hbottom_g<=horizontal_sum(hp_bottom_base[15:8],hp_bottom_g);
                hbottom_b<=horizontal_sum(hp_bottom_base[7:0],hp_bottom_b);
            end
            hp_valid<=h0_valid;hp_sof<=h0_sof;hp_eol<=h0_eol;hp_eof<=h0_eof;hp_x<=h0_x;hp_y<=h0_y;
            if(h0_valid)begin
                hp_wy<=h0_wy;hp_top_base<=h0_top_left;hp_bottom_base<=h0_bottom_left;
                hp_top_r<=horizontal_product(h0_top_left[23:16],h0_top_right[23:16],h0_wx);
                hp_top_g<=horizontal_product(h0_top_left[15:8],h0_top_right[15:8],h0_wx);
                hp_top_b<=horizontal_product(h0_top_left[7:0],h0_top_right[7:0],h0_wx);
                hp_bottom_r<=horizontal_product(h0_bottom_left[23:16],h0_bottom_right[23:16],h0_wx);
                hp_bottom_g<=horizontal_product(h0_bottom_left[15:8],h0_bottom_right[15:8],h0_wx);
                hp_bottom_b<=horizontal_product(h0_bottom_left[7:0],h0_bottom_right[7:0],h0_wx);
            end
            h0_valid<=0;
            if(s0_valid)begin
                previous_rgb<=s0_rgb;previous_top<=s0_top;
                if(s0_sof)begin dest_x<=0;dest_y<=0;end
                if(s0_cx[11]&&s0_cy[11])begin
                    h0_valid<=1;h0_x<=dest_x;h0_y<=dest_y;
                    h0_sof<=dest_x==0&&dest_y==0;h0_eol<=dest_x==568;h0_eof<=dest_x==568&&dest_y==639;
                    h0_wx<=s0_cx[10:0];h0_wy<=s0_cy[10:0];
                    h0_top_left<=previous_top;h0_top_right<=s0_top;
                    h0_bottom_left<=previous_rgb;h0_bottom_right<=s0_rgb;
                    if(dest_x==568)begin dest_x<=0;dest_y<=dest_y+1'b1;end
                    else dest_x<=dest_x+1'b1;
                end
            end
            s0_valid<=0;
            if(accept)begin
                // Distinct read/write banks, including x=0 and x=959.
                if(in_y[0])begin row_odd[in_x[9:0]]<=in_rgb;even_read<=row_even[in_x[9:0]];end
                else begin row_even[in_x[9:0]]<=in_rgb;odd_read<=row_odd[in_x[9:0]];end
                s0_bank<=in_y[0];
                s0_valid<=1;s0_sof<=in_sof;s0_rgb<=in_rgb;
                s0_cx<=x_coeff[in_x[9:0]];s0_cy<=y_coefficient(start?5'd0:row_phase);
                if(start)begin input_active<=1;expected_x<=1;expected_y<=0;row_phase<=0;end
                else if(in_eol)begin
                    expected_x<=0;
                    if(in_y==1079)begin input_active<=0;expected_y<=0;row_phase<=0;end
                    else begin expected_y<=expected_y+1'b1;row_phase<=row_phase==26?0:row_phase+1'b1;end
                end else expected_x<=expected_x+1'b1;
            end
            if(restart)begin
                // Keep this new SOF in s0 while invalidating all old work.
                h0_valid<=0;hp_valid<=0;s1_valid<=0;vp_valid<=0;s2_valid<=0;out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_abort<=1;
                dest_x<=0;dest_y<=0;
            end
            if(in_abort||bad)begin
                input_active<=0;s0_valid<=0;h0_valid<=0;hp_valid<=0;s1_valid<=0;vp_valid<=0;s2_valid<=0;
                out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_abort<=1;
                dest_x<=0;dest_y<=0;expected_x<=0;expected_y<=0;row_phase<=0;
            end
        end
    end
endmodule
