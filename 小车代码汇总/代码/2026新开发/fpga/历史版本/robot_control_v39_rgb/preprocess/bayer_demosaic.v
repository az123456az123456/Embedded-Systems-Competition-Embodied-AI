// RAW8 Bayer -> RGB888, independent 3x3 bilinear implementation.
// Algorithm/reference review: https://github.com/amd/mini-isp (MIT, MHC 5x5).
// This implementation is NOT AMD's 5x5 MHC filter and contains no AMD IP.
// Reflect-101 boundaries preserve Bayer colour parity at all four edges.
// Three rotating RAW8 rows, synchronous read ports; no whole-frame storage.
module bayer_demosaic #(
    parameter integer WIDTH=960,
    parameter integer HEIGHT=1080,
    // Same phase enumeration as Mini-ISP: BGGR=0, GBRG=1, GRBG=2, RGGB=3.
    parameter integer BAYER=0
)(
    input wire clk,
    input wire rst_n,
    input wire in_valid,
    input wire in_sof,
    input wire in_eol,
    input wire [7:0] in_pixel,
    input wire in_abort,
    // Normally ready through all active input pixels. Low only while the
    // mirrored final row is replayed/drained (WIDTH+7 clocks in VBLANK).
    // A new SOF always preempts that replay, aborting the incomplete frame.
    output wire in_ready,
    output reg out_valid,
    output reg out_sof,
    output reg out_eol,
    output reg out_eof,
    output reg [10:0] out_x,
    output reg [10:0] out_y,
    output reg [23:0] out_rgb,
    output reg out_abort
);
    localparam integer AW=(WIDTH<=2)?1:$clog2(WIDTH);
    localparam [10:0] LAST_X=WIDTH[10:0]-11'd1;
    localparam [10:0] LAST_Y=HEIGHT[10:0]-11'd1;
    localparam [10:0] FLUSH_Y=HEIGHT[10:0];
    (* syn_ramstyle="block_ram" *) reg [7:0] row0[0:WIDTH-1];
    (* syn_ramstyle="block_ram" *) reg [7:0] row1[0:WIDTH-1];
    (* syn_ramstyle="block_ram" *) reg [7:0] row2[0:WIDTH-1];
    reg [7:0] q0,q1,q2;
    reg active,pending_frame,flush_active;
    reg [10:0] input_x,input_y,flush_x;
    reg [1:0] input_bank,last_bank;

    wire new_sof=in_valid && in_sof;
    wire [10:0] accept_x=new_sof?11'd0:input_x;
    wire [10:0] accept_y=new_sof?11'd0:input_y;
    wire [1:0] accept_bank=new_sof?2'd0:input_bank;
    wire line_bad=in_valid && ((new_sof && in_eol) ||
                  (!new_sof && active && (in_eol!=(input_x==LAST_X))) ||
                  (!new_sof && pending_frame && !active));
    wire accept_pixel=in_valid && !in_abort && !line_bad && (new_sof || active);
    wire issue_bottom=flush_active && !new_sof && !in_abort && !line_bad;
    wire issue_read=accept_pixel || issue_bottom;
    wire [10:0] read_x=issue_bottom?flush_x:accept_x;
    wire [AW-1:0] read_address=read_x[AW-1:0];
    wire clear_pipe=in_abort || line_bad || new_sof;
    assign in_ready=active || !pending_frame;

    // Each bank has one synchronous read port and one write port. The current
    // input row is never used as a read source until a later row; old contents
    // from read-during-write are deliberately not consumed.
    always @(posedge clk) begin
        if(issue_read)q0<=row0[read_address];
        if(accept_pixel && accept_bank==0)row0[read_address]<=in_pixel;
    end
    always @(posedge clk) begin
        if(issue_read)q1<=row1[read_address];
        if(accept_pixel && accept_bank==1)row1[read_address]<=in_pixel;
    end
    always @(posedge clk) begin
        if(issue_read)q2<=row2[read_address];
        if(accept_pixel && accept_bank==2)row2[read_address]<=in_pixel;
    end

    reg strip_valid,strip_bottom;
    reg [10:0] strip_x,strip_y;
    reg [1:0] strip_bank;
    reg [7:0] strip_pixel;
    reg [7:0] top_now,middle_now,bottom_now;
    always @* begin
        top_now=0;middle_now=0;bottom_now=strip_pixel;
        if(strip_bottom)begin
            case(strip_bank)
              0:begin middle_now=q0;top_now=q2;end
              1:begin middle_now=q1;top_now=q0;end
              default:begin middle_now=q2;top_now=q1;end
            endcase
            bottom_now=top_now; // reflect HEIGHT -> HEIGHT-2
        end else begin
            case(strip_bank)
              0:begin middle_now=q2;top_now=q1;end
              1:begin middle_now=q0;top_now=q2;end
              default:begin middle_now=q1;top_now=q0;end
            endcase
            if(strip_y==1)top_now=strip_pixel; // reflect -1 -> 1
        end
    end

    reg [7:0] top_previous,middle_previous,bottom_previous;
    reg [7:0] top_older,middle_older,bottom_older;
    reg right_pending;
    reg [10:0] right_y;
    // Separate the BRAM bank/reflect mux, pair sums, final rounded sums and
    // colour-phase mux. No BRAM DO path contains interpolation arithmetic.
    // Every stage carries the same pixel's coordinates and boundary flags.
    reg window_valid,window_sof,window_eol,window_eof;
    reg [10:0] window_x,window_y;
    reg [7:0] window_tl,window_ml,window_bl,window_tc,window_mc,window_bc;
    reg [7:0] window_tr,window_mr,window_br;
    reg sum_valid,sum_sof,sum_eol,sum_eof;
    reg [10:0] sum_x,sum_y;
    reg [1:0] sum_phase;
    reg [7:0] sum_center;
    reg [8:0] sum_horizontal,sum_vertical,sum_diagonal_a,sum_diagonal_b;
    reg average_valid,average_sof,average_eol,average_eof;
    reg [10:0] average_x,average_y;
    reg [1:0] average_phase;
    reg [7:0] average_center,average_horizontal,average_vertical;
    reg [9:0] average_cross,average_diagonal;
    wire [9:0] rounded_horizontal={1'b0,sum_horizontal}+10'd1;
    wire [9:0] rounded_vertical={1'b0,sum_vertical}+10'd1;
    wire emit_eof=average_valid && average_eof && !clear_pipe;

    always @(posedge clk)begin
        if(!rst_n)begin
            sum_valid<=0;sum_sof<=0;sum_eol<=0;sum_eof<=0;sum_x<=0;sum_y<=0;
            sum_phase<=0;sum_center<=0;sum_horizontal<=0;sum_vertical<=0;
            sum_diagonal_a<=0;sum_diagonal_b<=0;
            average_valid<=0;average_sof<=0;average_eol<=0;average_eof<=0;
            average_x<=0;average_y<=0;average_phase<=0;average_center<=0;
            average_horizontal<=0;average_vertical<=0;average_cross<=0;average_diagonal<=0;
            out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_x<=0;out_y<=0;out_rgb<=0;
        end else begin
            sum_valid<=window_valid;
            sum_sof<=window_sof;sum_eol<=window_eol;sum_eof<=window_eof;
            if(window_valid)begin
                sum_x<=window_x;sum_y<=window_y;
                sum_phase<={window_y[0]^BAYER[1],window_x[0]^BAYER[0]};
                sum_center<=window_mc;
                sum_horizontal<={1'b0,window_ml}+{1'b0,window_mr};
                sum_vertical<={1'b0,window_tc}+{1'b0,window_bc};
                sum_diagonal_a<={1'b0,window_tl}+{1'b0,window_tr};
                sum_diagonal_b<={1'b0,window_bl}+{1'b0,window_br};
            end
            average_valid<=sum_valid;
            average_sof<=sum_sof;average_eol<=sum_eol;average_eof<=sum_eof;
            if(sum_valid)begin
                average_x<=sum_x;average_y<=sum_y;average_phase<=sum_phase;
                average_center<=sum_center;
                average_horizontal<=rounded_horizontal[8:1];
                average_vertical<=rounded_vertical[8:1];
                average_cross<={1'b0,sum_horizontal}+{1'b0,sum_vertical}+10'd2;
                average_diagonal<={1'b0,sum_diagonal_a}+{1'b0,sum_diagonal_b}+10'd2;
            end
            out_valid<=average_valid;
            out_sof<=average_valid && average_sof;
            out_eol<=average_valid && average_eol;
            out_eof<=average_valid && average_eof;
            if(average_valid)begin
                out_x<=average_x;out_y<=average_y;
                case(average_phase)
                    2'b00:out_rgb<={average_diagonal[9:2],average_cross[9:2],average_center};
                    2'b11:out_rgb<={average_center,average_cross[9:2],average_diagonal[9:2]};
                    2'b01:out_rgb<={average_vertical,average_center,average_horizontal};
                    default:out_rgb<={average_horizontal,average_center,average_vertical};
                endcase
            end
            // A new physical SOF may preempt a still-draining old frame.
            // Clear every old valid at once; the accepted new RAW pixel lives
            // in strip_valid and is deliberately not cleared here.
            if(clear_pipe)begin
                sum_valid<=0;average_valid<=0;
                out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;
            end
        end
    end

    always @(posedge clk) begin
        if(!rst_n)begin
            active<=0;pending_frame<=0;flush_active<=0;input_x<=0;input_y<=0;
            input_bank<=0;last_bank<=0;flush_x<=0;strip_valid<=0;strip_bottom<=0;
            strip_x<=0;strip_y<=0;strip_bank<=0;strip_pixel<=0;
            top_previous<=0;middle_previous<=0;bottom_previous<=0;
            top_older<=0;middle_older<=0;bottom_older<=0;right_pending<=0;right_y<=0;
            window_valid<=0;window_sof<=0;window_eol<=0;window_eof<=0;window_x<=0;window_y<=0;
            window_tl<=0;window_ml<=0;window_bl<=0;window_tc<=0;window_mc<=0;window_bc<=0;
            window_tr<=0;window_mr<=0;window_br<=0;out_abort<=0;
        end else begin
            window_valid<=0;window_sof<=0;window_eol<=0;window_eof<=0;out_abort<=0;
            if(emit_eof)pending_frame<=0;
            strip_valid<=issue_read;
            if(issue_read)begin
                strip_bottom<=issue_bottom;strip_x<=read_x;
                strip_y<=issue_bottom?FLUSH_Y:accept_y;
                strip_bank<=issue_bottom?last_bank:accept_bank;strip_pixel<=in_pixel;
            end

            if(issue_bottom)begin
                if(flush_x==LAST_X)begin flush_active<=0;flush_x<=0;end
                else flush_x<=flush_x+1'b1;
            end
            if(accept_pixel)begin
                pending_frame<=1;active<=1;
                if(accept_x==LAST_X)begin
                    input_x<=0;
                    if(accept_y==LAST_Y)begin
                        active<=0;flush_active<=1;flush_x<=0;last_bank<=accept_bank;
                    end else begin
                        input_y<=accept_y+1'b1;
                        input_bank<=(accept_bank==2)?0:accept_bank+1'b1;
                    end
                end else begin
                    input_x<=accept_x+1'b1;input_y<=accept_y;input_bank<=accept_bank;
                end
                if(new_sof)begin flush_active<=0;right_pending<=0;end
            end

            if(!clear_pipe)begin
                // Emit the reflected right edge in the next cycle. A new
                // row's x=0 may be captured in this same cycle, preserving
                // continuous one-pixel-per-clock input without EOL bubbles.
                if(right_pending)begin
                    window_valid<=1;window_x<=LAST_X;window_y<=right_y;window_eol<=1;
                    window_tl<=top_older;window_ml<=middle_older;window_bl<=bottom_older;
                    window_tc<=top_previous;window_mc<=middle_previous;window_bc<=bottom_previous;
                    window_tr<=top_older;window_mr<=middle_older;window_br<=bottom_older;
                    right_pending<=0;
                    if(right_y==LAST_Y)window_eof<=1;
                end
                if(strip_valid && strip_y!=0)begin
                    if(strip_x!=0)begin
                        window_valid<=1;window_x<=strip_x-1'b1;window_y<=strip_y-1'b1;
                        window_sof<=(strip_y==1 && strip_x==1);
                        window_tl<=(strip_x==1)?top_now:top_older;
                        window_ml<=(strip_x==1)?middle_now:middle_older;
                        window_bl<=(strip_x==1)?bottom_now:bottom_older;
                        window_tc<=top_previous;window_mc<=middle_previous;window_bc<=bottom_previous;
                        window_tr<=top_now;window_mr<=middle_now;window_br<=bottom_now;
                    end
                    top_older<=top_previous;middle_older<=middle_previous;bottom_older<=bottom_previous;
                    top_previous<=top_now;middle_previous<=middle_now;bottom_previous<=bottom_now;
                    if(strip_x==LAST_X)begin right_pending<=1;right_y<=strip_y-1'b1;end
                end
            end else begin
                right_pending<=0;
            end
            if(in_abort || line_bad)begin
                active<=0;pending_frame<=0;flush_active<=0;strip_valid<=0;
                right_pending<=0;window_valid<=0;window_sof<=0;window_eol<=0;window_eof<=0;out_abort<=1;
                input_x<=0;input_y<=0;input_bank<=0;
            end else if(new_sof && pending_frame)begin
                // The newly accepted first pixel survives; only old pipeline
                // values and downstream incomplete-frame state are discarded.
                window_valid<=0;window_sof<=0;window_eol<=0;window_eof<=0;out_abort<=1;
            end
        end
    end
endmodule
