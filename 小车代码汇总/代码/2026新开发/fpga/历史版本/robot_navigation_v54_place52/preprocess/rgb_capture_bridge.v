`timescale 1ns/1ps
// One physical camera, one pixel clock; RGB typed words feed a CDC FIFO.
// The camera is never backpressured. Any malformed/overflowed frame is
// explicitly aborted, rather than completing a tensor from shifted pixels.
module rgb_capture_bridge #(
 parameter WIDTH=960,HEIGHT=1080,BAYER=0
)(
 input wire clk,rst_n,init_done,
 input wire href,vsync,input wire [7:0] raw,
 input wire session_valid,input wire [15:0] session_epoch,
 input wire [31:0] uptime_gray,
 input wire fifo_full,output reg fifo_write,output reg [35:0] fifo_word,
 output reg fault,output reg [31:0] completed_frames,dropped_frames
);
 (* syn_async_reg=1 *) reg [31:0] time_sync0,time_sync1;
 reg [15:0] epoch;
 reg href_d,seen_vsync,active,drop_frame;
 reg [10:0] line_count,row;
 reg [31:0] frame_sequence,saved_sequence,saved_stamp;
 reg [1:0] meta_pending;
 reg abort_pending,eof_pending;
 reg [9:0] expected_x,expected_y;
 // The coherent mailbox payload is stable when session_valid arrives, but
 // is still a cross-domain held bus. Terminate it ONLY at this sampling bank.
 // Comparison and the wide ISP abort/CE fanout are driven by local registers.
 reg [15:0] sampled_session_epoch,accepted_session_epoch;
 reg sampled_session_valid,session_change,epoch_active;
 wire new_session=session_change;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   sampled_session_epoch<=0;accepted_session_epoch<=0;
   sampled_session_valid<=0;session_change<=0;epoch_active<=0;
  end else begin
   if(new_session)epoch_active<=accepted_session_epoch!=0;else epoch_active<=epoch!=0;
   sampled_session_valid<=session_valid;
   if(session_valid)sampled_session_epoch<=session_epoch;
   session_change<=0;
   // Compare with the last accepted request, not the later applied epoch:
   // repeated strobes must remain idempotent while the pipeline is pending.
   if(sampled_session_valid && sampled_session_epoch!=accepted_session_epoch)begin
    accepted_session_epoch<=sampled_session_epoch;session_change<=1;
   end
  end
 end
 wire raw_sof=href && !href_d && seen_vsync && init_done && epoch_active;
 wire raw_valid=href && (active || raw_sof) && line_count<WIDTH;
 wire raw_eol=raw_valid && line_count==WIDTH-1;
 wire raw_eof=raw_eol && row==HEIGHT-1;
 wire raw_fault=(href && active && line_count>=WIDTH) ||
                (!href && href_d && active && line_count!=WIDTH);
 wire dem_ready,dem_valid,dem_sof,dem_eol,dem_eof,dem_abort;
 wire [10:0] dem_x,dem_y;wire [23:0] dem_rgb;
 wire scale_valid,scale_sof,scale_eol,scale_eof,scale_abort;
 wire [9:0] scale_x,scale_y;wire [23:0] scale_rgb;
 wire out_valid,out_sof,out_eol,out_eof,out_abort;
 wire [9:0] out_x,out_y;wire [23:0] out_rgb;
 // A VSYNC before the last complete source row truncates the frame now,
 // rather than leaving a pending demosaic frame to abort the NEXT SOF.
 // VSYNC coincident with the falling edge of a complete final HREF is legal.
 wire vsync_truncated=vsync && active && !(row==HEIGHT-1 && line_count==WIDTH);
 wire source_abort=raw_fault || vsync_truncated || new_session;
 wire demosaic_abort=source_abort || (raw_valid && !dem_ready && !raw_sof);
 function [31:0] gray_decode;
  input [31:0] value;integer k;
  begin for(k=0;k<32;k=k+1)gray_decode[k]=^(value>>k);end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin time_sync0<=0;time_sync1<=0;end
  else begin time_sync0<=uptime_gray;time_sync1<=time_sync0;end
 end
 bayer_demosaic #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BAYER(BAYER)) demosaic(
  .clk(clk),.rst_n(rst_n),.in_valid(raw_valid),.in_sof(raw_sof),.in_eol(raw_eol),.in_pixel(raw),
  .in_abort(demosaic_abort),.in_ready(dem_ready),.out_valid(dem_valid),.out_sof(dem_sof),
  .out_eol(dem_eol),.out_eof(dem_eof),.out_x(dem_x),.out_y(dem_y),.out_rgb(dem_rgb),.out_abort(dem_abort));
 rgb_bilinear_resize resize(
  .clk(clk),.rst_n(rst_n),.in_valid(dem_valid),.in_sof(dem_sof),.in_eol(dem_eol),.in_eof(dem_eof),
  .in_x(dem_x),.in_y(dem_y),.in_rgb(dem_rgb),.in_abort(dem_abort),
  .out_valid(scale_valid),.out_sof(scale_sof),.out_eol(scale_eol),.out_eof(scale_eof),
  .out_x(scale_x),.out_y(scale_y),.out_rgb(scale_rgb),.out_abort(scale_abort));
 rgb_letterbox letterbox(
  .clk(clk),.rst_n(rst_n),.in_valid(scale_valid),.in_sof(scale_sof),.in_eol(scale_eol),.in_eof(scale_eof),
  .in_x(scale_x),.in_y(scale_y),.in_rgb(scale_rgb),.in_abort(scale_abort),
  .out_valid(out_valid),.out_sof(out_sof),.out_eol(out_eol),.out_eof(out_eof),
  .out_x(out_x),.out_y(out_y),.out_rgb(out_rgb),.out_abort(out_abort),.fifo_peak());
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   epoch<=0;href_d<=0;seen_vsync<=0;active<=0;line_count<=0;row<=0;frame_sequence<=0;
   fifo_write<=0;fifo_word<=0;meta_pending<=0;abort_pending<=0;eof_pending<=0;
   saved_sequence<=0;saved_stamp<=0;drop_frame<=1;expected_x<=0;expected_y<=0;
   completed_frames<=0;dropped_frames<=0;fault<=0;
  end else begin
   fifo_write<=0;href_d<=href;
   if(vsync)begin seen_vsync<=1;line_count<=0;row<=0;active<=0;end
   if(!href && href_d)begin
    line_count<=0;
    if(active && row==HEIGHT-1)active<=0;else if(active)row<=row+1'b1;
   end
   if(raw_valid)line_count<=line_count+1'b1;
   if(raw_sof)begin
    seen_vsync<=0;active<=1;row<=0;frame_sequence<=frame_sequence+1'b1;
    saved_sequence<=frame_sequence+1'b1;saved_stamp<=gray_decode(time_sync1);
    meta_pending<=2;eof_pending<=0;drop_frame<=abort_pending;
    expected_x<=0;expected_y<=0;
   end
   if(new_session)begin
    epoch<=accepted_session_epoch;frame_sequence<=0;active<=0;seen_vsync<=0;line_count<=0;row<=0;
    meta_pending<=0;abort_pending<=0;eof_pending<=0;drop_frame<=1;
   end else if(source_abort || out_abort || dem_abort)begin
    active<=0;drop_frame<=1;abort_pending<=1;meta_pending<=0;eof_pending<=0;
    fault<=1;if(!drop_frame)dropped_frames<=dropped_frames+1'b1;
   end else if((out_valid || meta_pending!=0 || eof_pending) && fifo_full && !drop_frame)begin
    drop_frame<=1;abort_pending<=1;meta_pending<=0;eof_pending<=0;
    fault<=1;dropped_frames<=dropped_frames+1'b1;
   end else if(abort_pending && !fifo_full)begin
    fifo_write<=1;fifo_word<={4'hf,frame_sequence};abort_pending<=0;
   end else if(meta_pending!=0 && !fifo_full && !raw_sof)begin
    fifo_write<=1;
    if(meta_pending==2)fifo_word<={4'h1,saved_sequence};else fifo_word<={4'h2,saved_stamp};
    meta_pending<=meta_pending-1'b1;
   end else if(out_valid && !drop_frame && !fifo_full)begin
    if(out_x!=expected_x || out_y!=expected_y || out_sof!=(expected_x==0 && expected_y==0) ||
       out_eol!=(expected_x==639) || out_eof!=(expected_x==639 && expected_y==639))begin
     drop_frame<=1;abort_pending<=1;fault<=1;dropped_frames<=dropped_frames+1'b1;
    end else begin
     fifo_write<=1;fifo_word<={4'h0,8'h00,out_rgb};
     if(expected_x==639)begin expected_x<=0;expected_y<=expected_y+1'b1;end
     else expected_x<=expected_x+1'b1;
     if(out_eof)eof_pending<=1;
    end
   end else if(eof_pending && !fifo_full && !drop_frame)begin
    fifo_write<=1;fifo_word<={4'h3,saved_sequence};eof_pending<=0;completed_frames<=completed_frames+1'b1;
   end
  end
 end
endmodule
