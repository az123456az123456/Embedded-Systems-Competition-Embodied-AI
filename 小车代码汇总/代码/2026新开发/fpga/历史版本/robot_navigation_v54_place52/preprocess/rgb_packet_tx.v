// RGP1 raw-Ethernet RGB transmitter. Two 1KiB synchronous RAM banks allow
// packet packing/CRC and Ethernet sending to overlap. No whole-frame store.
// FIFO word tags: 1=SOF/seq32, 2=capture uptime32, 0=RGB888, 3=EOF, f=ABORT.
module rgb_packet_tx #(
    parameter [47:0] DEST_MAC=48'hffffffffffff,
    parameter [47:0] SOURCE_MAC=48'h02ac13800001,
    parameter integer IMAGE_HEIGHT=640,
    parameter integer STAMP_TOLERANCE_MS=8,
    parameter integer WAIT_CYCLES=12500000
)(
    input wire clk,input wire rst_n,input wire [15:0] epoch,
    input wire tx_allow,output wire tx_busy,output wire tx_pending,
    output reg [7:0] gmii_data,output reg gmii_valid,
    input wire [35:0] fifo0_data,input wire fifo0_empty,output reg fifo0_pop,
    input wire [35:0] fifo1_data,input wire fifo1_empty,output reg fifo1_pop,
    output reg pair_fault,output reg [31:0] pair_fault_count,
    output reg [31:0] packets_sent,output reg [31:0] pairs_started
);
    localparam [2:0] L_IDLE=0,L_HEADER=1,L_PIXELS=2,L_EOF=3,L_CRC=4,L_HEADER_PRIME=5,
                     L_PAYLOAD_DRAIN=6,L_COMMIT=7;
    localparam [10:0] LAST_CHUNK=IMAGE_HEIGHT[9:0]*11'd2-11'd1;
    localparam [31:0] STAMP_LIMIT=STAMP_TOLERANCE_MS;
    // Saturates at WAIT_CYCLES; the default 12,500,000 needs 24 bits.
    // Keep +1 wide to avoid a signed-int overflow at the parameter boundary.
    localparam integer WAIT_BITS=(WAIT_CYCLES==0)?1:$clog2({1'b0,WAIT_CYCLES}+33'd1);
    localparam [WAIT_BITS-1:0] WAIT_LIMIT=WAIT_CYCLES;
    reg [15:0] current_epoch;
    wire epoch_ok=(epoch!=0 && epoch==current_epoch);
    reg [1:0] meta0,meta1;
    reg [31:0] seq0,seq1,stamp0,stamp1,pair_seq,pair_stamp;
    reg pair_active,done0,done1,next_eye;
    // Configuration changes are atomic for a complete left/right tensor pair.
    reg [10:0] chunk0,chunk1;
    reg [WAIT_BITS-1:0] wait_count;
    wire [WAIT_BITS-1:0] wait_increment;
    // Parallel 8-bit increments with reduction carries replace the long
    // carry chain; no delayed carry state or changed timeout edge.
    genvar wait_bit;
    generate for(wait_bit=0;wait_bit<WAIT_BITS;wait_bit=wait_bit+8)begin:wait_segments
        localparam integer BITS=(WAIT_BITS-wait_bit>8)?8:WAIT_BITS-wait_bit;
        if(wait_bit==0)assign wait_increment[wait_bit +:BITS]=wait_count[wait_bit +:BITS]+1'b1;
        else assign wait_increment[wait_bit +:BITS]=wait_count[wait_bit +:BITS]+(&wait_count[wait_bit-1:0]);
    end endgenerate

    (* syn_ramstyle="block_ram" *) reg [7:0] bank0[0:1023];
    (* syn_ramstyle="block_ram" *) reg [7:0] bank1[0:1023];
    reg [7:0] bank0_q,bank1_q;
    reg [1:0] ready;
    reg [2:0] load_state;
    reg load_bank,load_eye;
    reg [10:0] load_chunk;
    reg [4:0] header_index;
    reg [7:0] header_hold;
    reg [9:0] payload_index;
    reg [1:0] channel,crc_index;
    reg [23:0] rgb_hold;
    reg [31:0] app_crc;
    reg sending,send_bank,send_prefer;
    reg [10:0] send_position;
    reg [3:0] gap;
    reg [31:0] ether_crc;

    function [31:0] crc_byte;
        input [31:0] crc;
        input [7:0] data;
        reg [31:0] c;
        integer bit_index;
        begin
            c=crc^{24'd0,data};
            for(bit_index=0;bit_index<8;bit_index=bit_index+1)
                c=c[0]?((c>>1)^32'hedb88320):(c>>1);
            crc_byte=c;
        end
    endfunction
    function [7:0] header_byte;
        input [4:0] n;
        input eye;
        input [10:0] chunk;
        input [31:0] seq,stamp;
        input [15:0] ep;
        reg [15:0] row;
        begin
            row={6'd0,chunk[10:1]};
            case(n)
              0:header_byte=8'h52;1:header_byte=8'h47;2:header_byte=8'h50;3:header_byte=8'h31;
              4:header_byte=1;5:header_byte={7'd0,eye};6:header_byte=1;7:header_byte=(seq==1)?1:0;
              8:header_byte=seq[31:24];9:header_byte=seq[23:16];10:header_byte=seq[15:8];11:header_byte=seq[7:0];
              12:header_byte=stamp[31:24];13:header_byte=stamp[23:16];14:header_byte=stamp[15:8];15:header_byte=stamp[7:0];
              16:header_byte=row[15:8];17:header_byte=row[7:0];18:header_byte={7'd0,chunk[0]};19:header_byte=2;
              20:header_byte=8'h02;21:header_byte=8'h80;
              22:header_byte=IMAGE_HEIGHT[15:8];23:header_byte=IMAGE_HEIGHT[7:0];
              24:header_byte=8'h03;25:header_byte=8'hc0;
              26:header_byte=ep[15:8];27:header_byte=ep[7:0];
              default:header_byte=0;
            endcase
        end
    endfunction
    function [7:0] ether_header;
        input [3:0] n;
        begin
            case(n)
              0:ether_header=DEST_MAC[47:40];1:ether_header=DEST_MAC[39:32];2:ether_header=DEST_MAC[31:24];
              3:ether_header=DEST_MAC[23:16];4:ether_header=DEST_MAC[15:8];5:ether_header=DEST_MAC[7:0];
              6:ether_header=SOURCE_MAC[47:40];7:ether_header=SOURCE_MAC[39:32];8:ether_header=SOURCE_MAC[31:24];
              9:ether_header=SOURCE_MAC[23:16];10:ether_header=SOURCE_MAC[15:8];11:ether_header=SOURCE_MAC[7:0];
              12:ether_header=8'h88;default:ether_header=8'hba;
            endcase
        end
    endfunction

    // One-word input stages sever BRAM DO -> tag/fatal decode -> FIFO pointer
    // and CRC feedback. FIFO words are removed only when captured here; the
    // existing seek/pair state machine consumes the registered copy. Refill
    // only after consumption: two cycles per word still exceed payload's one
    // word per three clocks, and keep the late seek/load-state decision out
    // of the FIFO BRAM address and pointer timing cone.
    reg [35:0] head0_data,head1_data;
    reg head0_valid,head1_valid;
    // Classify in parallel with the input register, not in the byte/CRC and
    // next-state cone. These flags describe exactly the stored head word.
    reg head0_sof,head0_stamp,head0_pixel,head0_eof,head0_abort;
    reg head1_sof,head1_stamp,head1_pixel,head1_eof,head1_abort;
    reg head0_take,head1_take;
    wire head0_empty=!head0_valid,head1_empty=!head1_valid;
    always @* begin
        fifo0_pop=epoch_ok && !fifo0_empty && !head0_valid;
        fifo1_pop=epoch_ok && !fifo1_empty && !head1_valid;
    end
    always @(posedge clk)begin
        if(!rst_n)begin
            head0_valid<=0;head1_valid<=0;
            head0_data<=0;head1_data<=0;
            head0_sof<=0;head0_stamp<=0;head0_pixel<=0;head0_eof<=0;head0_abort<=0;
            head1_sof<=0;head1_stamp<=0;head1_pixel<=0;head1_eof<=0;head1_abort<=0;
        end else begin
            if(fifo0_pop)begin
                head0_data<=fifo0_data;head0_valid<=1;
                head0_sof<=(fifo0_data[35:32]==1);head0_stamp<=(fifo0_data[35:32]==2);
                head0_pixel<=(fifo0_data[35:32]==0);head0_eof<=(fifo0_data[35:32]==3);
                head0_abort<=(fifo0_data[35:32]==15);
            end
            else if(head0_take)head0_valid<=0;
            if(fifo1_pop)begin
                head1_data<=fifo1_data;head1_valid<=1;
                head1_sof<=(fifo1_data[35:32]==1);head1_stamp<=(fifo1_data[35:32]==2);
                head1_pixel<=(fifo1_data[35:32]==0);head1_eof<=(fifo1_data[35:32]==3);
                head1_abort<=(fifo1_data[35:32]==15);
            end
            else if(head1_take)head1_valid<=0;
            if(epoch!=current_epoch && !sending)begin
                head0_valid<=0;head1_valid<=0;
            end
        end
    end

    wire [35:0] selected_data=load_eye?head1_data:head0_data;
    wire selected_empty=load_eye?head1_empty:head0_empty;
    wire selected_pixel=load_eye?head1_pixel:head0_pixel;
    wire selected_eof=load_eye?head1_eof:head0_eof;
    wire selected_sof=load_eye?head1_sof:head0_sof;
    wire payload_available=(channel!=0 || !selected_empty);
    wire abort_head=pair_active && ((!head0_empty && head0_abort) ||
                                             (!head1_empty && head1_abort));
    wire bad_pixel=(load_state==L_PIXELS && channel==0 && !selected_empty && !selected_pixel);
    wire bad_eof=(load_state==L_EOF && !selected_empty && !selected_eof);
    wire waiting_word=pair_active && (((load_state==L_PIXELS && channel==0) || load_state==L_EOF) && selected_empty);
    wire waiting_meta=!pair_active && ((meta0!=0 || meta1!=0) && !(meta0==2 && meta1==2));
    wire timed_out=wait_count>=WAIT_LIMIT;
    wire fatal_pair=epoch_ok && (abort_head || bad_pixel || bad_eof || timed_out);
    wire both_meta=(meta0==2 && meta1==2 && !pair_active && load_state==L_IDLE);
    // Metadata is immutable once both source heads have supplied SOF/stamp.
    // Split 32-bit modular differences from the bounded-skew decision so a
    // subtract/absolute/compare chain cannot drive every chunk counter CE.
    reg [31:0] stamp_delta_q,sequence_delta_q;
    reg sequences_equal_q,sequence0_older_q,stamp0_earlier_q,stamp_skew_bad_q;
    reg [1:0] metadata_settle;
    always @(posedge clk)begin
        if(!rst_n)begin
            stamp_delta_q<=0;sequence_delta_q<=0;sequences_equal_q<=0;
            sequence0_older_q<=0;stamp0_earlier_q<=0;stamp_skew_bad_q<=0;metadata_settle<=0;
        end else begin
            stamp_delta_q<=stamp0-stamp1;sequence_delta_q<=seq0-seq1;
            sequences_equal_q<=seq0==seq1;sequence0_older_q<=sequence_delta_q[31];
            stamp0_earlier_q<=stamp_delta_q[31];
            stamp_skew_bad_q<=(stamp_delta_q>STAMP_LIMIT) &&
                              ((STAMP_LIMIT==0)||(stamp_delta_q<(32'd0-STAMP_LIMIT)));
            if(!both_meta)metadata_settle<=0;
            else if(metadata_settle!=3)metadata_settle<=metadata_settle+1'b1;
        end
    end
    wire metadata_bad=epoch_ok && !pair_active &&
        ((meta0==1 && !head0_empty && !head0_stamp && !head0_sof) ||
         (meta1==1 && !head1_empty && !head1_stamp && !head1_sof));

    always @* begin
        head0_take=0;head1_take=0;
        if(epoch_ok)begin
            if(!pair_active)begin
                if(meta0<2 && !head0_empty)head0_take=1;
                if(meta1<2 && !head1_empty)head1_take=1;
            end else begin
                if((load_state==L_PIXELS && channel==0) || load_state==L_EOF)begin
                    // Preserve an unexpected new SOF for the seek state.
                    if(!selected_empty && !selected_sof)begin
                        if(load_eye)head1_take=1;else head0_take=1;
                    end
                end
                if(abort_head)begin
                    head0_take=(!head0_empty && head0_abort);
                    head1_take=(!head1_empty && head1_abort);
                end
            end
        end
    end

    reg write_enable;reg [9:0] write_address;reg [7:0] write_data;
    wire [31:0] app_final=~app_crc;
    always @* begin
        write_enable=0;write_address=0;write_data=0;
        if(epoch_ok)begin
            case(load_state)
              L_HEADER:begin
                  write_enable=1;write_address={5'd0,header_index};
                  write_data=header_hold;
              end
              L_PIXELS:begin
                  write_address=10'd32+payload_index;
                  if(channel==0)begin write_enable=!selected_empty;write_data=selected_data[23:16];end
                  else begin write_enable=1;write_data=(channel==1)?rgb_hold[15:8]:rgb_hold[7:0];end
              end
              L_CRC:begin
                  write_enable=1;write_address=10'd28+{8'd0,crc_index};
                  case(crc_index)
                    0:write_data=app_final[31:24];1:write_data=app_final[23:16];
                    2:write_data=app_final[15:8];default:write_data=app_final[7:0];
                  endcase
              end
              default:begin end
            endcase
        end
        if(fatal_pair)write_enable=0;
    end
    // One byte pipeline retained to align RAM writes, CRC and packet tags.
    // No correction ROM or runtime image-calibration command remains.
    reg byte_valid; reg [7:0] byte_data;
    reg [9:0] byte_address;reg byte_bank,byte_crc_data;
    always @(posedge clk)begin
        if(!rst_n || !epoch_ok || fatal_pair)byte_valid<=0;
        else byte_valid<=write_enable;
        if(write_enable)begin
            byte_data<=write_data;byte_address<=write_address;byte_bank<=load_bank;
            byte_crc_data<=(load_state==L_HEADER || load_state==L_PIXELS);
        end
    end
    wire byte_commit=byte_valid && epoch_ok && !fatal_pair;
    wire read_enable=sending && send_position>=21 && send_position<=1012;
    wire [10:0] read_offset=send_position-11'd21;
    wire [9:0] read_address=read_offset[9:0];
    always @(posedge clk)begin
        if(read_enable)bank0_q<=bank0[read_address];
        if(byte_commit && !byte_bank)bank0[byte_address]<=byte_data;
    end
    always @(posedge clk)begin
        if(read_enable)bank1_q<=bank1[read_address];
        if(byte_commit && byte_bank)bank1[byte_address]<=byte_data;
    end
    wire [7:0] packet_byte=send_bank?bank1_q:bank0_q;
    reg [7:0] wire_byte;
    wire [31:0] ether_final=~ether_crc;
    always @* begin
        if(send_position<7)wire_byte=8'h55;
        else if(send_position==7)wire_byte=8'hd5;
        else if(send_position<22)wire_byte=ether_header(send_position[3:0]-4'd8);
        else if(send_position<=1013)wire_byte=packet_byte;
        else case(send_position)
            1014:wire_byte=ether_final[7:0];1015:wire_byte=ether_final[15:8];
            1016:wire_byte=ether_final[23:16];default:wire_byte=ether_final[31:24];
        endcase
    end
    assign tx_busy=sending || gap!=0;
    assign tx_pending=epoch_ok && ready!=0;
    wire selected_bank=ready[send_prefer]?send_prefer:!send_prefer;

    always @(posedge clk)begin
        if(!rst_n)begin
            current_epoch<=0;meta0<=0;meta1<=0;seq0<=0;seq1<=0;stamp0<=0;stamp1<=0;
            pair_seq<=0;pair_stamp<=0;pair_active<=0;done0<=0;done1<=0;next_eye<=0;
            chunk0<=0;chunk1<=0;wait_count<=0;ready<=0;load_state<=L_IDLE;
            load_bank<=0;load_eye<=0;load_chunk<=0;header_index<=0;header_hold<=0;payload_index<=0;channel<=0;
            crc_index<=0;rgb_hold<=0;app_crc<=32'hffffffff;
            sending<=0;send_bank<=0;send_prefer<=0;send_position<=0;gap<=0;ether_crc<=32'hffffffff;
            gmii_data<=0;gmii_valid<=0;pair_fault<=0;pair_fault_count<=0;packets_sent<=0;pairs_started<=0;
        end else begin
            gmii_valid<=0;pair_fault<=0;
            if(gap!=0)gap<=gap-1'b1;
            if(sending)begin
                gmii_valid<=1;gmii_data<=wire_byte;
                if(send_position>=8 && send_position<=1013)ether_crc<=crc_byte(ether_crc,wire_byte);
                if(send_position==1017)begin
                    sending<=0;gap<=12;packets_sent<=packets_sent+1'b1;
                end else send_position<=send_position+1'b1;
            end else if(gap==0 && tx_allow && tx_pending)begin
                send_bank<=selected_bank;ready[selected_bank]<=0;send_prefer<=!selected_bank;
                sending<=1;send_position<=0;ether_crc<=32'hffffffff;
            end

            if(epoch_ok)begin
                // CRC covers exactly the corrected bytes committed to RAM,
                // never the pre-Gamma source or its one-cycle-ahead tags.
                if(byte_commit && byte_crc_data)app_crc<=crc_byte(app_crc,byte_data);
                if(head0_take || head1_take || !(waiting_word || waiting_meta))wait_count<=0;
                else if(wait_count<WAIT_LIMIT)wait_count<=wait_increment;
                if(!pair_active)begin
                    if(meta0<2 && head0_valid)begin
                        if(head0_sof)begin seq0<=head0_data[31:0];meta0<=1;end
                        else if(meta0==1 && head0_stamp)begin stamp0<=head0_data[31:0];meta0<=2;end
                        else meta0<=0;
                    end
                    if(meta1<2 && head1_valid)begin
                        if(head1_sof)begin seq1<=head1_data[31:0];meta1<=1;end
                        else if(meta1==1 && head1_stamp)begin stamp1<=head1_data[31:0];meta1<=2;end
                        else meta1<=0;
                    end
                    if(both_meta && metadata_settle==3)begin
                        if(!sequences_equal_q)begin
                            if(sequence0_older_q)meta0<=0;else meta1<=0;
                            pair_fault<=1;pair_fault_count<=pair_fault_count+1'b1;
                        end else if(stamp_skew_bad_q)begin
                            meta0<=0;meta1<=0;pair_fault<=1;pair_fault_count<=pair_fault_count+1'b1;
                        end else begin
                            // Stamps have already passed the bounded skew check;
                            // signed modular ordering also works at uptime wrap.
                            pair_active<=1;pair_seq<=seq0;pair_stamp<=stamp0_earlier_q?stamp0:stamp1;
                            chunk0<=0;chunk1<=0;done0<=0;done1<=0;next_eye<=0;
                            wait_count<=0;pairs_started<=pairs_started+1'b1;
                        end
                    end
                end
                case(load_state)
                  L_IDLE:if(pair_active && (!ready[0] && !(sending && !send_bank) || !ready[1] && !(sending && send_bank)))begin
                      load_bank<=(!ready[0] && !(sending && !send_bank))?0:1;
                      load_eye<=(next_eye && !done1) || done0;
                      load_chunk<=((next_eye && !done1) || done0)?chunk1:chunk0;
                      header_index<=0;app_crc<=32'hffffffff;load_state<=L_HEADER_PRIME;
                  end
                  L_HEADER_PRIME:begin
                      header_hold<=header_byte(header_index,load_eye,load_chunk,pair_seq,pair_stamp,current_epoch);
                      load_state<=L_HEADER;
                  end
                  L_HEADER:begin
                      header_hold<=header_byte(header_index+1'b1,load_eye,load_chunk,pair_seq,pair_stamp,current_epoch);
                      if(header_index==27)begin load_state<=L_PIXELS;payload_index<=0;channel<=0;end
                      else header_index<=header_index+1'b1;
                  end
                  // Byte counters use availability, not the global fatal gate.
                  // A fatal event below discards all uncommitted banks and
                  // overrides load_state in this same edge. Updating private
                  // counters on that discarded edge cannot publish a packet,
                  // and keeps tag/error fanout out of the CRC/index CE cone.
                  L_PIXELS:if(payload_available)begin
                      payload_index<=payload_index+1'b1;
                      if(channel==0)begin rgb_hold<=selected_data[23:0];channel<=1;end
                      else if(channel==1)channel<=2;
                      else begin
                          channel<=0;
                          if(payload_index==959)begin
                              load_state<=L_PAYLOAD_DRAIN;
                          end
                      end
                  end
                  // Last payload byte retires through ROM / CRC this edge.
                  // Wait before reading ~app_crc to serialize its four bytes.
                  L_PAYLOAD_DRAIN:begin
                      if(load_chunk==LAST_CHUNK)load_state<=L_EOF;
                      else begin load_state<=L_CRC;crc_index<=0;end
                  end
                  L_EOF:if(!selected_empty && selected_eof)begin load_state<=L_CRC;crc_index<=0;end
                  L_CRC:begin
                      if(crc_index==3)begin
                          load_state<=L_COMMIT;
                      end else crc_index<=crc_index+1'b1;
                  end
                  // Publish RAM ownership only when the final CRC byte has
                  // reached the same bank through the synchronous ROM stage.
                  L_COMMIT:begin
                          ready[load_bank]<=1;load_state<=L_IDLE;next_eye<=!load_eye;
                          if(load_eye)begin chunk1<=chunk1+1'b1;if(load_chunk==LAST_CHUNK)done1<=1;end
                          else begin chunk0<=chunk0+1'b1;if(load_chunk==LAST_CHUNK)done0<=1;end
                          if(load_chunk==LAST_CHUNK && (load_eye?done0:done1))begin
                              pair_active<=0;meta0<=0;meta1<=0;
                          end
                  end
                  default:load_state<=L_IDLE;
                endcase
                if(metadata_bad)begin pair_fault<=1;pair_fault_count<=pair_fault_count+1'b1;end
                if(fatal_pair)begin
                    pair_fault<=1;pair_fault_count<=pair_fault_count+1'b1;
                    pair_active<=0;load_state<=L_IDLE;ready<=0;wait_count<=0;
                end
                // Pair metadata is no longer used once pair_seq/stamp have
                // been latched. Clear it from local state, not cross-eye
                // head/fatal decode; timeout still resets an incomplete seek.
                if(pair_active || timed_out)begin meta0<=0;meta1<=0;end
            end
            // Finish an already active Ethernet frame before clearing its RAM
            // ownership. New-epoch packing starts only after that boundary.
            if(epoch!=current_epoch && !sending)begin
                current_epoch<=epoch;pair_active<=0;meta0<=0;meta1<=0;
                load_state<=L_IDLE;ready<=0;wait_count<=0;done0<=0;done1<=0;send_prefer<=0;
            end
        end
    end
endmodule
