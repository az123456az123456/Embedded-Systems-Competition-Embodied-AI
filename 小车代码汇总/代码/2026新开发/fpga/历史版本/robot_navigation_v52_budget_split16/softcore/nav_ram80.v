`timescale 1ns/1ps
// 80 KiB CPU memory: preserve the proven 64 KiB bit-plane inference, then add
// eight 4096 x 4-bit tail planes. No read-before-write and no index truncation.
module nav_ram80 #(
    parameter LOW_HEX="softcore/firmware/firmware_low.hex",
    parameter TAIL_PREFIX="softcore/firmware/firmware_tail"
)(
    input wire clk,request,
    input wire [31:0] address,wdata,
    input wire [3:0] wstrb,
    output wire [31:0] rdata,
    output wire address_valid
);
    wire low_select=(address[31:16]==16'd0);
    wire tail_select=(address[31:14]==18'd4);
    assign address_valid=low_select || tail_select;
    wire read_request=request && address_valid && wstrb==0;

    // Keep the low-bank source structure identical to the reviewed 64 KiB RAM.
    (* syn_ramstyle="block_ram" *) reg [31:0] low [0:16383];
    initial $readmemh(LOW_HEX,low);
    reg [31:0] low_q;
    reg read_tail=0;
    always @(posedge clk)begin
        if(read_request)read_tail<=tail_select;
        if(request && low_select)begin
            if(wstrb==0)low_q<=low[address[15:2]];
            if(wstrb[0])low[address[15:2]][7:0]<=wdata[7:0];
            if(wstrb[1])low[address[15:2]][15:8]<=wdata[15:8];
            if(wstrb[2])low[address[15:2]][23:16]<=wdata[23:16];
            if(wstrb[3])low[address[15:2]][31:24]<=wdata[31:24];
        end
    end

    wire [31:0] tail_q;
    genvar lane,half;
    generate for(lane=0;lane<4;lane=lane+1)begin:g_tail_lane
        for(half=0;half<2;half=half+1)begin:g_half
            localparam [7:0] LANE_CHAR=8'h30+lane;
            localparam INIT_FILE={TAIL_PREFIX,"_lane",LANE_CHAR,
                                  half==0 ? "_lo.hex" : "_hi.hex"};
            (* syn_ramstyle="block_ram" *) reg [3:0] ram [0:4095];
            initial $readmemh(INIT_FILE,ram);
            reg [3:0] q;
            always @(posedge clk)begin
                if(request && tail_select)begin
                    if(wstrb==0)q<=ram[address[13:2]];
                    if(wstrb[lane])ram[address[13:2]]<=wdata[lane*8+half*4 +:4];
                end
            end
            assign tail_q[lane*8+half*4 +:4]=q;
        end
    end endgenerate
    // The selected bank belongs to the READ request, never to a live bus address.
    // A store (including a masked store to the other bank) holds the whole rdata.
    assign rdata=read_tail ? tail_q : low_q;
endmodule
