// ============================================================
// protocol_rx.v — FPGA↔RK3588 协议 v2 接收帧解析
// 帧: AA 55 | CMD | SEQ | LEN | PAYLOAD | CRC_L CRC_H
// payload 最大 32 字节，CRC-16/CCITT-FALSE
// ============================================================
module protocol_rx #(
    parameter MAX_PAYLOAD = 32
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         rx_valid,
    input  wire [7:0]   rx_data,
    output reg  [7:0]   o_cmd,
    output reg  [7:0]   o_seq,
    output reg  [7:0]   o_len,
    output reg  [255:0] o_payload,
    output reg          o_frame_valid,
    output reg          o_frame_error
);
    localparam S_WAIT_AA = 3'd0;
    localparam S_WAIT_55 = 3'd1;
    localparam S_CMD     = 3'd2;
    localparam S_SEQ     = 3'd3;
    localparam S_LEN     = 3'd4;
    localparam S_PAYLOAD = 3'd5;
    localparam S_CRC_L   = 3'd6;
    localparam S_CRC_H   = 3'd7;

    reg [2:0]  state;
    reg [7:0]  idx;
    reg [7:0]  crc_low;
    reg [15:0] crc_reg;
    wire [15:0] crc_next;

    crc16_byte u_crc (
        .crc_in(crc_reg),
        .data(rx_data),
        .crc_out(crc_next)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_WAIT_AA;
            idx           <= 8'd0;
            crc_low       <= 8'd0;
            crc_reg       <= 16'hFFFF;
            o_cmd         <= 8'd0;
            o_seq         <= 8'd0;
            o_len         <= 8'd0;
            o_payload     <= 256'd0;
            o_frame_valid <= 1'b0;
            o_frame_error <= 1'b0;
        end else begin
            o_frame_valid <= 1'b0;
            o_frame_error <= 1'b0;

            if (rx_valid) begin
                case (state)
                    S_WAIT_AA: begin
                        if (rx_data == 8'hAA) begin
                            crc_reg   <= 16'hFFFF;
                            idx       <= 8'd0;
                            o_payload <= 256'd0;
                            state <= S_WAIT_55;
                        end
                    end
                    S_WAIT_55: begin
                        if (rx_data == 8'h55)
                            state <= S_CMD;
                        else if (rx_data != 8'hAA)
                            state <= S_WAIT_AA;
                    end
                    S_CMD: begin
                        o_cmd  <= rx_data;
                        crc_reg <= crc_next;
                        state  <= S_SEQ;
                    end
                    S_SEQ: begin
                        o_seq  <= rx_data;
                        crc_reg <= crc_next;
                        state  <= S_LEN;
                    end
                    S_LEN: begin
                        if (rx_data > MAX_PAYLOAD) begin
                            o_frame_error <= 1'b1;
                            state <= S_WAIT_AA;
                        end else begin
                            o_len     <= rx_data;
                            o_payload <= 256'd0;
                            crc_reg   <= crc_next;
                            idx       <= 8'd0;
                            if (rx_data == 8'd0)
                                state <= S_CRC_L;
                            else
                                state <= S_PAYLOAD;
                        end
                    end
                    S_PAYLOAD: begin
                        o_payload[idx*8 +: 8] <= rx_data;
                        crc_reg <= crc_next;
                        if (idx == o_len - 1)
                            state <= S_CRC_L;
                        else
                            idx <= idx + 8'd1;
                    end
                    S_CRC_L: begin
                        crc_low <= rx_data;
                        state   <= S_CRC_H;
                    end
                    S_CRC_H: begin
                        if ({rx_data, crc_low} == crc_reg)
                            o_frame_valid <= 1'b1;
                        else
                            o_frame_error <= 1'b1;
                        state <= S_WAIT_AA;
                    end
                    default: state <= S_WAIT_AA;
                endcase
            end
        end
    end
endmodule
