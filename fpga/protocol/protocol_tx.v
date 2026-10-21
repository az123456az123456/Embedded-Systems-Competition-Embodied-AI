// ============================================================
// protocol_tx.v — FPGA↔RK3588 协议 v2 发送器
// 输入一帧命令/数据，自动附加长度和 CRC16 后交给 uart_tx。
// ============================================================
module protocol_tx (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    input  wire [7:0]   cmd,
    input  wire [7:0]   seq,
    input  wire [7:0]   len,
    input  wire [255:0] payload,
    output reg          tx_start,
    output reg  [7:0]   tx_data,
    input  wire         tx_busy,
    output reg          busy,
    output reg          done
);
    localparam S_IDLE      = 2'd0;
    localparam S_SEND      = 2'd1;
    localparam S_WAIT_BUSY = 2'd2;
    localparam S_WAIT_DONE = 2'd3;

    reg [1:0] state;
    reg [7:0] cmd_r, seq_r, len_r;
    reg [255:0] payload_r;
    reg [7:0] idx;
    reg [15:0] crc_reg;
    reg [7:0]  cur_byte;
    wire [15:0] crc_next;

    crc16_byte u_crc (
        .crc_in(crc_reg),
        .data(cur_byte),
        .crc_out(crc_next)
    );

    always @(*) begin
        if (idx == 8'd0)
            cur_byte = 8'hAA;
        else if (idx == 8'd1)
            cur_byte = 8'h55;
        else if (idx == 8'd2)
            cur_byte = cmd_r;
        else if (idx == 8'd3)
            cur_byte = seq_r;
        else if (idx == 8'd4)
            cur_byte = len_r;
        else if (idx < (8'd5 + len_r))
            cur_byte = payload_r[(idx - 8'd5)*8 +: 8];
        else if (idx == (8'd5 + len_r))
            cur_byte = crc_reg[7:0];
        else
            cur_byte = crc_reg[15:8];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            tx_start <= 1'b0;
            tx_data  <= 8'd0;
            busy     <= 1'b0;
            done     <= 1'b0;
            idx      <= 8'd0;
            crc_reg  <= 16'hFFFF;
            cmd_r    <= 8'd0;
            seq_r    <= 8'd0;
            len_r    <= 8'd0;
            payload_r <= 256'd0;
        end else begin
            tx_start <= 1'b0;
            done     <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start && !busy) begin
                        cmd_r     <= cmd;
                        seq_r     <= seq;
                        len_r     <= len;
                        payload_r <= payload;
                        idx       <= 8'd0;
                        crc_reg   <= 16'hFFFF;
                        busy      <= 1'b1;
                        state     <= S_SEND;
                    end
                end
                S_SEND: begin
                    tx_data  <= cur_byte;
                    tx_start <= 1'b1;
                    if (idx >= 8'd2 && idx < (8'd5 + len_r))
                        crc_reg <= crc_next;
                    state <= S_WAIT_BUSY;
                end
                S_WAIT_BUSY: begin
                    if (tx_busy)
                        state <= S_WAIT_DONE;
                end
                S_WAIT_DONE: begin
                    if (!tx_busy) begin
                        if (idx >= (8'd6 + len_r)) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            idx   <= idx + 8'd1;
                            state <= S_SEND;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
