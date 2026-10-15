`timescale 1ns/1ps
// ACG138, 50MHz. Three A02YYUW automatic UART sensors, 9600 8N1.
// RX0=P2.1/Y18, RX1=P2.3/N20, RX2=P2.5/U20 (FMCtoGPIO).
// Channel 0=FRONT, 1=LEFT, 2=RIGHT.
// Sensor RX mode-select pins float; VCC/GND are board power nets, not HDL ports.
module top #(
    parameter integer CLK_FREQ = 50_000_000
)(
    input wire clk,
    input wire [2:0] us_rx,
    output wire dbg_tx,
    output wire [7:0] led
);
    reg [15:0] por_cnt = 16'hffff;
    always @(posedge clk)
        if (por_cnt != 0) por_cnt <= por_cnt - 1'b1;
    wire rst_n = (por_cnt == 0);
    reg [31:0] hb_cnt = 0;
    reg heartbeat = 0;
    always @(posedge clk) begin
        if (hb_cnt == CLK_FREQ/2 - 1) begin
            hb_cnt <= 0; heartbeat <= ~heartbeat;
        end else hb_cnt <= hb_cnt + 1'b1;
    end

    wire [47:0] distance;
    wire [2:0] dist_valid, online, sensor_fault, obstacle, slow_down, stop_req;
    wire any_obstacle, any_slow_down, any_stop_req;
    wire [15:0] front_distance_mm, left_distance_mm, right_distance_mm;
    wire forward_slow_req, forward_stop_req, left_blocked, right_blocked;
    ultrasonic_3ch #(.CLK_FREQ(CLK_FREQ)) u_ultrasonic (
        .clk(clk), .rst_n(rst_n), .rx(us_rx), .distance_mm(distance),
        .distance_valid(dist_valid), .online(online), .sensor_fault(sensor_fault),
        .obstacle(obstacle), .slow_down(slow_down), .stop_req(stop_req),
        .any_obstacle(any_obstacle), .any_slow_down(any_slow_down),
        .any_stop_req(any_stop_req),
        .front_distance_mm(front_distance_mm), .left_distance_mm(left_distance_mm),
        .right_distance_mm(right_distance_mm), .forward_slow_req(forward_slow_req),
        .forward_stop_req(forward_stop_req), .left_blocked(left_blocked),
        .right_blocked(right_blocked)
    );

    // Every 100ms latch all channels atomically and print all three.
    // Line: "F:07A1 0\r\n". Status hex: bit3 fault, bit2 stop, bit1 slow, bit0 obstacle.
    // Periodic snapshots report disconnected sensors too; simultaneous updates cannot
    // compete for the debug transmitter. Control outputs remain live and independent.
    reg [31:0] report_cnt;
    reg [47:0] snapshot;
    reg [11:0] status_snapshot;
    reg [1:0] channel_idx;
    reg [3:0] char_idx;
    reg printing, tx_send;
    reg [7:0] tx_byte;
    wire tx_busy;
    wire [15:0] current_distance = snapshot[channel_idx*16 +: 16];
    wire [3:0] current_status = status_snapshot[channel_idx*4 +: 4];
    function [7:0] hex2ascii;
        input [3:0] v;
        begin hex2ascii = (v < 10) ? 8'h30 + {4'b0,v} : 8'h37 + {4'b0,v}; end
    endfunction
    a02_uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(115200)) u_uart_tx (
        .clk(clk), .rst_n(rst_n), .send(tx_send), .data(tx_byte),
        .tx(dbg_tx), .busy(tx_busy)
    );
    always @(posedge clk) begin
        if (!rst_n) begin
            report_cnt <= 0; snapshot <= 0; status_snapshot <= 0;
            channel_idx <= 0; char_idx <= 0; printing <= 0;
            tx_send <= 0; tx_byte <= 0;
        end else begin
            tx_send <= 0;
            if (report_cnt < CLK_FREQ/10 - 1) report_cnt <= report_cnt + 1'b1;
            if (!printing) begin
                if (report_cnt >= CLK_FREQ/10 - 1) begin
                    report_cnt <= 0; snapshot <= distance;
                    status_snapshot <= {
                        sensor_fault[2],stop_req[2],slow_down[2],obstacle[2],
                        sensor_fault[1],stop_req[1],slow_down[1],obstacle[1],
                        sensor_fault[0],stop_req[0],slow_down[0],obstacle[0]};
                    channel_idx <= 0; char_idx <= 0; printing <= 1;
                end
            end else if (!tx_busy && !tx_send) begin
                case (char_idx)
                    0: case (channel_idx)
                        0: tx_byte <= "F"; // front
                        1: tx_byte <= "L"; // left
                        default: tx_byte <= "R"; // right
                    endcase
                    1: tx_byte <= ":";
                    2: tx_byte <= hex2ascii(current_distance[15:12]);
                    3: tx_byte <= hex2ascii(current_distance[11:8]);
                    4: tx_byte <= hex2ascii(current_distance[7:4]);
                    5: tx_byte <= hex2ascii(current_distance[3:0]);
                    6: tx_byte <= " ";
                    7: tx_byte <= hex2ascii(current_status);
                    8: tx_byte <= 8'h0d;
                    default: tx_byte <= 8'h0a;
                endcase
                tx_send <= 1;
                if (char_idx == 9) begin
                    char_idx <= 0;
                    if (channel_idx == 2) printing <= 0;
                    else channel_idx <= channel_idx + 1'b1;
                end else char_idx <= char_idx + 1'b1;
            end
        end
    end
    assign led = {|sensor_fault, any_stop_req, any_slow_down, any_obstacle,
                  online[2], online[1], online[0], heartbeat};
endmodule

