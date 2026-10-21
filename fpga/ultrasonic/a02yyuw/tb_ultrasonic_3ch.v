`timescale 1ns/1ps
module tb_ultrasonic_3ch;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0;
    reg [2:0] rx=3'b111;
    wire [47:0] distance;
    wire [2:0] valid, online, fault, obstacle, slow, stop_req;
    wire any_obstacle, any_slow, any_stop;
    wire [15:0] front_mm, left_mm, right_mm;
    wire forward_stop, forward_slow, left_blocked, right_blocked;
    ultrasonic_3ch #(.CLK_FREQ(1000000), .DATA_TIMEOUT_MS(20)) dut (
        .clk(clk),.rst_n(rst_n),.rx(rx),.distance_mm(distance),
        .distance_valid(valid),.online(online),.sensor_fault(fault),
        .obstacle(obstacle),.slow_down(slow),.stop_req(stop_req),
        .any_obstacle(any_obstacle),.any_slow_down(any_slow),.any_stop_req(any_stop),
        .front_distance_mm(front_mm),.left_distance_mm(left_mm),.right_distance_mm(right_mm),
        .forward_stop_req(forward_stop),.forward_slow_req(forward_slow),
        .left_blocked(left_blocked),.right_blocked(right_blocked)
    );
    wire f_obs, l_obs, r_obs, f_valid, l_valid, r_valid;
    wire [15:0] if_front, if_left, if_right;
    wire [2:0] if_updates, if_fault;
    wire if_stop, if_slow, if_left_blocked, if_right_blocked;
    a02yyuw_obstacle_if #(.CLK_FREQ(1000000), .DATA_TIMEOUT_MS(20)) motor_if (
        .clk(clk),.rst_n(rst_n),.us_rx_front(rx[0]),.us_rx_left(rx[1]),.us_rx_right(rx[2]),
        .front_obstacle(f_obs),.left_obstacle(l_obs),.right_obstacle(r_obs),
        .front_distance_mm(if_front),.left_distance_mm(if_left),.right_distance_mm(if_right),
        .front_data_valid(f_valid),.left_data_valid(l_valid),.right_data_valid(r_valid),
        .distance_updated(if_updates),.sensor_fault(if_fault),
        .forward_stop_req(if_stop),.forward_slow_req(if_slow),
        .left_blocked(if_left_blocked),.right_blocked(if_right_blocked)
    );
    always @(negedge clk) if(rst_n) begin
        if ({if_right,if_left,if_front} !== distance || if_updates !== valid || if_fault !== fault)
            $fatal(1,"motor interface channel mapping");
        if ({r_valid,l_valid,f_valid} !== online || {r_obs,l_obs,f_obs} !== (online & obstacle))
            $fatal(1,"motor interface confirmed obstacle/valid state");
        if ({if_stop,if_slow,if_left_blocked,if_right_blocked} !==
            {forward_stop,forward_slow,left_blocked,right_blocked})
            $fatal(1,"motor interface motion request mapping");
    end
    integer counts[0:2];
    integer k;
    always @(posedge clk) begin
        for(integer j=0;j<3;j=j+1) if(valid[j]) counts[j]=counts[j]+1;
    end
    task check;
        input condition;
        input [511:0] message;
        begin if(condition !== 1'b1) begin
            $display("FAIL: %0s",message); $fatal(1);
        end end
    endtask
    task automatic send_byte;
        input integer channel;
        input [7:0] data;
        integer bit_idx;
        begin
            @(negedge clk); rx[channel]=0;
            repeat(104) @(negedge clk);
            for(bit_idx=0;bit_idx<8;bit_idx=bit_idx+1) begin
                rx[channel]=data[bit_idx]; repeat(104) @(negedge clk);
            end
            rx[channel]=1; repeat(104) @(negedge clk);
        end
    endtask
    task automatic frame;
        input integer channel;
        input [15:0] mm;
        input corrupt;
        begin
            send_byte(channel,8'hff); send_byte(channel,mm[15:8]);
            send_byte(channel,mm[7:0]);
            send_byte(channel,(8'hff+mm[15:8]+mm[7:0]) ^ corrupt);
            repeat(10) @(negedge clk);
        end
    endtask

    // Also check the board top's entire debug stream, including channel labels.
    wire debug_tx;
    wire [7:0] led;
    top #(.CLK_FREQ(1000000)) board(.clk(clk),.us_rx(3'b111),.dbg_tx(debug_tx),.led(led));
    wire [7:0] debug_byte;
    wire debug_valid;
    a02_uart_rx #(.CLK_FREQ(1000000),.BAUD(115200)) monitor (
        .clk(clk),.rst_n(rst_n),.rx(debug_tx),.data(debug_byte),.valid(debug_valid)
    );
    integer debug_count=0;
    reg [7:0] expected;
    always @(posedge clk) if(debug_valid && debug_count<30) begin
        case(debug_count%10)
            0: case(debug_count/10)
                0: expected="F";
                1: expected="L";
                default: expected="R";
            endcase
            1: expected=":";
            2,3,4,5: expected="0";
            6: expected=" ";
            7: expected="F";
            8: expected=8'h0d;
            default: expected=8'h0a;
        endcase
        if(debug_byte !== expected) $fatal(1,"debug byte %0d: got %h expected %h",debug_count,debug_byte,expected);
        debug_count=debug_count+1;
    end
    initial begin
        for(k=0;k<3;k=k+1) counts[k]=0;
        repeat(5) @(negedge clk); rst_n=1;
        repeat(5) @(negedge clk);
        check(fault==3'b111 && any_stop,"all channels fail safe at startup");
        fork frame(0,1953,0); frame(1,600,0); frame(2,200,0); join
        check(distance=={16'd200,16'd600,16'd1953},"three simultaneous distinct distances");
        check(online==7 && stop_req==4 && slow==6 && obstacle==6,"independent flags and aggregate");
        check(any_stop && any_slow && any_obstacle,"aggregate requests");
        check(front_mm==1953 && left_mm==600 && right_mm==200,"front/left/right distance mapping");
        check(!forward_stop && !forward_slow && !left_blocked && right_blocked,
              "right proximity isolated from forward request");
        check(!f_obs && l_obs && r_obs && if_front==1953 && if_left==600 && if_right==200,
              "motor interface obstacle direction and mm distance");
        fork frame(0,1400,0); frame(1,1500,1); frame(2,1600,0); join
        check(distance=={16'd1600,16'd600,16'd1400},"bad checksum isolated to channel1");
        check(counts[0]==2 && counts[1]==1 && counts[2]==2,"no dropped simultaneous valid pulses");
        // Only channel1 disconnects. Keep 0 and 2 fresh past channel1 timeout.
        repeat(12000) @(negedge clk);
        fork frame(0,1500,0); frame(2,1700,0); join
        check(online==3'b101 && fault==3'b010 && stop_req==3'b010 && any_stop,"one disconnect isolated; aggregate stops");
        check(forward_stop && forward_slow && left_blocked,"left fault inhibits forward motion");
        check(!l_valid && !l_obs && if_fault[1] && if_stop,
              "invalid left is a fault rather than a confirmed detection");
        frame(1,1800,0);
        check(online==7 && !any_stop && !any_slow && !any_obstacle,"reconnect clears aggregate");
        // Partial packet on one channel cannot block complete packets on the others.
        fork
            begin send_byte(1,8'hff); send_byte(1,8'h01); end
            frame(0,30,0);
            frame(2,4500,0);
        join
        check(distance[15:0]==30 && distance[47:32]==4500 && distance[31:16]==1800,"partial packet isolation");
        repeat(22000) @(negedge clk);
        frame(1,1300,0);
        check(distance[31:16]==1300 && online[1],"partial packet timeout recovery");
        fork frame(0,200,0); frame(1,1500,0); frame(2,1600,0); join
        check(forward_stop && forward_slow && !left_blocked && !right_blocked,
              "front obstacle requests stop with both sides clear");
        fork frame(0,600,0); frame(1,200,0); frame(2,1600,0); join
        check(!forward_stop && forward_slow && left_blocked && !right_blocked,
              "front slow and left blocked reported independently");
        fork frame(0,1500,0); frame(1,200,0); frame(2,1600,0); join
        check(!forward_stop && !forward_slow && left_blocked && !right_blocked,
              "left proximity alone is not a forward stop");
        wait(debug_count==30);
        check(led[7] && led[6],"board disconnected fault LEDs");
        $display("PASS: three concurrent drivers, per-channel failure isolation, aggregate, recovery, top debug and motor-facing obstacle interface");
        $finish;
    end
    initial begin #10000000; $fatal(1,"test watchdog"); end
endmodule
