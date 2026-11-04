`timescale 1ns/1ps
// Single-master I2C register access. Both wires are open drain, with external
// pull-ups. Burst reads use repeated START and NACK on the final byte.
module i2c_byte_engine #(
    parameter integer CLK_HZ=50000000, I2C_HZ=100000,
    parameter integer TIMEOUT_CYCLES=CLK_HZ/100
)(
    input wire clk, rst_n, start,
    input wire [1:0] op, // 0 START, 1 WRITE, 2 READ, 3 STOP
    input wire [7:0] tx,
    input wire nack,
    output reg busy, done, error, ack,
    output reg [7:0] rx,
    inout wire scl, sda
);
    localparam integer QUARTER=CLK_HZ/I2C_HZ/4;
    reg scl_low, sda_low;
    assign scl=scl_low ? 1'b0 : 1'bz;
    assign sda=sda_low ? 1'b0 : 1'bz;
    (* syn_async_reg=1 *) reg [1:0] scl_sync, sda_sync;
    reg [31:0] divider, watchdog;
    reg [2:0] phase;
    reg [3:0] bit_index;
    reg [1:0] command;
    reg [7:0] tx_latched, shift;
    reg nack_latched;
    always @(posedge clk) begin
        scl_sync <= {scl_sync[0],scl};
        sda_sync <= {sda_sync[0],sda};
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_low<=0; sda_low<=0; busy<=0; done<=0; error<=0; ack<=0;
            rx<=0; divider<=0; watchdog<=0; phase<=0; bit_index<=0;
            command<=0; tx_latched<=0; shift<=0; nack_latched<=1;
        end else begin
            done<=0;
            if (!busy) begin
                divider<=0; watchdog<=0;
                if (start) begin
                    busy<=1; error<=0; ack<=0; phase<=0; bit_index<=0;
                    command<=op; tx_latched<=tx; nack_latched<=nack; shift<=0;
                end
            end else if (watchdog>=TIMEOUT_CYCLES-1) begin
                // Release the bus after a missing pull-up / stuck SCL.
                busy<=0; done<=1; error<=1; scl_low<=0; sda_low<=0;
            end else begin
                watchdog<=watchdog+1'b1;
                if (divider<QUARTER-1) divider<=divider+1'b1;
                else begin
                    divider<=0;
                    case (command)
                        0: case (phase)
                            0: begin sda_low<=0; phase<=1; end
                            1: begin scl_low<=0; phase<=2; end
                            2: if (scl_sync[1] && sda_sync[1]) begin
                                sda_low<=1; phase<=3;
                            end
                            3: begin scl_low<=1; phase<=4; end
                            4: begin busy<=0; done<=1; end
                        endcase
                        3: case (phase)
                            0: begin scl_low<=1; phase<=1; end
                            1: begin sda_low<=1; phase<=2; end
                            2: begin scl_low<=0; phase<=3; end
                            3: if (scl_sync[1]) begin sda_low<=0; phase<=4; end
                            4: begin busy<=0; done<=1; end
                        endcase
                        default: case (phase)
                            0: begin scl_low<=1; phase<=1; end
                            1: begin
                                if (command==1)
                                    sda_low <= (bit_index<8) ? !tx_latched[7-bit_index] : 1'b0;
                                else
                                    sda_low <= (bit_index==8) ? !nack_latched : 1'b0;
                                phase<=2;
                            end
                            2: begin scl_low<=0; phase<=3; end
                            3: if (scl_sync[1]) begin
                                if (command==2 && bit_index<8)
                                    shift<={shift[6:0],sda_sync[1]};
                                if (command==1 && bit_index==8) ack<=!sda_sync[1];
                                phase<=4;
                            end
                            4: begin
                                scl_low<=1;
                                if (bit_index==8) phase<=5;
                                else begin bit_index<=bit_index+1'b1; phase<=1; end
                            end
                            5: begin
                                sda_low<=0; rx<=shift; busy<=0; done<=1;
                            end
                        endcase
                    endcase
                end
            end
        end
    end
endmodule

module i2c_reg_master #(
    parameter integer CLK_HZ=50000000, I2C_HZ=100000
)(
    input wire clk, rst_n, start, read,
    input wire [6:0] address,
    input wire [7:0] register_address, write_data,
    input wire [4:0] read_length, // 1..16 bytes
    output reg busy, done, error,
    output reg [127:0] read_data, // First byte highest; last byte [7:0].
    inout wire scl, sda
);
    reg [6:0] addr;
    reg [7:0] reg_addr, wr;
    reg rd, issued;
    reg [4:0] length, count;
    reg [2:0] step;
    reg command_start;
    reg [1:0] command;
    reg [7:0] command_data;
    reg command_nack;
    wire engine_busy, engine_done, engine_error, engine_ack;
    wire [7:0] engine_data;
    i2c_byte_engine #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ)) engine (
        .clk(clk),.rst_n(rst_n),.start(command_start),.op(command),
        .tx(command_data),.nack(command_nack),.busy(engine_busy),
        .done(engine_done),.error(engine_error),.ack(engine_ack),
        .rx(engine_data),.scl(scl),.sda(sda)
    );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy<=0; done<=0; error<=0; read_data<=0; issued<=0; step<=0;
            command_start<=0; command<=0; command_data<=0; command_nack<=1;
            addr<=0; reg_addr<=0; wr<=0; rd<=0; length<=0; count<=0;
        end else begin
            done<=0; command_start<=0;
            if (!busy) begin
                if (start) begin
                    addr<=address; reg_addr<=register_address; wr<=write_data;
                    rd<=read; length<=read_length; count<=0; read_data<=0;
                    error<=0; step<=0; issued<=0;
                    if (read && (read_length==0 || read_length>16)) begin
                        done<=1; error<=1;
                    end else busy<=1;
                end
            end else if (!issued && !engine_busy) begin
                command_start<=1; issued<=1; command_nack<=1;
                case (step)
                    0: command<=0;
                    1: begin command<=1; command_data<={addr,1'b0}; end
                    2: begin command<=1; command_data<=reg_addr; end
                    3: if (rd) command<=0;
                       else begin command<=1; command_data<=wr; end
                    4: if (rd) begin command<=1; command_data<={addr,1'b1}; end
                       else command<=3;
                    5: begin command<=2; command_nack<=(count==length-1); end
                    default: command<=3;
                endcase
            end else if (issued && engine_done) begin
                issued<=0;
                if (step==6 || (step==4 && !rd)) begin
                    error<=error || engine_error; busy<=0; done<=1;
                end else if (engine_error || (command==1 && !engine_ack)) begin
                    error<=1; step<=6; // Always try STOP after an error.
                end else if (step==5) begin
                    read_data<={read_data[119:0],engine_data};
                    if (count==length-1) step<=6;
                    else count<=count+1'b1;
                end else step<=step+1'b1;
            end
        end
    end
endmodule
