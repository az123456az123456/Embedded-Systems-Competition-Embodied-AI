// ============================================================
// crc16_byte.v — CRC-16/CCITT-FALSE 单字节组合更新
// 多项式 0x1021，初值 0xFFFF，不反射，无最终异或
// ============================================================
module crc16_byte (
    input  wire [15:0] crc_in,
    input  wire [7:0]  data,
    output reg  [15:0] crc_out
);
    integer i;

    always @(*) begin
        crc_out = crc_in ^ {data, 8'h00};
        for (i = 0; i < 8; i = i + 1) begin
            if (crc_out[15])
                crc_out = {crc_out[14:0], 1'b0} ^ 16'h1021;
            else
                crc_out = {crc_out[14:0], 1'b0};
        end
    end
endmodule
