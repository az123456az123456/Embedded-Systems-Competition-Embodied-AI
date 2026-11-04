// Exact unsigned bit-vector equality. Isolated proposal, no owner integration.
// WIDTH is not truncated. No clock, state, latency, carry chain, RAM or DSP.
// Synthesis: Gowin native LUT4 primitive; do not add the simulation model.
module nav_eq_lut #(parameter integer WIDTH=64)(
 input wire [WIDTH-1:0] a,b,
 output wire eq,ne
);
 localparam integer PAIRS=(WIDTH+1)/2;
 wire [PAIRS-1:0] pair_equal;
 genvar i;
 generate for(i=0;i<PAIRS;i=i+1)begin: pair
  if(2*i+1<WIDTH)begin: full_pair
   LUT4 #(.INIT(16'h9009)) same(.I0(a[2*i]),.I1(b[2*i]),
    .I2(a[2*i+1]),.I3(b[2*i+1]),.F(pair_equal[i]));
  end else begin: last_bit
   LUT4 #(.INIT(16'h9009)) same(.I0(a[2*i]),.I1(b[2*i]),
    .I2(1'b0),.I3(1'b0),.F(pair_equal[i]));
  end
 end endgenerate
 nav_eq_reduce_lut4 #(.N(PAIRS)) reduce_equal(.v(pair_equal),.q(eq));
 assign ne=~eq;
endmodule
module nav_eq_reduce_lut4 #(parameter integer N=32)(
 input wire [N-1:0] v, output wire q
);
 generate if(N==1)begin: last
  assign q=v[0];
 end else begin: level
  localparam integer M=(N+3)/4;
  wire [4*M-1:0] padded;
  wire [M-1:0] partial;
  assign padded={{(4*M-N){1'b1}},v};
  genvar i;
  for(i=0;i<M;i=i+1)begin: four
   LUT4 #(.INIT(16'h8000)) all_equal(.I0(padded[4*i]),
    .I1(padded[4*i+1]),.I2(padded[4*i+2]),.I3(padded[4*i+3]),.F(partial[i]));
  end
  nav_eq_reduce_lut4 #(.N(M)) next(.v(partial),.q(q));
 end endgenerate
endmodule
