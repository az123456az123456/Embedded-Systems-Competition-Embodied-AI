// SIMULATION / FORMAL ONLY. Do not synthesize together with Gowin built-ins.
// Cascaded conditional muxes match official MUX16 four-state merging.
// An index with X/Z would be overly pessimistic and is deliberately not used.
module LUT4 #(parameter [15:0] INIT=16'h0000)(
 input wire I0,I1,I2,I3,output wire F
);
 wire [7:0] m0;
 wire [3:0] m1;
 wire [1:0] m2;
 genvar i;
 generate
  for(i=0;i<8;i=i+1)begin: l0
   assign m0[i]=I0?INIT[2*i+1]:INIT[2*i];
  end
  for(i=0;i<4;i=i+1)begin: l1
   assign m1[i]=I1?m0[2*i+1]:m0[2*i];
  end
  for(i=0;i<2;i=i+1)begin: l2
   assign m2[i]=I2?m1[2*i+1]:m1[2*i];
  end
 endgenerate
 assign F=I3?m2[1]:m2[0];
endmodule
