`timescale 1ns/1ps
module tb_layered_mmio_v28;
 reg clk=0,rst_n=0,iv=0;always #5 clk=~clk;
 reg [511:0] ip={16{32'h11223344}};
 reg [4:0] flags=5'b00111;reg [31:0] epoch=9,cseq=77,motion={16'd100,16'd300};
 reg [127:0] counts={32'd4,32'd3,32'd2,32'd1};reg [3:0] illegal=4;
 reg [47:0] sonar={16'd3000,16'd2000,16'd1000};reg [95:0] seqs={32'd33,32'd22,32'd11},stamps={32'd30,32'd20,32'd10};reg [2:0] online=7;
 wire [63:0] pwm;wire ready;wire [47:0] filtered;wire [95:0] fseq;wire [2:0] fvalid;wire trap;wire [127:0] diag;
 robot_nav_soc #(.CLK_HZ(100000)) dut(.clk(clk),.reset_n(rst_n),.imu_valid(iv),.imu_packet(ip),.motor_valid(1'b0),.motor_packet(128'd0),
 .sensor_valid(1'b0),.sensor_packet(512'd0),.nav_request_valid(1'b0),.nav_request_packet(512'd0),.nav_request_ready(),.nav_reply_valid(),.nav_reply_packet(),.nav_reply_ready(1'b1),.nav_status(),.cpu_trap(trap),
 .control_flags(flags),.control_epoch(epoch),.control_sequence(cseq),.control_motion(motion),.control_masks(8'hfa),.encoder_positions(counts),.encoder_illegal(illegal),
 .sonar_mm(sonar),.sonar_seq(seqs),.sonar_stamp(stamps),.sonar_online(online),.control_pwm(pwm),.control_ready(ready),.filter_mm(filtered),.filter_seq(fseq),.filter_valid(fvalid),.control_diagnostics(diag));
 reg req=0;reg [31:0] addr=0,wd=0;reg [3:0] ws=0;
 task tx(input [31:0] offset,input [31:0] w,input [3:0] strobes,output [31:0] value);
 begin @(negedge clk);addr=32'h10000000+offset;wd=w;ws=strobes;req=1;
 @(negedge clk);if(!dut.mem_ready)$fatal(1,"missingMMIOack");value=dut.mem_rdata;req=0;@(negedge clk);end endtask
 task wr(input [31:0] offset,input [31:0] w);reg [31:0] ignored;begin tx(offset,w,15,ignored);end endtask
 task rd(input [31:0] offset,input [31:0] expectval);reg [31:0] r;begin tx(offset,0,0,r);if(r!==expectval)$fatal(1,"MMIO%h expected%h got%h",offset,expectval,r);end endtask
 integer i;reg [31:0] commit_at,old_stamp,r;
 initial begin
 force dut.mem_valid=req;force dut.mem_instr=1'b0;force dut.mem_addr=addr;force dut.mem_wstrb=ws;force dut.mem_wdata=wd;force dut.core_trap=1'b0;
 repeat(3)@(negedge clk);rst_n=1;@(negedge clk);iv=1;@(negedge clk);iv=0;
 wr('h200,1);counts=128'hffffffff;sonar=0;seqs=0;stamps=0;flags=0;ip=0;iv=1;@(negedge clk);iv=0;
 rd('h208,9);rd('h20c,7);rd('h210,77);rd('h214,{16'd100,16'd300});rd('h218,'hfa);
 for(i=0;i<4;i=i+1)rd('h220+i*4,i+1);rd('h230,4);rd('h240,'h11223344);rd('h27c,'h11223344);
 rd('h280,{16'd11,16'd1000});rd('h284,{16'd22,16'd2000});rd('h288,{16'd33,16'd3000});
 rd('h290,10);rd('h294,20);rd('h298,30);rd('h2a0,7);rd('h2a4,11);rd('h2a8,22);rd('h2ac,33);
 wr('h300,100);wr('h304,-200);wr('h308,300);wr('h30c,-400);wr('h310,990);wr('h314,1990);wr('h318,2990);
 wr('h31c,'h703);wr('h320,11);wr('h324,22);wr('h328,33);wr('h330,-90000);wr('h334,-100);wr('h338,100);wr('h33c,5);
 if(pwm!=0 || ready)$fatal(1,"partialshadowwriteescaped");wr('h32c,1);commit_at=dut.commit_ms;
 if(!ready || pwm!=={-16'sd400,16'd300,-16'sd200,16'd100} || filtered!=={16'd2990,16'd1990,16'd990} || fseq!=={32'd33,32'd22,32'd11} || fvalid!=7)$fatal(1,"atomiccommitbad");
 if(diag!=={32'd5,32'd100,-32'sd100,-32'sd90000})$fatal(1,"diagnostics atomic bad");
 // Encoder snapshot consumes absolute local counts, independently of SNS1 cadence.
 wr('h200,1);rd('h220,32'hffffffff);rd('h240,0);
 wr('h300,555);if(pwm[15:0]!=100)$fatal(1,"uncommittedoutput changed");
 while(dut.uptime_ms-commit_at<59)@(negedge clk);if(!ready)$fatal(1,"early watchdogexpiry");
 wr('h32c,1);if(dut.commit_ms!=commit_at)$fatal(1,"duplicatecommit refreshedtimer");
 while(dut.uptime_ms-commit_at<60)@(negedge clk);#1;if(ready)$fatal(1,"CPUstall notblocked at60ms");
 wr('h32c,2);if(!ready || pwm[15:0]!=555)$fatal(1,"freshcommit recovery");
 force dut.core_trap=1'b1;#1;if(ready)$fatal(1,"trap didn'tgate");force dut.core_trap=1'b0;
 wr('h31c,'h701);wr('h32c,3);if(ready)$fatal(1,"IMUnotready didn'tgate");
 $display("PASS v28 actualSoC coherent directinput snapshot/shadowatomiccommit/duplicateheartbeat/60msCPUstall/trap/IMUreadiness; oldaddressspace retained");$finish;end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
