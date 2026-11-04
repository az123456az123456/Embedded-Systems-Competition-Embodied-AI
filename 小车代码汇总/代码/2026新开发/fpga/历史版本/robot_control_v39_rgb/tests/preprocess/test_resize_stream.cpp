#include "Vresize_top.h"
#include "verilated.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>

static uint64_t assertions=0;
#define CHECK(x) do{++assertions;if(!(x)){std::fprintf(stderr,"FAIL %s line=%d\n",#x,__LINE__);std::abort();}}while(0)
struct Axis {int lo,hi,q;};
static Axis axis(int at,int src,int dst){
    int n=(2*at+1)*src-dst;int d=2*dst;int lo=n/d;
    return {lo,lo+1,((n%d)*2048+d/2)/d};
}
static uint32_t pixel(int x,int y,int frame){
    switch(frame%6){
        case 0:return 0x7f7f7f;
        case 1:return ((x&255)<<16)|((y&255)<<8)|((x+y)&255);
        case 2:return ((x+y)&1)?0xffffff:0;
        case 3:{uint32_t h=uint32_t(x)*2654435761u^uint32_t(y)*2246822519u^uint32_t(frame)*3266489917u;return h&0xffffff;}
        case 4:return x==959&&y==1079?0xffffff:0;
        default:return y<540?0xff0000:x<480?0x00ff00:0x0000ff;
    }
}
static uint32_t expected(int x,int y,int frame){
    if(x<35||x>=604)return 0x727272;
    Axis a=axis(x-35,960,569),b=axis(y,1080,640);uint32_t result=0;
    for(int shift:{16,8,0}){
        uint64_t p00=(pixel(a.lo,b.lo,frame)>>shift)&255,p01=(pixel(a.hi,b.lo,frame)>>shift)&255;
        uint64_t p10=(pixel(a.lo,b.hi,frame)>>shift)&255,p11=(pixel(a.hi,b.hi,frame)>>shift)&255;
        uint64_t top=p00*(2048-a.q)+p01*a.q,bottom=p10*(2048-a.q)+p11*a.q;
        result|=uint32_t((top*(2048-b.q)+bottom*b.q+(1u<<21))>>22)<<shift;
    }
    return result;
}
struct Simulation {
    std::unique_ptr<Vresize_top> dut{new Vresize_top};
    int next_frame=0,current_frame=-1,x=0,y=0,completed=0,aborts=0;bool checking=true;
    uint64_t cycles=0,outputs=0;unsigned maximum_fifo=0;
    void tick(){
        dut->clk=0;dut->eval();dut->clk=1;dut->eval();++cycles;
        maximum_fifo=std::max(maximum_fifo,unsigned(dut->fifo_peak));
        if(dut->out_abort){++aborts;current_frame=-1;x=y=0;}
        if(!dut->out_valid)return;
        ++outputs;
        if(dut->out_sof){CHECK(current_frame==-1);current_frame=next_frame++;x=y=0;}
        CHECK(current_frame>=0);
        CHECK(dut->out_x==x&&dut->out_y==y);
        if(checking&&dut->out_rgb!=expected(x,y,current_frame)){
            std::fprintf(stderr,"pixel mismatch frame=%d x=%d y=%d got=%06x expected=%06x cycles=%llu\n",current_frame,x,y,dut->out_rgb,expected(x,y,current_frame),(unsigned long long)cycles);CHECK(false);
        }
        CHECK(bool(dut->out_sof)==(x==0&&y==0));CHECK(bool(dut->out_eol)==(x==639));
        CHECK(bool(dut->out_eof)==(x==639&&y==639));
        if(x==639){x=0;if(y==639){++completed;current_frame=-1;y=0;}else ++y;}else ++x;
    }
    void reset(int first=0){
        dut->bypass=0;
        dut->in_valid=0;dut->in_sof=0;dut->in_eol=0;dut->in_eof=0;dut->in_abort=0;
        dut->rst_n=0;for(int i=0;i<5;i++)tick();dut->rst_n=1;
        next_frame=first;current_frame=-1;x=y=0;completed=0;aborts=0;outputs=0;maximum_fifo=0;
    }
    void input(int px,int py,int frame,bool gap=false){
        if(gap&&((px*13+py*7+frame)%17==0)){dut->in_valid=0;tick();}
        dut->in_valid=1;dut->in_sof=px==0&&py==0;dut->in_eol=px==959;
        dut->in_eof=px==959&&py==1079;dut->in_x=px;dut->in_y=py;dut->in_rgb=pixel(px,py,frame);tick();
    }
    void frame(int index,bool gaps=false){for(int py=0;py<1080;py++)for(int px=0;px<960;px++)input(px,py,index,gaps);}
    void flush(){dut->in_valid=0;dut->in_sof=0;dut->in_eol=0;dut->in_eof=0;for(int i=0;i<3000;i++)tick();}
};
int main(int argc,char **argv){
    Verilated::commandArgs(argc,argv);Simulation s;
    s.reset();for(int i=0;i<30;i++)s.frame(i);s.flush();
    CHECK(s.completed==30&&s.aborts==0&&s.maximum_fifo<64);
    std::printf("PASS 30 full RGB frames continuous valid/no Hblank/back-to-back; pixels=%llu cycles=%llu fifo_peak=%u\n",(unsigned long long)s.outputs,(unsigned long long)s.cycles,s.maximum_fifo);
    s.reset(30);for(int i=30;i<33;i++)s.frame(i,true);s.flush();
    CHECK(s.completed==3&&s.aborts==0&&s.maximum_fifo<64);
    std::printf("PASS 3 full frames with bubbles, exact same pixel/coordinate/sof/eol/eof contract; fifo_peak=%u\n",s.maximum_fifo);
    s.reset();for(int py=0;py<301;py++)for(int px=0;px<960;px++)s.input(px,py,0);
    s.dut->in_valid=0;s.dut->in_abort=1;s.tick();s.dut->in_abort=0;s.flush();CHECK(s.aborts>0);
    s.next_frame=7;s.frame(7);s.frame(8);s.flush();CHECK(s.completed==2);
    std::printf("PASS explicit interrupted frame dropped, then clean SOF recovery\n");
    s.reset();for(int py=0;py<301;py++)for(int px=0;px<960;px++)s.input(px,py,0);
    // Out-of-order input (a missing pixel) must abort, never shift the map.
    s.input(1,301,0);s.dut->in_valid=0;s.flush();CHECK(s.aborts>0);
    s.next_frame=13;s.frame(13);s.flush();CHECK(s.completed==1);
    std::printf("PASS missing source pixel abort and recovery\n");
    s.reset();for(int py=0;py<301;py++)for(int px=0;px<960;px++)s.input(px,py,0);
    s.next_frame=19;s.frame(19);s.frame(20);s.flush();CHECK(s.completed==2&&s.aborts>0);
    std::printf("PASS unexpected SOF interrupts incomplete frame then clean recovery\n");
    s.reset();s.dut->bypass=1;s.checking=false;
    // A deliberately invalid sustained dense stream exceeds the proven
    // source timing envelope. FIFO overflow must abort, never wrap/corrupt.
    for(int py=0;py<5&&!s.aborts;py++)for(int px=0;px<569&&!s.aborts;px++){
        s.dut->in_valid=1;s.dut->in_sof=px==0&&py==0;s.dut->in_eol=px==568;
        s.dut->in_eof=0;s.dut->in_x=px;s.dut->in_y=py;s.dut->in_rgb=0x102030;s.tick();
    }
    CHECK(s.aborts>0);s.dut->in_valid=0;s.flush();s.next_frame=23;
    for(int py=0;py<640;py++)for(int px=0;px<569;px++){
        s.dut->in_valid=0;s.tick();s.dut->in_valid=1;s.dut->in_sof=px==0&&py==0;
        s.dut->in_eol=px==568;s.dut->in_eof=px==568&&py==639;
        s.dut->in_x=px;s.dut->in_y=py;s.dut->in_rgb=0x102030;s.tick();
    }
    s.flush();CHECK(s.completed==1);
    std::printf("PASS letterbox deliberate overflow abort and clean source recovery; checks=%llu\n",(unsigned long long)assertions);
}
