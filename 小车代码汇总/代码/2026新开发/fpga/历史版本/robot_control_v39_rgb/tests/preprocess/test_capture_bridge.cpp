#include "Vrgb_capture_bridge.h"
#include "verilated.h"
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <vector>
static uint64_t checks=0;
#define CHECK(x)do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %s line=%d\n",#x,__LINE__);std::abort();}}while(0)
static int mirror(int x,int size){return x<0?-x:x>=size?2*size-2-x:x;}
static unsigned raw_pixel(int x,int y,int frame){
    x=mirror(x,960);y=mirror(y,1080);
    if(frame%3==0)return unsigned(x*3+y*5+frame*7)&255;
    if(frame%3==1)return ((x+y)&1)?220:31;
    return (uint32_t(x)*2654435761u^uint32_t(y)*2246822519u^uint32_t(frame)*3266489917u)&255;
}
static uint32_t demosaic(int x,int y,int frame){
    unsigned mc=raw_pixel(x,y,frame),ml=raw_pixel(x-1,y,frame),mr=raw_pixel(x+1,y,frame);
    unsigned tc=raw_pixel(x,y-1,frame),bc=raw_pixel(x,y+1,frame);
    unsigned cross=(tc+ml+mr+bc+2)>>2;
    unsigned diagonal=(raw_pixel(x-1,y-1,frame)+raw_pixel(x+1,y-1,frame)+
                       raw_pixel(x-1,y+1,frame)+raw_pixel(x+1,y+1,frame)+2)>>2;
    unsigned horizontal=(ml+mr+1)>>1,vertical=(tc+bc+1)>>1,r,g,b;
    switch(((y&1)<<1)|(x&1)){
        case 0:r=diagonal;g=cross;b=mc;break;
        case 3:r=mc;g=cross;b=diagonal;break;
        case 1:r=vertical;g=mc;b=horizontal;break;
        default:r=horizontal;g=mc;b=vertical;break;
    }
    return (r<<16)|(g<<8)|b;
}
struct Axis{int lo,hi,q;};
static Axis axis(int at,int src,int dst){int n=(2*at+1)*src-dst,d=2*dst;return {n/d,n/d+1,((n%d)*2048+d/2)/d};}
static std::vector<uint32_t> reference(int frame){
    std::vector<uint32_t> pixels(640*640,0x727272);
    for(int y=0;y<640;y++)for(int x=0;x<569;x++){
        Axis a=axis(x,960,569),b=axis(y,1080,640);
        uint32_t p00=demosaic(a.lo,b.lo,frame),p01=demosaic(a.hi,b.lo,frame);
        uint32_t p10=demosaic(a.lo,b.hi,frame),p11=demosaic(a.hi,b.hi,frame),result=0;
        for(int shift:{16,8,0}){
            uint64_t top=((p00>>shift)&255)*(2048-a.q)+((p01>>shift)&255)*a.q;
            uint64_t bottom=((p10>>shift)&255)*(2048-a.q)+((p11>>shift)&255)*a.q;
            result|=uint32_t((top*(2048-b.q)+bottom*b.q+(1<<21))>>22)<<shift;
        }
        pixels[y*640+x+35]=result;
    }
    return pixels;
}
struct Frame{int pattern;uint32_t stamp;std::vector<uint32_t> pixels;};
struct Test{
    std::unique_ptr<Vrgb_capture_bridge> d{new Vrgb_capture_bridge};
    std::map<uint32_t,Frame> frames;
    uint32_t next_sequence=0,sequence=0;bool open=false,stamped=false;
    size_t index=0;int completed=0,aborts=0,metadata=0;uint64_t cycles=0;
    bool discard_registered=false;
    bool repeat_epoch_once=false;
    void tick(){
        // A real CDC FIFO samples the bridge's REGISTERED write at this edge,
        // before the bridge updates its output registers for the next edge.
        bool write=d->fifo_write;uint64_t word=d->fifo_word;
        bool discard=discard_registered;discard_registered=false;
        d->clk=0;d->eval();d->clk=1;d->eval();++cycles;
        if(!write||!d->rst_n||discard)return;
        unsigned type=unsigned(word>>32);uint32_t value=word;
        if(type==15){open=false;stamped=false;index=0;++aborts;return;}
        if(type==1){
            CHECK(!open);CHECK(frames.count(value));sequence=value;open=true;stamped=false;index=0;++metadata;return;
        }
        CHECK(open);
        if(type==2){CHECK(!stamped&&index==0);CHECK(value==frames.at(sequence).stamp);stamped=true;return;}
        CHECK(stamped);
        if(type==0){
            CHECK((value&0xff000000)==0);CHECK(index<640*640);
            if(value!=frames.at(sequence).pixels[index]){
                std::fprintf(stderr,"tensor mismatch seq=%u pattern=%d index=%zu x=%zu y=%zu actual=%06x expected=%06x\n",sequence,frames.at(sequence).pattern,index,index%640,index/640,value,frames.at(sequence).pixels[index]);CHECK(false);
            }
            ++index;return;
        }
        CHECK(type==3);CHECK(value==sequence);CHECK(index==640*640);++completed;open=false;stamped=false;index=0;
    }
    void idle(int n){d->href=0;d->vsync=0;for(int i=0;i<n;i++)tick();}
    void reset(){
        d->rst_n=0;d->href=0;d->vsync=0;d->session_valid=0;d->session_epoch=0;d->init_done=0;
        d->fifo_full=0;d->uptime_gray=0;for(int i=0;i<8;i++)tick();
        d->rst_n=1;d->init_done=1;d->session_valid=1;d->session_epoch=1;idle(1);
        d->session_valid=0;idle(8);
        next_sequence=0;sequence=0;open=stamped=false;index=0;completed=aborts=metadata=0;frames.clear();
    }
    void vsync(){d->href=0;d->vsync=1;for(int i=0;i<4;i++)tick();d->vsync=0;idle(4);}
    void begin(int pattern){
        uint32_t stamp=1000+pattern;d->uptime_gray=stamp^(stamp>>1);idle(4);vsync();
        frames.emplace(++next_sequence,Frame{pattern,stamp,reference(pattern)});
    }
    void line(int y,int pattern,int width=960,int hblank=2){
        d->href=1;d->vsync=0;
        for(int x=0;x<width;x++){
            // Mailbox valid is a one-cycle update strobe, not link-alive.
            d->session_valid=repeat_epoch_once&&y==300&&x==400;
            d->raw=raw_pixel(x,y,pattern);tick();
        }
        d->session_valid=0;
        idle(hblank);
    }
    void frame(int pattern){begin(pattern);for(int y=0;y<1080;y++)line(y,pattern);idle(1400);}
    void partial(int pattern,int rows=100){begin(pattern);for(int y=0;y<rows;y++)line(y,pattern);}
    void flush(){idle(1500);}
};
int main(int argc,char **argv){
    std::setvbuf(stdout,nullptr,_IONBF,0);
    Verilated::commandArgs(argc,argv);Test t;t.reset();
    t.frame(0);CHECK(t.completed==1&&t.d->completed_frames==1&&t.d->dropped_frames==0&&t.aborts==0);
    t.repeat_epoch_once=true;t.frame(1);t.repeat_epoch_once=false;
    CHECK(t.completed==2&&t.d->completed_frames==2&&t.d->dropped_frames==0&&t.aborts==0);
    std::printf("PASS actual DVP Bayer gradient/checker -> demosaic -> Q11 resize -> 640x640 letterbox; two full tensors, all RGB bytes verified\n");
    t.reset();t.begin(12);for(int y=0;y<1079;y++)t.line(y,12);
    t.d->href=1;
    for(int x=0;x<960;x++){t.d->raw=raw_pixel(x,1079,12);t.tick();}
    t.d->href=0;t.d->vsync=1;for(int i=0;i<4;i++)t.tick();t.d->vsync=0;t.flush();
    CHECK(t.completed==1&&t.d->dropped_frames==0&&t.aborts==0);
    std::printf("PASS VSYNC coincident with final complete HREF falling edge preserves legal frame\n");
    t.reset();t.partial(2,100);t.vsync();t.flush();CHECK(t.aborts>0&&t.d->dropped_frames==1&&t.completed==0);
    t.frame(3);CHECK(t.completed==1);std::printf("PASS early VSYNC truncation aborts old frame, next healthy frame succeeds\n");
    t.reset();t.begin(4);t.line(0,4);t.line(1,4);t.line(2,4,959);t.flush();
    CHECK(t.aborts>0&&t.d->dropped_frames==1&&t.completed==0);t.frame(5);CHECK(t.completed==1);
    std::printf("PASS short HREF abort and recovery; one dropped-frame count despite propagated aborts\n");
    t.reset();t.begin(6);t.line(0,6);t.line(1,6);t.line(2,6,961);t.flush();
    CHECK(t.aborts>0&&t.d->dropped_frames==1&&t.completed==0);t.frame(7);CHECK(t.completed==1);
    std::printf("PASS long HREF abort and recovery\n");
    t.reset();t.partial(8,100);t.d->fifo_full=1;t.line(100,8);t.line(101,8);t.idle(40);t.d->fifo_full=0;t.flush();
    CHECK(t.aborts>0&&t.d->dropped_frames==1&&t.completed==0);t.frame(9);CHECK(t.completed==1);
    std::printf("PASS almost-full input backpressure abort; camera never stalled; next frame succeeds\n");
    t.reset();t.partial(10,100);
    // The session owner clears its FIFO/parser when epoch changes. Ignore the
    // one already-registered write crossing the reset edge, as that FIFO does.
    t.open=false;t.stamped=false;t.discard_registered=true;
    t.d->session_valid=1;t.d->session_epoch=2;t.d->href=0;t.d->vsync=0;t.tick();
    t.d->session_valid=0;t.next_sequence=0;t.frames.clear();
    t.idle(12);int old_metadata=t.metadata;
    // Mid-frame HREF after a new epoch cannot invent a source SOF.
    t.line(0,11);t.line(1,11);t.flush();CHECK(t.metadata==old_metadata&&t.completed==0);
    t.frame(11);CHECK(t.completed==1&&t.sequence==1);
    std::printf("PASS epoch change waits for next physical VSYNC and resets source sequence; Gray capture timestamp checked after 2FF latency\n");
    std::printf("PASS capture bridge checks=%llu cycles=%llu (functional simulation, not hardware FPS)\n",(unsigned long long)checks,(unsigned long long)t.cycles);
}
