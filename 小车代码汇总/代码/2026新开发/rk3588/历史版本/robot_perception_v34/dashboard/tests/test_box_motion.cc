#include "box_motion.h"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>
using box_motion::Gray;
static unsigned random_value=0x12345678;
static uint8_t random_byte(){random_value^=random_value<<13;random_value^=random_value>>17;random_value^=random_value<<5;return random_value>>24;}
static Gray scene(int object_x,int object_y,int width=32,int height=42,bool strong_background=false){
    Gray image;
    auto wall=[](int x,int y){unsigned v=unsigned(x)*374761393u+unsigned(y)*668265263u+918273u;v=(v^(v>>13))*1274126177u;return 20+int((v^(v>>16))%215);};
    for(int y=0;y<box_motion::H;y++)for(int x=0;x<box_motion::W;x++){
        int gx=x/4,gy=y/4,fx=x%4,fy=y%4;
        image.full[y*box_motion::W+x]=strong_background?
            (wall(gx,gy)*(4-fx)*(4-fy)+wall(gx+1,gy)*fx*(4-fy)+wall(gx,gy+1)*(4-fx)*fy+wall(gx+1,gy+1)*fx*fy)/16:
            35+(x*3+y*5)%11;
    }
    random_value=0x87654321;
    std::array<unsigned char,20*20> texture{};for(auto &v:texture)v=30+random_byte()%190;
    for(int y=0;y<height;y++)for(int x=0;x<width;x++){
        int gx=x/4,gy=y/4,fx=x%4,fy=y%4;
        uint8_t p=(texture[gy*20+gx]*(4-fx)*(4-fy)+texture[gy*20+gx+1]*fx*(4-fy)+texture[(gy+1)*20+gx]*(4-fx)*fy+texture[(gy+1)*20+gx+1]*fx*fy)/16;
        if(object_x+x>=0&&object_x+x<box_motion::W&&object_y+y>=0&&object_y+y<box_motion::H)
            image.full[(object_y+y)*box_motion::W+object_x+x]=p;
    }
    image.finish();return image;
}
static vision_object object(int x,int y,int w=32,int h=42,int eye=0){
    vision_object o{};o.left=x*3;o.top=y*3;o.right=(x+w)*3;o.bottom=(y+h)*3;o.score=.8;o.eye=eye;
    return o;
}
static void movement(int dx,int dy,uint64_t age,int eye=0){
    int x=eye==1?180:45,y=55;
    Gray old=scene(x,y),now=scene(x+dx,y+dy);
    auto estimate=box_motion::estimate(old,now,object(x,y,32,42,eye),age);
    if(!estimate.valid||std::abs(estimate.dx-dx*3)>3||std::abs(estimate.dy-dy*3)>3){
        std::fprintf(stderr,"motion failed dx=%d dy=%d got=%f/%f valid=%d features=%d\n",dx,dy,estimate.dx,estimate.dy,estimate.valid,estimate.features);
        for(int quadrant=0;quadrant<4;quadrant++){
            int best=-1;box_motion::detail::Point point;
            for(int yy=0;yy<4;yy++)for(int xx=0;xx<4;xx++){
                float fx=.15f+((quadrant&1)*4+xx)*.1f,fy=.15f+((quadrant/2)*4+yy)*.1f;
                int px=std::lround(x+32*fx),py=std::lround(y+42*fy);
                int v=box_motion::detail::texture(old,px,py);if(v>best){best=v;point={px,py};}
            }
            auto m=box_motion::detail::match(old,now,point,std::min(28,std::max(8,int(age/7))),eye?144:0,eye?288:144);
            std::fprintf(stderr," point%d %d,%d var%d match%d dx%d dy%d\n",quadrant,point.x,point.y,best,m.valid,m.dx,m.dy);
        }
        std::abort();
    }
    vision_object out{};assert(box_motion::apply(estimate,object(x,y,32,42,eye),out));
    assert(out.eye==eye&&out.score==.8f);
}
int main(){
    int movement_cases=0;
    for(int eye=0;eye<2;eye++)for(int dx=-15;dx<=15;dx+=3)for(int dy=-7;dy<=7;dy+=7){movement(dx,dy,140,eye);++movement_cases;}
    movement(23,12,220);movement(-24,-9,240);movement(0,0,80);
    // The surrounding wall stays fixed while just the object moves. This must
    // follow foreground evidence, rather than infer a global camera pan.
    int padded_matches=0;
    for(int eye=0;eye<2;eye++)for(int dx:{-13,0,14}){
        int x=eye?180:45,dy=dx?5:0;
        Gray reference=scene(x,55,32,42,true),preview=scene(x+dx,55+dy,32,42,true);
        for(int padding:{0,3}){
            auto translated=box_motion::estimate(reference,preview,object(x-padding,55-padding,32+padding*2,42+padding*2,eye),160);
            assert(padding||translated.valid);
            if(translated.valid){assert(std::abs(translated.dx-dx*3)<=3&&std::abs(translated.dy-dy*3)<=3);if(padding)++padded_matches;}
        }
    }
    assert(padded_matches>=4);
    // Half occlusion can keep a bounded, correct estimate from visible texture;
    // changed geometry must either be rejected or avoid an invented large move.
    Gray foreground=scene(45,55,32,42,true),occluded=scene(55,50,32,42,true);
    for(int y=50;y<92;y++)for(int x=71;x<87;x++)occluded.full[y*box_motion::W+x]=120;
    occluded.finish();auto partial=box_motion::estimate(foreground,occluded,object(45,55),160);
    assert(!partial.valid||(std::abs(partial.dx-30)<=3&&std::abs(partial.dy+15)<=3));
    Gray scaled=scene(-1000,-1000,32,42,true);
    for(int y=0;y<52;y++)for(int x=0;x<40;x++)scaled.full[(50+y)*box_motion::W+41+x]=foreground.full[(55+y*42/52)*box_motion::W+45+x*32/40];
    scaled.finish();auto zoom=box_motion::estimate(foreground,scaled,object(45,55),160);
    assert(!zoom.valid||(std::abs(zoom.dx)<=9&&std::abs(zoom.dy)<=9));
    Gray removed=scene(-1000,-1000,32,42,true);
    assert(!box_motion::estimate(foreground,removed,object(45,55),160).valid);
    Gray old=scene(45,55),current=scene(55,50),blank;
    blank.full.fill(90);blank.finish();
    assert(!box_motion::estimate(blank,blank,object(45,55),100).valid);
    assert(!box_motion::estimate(old,blank,object(45,55),100).valid);
    assert(!box_motion::estimate(old,current,object(45,55),VISION_BOX_MAX_AGE_MS+1).valid);
    assert(!box_motion::estimate(old,current,object(45,55,2,2),100).valid);
    auto bad=object(45,55);bad.left=std::numeric_limits<float>::quiet_NaN();
    assert(!box_motion::estimate(old,current,bad,100).valid);
    // An identical-looking object in the OTHER eye must never attract a box.
    Gray opposite=scene(180,55);
    assert(!box_motion::estimate(old,opposite,object(45,55),150).valid);
    auto clipped=object(130,55,13,42,0);vision_object out{};
    assert(box_motion::apply({12,0,3,true},clipped,out));
    assert(out.right<=VISION_W/2&&out.left<VISION_W/2);
    assert(!box_motion::apply({300,0,3,true},clipped,out));
    // Gray thumbnails sampled from source and preview must match exactly,
    // including non-16:9 letterboxing and channel weighting.
    const int width=320,height=240;
    std::vector<uint8_t> rgb(width*height*3);for(auto &v:rgb)v=random_byte();
    const double scale=std::min(VISION_W/double(width),VISION_H/double(height));
    int ww=int(width*scale),hh=int(height*scale),ox=(VISION_W-ww)/2,oy=(VISION_H-hh)/2;
    std::vector<uint32_t> preview(VISION_W*VISION_H,0xff14232d);
    for(int y=0;y<hh;y++)for(int x=0;x<ww;x++){
        const auto *p=rgb.data()+(std::min(height-1,int(y/scale))*width+std::min(width-1,int(x/scale)))*3;
        preview[(y+oy)*VISION_W+x+ox]=0xff000000|(p[0]<<16)|(p[1]<<8)|p[2];
    }
    Gray source,rendered;source.from_rgb(rgb.data(),width,height,ox,oy,ww,hh,scale);rendered.from_preview(preview.data());
    assert(source.full==rendered.full&&source.half==rendered.half);
    // Repeat the complete 32-box budget: timings are host-only, not NPU FPS.
    int good=0;auto began=std::chrono::steady_clock::now();
    for(int frame=0;frame<30;frame++)for(int i=0;i<32;i++)good+=box_motion::estimate(old,current,object(45,55),140).valid;
    double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()/30;
    assert(good==30*32);
    began=std::chrono::steady_clock::now();good=0;
    for(int frame=0;frame<30;frame++)for(int i=0;i<32;i++)good+=box_motion::estimate(old,current,object(45,55),350).valid;
    double worst=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()/30;
    assert(good==30*32);
    std::printf("PASS image motion: %d translations + 12 strong stationary background/padded foreground cases, fast movement, static, full/partial occlusion, scale change, textureless, stereo boundaries, age/NaN/small boxes, exact gray letterbox mapping; padded_accepted=%d/6 partial_occlusion_accepted=%d scale_change_accepted=%d host32box_ms=%.3f worst_radius_host32box_ms=%.3f\n",movement_cases+3,padded_matches,partial.valid,zoom.valid,elapsed,worst);
}
