#include "controller.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace robotphone;
static unsigned checks=0;
static void ck(bool condition,const char *message){++checks;if(!condition)throw std::runtime_error(message);}
static void near(double a,double b,const char *message){ck(std::abs(a-b)<1e-9,message);}
static Config config(){Config c;c.left_motor_mask=10;c.vision_fresh_ms=350;c.bearing.enabled=true;for(auto &e:c.bearing.eyes){e.calibrated=true;e.fx_px=600;e.cx_px=480;}return c;}
static ImageGeometry prepared(){return {1,640,640,960,1080,35,0,569,640};}
static Detection box(int eye,double u,double half,bool fpga){
    Detection b;b.class_id=2;b.score=.9;b.eye=eye;b.name="no_vest";
    auto x=[&](double raw){return fpga?(eye*640+35+raw*569/960)*(864./1280):(eye*960+raw)*.45;};
    b.x1=x(u-half);b.x2=x(u+half);b.y1=500*.45;b.y2=580*.45;return b;
}
static BearingResult install(Controller &c,Detection b,bool fpga,VisionFrame *saved=nullptr){
    c.updateImu({1000,true,0,0,0});VisionFrame frame;frame.frame_id=1;frame.time_ms=1000;frame.boxes={b};if(fpga)frame.geometry=prepared();
    auto result=c.updateVision(frame);ck(result.boxes.size()==1,"valid target disappeared");
    ck(c.select(result.frame_id,result.boxes[0].track_id,1000),"target selection failed");
    if(saved)*saved=result;
    return c.selectedBearing(1000);
}
int main(){try{
    for(int eye=0;eye<2;++eye)for(double u:{1.,40.,120.,240.,480.,720.,900.,958.}){
        Controller legacy(config()),fpga(config());VisionFrame before,after;
        const auto a=install(legacy,box(eye,u,.25,false),false,&before),b=install(fpga,box(eye,u,.25,true),true,&after);
        ck(a.valid&&b.valid,"calibrated bearing invalid");near(a.u_px,b.u_px,"inverse letterbox physical u differs");near(a.v_px,b.v_px,"physical v differs");
        near(a.camera_deg,b.camera_deg,"optical bearing changed");near(a.turn_deg,b.turn_deg,"mount/lookahead bearing changed");
        near(after.boxes[0].x1,box(eye,u,.25,true).x1,"public x left converted out of tensor coordinates");
        near(before.boxes[0].center_x,after.boxes[0].center_x,"normalized follow center differs");
        near(before.boxes[0].height_fraction,after.boxes[0].height_fraction,"normalized follow height differs");
        near(before.boxes[0].area_fraction,after.boxes[0].area_fraction,"normalized follow area differs");
    }
    for(int eye=0;eye<2;++eye)for(bool right:{false,true}){
        Controller c(config());VisionFrame frame;frame.frame_id=1;frame.time_ms=1000;frame.geometry=prepared();Detection b;b.class_id=2;b.score=.9;b.eye=eye;
        const double origin=eye*432.;b.x1=origin+(right?610:2)*.675;b.x2=origin+(right?630:30)*.675;b.y1=10;b.y2=100;frame.boxes={b};
        ck(c.updateVision(frame).boxes.empty(),"padding-only target accepted for follow");
    }
    Controller edge(config());Detection crossing=box(0,0,30,true);VisionFrame saved;const auto b=install(edge,crossing,true,&saved);
    ck(b.valid,"clipped content edge target rejected");near(saved.boxes[0].x1,35*.675,"box not clipped at valid content start");near(b.u_px,15,"partly padded box physical center incorrect");
    double u,v;std::string error;
    ck(!cameraPixel(prepared(),.01,.5,u,v,error)&&error=="target_in_model_padding","padding pixel ray accepted");
    auto unsupported=prepared();unsupported.content_x=36;ck(!cameraPixel(unsupported,.5,.5,u,v,error),"incorrect announced letterbox accepted");
    // The 1280x640 preview is letterboxed into 864x486 (27 top,432 high).
    for(int eye=0;eye<2;eye++){
        Controller legacy(config()),viewport(config());VisionFrame old;const auto before=install(legacy,box(eye,720,20,false),false,&old);
        viewport.updateImu({1000,true,0,0,0});VisionFrame frame;frame.frame_id=1;frame.time_ms=1000;frame.geometry=prepared();frame.geometry.viewport_y=27./486;frame.geometry.viewport_height=432./486;
        double pu=0,pv=0;std::string padding_error;
        ck(!cameraPixel(frame.geometry,.5,10./486,pu,pv,padding_error),"top display black border accepted as camera pixel");
        ck(!cameraPixel(frame.geometry,.5,480./486,pu,pv,padding_error),"bottom display black border accepted as camera pixel");
        auto b=box(eye,720,20,true);b.y1=27+500./1080*432;b.y2=27+580./1080*432;frame.boxes={b};auto saved=viewport.updateVision(frame);
        ck(saved.boxes.size()==1,"letterboxed viewport dropped target");ck(viewport.select(1,saved.boxes[0].track_id,1000),"viewport target selection failed");auto after=viewport.selectedBearing(1000);
        near(before.u_px,after.u_px,"viewport physical u differs");near(before.v_px,after.v_px,"viewport physical v differs");near(old.boxes[0].height_fraction,saved.boxes[0].height_fraction,"viewport normalized height differs");near(old.boxes[0].area_fraction,saved.boxes[0].area_fraction,"viewport normalized area differs");
        frame.frame_id=2;frame.time_ms=1010;frame.boxes.clear();frame.complete_mask=1;frame.requested_mask=3;frame.control_eligible=false;
        const auto partial=viewport.updateVision(frame);ck(partial.frame_id==1&&partial.time_ms==1000,"partial renewed or replaced complete control");ck(viewport.state().target_id==saved.boxes[0].track_id,"partial cleared existing target");
        frame.frame_id=3;frame.time_ms=1020;frame.complete_mask=3;const auto late=viewport.updateVision(frame);ck(late.frame_id==1,"late complete renewed control");
        ck(viewport.selectedBearing(1351).reason=="vision_stale","partial or late pair renewed control age");
    }
    std::cout<<"PASS FPGA tensor to calibrated camera geometry "<<checks<<" checks; no motor commands sent\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<" after "<<checks<<" checks\n";return 1;}}
