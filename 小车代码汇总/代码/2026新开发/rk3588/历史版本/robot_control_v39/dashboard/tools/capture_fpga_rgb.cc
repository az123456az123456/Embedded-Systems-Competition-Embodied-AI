#include "rgb_camera.h"
#include "rgb_capture_files.h"
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
using namespace fpga_rgb;
static volatile std::sig_atomic_t interrupted=0;
static void signal_stop(int){interrupted=1;}
int main(int argc,char **argv){try{
    std::string interface="eth0",output;unsigned timeout=10;
    for(int i=1;i<argc;++i){std::string arg=argv[i];if((arg=="--interface"||arg=="--output"||arg=="--timeout")&&i+1<argc){std::string value=argv[++i];if(arg=="--interface")interface=value;else if(arg=="--output")output=value;else timeout=std::stoul(value);}else throw std::runtime_error("Usage: capture-fpga-rgb --interface eth0 --output NEWDIR [--timeout 10]");}
    if(output.empty()||timeout<1||timeout>30)throw std::runtime_error("A new output directory and timeout 1..30s are required");
    std::signal(SIGINT,signal_stop);std::signal(SIGTERM,signal_stop);
    RgbCamera camera;std::string error;if(!camera.open(interface,error))throw std::runtime_error(error);
    std::cerr<<"Capture only: RGC1 starts camera session; no NPU, IMU, motor or navigation commands. Exit dashboard first. "<<camera.description()<<'\n';
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(timeout);std::vector<uint8_t> rgb;ReadTiming timing;
    while(!interrupted&&std::chrono::steady_clock::now()<deadline){const int result=camera.read(rgb,error,&timing);if(result<0)throw std::runtime_error(error);if(result>0){save_bundle(output,rgb,timing.frame,camera.description());std::cout<<"Saved complete pair sequence="<<timing.frame.sequence<<" epoch="<<timing.frame.epoch<<" output="<<output<<" (1280x640 RGB, complete_mask=3)\n";return 0;}}
    throw std::runtime_error(interrupted?"Capture interrupted":"Timed out without a complete stereo RGP1 pair; no partial image saved");
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
