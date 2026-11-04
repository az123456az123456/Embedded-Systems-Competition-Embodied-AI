#ifndef PERCEPTION_RGB_CAPTURE_FILES_H
#define PERCEPTION_RGB_CAPTURE_FILES_H
#include "rgb_camera.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>

namespace fpga_rgb {
inline std::string json_string(const std::string &text){std::string out="\"";for(char c:text){if(c=='\\'||c=='\"')out+='\\';if(c=='\n'){out+="\\n";continue;}if(c=='\r'){out+="\\r";continue;}out+=c;}return out+'\"';}
inline std::string crc_hex(const uint8_t *data,size_t size){std::ostringstream out;out<<std::hex<<std::setw(8)<<std::setfill('0')<<(crc32_update(0xffffffffu,data,size)^0xffffffffu);return out.str();}
inline void save_bundle(const std::string &directory,const std::vector<uint8_t> &rgb,const FrameInfo &info,const std::string &source) {
    if(rgb.size()!=FrameBytes)throw std::runtime_error("Cannot save incomplete RGB pair");
    if(mkdir(directory.c_str(),0755))throw std::runtime_error("Output directory must be new and its parent must exist");
    std::ostringstream files;bool first=true;
    auto write=[&](const std::string &name,const std::vector<uint8_t> &bytes){
        std::ofstream out(directory+"/"+name,std::ios::binary);out.write(reinterpret_cast<const char *>(bytes.data()),bytes.size());out.close();
        if(!out)throw std::runtime_error("Write failed: "+name);
        if(!first)files<<',';
        first=false;files<<"\n    "<<json_string(name)<<":{\"bytes\":"<<bytes.size()<<",\"crc32\":"<<json_string(crc_hex(bytes.data(),bytes.size()))<<'}';
    };
    auto ppm=[&](const std::string &name,unsigned width,const std::vector<uint8_t> &pixels){
        const std::string header="P6\n"+std::to_string(width)+" 640\n255\n";
        std::vector<uint8_t> bytes(header.begin(),header.end());bytes.insert(bytes.end(),pixels.begin(),pixels.end());write(name,bytes);
    };
    write("stereo.rgb",rgb);ppm("stereo.ppm",1280,rgb);
    for(unsigned eye=0;eye<2;++eye){std::vector<uint8_t> pixels(Width*Height*3);for(unsigned y=0;y<Height;++y) {
        std::copy_n(rgb.data()+(static_cast<size_t>(y)*Width*2+eye*Width)*3,Width*3,pixels.data()+static_cast<size_t>(y)*Width*3);
        }
        const std::string name=eye?"right":"left";write(name+".rgb",pixels);ppm(name+".ppm",640,pixels);
    }
    std::ofstream meta(directory+"/metadata.json");meta<<"{\n  \"schema\":1,\n  \"source\":"<<json_string(source)
       <<",\n  \"format\":\"RGB888\",\n  \"width\":1280,\n  \"height\":640,\n  \"sequence\":"<<info.sequence<<",\n  \"epoch\":"<<info.epoch
       <<",\n  \"complete_mask\":"<<info.complete_mask<<",\n  \"device_uptime_ms\":"<<info.device_uptime_ms<<",\n  \"source_ms\":"<<info.source_ms
       <<",\n  \"first_rx_ms\":"<<info.first_rx_ms<<",\n  \"completed_ms\":"<<info.completed_ms
       <<",\n  \"clock_semantics\":\"transport offset lower bound, not synchronized exposure\",\n  \"files\":{"<<files.str()<<"\n  }\n}\n";meta.close();
    if(!meta)throw std::runtime_error("Metadata write failed");
}
}
#endif
