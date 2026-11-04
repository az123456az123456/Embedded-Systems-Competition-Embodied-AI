#include "rgb_camera.h"
#include "rgb_capture_files.h"
#include <fstream>
#include <cstring>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
using namespace fpga_rgb;
namespace {
uint16_t be16(const uint8_t *p){return (p[0]<<8)|p[1];}
uint32_t be32(const uint8_t *p){return (uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3];}
std::vector<uint8_t> hex(const std::string &line){std::vector<uint8_t> out;int high=-1;for(char c:line){if(c=='#')break;if(c==' '||c=='\r'||c=='\t'||c=='\n')continue;int v=c>='0'&&c<='9'?c-'0':c>='a'&&c<='f'?c-'a'+10:c>='A'&&c<='F'?c-'A'+10:-1;if(v<0)throw std::runtime_error("Invalid hex input");if(high<0)high=v;else{out.push_back((high<<4)|v);high=-1;}}if(high>=0)throw std::runtime_error("Odd hex input");return out;}
}
int main(int argc,char **argv){try{
    std::string format="records",input,output;unsigned epoch=0;bool with_fcs=false,allow_errors=false;
    for(int i=1;i<argc;++i){std::string arg=argv[i];if(arg=="--with-fcs")with_fcs=true;else if(arg=="--allow-errors")allow_errors=true;
        else if((arg=="--format"||arg=="--input"||arg=="--output"||arg=="--epoch")&&i+1<argc){std::string value=argv[++i];if(arg=="--format")format=value;else if(arg=="--input")input=value;else if(arg=="--output")output=value;else epoch=std::stoul(value);}
        else throw std::runtime_error("Usage: rgb-codec-probe --epoch N --format records|hex|gmii --input FILE [--with-fcs] [--output NEWDIR] [--allow-errors]");}
    if(epoch<1||epoch>65535||input.empty())throw std::runtime_error("A nonzero uint16 epoch and input are required");
    if(format!="records"&&format!="hex"&&format!="gmii")throw std::runtime_error("Unknown input format");
    std::ifstream source(input,std::ios::binary);if(!source)throw std::runtime_error("Input open failed");
    Assembler assembler(epoch);uint64_t total=0,other=0,bad_fcs=0,rgp=0;std::vector<uint8_t> last_rgb;FrameInfo last_info;bool have_pair=false;
    auto consume=[&](std::vector<uint8_t> frame){++total;
        if(with_fcs||format=="gmii"){
            if(frame.size()<18)throw std::runtime_error("Short Ethernet/FCS frame");
            const size_t n=frame.size()-4;
            const uint32_t expected=uint32_t(frame[n])|(uint32_t(frame[n+1])<<8)|(uint32_t(frame[n+2])<<16)|(uint32_t(frame[n+3])<<24);
            if((crc32_update(0xffffffffu,frame.data(),n)^0xffffffffu)!=expected){++bad_fcs;return;}frame.resize(n);
        }
        if(frame.size()<14)throw std::runtime_error("Short Ethernet header");
        if(be16(frame.data()+12)!=EtherType||frame.size()<18||std::memcmp(frame.data()+14,"RGP1",4)){++other;return;}
        ++rgp;const uint64_t replay_host_ms=1000000+rgp/64;
        assembler.ingest(frame.data()+14,frame.size()-14,replay_host_ms);
        if(assembler.take_latest(last_rgb,last_info,replay_host_ms))have_pair=true;
    };
    if(format=="hex"){std::string line;while(std::getline(source,line)){auto bytes=hex(line);if(!bytes.empty())consume(std::move(bytes));}}
    else if(format=="gmii"){
        std::array<uint8_t,1018> raw{};while(source.read(reinterpret_cast<char *>(raw.data()),raw.size())){
            for(unsigned i=0;i<7;++i)if(raw[i]!=0x55)throw std::runtime_error("Invalid GMII preamble");
            if(raw[7]!=0xd5)throw std::runtime_error("Invalid GMII SFD");
            consume({raw.begin()+8,raw.end()});
        }if(source.gcount())throw std::runtime_error("Truncated GMII record");
    }else {
        std::array<uint8_t,4> header{};while(source.read(reinterpret_cast<char *>(header.data()),4)){
            const size_t size=be32(header.data());if(size<14||size>2048)throw std::runtime_error("Invalid record extent");std::vector<uint8_t> frame(size);if(!source.read(reinterpret_cast<char *>(frame.data()),size))throw std::runtime_error("Truncated frame record");consume(std::move(frame));
        }if(source.gcount())throw std::runtime_error("Truncated record length");
    }
    const auto &s=assembler.stats();
    if(have_pair&&!output.empty())save_bundle(output,last_rgb,last_info,"RTL/C++ offline replay, synthetic host time; no hardware execution");
    std::cout<<"{\"epoch\":"<<epoch<<",\"ethernet_frames\":"<<total<<",\"other_ethertypes\":"<<other<<",\"bad_fcs\":"<<bad_fcs<<",\"rgp_packets\":"<<rgp<<",\"accepted\":"<<s.accepted<<",\"complete_pairs\":"<<s.completed<<",\"crc_bad\":"<<s.crc_bad<<",\"malformed\":"<<s.malformed<<",\"wrong_epoch\":"<<s.wrong_epoch<<",\"conflicts\":"<<s.conflicts<<",\"last_sequence\":"<<(have_pair?last_info.sequence:0)<<",\"complete_mask\":"<<(have_pair?last_info.complete_mask:0)<<",\"last_rgb_crc32\":"<<json_string(have_pair?crc_hex(last_rgb.data(),last_rgb.size()):"")<<",\"hardware_tested\":false}\n";
    return (!have_pair||(!allow_errors&&(bad_fcs||s.crc_bad||s.malformed||s.wrong_epoch||s.conflicts)))?1:0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}}
