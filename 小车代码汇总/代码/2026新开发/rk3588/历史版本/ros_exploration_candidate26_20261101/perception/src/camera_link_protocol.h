#ifndef PERCEPTION_CAMERA_LINK_PROTOCOL_H
#define PERCEPTION_CAMERA_LINK_PROTOCOL_H
#include <cstddef>
#include <cstdint>
#include <cstring>
namespace camera_link {
enum class Mode { Unknown, Raw, PreparedRgb };
inline unsigned be16(const uint8_t *p){return unsigned(p[0])*256+p[1];}
// Inspect complete Ethernet frames, never confuse sensor/control EtherTypes
// with image traffic. RAW1080 is the established 1922-byte UDP payload.
inline Mode classify(const uint8_t *p,size_t n){
 static constexpr uint8_t rgb_peer[6]={2,0xac,0x13,0x80,0,1};
 static constexpr uint8_t raw_peer[6]={0,0x0a,0x35,1,0xfe,0xc0};
 if(!p||n<14)return Mode::Unknown;
 const auto type=be16(p+12);
 if(type==0x88ba&&!std::memcmp(p+6,rgb_peer,6)&&n>=14+992&&std::memcmp(p+14,"RGP1",4)==0&&p[18]==1&&p[19]<=1&&p[20]==1&&be16(p+34)==640&&be16(p+36)==640)return Mode::PreparedRgb;
 if(type!=0x0800||std::memcmp(p+6,raw_peer,6)||n<14+20+8+1922)return Mode::Unknown;
 const auto ip=p+14;const unsigned ihl=(ip[0]&15)*4;
 if((ip[0]>>4)!=4||ihl<20||n<14+ihl+8+1922||ip[9]!=17||(be16(ip+6)&0x3fff))return Mode::Unknown;
 const auto udp=ip+ihl;
 if(be16(udp+2)!=6102||be16(udp+4)!=1930)return Mode::Unknown;
 return (unsigned(udp[8])+(unsigned(udp[9])<<8)<1080)?Mode::Raw:Mode::Unknown;
}
}
#endif
