// FPC1 native host tool: calibration control only, no motor/navigation protocol.
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include "picojson.h"

namespace fpc1 {
using Body=std::array<uint8_t,64>;
using Mac=std::array<uint8_t,6>;
using Clock=std::chrono::steady_clock;
constexpr unsigned Width=960, EtherType=0x88bb;
constexpr Mac Target={0x02,0xac,0x13,0x80,0x00,0x01};
struct Coeff {unsigned dark,gain;};
void require(bool ok,const std::string& msg) {if(!ok)throw std::runtime_error(msg);}
uint16_t crc16(const uint8_t* p,size_t n) {
    uint16_t crc=0xffff;
    for(size_t i=0;i<n;++i) {crc^=uint16_t(p[i])<<8;for(int bit=0;bit<8;++bit)crc=(crc&0x8000)?uint16_t((crc<<1)^0x1021):uint16_t(crc<<1);}
    return crc;
}
unsigned be16(const Body& b,unsigned i) {return unsigned(b[i])<<8|b[i+1];}
uint32_t be32(const Body& b,unsigned i) {return uint32_t(b[i])<<24|uint32_t(b[i+1])<<16|uint32_t(b[i+2])<<8|b[i+3];}
void put16(Body& b,unsigned i,unsigned x) {b[i]=uint8_t(x>>8);b[i+1]=uint8_t(x);}
void put32(Body& b,unsigned i,uint32_t x) {for(unsigned n=0;n<4;++n)b[i+n]=uint8_t(x>>(24-8*n));}
uint32_t next_id(uint32_t id) {return id==0xffffffffU?1:id+1;}
Mac mac(const std::string& text) {
    Mac m{};require(text.size()==17,"MAC must be six colon-separated bytes");
    for(unsigned i=0;i<6;++i) {
        if(i)require(text[3*i-1]==':',"Invalid MAC separator");
        auto digit=[](char c)->int {if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;return -1;};
        int a=digit(text[3*i]),b=digit(text[3*i+1]);require(a>=0&&b>=0,"Invalid MAC digit");m[i]=uint8_t(a*16+b);
    }return m;
}
Body request(unsigned op,unsigned eye,uint32_t id,unsigned col=0,unsigned dark=0,unsigned gain=0,unsigned value=0) {
    require(op>=1&&op<=6&&eye<2&&id!=0,"Invalid op/eye/request ID");
    if(op==2)require(col<Width&&dark<=255&&gain>=1&&gain<=1023&&value==0,"Invalid column/dark/Q2.8 gain");
    else if(op==5)require(col==0&&dark==0&&gain==0&&value<=1,"Invalid gamma value");
    else require(col==0&&dark==0&&gain==0&&value==0,"Unused request fields must be zero");
    Body b{};b[0]='F';b[1]='P';b[2]='C';b[3]='1';b[4]=1;b[5]=uint8_t(op);b[6]=uint8_t(eye);
    put32(b,8,id);put16(b,12,col);b[14]=uint8_t(dark);b[15]=uint8_t(value);put16(b,16,gain);return b;
}
std::vector<uint8_t> frame(const Body& b,const Mac& source,const Mac& target) {
    std::vector<uint8_t> f(80);std::copy(target.begin(),target.end(),f.begin());std::copy(source.begin(),source.end(),f.begin()+6);
    f[12]=0x88;f[13]=0xbb;std::copy(b.begin(),b.end(),f.begin()+14);unsigned c=crc16(b.data(),64);f[78]=uint8_t(c>>8);f[79]=uint8_t(c);return f;
}
bool decode(const uint8_t* f,size_t n,const Mac& target,const Mac& host,Body& b) {
    if(n!=80||!std::equal(target.begin(),target.end(),f+6)||f[12]!=0x88||f[13]!=0xbb)return false;
    if(!std::equal(host.begin(),host.end(),f)&&!std::all_of(f,f+6,[](uint8_t v){return v==255;}))return false;
    std::copy(f+14,f+78,b.begin());
    if(crc16(b.data(),64)!=(unsigned(f[78])<<8|f[79]))return false;
    if(b[0]!='F'||b[1]!='P'||b[2]!='C'||b[3]!='1'||b[4]!=1||b[5]<1||b[5]>6||b[6]>1||b[7]!=0x80)return false;
    return b[23]==0&&std::all_of(b.begin()+44,b.end(),[](uint8_t v){return v==0;});
}
bool matches(const Body& b,const Body& r) {
    return b[5]==r[5]&&b[6]==r[6]&&std::equal(b.begin()+8,b.begin()+18,r.begin()+8);
}
const char* status_name(unsigned s) {
    static const char* names[]={"ok","fpn_transaction_state","column_order","gain_out_of_range","table_incomplete","busy","internal_ack_mismatch","request_id_conflict","stale_request_id"};
    return s<9?names[s]:"unknown_status";
}
picojson::value ack_json(const Body& b) {
    picojson::object o;
    auto num=[&](const char* k,uint32_t x){o[k]=picojson::value(double(x));};
    num("op",b[5]);num("eye",b[6]);num("request_id",be32(b,8));num("column",be16(b,12));num("dark",b[14]);num("value",b[15]);num("gain_q8",be16(b,16));num("status",b[18]);
    o["status_name"]=picojson::value(std::string(status_name(b[18])));o["table_valid"]=picojson::value(bool(b[19]&1));
    num("expected_column",be16(b,20));o["gamma_enabled"]=picojson::value(bool(b[22]&1));
    num("table_generation",be32(b,24));num("applied_frame",be32(b,28));num("duplicate_count",be32(b,32));num("accepted_commands",be32(b,36));num("last_request_id",be32(b,40));return picojson::value(o);
}
unsigned integer(const picojson::value& v,unsigned lo,unsigned hi,const char* name) {
    require(v.is<double>(),std::string(name)+" must be integer");double d=v.get<double>();
    require(std::isfinite(d)&&std::floor(d)==d&&d>=lo&&d<=hi,std::string(name)+" out of range");return unsigned(d);
}
std::vector<Coeff> coefficients(const std::string& file,unsigned eye) {
    std::ifstream in(file,std::ios::binary);require(bool(in),"Cannot read coefficient JSON: "+file);
    in.seekg(0,std::ios::end);auto size=in.tellg();require(size>0&&size<=1024*1024,"Coefficient JSON empty or larger than 1 MiB");in.seekg(0);
    std::string text((std::istreambuf_iterator<char>(in)),{});picojson::value doc;auto err=picojson::parse(doc,text);require(err.empty(),"Invalid JSON: "+err);require(doc.is<picojson::object>(),"JSON object required");
    require(integer(doc.get("width"),Width,Width,"width")==Width,"Width mismatch");
    const auto& e=doc.get("eye");if(!e.is<picojson::null>()) {
        if(e.is<std::string>())require(e.get<std::string>()==(eye?"right":"left"),"Coefficient eye mismatch");
        else require(integer(e,0,1,"eye")==eye,"Coefficient eye mismatch");
    }
    const auto& sv=doc.get("schema");require(sv.is<std::string>(),"Missing coefficient schema");const auto schema=sv.get<std::string>();
    std::vector<Coeff> result;result.reserve(Width);
    if(schema=="FPN-COLUMN-RAW8-Q2.8-v1") {
        integer(doc.get("raw_bits"),8,8,"raw_bits");integer(doc.get("gain_fraction_bits"),8,8,"gain_fraction_bits");
        const auto& d=doc.get("dark");const auto& g=doc.get("gain");require(d.is<picojson::array>()&&g.is<picojson::array>(),"Dark/gain arrays required");
        require(d.get<picojson::array>().size()==Width&&g.get<picojson::array>().size()==Width,"Require 960 dark/gain values");
        for(unsigned i=0;i<Width;++i)result.push_back({integer(d.get(i),0,255,"dark"),integer(g.get(i),1,1023,"gain")});
    } else if(schema=="fpc1-column-q8-v1") {
        const auto& c=doc.get("columns");require(c.is<picojson::array>()&&c.get<picojson::array>().size()==Width,"Require 960 columns");
        for(const auto& v:c.get<picojson::array>()) {
            require(v.is<picojson::object>()&&v.get<picojson::object>().size()==2,"Column requires dark/gain_q8 only");
            result.push_back({integer(v.get("dark"),0,255,"dark"),integer(v.get("gain_q8"),1,1023,"gain_q8")});
        }
    } else throw std::runtime_error("Unknown coefficient schema");
    return result;
}
class Link {
    int fd_=-1;Mac host_{};Mac target_;double timeout_;unsigned retries_;uint32_t next_=0;
public:
    Link(const std::string& iface,Mac target,double timeout,unsigned retries):target_(target),timeout_(timeout),retries_(retries) {
        require(!iface.empty()&&iface.size()<IFNAMSIZ&&iface.find('/')==std::string::npos,"Invalid network interface");
        unsigned index=if_nametoindex(iface.c_str());require(index!=0,"Network interface not found: "+iface);
        fd_=socket(AF_PACKET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,htons(EtherType));
        if(fd_<0)throw std::runtime_error("Raw socket failed (run as root): "+std::string(strerror(errno)));
        struct ifreq ifr{};std::copy(iface.begin(),iface.end(),ifr.ifr_name);
        if(ioctl(fd_,SIOCGIFHWADDR,&ifr)<0){auto e=errno;close(fd_);fd_=-1;throw std::runtime_error(strerror(e));}
        std::copy_n(reinterpret_cast<uint8_t*>(ifr.ifr_hwaddr.sa_data),6,host_.begin());
        struct sockaddr_ll addr{};addr.sll_family=AF_PACKET;addr.sll_protocol=htons(EtherType);addr.sll_ifindex=int(index);
        if(bind(fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0){auto e=errno;close(fd_);fd_=-1;throw std::runtime_error(strerror(e));}
    }
    Link(const Link&)=delete;Link& operator=(const Link&)=delete;
    ~Link(){if(fd_>=0)close(fd_);}
    Body transact(const Body& r) {
        const auto wire=frame(r,host_,target_);
        for(unsigned attempt=0;attempt<retries_;++attempt) {
            ssize_t sent;do {sent=send(fd_,wire.data(),wire.size(),0);}while(sent<0&&errno==EINTR);
            require(sent==ssize_t(wire.size()),"FPC1 send failed: "+std::string(strerror(errno)));
            auto deadline=Clock::now()+std::chrono::duration<double>(timeout_);
            while(Clock::now()<deadline) {
                auto left=std::chrono::duration<double,std::milli>(deadline-Clock::now()).count();
                int ms=std::max(1,int(std::ceil(left)));struct pollfd pfd{fd_,POLLIN,0};int ready=poll(&pfd,1,ms);
                if(ready<0&&errno==EINTR)continue;
                require(ready>=0,"FPC1 poll failed");if(ready==0)break;
                require(!(pfd.revents&(POLLERR|POLLHUP|POLLNVAL)),"FPC1 network socket error");
                std::array<uint8_t,2048> buffer{};struct sockaddr_ll addr{};socklen_t len=sizeof(addr);
                ssize_t n=recvfrom(fd_,buffer.data(),buffer.size(),0,reinterpret_cast<sockaddr*>(&addr),&len);
                if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR))continue;
                require(n>=0,"FPC1 receive failed");if(addr.sll_pkttype==PACKET_OUTGOING)continue;
                Body ack{};if(!decode(buffer.data(),size_t(n),target_,host_,ack)||!matches(ack,r))continue;
                require(ack[18]==0,"FPGA rejected request: "+ack_json(ack).serialize());return ack;
            }
        }
        throw std::runtime_error("No FPC1 ACK after "+std::to_string(retries_)+" identical retries; no further writes sent. Check v40 bitstream and internal eth0.");
    }
    Body synchronize(unsigned eye) {
        uint32_t id=uint32_t(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());if(!id)id=1;
        auto ack=transact(request(6,eye,id));next_=next_id(be32(ack,40));return ack;
    }
    Body command(unsigned op,unsigned eye,unsigned col=0,unsigned dark=0,unsigned gain=0,unsigned value=0) {
        require(next_!=0,"INFO synchronization required");auto r=request(op,eye,next_,col,dark,gain,value);auto ack=transact(r);next_=next_id(next_);return ack;
    }
};
void self_test() {
    const std::string check="123456789";require(crc16(reinterpret_cast<const uint8_t*>(check.data()),check.size())==0x29b1,"CRC16 self-test");
    auto host=mac("02:ac:13:80:00:02");Body ack=request(2,1,0x12345678,959,255,1023);ack[7]=0x80;ack[19]=1;put16(ack,20,960);
    auto f=frame(ack,Target,host);Body got{};require(decode(f.data(),f.size(),Target,host,got)&&got==ack,"ACK self-test");
    f[78]^=1;require(!decode(f.data(),f.size(),Target,host,got),"Bad CRC rejection");
    require(next_id(0xffffffff)==1&&next_id(1)==2,"ID wrap self-test");
}
int application(int argc,char** argv) {
    std::string iface="eth0",cmd,file;Mac target=Target;unsigned eye=0,retries=10;double timeout=0.5;std::vector<std::string> pos;
    for(int i=1;i<argc;++i) {
        std::string s=argv[i];auto value=[&](){require(i+1<argc,"Missing value for "+s);return std::string(argv[++i]);};
        if(s=="--help"||s=="-h") {
            std::cout<<"FPC1 v40.1 native ARM64 (no Python), calibration only\nUsage: fpc1 [--interface eth0] [--timeout 0.5] [--retries 10] COMMAND [--eye 0|1]\nCommands: info | gamma on|off | bypass | upload coefficients.json\nOffline: --self-test | --check coefficients.json --eye 0|1\nprepare: generate real RAW8 dark/flat coefficients on the PC with the existing Python tools.\n";return 0;
        } else if(s=="--self-test") {require(argc==2,"--self-test accepts no additional options");self_test();std::cout<<"PASS FPC1 CRC16 / frame codec / malformed ACK / request-ID wrap (no network)\n";return 0;}
        else if(s=="--interface")iface=value();
        else if(s=="--target")target=mac(value());
        else if(s=="--eye") {auto v=value();require(v=="0"||v=="1","eye must be 0 or 1");eye=unsigned(v[0]-'0');}
        else if(s=="--timeout") {auto v=value();size_t n=0;timeout=std::stod(v,&n);require(n==v.size(),"Invalid timeout");}
        else if(s=="--retries") {auto v=value();size_t n=0;double d=std::stod(v,&n);require(n==v.size()&&std::isfinite(d)&&std::floor(d)==d&&d>=1&&d<=100,"Retries must be 1..100");retries=unsigned(d);}
        else if(s=="--check") {require(cmd.empty(),"Only one command allowed");cmd="check";file=value();}
        else {require(s.empty()||s[0]!='-',"Unknown option: "+s);pos.push_back(s);}
    }
    require(std::isfinite(timeout)&&timeout>=0.05&&timeout<=30,"Timeout must be 0.05..30 seconds");
    if(cmd=="check") {require(pos.empty(),"Unexpected arguments");auto c=coefficients(file,eye);std::cout<<"PASS "<<c.size()<<" column coefficients, eye="<<eye<<" (no network)\n";return 0;}
    require(!pos.empty(),"Missing command; use --help");cmd=pos[0];
    require(cmd=="info"||cmd=="bypass"||cmd=="gamma"||cmd=="upload","Unknown command");
    require(pos.size()==((cmd=="gamma"||cmd=="upload")?2u:1u),"Incorrect command argument count");
    unsigned value=0;std::vector<Coeff> table;
    if(cmd=="gamma") {require(pos[1]=="on"||pos[1]=="off","gamma requires on or off");value=pos[1]=="on";}
    if(cmd=="upload")table=coefficients(pos[1],eye); // Validate everything before any socket or BEGIN.
    Link link(iface,target,timeout,retries);auto initial=link.synchronize(eye);Body ack{};
    if(cmd=="info")ack=initial;
    else if(cmd=="bypass")ack=link.command(4,eye);
    else if(cmd=="gamma")ack=link.command(5,eye,0,0,0,value);
    else {
        auto begin=link.command(1,eye);require(!(begin[19]&1)&&be16(begin,20)==0,"BEGIN did not disable table/reset column counter; upload aborted");
        for(unsigned col=0;col<Width;++col) {
            auto w=link.command(2,eye,col,table[col].dark,table[col].gain);require(be16(w,20)==col+1,"WRITE column counter mismatch; COMMIT not sent");
            if((col+1)%120==0)std::cerr<<"Uploaded eye "<<eye<<": "<<col+1<<"/960\n";
        }
        ack=link.command(3,eye);std::cerr<<"COMMIT accepted; query INFO after next camera SOF to confirm table application.\n";
    }
    std::cout<<ack_json(ack).serialize(true)<<'\n';return 0;
}
} // namespace fpc1
#ifndef FPC1_LIBRARY_ONLY
int main(int argc,char** argv) {try{return fpc1::application(argc,argv);}catch(const std::exception& e){std::cerr<<"FPC1 error: "<<e.what()<<'\n';return 1;}}
#endif
