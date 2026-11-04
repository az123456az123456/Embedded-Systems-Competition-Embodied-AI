#define FPC1_LIBRARY_ONLY
#include "../src/fpc1.cpp"
#include <iomanip>
int main(int argc,char** argv) {
    try {
        fpc1::require(argc>=2,"probe mode required");std::string mode=argv[1];
        if(mode=="encode") {
            fpc1::require(argc==10,"encode args");std::array<unsigned long,8> p{};
            for(int i=0;i<8;++i)p[i]=std::stoul(argv[i+2]);
            auto b=fpc1::request(unsigned(p[0]),unsigned(p[1]),uint32_t(p[2]),unsigned(p[3]),unsigned(p[4]),unsigned(p[5]),unsigned(p[6]));
            auto f=fpc1::frame(b,fpc1::mac("02:ac:13:80:00:02"),fpc1::Target);
            for(auto c:f)std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(c);
            std::cout<<'\n';return 0;
        }
        if(mode=="decode") {
            std::string hex;std::cin>>hex;fpc1::require(hex.size()%2==0,"odd hex");std::vector<uint8_t> f;
            for(size_t i=0;i<hex.size();i+=2)f.push_back(uint8_t(std::stoul(hex.substr(i,2),nullptr,16)));
            fpc1::Body b{};fpc1::require(fpc1::decode(f.data(),f.size(),fpc1::Target,fpc1::mac("02:ac:13:80:00:02"),b),"invalid ACK");
            std::cout<<fpc1::ack_json(b).serialize()<<'\n';return 0;
        }
        return 2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
