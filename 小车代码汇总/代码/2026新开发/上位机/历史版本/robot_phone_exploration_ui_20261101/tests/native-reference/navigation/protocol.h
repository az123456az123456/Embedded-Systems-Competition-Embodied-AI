#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#ifdef NAV_TEST_TRANSPORT
#include <functional>
#endif

namespace navwire {
constexpr uint16_t NAV_TYPE=0x88b9, SENSOR_TYPE=0x88b8;
using Bytes64=std::array<uint8_t,64>;
using Bitmap=std::array<uint8_t,512>;
struct Message {
    uint8_t op=0, status=0, count=0;
    uint32_t request=0, generation=0, arg0=0, arg1=0, arg2=0;
    std::array<uint8_t,32> data{};
};
uint16_t crc16(const uint8_t*,size_t);
uint32_t crc32(const uint8_t*,size_t);
Bytes64 encode(const Message&);
bool decode(const uint8_t*,size_t,Message&);
std::vector<uint8_t> frame(const uint8_t source[6],const Message&);
bool reply(const uint8_t*,size_t,const uint8_t host[6],Message&);
struct Sensors {
    uint32_t sequence=0,uptime_ms=0;
    uint8_t valid=0,fault=0,invert=0,illegal=0;
    std::array<int32_t,4> counts{},delta{};
    std::array<uint16_t,3> distance_mm{},age_ms{};
    uint16_t window_ms=0;
};
bool sensorFrame(const uint8_t*,size_t,const uint8_t host[6],Sensors&);
double monotonicSeconds();
std::string findInterface();
class RawSocket {
    int fd_=-1;
    std::array<uint8_t,6> mac_{};
public:
    RawSocket()=default;
    RawSocket(const RawSocket&)=delete;
    RawSocket& operator=(const RawSocket&)=delete;
    ~RawSocket();
    void open(const std::string&,uint16_t);
    void send(const std::vector<uint8_t>&);
    bool receive(std::vector<uint8_t>&,int timeout_ms,double* received_s=nullptr);
    const uint8_t* mac()const{return mac_.data();}
};
// Called only from one planner worker. Every transaction is correlated and
// bounded; cancellation stops retries and never emits a motor command.
class Planner {
    RawSocket socket_;
    uint32_t next_=1,generation_=0;
    Message exchange(Message,int,const std::atomic<bool>*);
public:
#ifdef NAV_TEST_TRANSPORT
    // Host-only seam. Not present in the shipped RK executable.
    std::function<Message(const Message&)> test_transport;
#endif
    void open(const std::string&);
    std::vector<uint16_t> plan(const Bitmap&,uint16_t,uint16_t,
                              const std::atomic<bool>* cancel=nullptr);
};
}
