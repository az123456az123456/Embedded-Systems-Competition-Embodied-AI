#pragma once
#include "acg138_fpga_map_bridge/map_core.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace acg138::wire {
constexpr uint16_t kEtherType=0x88b9;
using Mac=std::array<uint8_t,6>;
using Body=std::array<uint8_t,64>;
constexpr Mac kFpgaMac{{0x02,0xac,0x13,0x80,0,1}};
struct Message {
  uint8_t op=0,status=0,count=0;
  uint32_t request=0,generation=0,arg0=0,arg1=0,arg2=0;
  std::array<uint8_t,32> data{};
};
enum Op : uint8_t { Begin=1,Chunk=2,Commit=3,Plan=4,Page=5,Info=6 };
uint16_t crc16(const uint8_t *,size_t);
uint32_t crc32(const uint8_t *,size_t);
Body encode(const Message &);
bool decode(const Body &,Message &);
std::vector<uint8_t> ethernet(const Mac &source,const Message &);
bool decode_reply(const uint8_t *,size_t,const Mac &host,Message &);
// Swappable transport boundary; only this implementation uses AF_PACKET.
// Returned body has already passed link-envelope/FCS-independent CRC checks.
class Channel {
public:
  virtual ~Channel()=default;
  virtual void send(const Body &)=0;
  virtual bool receive(Body &,unsigned timeout_ms)=0;
};
class RawEthernet final : public Channel {
  int fd_=-1,lock_=-1;
  Mac mac_{};
  std::string host_proc_root_;
public:
  explicit RawEthernet(const std::string &interface,const std::string &host_proc_root="/proc");
  ~RawEthernet() override;
  void send(const Body &) override;
  bool receive(Body &,unsigned timeout_ms) override;
};
std::string find_interface();
bool legacy_navigation_running(const std::string &host_proc_root="/proc");
struct Statistics { uint64_t sent=0,retries=0,ignored=0; };
class Client {
  Channel &channel_;
  uint32_t next_=1,generation_=0;
  bool committed_=false;
  Statistics stats_;
  Message exchange(const Message &,unsigned timeout_ms);
  Message request(uint8_t op);
public:
  explicit Client(Channel &c):channel_(c){}
  // INFO synchronizes the global firmware request ID. An external client must
  // not use this ABI concurrently; there is one map/one previous-request cache.
  uint32_t upload(const Bitmap &);
  std::vector<uint16_t> plan(const Bitmap &,uint16_t start,uint16_t goal);
  uint32_t generation()const{return generation_;}
  const Statistics &statistics()const{return stats_;}
};
}  // namespace acg138::wire
