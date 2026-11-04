#pragma once
#include "acg138_fpga_map_bridge/nav3_wire.hpp"
namespace acg138::nav3 {
class RawEthernet final : public Channel {
  int fd_=-1, lock_=-1;
  std::array<std::uint8_t,6> mac_{};
  std::string proc_;
public:
  explicit RawEthernet(const std::string& interface,const std::string& proc="/proc");
  ~RawEthernet() override;
  RawEthernet(const RawEthernet&)=delete;
  RawEthernet& operator=(const RawEthernet&)=delete;
  void send(const Wire&) override;
  bool receive(Wire&,unsigned timeout_ms) override;
};
std::uint64_t random_session();
} // namespace acg138::nav3
