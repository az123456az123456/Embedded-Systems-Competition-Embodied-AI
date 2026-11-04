#pragma once
#include "nav_authority.h"
#include <string>
namespace robotnav {
// Separate protocol-family sender. It never takes or bypasses the map flock.
class NvaSocket {
 int fd_=-1,lock_=-1;unsigned index_=0;std::array<uint8_t,6>mac_{};
public:
 NvaSocket()=default;~NvaSocket();
 NvaSocket(const NvaSocket&)=delete;NvaSocket&operator=(const NvaSocket&)=delete;
 bool open(const std::string&,std::string& error);
 bool send(const acg138::nav3::Wire&);
 bool receive(acg138::nav3::Wire&);
 bool available()const{return fd_>=0;}
};
}
