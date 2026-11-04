#pragma once
#include "fpga_delegation.h"
#include "nva_socket.h"
namespace robotnav {
class PhoneNavAdapter {
 NvaSocket socket_;FpgaDelegation delegation_;
 bool active_=false;std::string error_;
public:
 PhoneNavAdapter();
 bool open(const std::string& interface);
 bool active()const{return active_;}
 bool quiet()const{return active_&&delegation_.quiet();}
 bool ready(uint64_t now)const{return active_&&delegation_.healthy(now);}
 void owner(const std::string&,uint64_t seq,uint64_t source,bool armed,uint64_t now);
 void tick(uint64_t now);
 void takeover();
 bool request(const std::string& method,const std::string& path,const std::string& body,
              const std::string& phone_session,bool idle,uint64_t now,int&code,std::string& response);
};
}
