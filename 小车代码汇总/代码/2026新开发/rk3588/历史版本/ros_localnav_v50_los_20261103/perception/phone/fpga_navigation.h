#pragma once
#ifdef HAVE_FPGA_NAV
#include "../../gateway/phone_nav_adapter.h"
using FpgaNavigationAdapter=robotnav::PhoneNavAdapter;
#else
#include <cstdint>
#include <string>
struct FpgaNavigationAdapter {
 bool open(const std::string&){return false;}
 bool active()const{return false;}bool quiet()const{return false;}bool ready(uint64_t)const{return false;}
 void owner(const std::string&,uint64_t,uint64_t,bool,uint64_t){}void tick(uint64_t){}void takeover(){}
 bool request(const std::string&,const std::string&,const std::string&,const std::string&,bool,uint64_t,int&,std::string&){return false;}
};
#endif
