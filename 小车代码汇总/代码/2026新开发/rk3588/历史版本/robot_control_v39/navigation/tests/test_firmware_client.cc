#include "../protocol.h"
extern "C" {
#include "nav_protocol.h"
}
#include <cassert>
#include <iostream>
#include <memory>
#include <stdexcept>
int main(){
    auto server=std::make_unique<nav_server>();nav_server_init(server.get());
    navwire::Planner client;unsigned calls=0;
    client.test_transport=[&](const navwire::Message&q){
        auto b=navwire::encode(q);uint8_t response[64]{};
        assert(nav_process(server.get(),b.data(),response,nullptr));
        navwire::Message r;assert(navwire::decode(response,64,r));++calls;return r;
    };
    navwire::Bitmap map{};
    auto p=client.plan(map,0,4095);assert(p.size()==127&&p.front()==0&&p.back()==4095);assert(calls==28);
    // A second full update must use a new generation and monotonically advance
    // request IDs learned from the actual firmware, not a hardcoded sequence.
    auto previous=server->previous_id;for(int y=0;y<64;++y)if(y!=45){unsigned i=y*64+32;map[i/8]|=1u<<(i%8);}
    p=client.plan(map,30*64+10,30*64+50);assert(p.size()==71);assert(server->previous_id>previous);
    bool passes=false;for(auto c:p)if(c==45*64+32)passes=true;assert(passes);
    unsigned barrier=45*64+32;map[barrier/8]|=1u<<(barrier%8);bool failed=false;
    try{client.plan(map,30*64+10,30*64+50);}catch(const std::runtime_error&){failed=true;}assert(failed);
    nav_server_init(server.get());map.fill(0);p=client.plan(map,100,100);assert(p.size()==1&&p[0]==100);
    std::atomic<bool> cancel{true};failed=false;try{client.plan(map,0,4095,&cancel);}catch(const std::runtime_error&){failed=true;}assert(failed);
    std::cout<<"PASS actual RK uploader/client -> actual FPGA firmware: full upload/commit/path pages, second map, barrier detour, no route, FPGA reset, cancellation\n";
}
