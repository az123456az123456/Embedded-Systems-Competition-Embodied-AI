#include "../gateway.h"
#include "../vendor/http_server.h"
#include <arpa/inet.h>
#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

static unsigned free_port(){
    int fd=socket(AF_INET,SOCK_STREAM,0);assert(fd>=0);
    sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    assert(bind(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))==0);
    socklen_t n=sizeof(a);assert(getsockname(fd,reinterpret_cast<sockaddr*>(&a),&n)==0);
    unsigned port=ntohs(a.sin_port);close(fd);return port;
}
int main(){
    auto backend_port=free_port(),public_port=free_port();while(public_port==backend_port)public_port=free_port();
    std::mutex mutex;std::string clock="\"timestamp_ms\":4294967295,";
    robotphone::HttpServer backend;std::string error;
    assert(backend.start("127.0.0.1",backend_port,[&](int fd,const robotphone::HttpRequest&){
        std::string source;{std::lock_guard<std::mutex> lock(mutex);source=clock;}
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        robotphone::httpReply(fd,200,"{\"ok\":true,\"imu_action_pending\":false,\"motor\":{\"velocity_protocol_supported\":true,\"softcore_ready\":true,\"ack_protocol\":4,\"link_ok\":true,\"left_mask\":10,\"invert_mask\":15,\"ack_age_ms\":5},\"imu\":{\"valid\":true,\"fresh\":true,\"bias_valid\":true,\"calibrating\":false,\"yaw\":179.9,\"roll\":0,\"pitch\":0,\"sequence\":100,"+source+"\"sample_age_ms\":10}}");
    },error));
    const auto token="/tmp/nav-v23-2-source-clock-"+std::to_string(getpid());
    {std::ofstream out(token);out<<"012345\n";}
    robotnav::GatewayOptions options;options.backend_port=backend_port;options.port=public_port;options.token_file=token;
    robotnav::Gateway gateway(options);assert(gateway.start(error));
    auto state=gateway.pollState();assert(state.available&&state.imu_valid&&state.imu_timestamp_valid);
    assert(state.imu_timestamp_ms==0xffffffffu&&state.imu_sequence==100);
    assert(state.request_started_ms>0&&state.received_ms>=state.request_started_ms+4);
    unsigned checks=5;
    for(const auto& field:{std::string(),std::string("\"timestamp_ms\":\"100\","),std::string("\"timestamp_ms\":-1,"),std::string("\"timestamp_ms\":1.5,"),std::string("\"timestamp_ms\":4294967296,")}){
        {std::lock_guard<std::mutex> lock(mutex);clock=field;}
        state=gateway.pollState();assert(state.available&&state.imu_valid&&!state.imu_timestamp_valid);++checks;
    }
    gateway.stop();backend.stop();unlink(token.c_str());
    std::cout<<"PASS "<<checks<<" real HTTP source-clock/timing validation checks\n";
}
