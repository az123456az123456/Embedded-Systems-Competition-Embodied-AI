#include "http_server.h"
#include <array>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
static uint64_t now(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(){int fd[2];if(socketpair(AF_UNIX,SOCK_STREAM,0,fd))return 2;int capacity=4096;setsockopt(fd[0],SOL_SOCKET,SO_SNDBUF,&capacity,sizeof(capacity));std::array<char,1024*1024> bytes{};
 const auto begin=now();bool written=robotphone::writeAllBefore(fd[0],bytes.data(),bytes.size(),begin+80);const auto elapsed=now()-begin;
 if(written||elapsed<70||elapsed>250)throw std::runtime_error("slow stream did not obey absolute deadline");
 close(fd[0]);close(fd[1]);std::printf("PASS slow receiver whole-frame deadline elapsed_ms=%llu; no physical hardware\n",(unsigned long long)elapsed);
}
