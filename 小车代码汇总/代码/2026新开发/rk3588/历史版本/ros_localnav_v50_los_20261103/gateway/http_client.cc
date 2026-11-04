#include "http_client.h"
#include <arpa/inet.h>
#include <chrono>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <algorithm>
namespace robotnav {
HttpResult dashboard_request(unsigned port,const std::string&token,const std::string&method,const std::string&path,const std::string&body,unsigned timeout){
 HttpResult result;if(port<1024||port>65535||body.size()>16384||token.find_first_of("\r\n")!=std::string::npos)return result;
 const int fd=socket(AF_INET,SOCK_STREAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0);if(fd<0)return result;
 struct Close{int fd;~Close(){::close(fd);}}close{fd};
 const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout);
 auto wait=[&](short events){const auto left=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()).count();if(left<=0)return false;pollfd p{fd,events,0};return poll(&p,1,int(left))>0&&!(p.revents&(POLLERR|POLLNVAL));};
 sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
 if(connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0){if(errno!=EINPROGRESS||!wait(POLLOUT))return result;int error=0;socklen_t size=sizeof(error);if(getsockopt(fd,SOL_SOCKET,SO_ERROR,&error,&size)||error)return result;}
 const std::string bytes=method+" "+path+" HTTP/1.1\r\nHost: 127.0.0.1\r\nAuthorization: Bearer "+token+"\r\nConnection: close\r\nContent-Type: application/octet-stream\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
 size_t sent=0;while(sent<bytes.size()){auto n=send(fd,bytes.data()+sent,bytes.size()-sent,MSG_DONTWAIT|MSG_NOSIGNAL);if(n>0)sent+=size_t(n);else if(n<0&&(errno==EINTR||errno==EAGAIN)){if(!wait(POLLOUT))return result;}else return result;}
 std::string response;
 while(response.size()<=262144){char b[8192];auto n=recv(fd,b,sizeof(b),MSG_DONTWAIT);if(n>0)response.append(b,size_t(n));else if(n==0)break;else if(errno==EINTR)continue;else if(errno==EAGAIN){if(!wait(POLLIN))return result;}else return result;}
 const auto split=response.find("\r\n\r\n"),space=response.find(' ');
 if(split==std::string::npos||space==std::string::npos||response.size()>262144)return result;
 try{result.status=std::stoi(response.substr(space+1,3));}catch(...){return result;}
 result.body=response.substr(split+4);result.transport_ok=true;return result;
}
}
