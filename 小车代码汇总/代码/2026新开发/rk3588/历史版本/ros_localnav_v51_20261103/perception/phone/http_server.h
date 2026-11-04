#ifndef ROBOT_PHONE_HTTP_SERVER_H
#define ROBOT_PHONE_HTTP_SERVER_H
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
namespace robotphone {
struct HttpRequest {std::string method,path,body;std::map<std::string,std::string> headers;};
bool parseHttpHead(const std::string &,HttpRequest &,size_t &body_size,std::string &error);
bool writeAll(int,const void *,size_t);
// A whole multipart frame shares one monotonic deadline; a slow receiver must
// reconnect to the latest frame instead of accumulating seconds of TCP data.
bool writeAllBefore(int,const void *,size_t,uint64_t deadline_ms);
void httpReply(int,int,const std::string &,const std::string &type="application/json; charset=utf-8");
class HttpServer {
public:
 using Handler=std::function<void(int,const HttpRequest &)>;
 ~HttpServer();
 bool start(const std::string &address,unsigned port,Handler,std::string &error);
 void stop();
private:
 int listen_=-1;std::atomic<bool> stopped_{true};std::thread acceptor_;std::vector<std::thread> workers_;
 std::mutex mutex_;std::condition_variable ready_;std::deque<int> queue_;std::set<int> active_;Handler handler_;
 void acceptLoop();void worker();
};
}
#endif
