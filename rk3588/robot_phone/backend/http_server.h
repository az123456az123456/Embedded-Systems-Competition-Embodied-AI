#ifndef ROBOT_PHONE_HTTP_SERVER_H
#define ROBOT_PHONE_HTTP_SERVER_H
#include <atomic>
#include <condition_variable>
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
