#include "navigation_map.h"
#include "picojson.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
namespace {
constexpr size_t MAX_JSON_BYTES=512*1024;
const picojson::value *field(const picojson::object &o,const char *key) {
    auto i=o.find(key);return i==o.end()?nullptr:&i->second;
}
bool number(const picojson::value *v,double &n) {
    if(!v||!v->is<double>())return false;
    n=v->get<double>();return std::isfinite(n);
}
bool integer(const picojson::value *v,uint64_t &n) {
    double d;if(!number(v,d)||d<0||d>9007199254740991.0||std::floor(d)!=d)return false;
    n=(uint64_t)d;return true;
}
bool boolean(const picojson::object &o,const char *key,int &out) {
    auto v=field(o,key);if(!v||!v->is<bool>())return false;out=v->get<bool>();return true;
}
bool vector(const picojson::value *v,double *out,size_t n) {
    if(!v||!v->is<picojson::array>()||v->get<picojson::array>().size()!=n)return false;
    const auto &a=v->get<picojson::array>();
    for(size_t i=0;i<n;i++)if(!number(&a[i],out[i])||std::fabs(out[i])>1e6)return false;
    return true;
}
bool points(const picojson::value *v,double out[][2],unsigned &count) {
    if(!v||!v->is<picojson::array>())return false;
    const auto &a=v->get<picojson::array>();if(a.size()>NAV_MAP_POINTS)return false;
    count=(unsigned)a.size();for(unsigned i=0;i<count;i++)if(!vector(&a[i],out[i],2))return false;
    return true;
}
bool string(const picojson::object &o,const char *key,char *out,size_t size,bool required) {
    auto v=field(o,key);if(!v)return !required;if(!v->is<std::string>())return false;
    auto &s=v->get<std::string>();if(s.size()>=size)return false;
    std::memcpy(out,s.c_str(),s.size()+1);return true;
}
}
extern "C" int navigation_map_parse(const char *data,size_t length,struct navigation_map *out) {
    if(!data||!out||length>MAX_JSON_BYTES)return 0;
    try {
        picojson::value root;std::string error;
        auto end=picojson::parse(root,data,data+length,&error);
        while(end<data+length&&(*end==' '||*end=='\n'||*end=='\r'||*end=='\t'))++end;
        if(!error.empty()||end!=data+length||!root.is<picojson::object>())return 0;
        const auto &o=root.get<picojson::object>();
        auto m=std::make_unique<navigation_map>();uint64_t schema=0,w=0,h=0;double sensor_age=-1;
        if(!integer(field(o,"schema"),schema)||schema!=1||!integer(field(o,"width"),w)||w!=64||
           !integer(field(o,"height"),h)||h!=64||!number(field(o,"resolution_m"),m->resolution_m)||
           m->resolution_m<.01||m->resolution_m>1||!vector(field(o,"origin"),m->origin,2)||
           !vector(field(o,"pose"),m->pose,3)||!boolean(o,"available",m->available)||
           !boolean(o,"pose_valid",m->pose_valid)||!integer(field(o,"updated_monotonic_ms"),m->updated_monotonic_ms)||
           !integer(field(o,"map_revision"),m->map_revision)||!number(field(o,"sensor_age_ms"),sensor_age)||
           sensor_age < -1 || sensor_age > 9007199254740991.0 || std::floor(sensor_age)!=sensor_age ||
           !string(o,"state",m->state,sizeof(m->state),true)||!string(o,"reason",m->reason,sizeof(m->reason),false)||
           !string(o,"session_id",m->session_id,sizeof(m->session_id),true)||!m->session_id[0]||
           !points(field(o,"path"),m->path,m->path_count)||!points(field(o,"trajectory"),m->trajectory,m->trajectory_count))return 0;
        auto cells=field(o,"cells"),goal=field(o,"goal");
        if(!cells||!cells->is<std::string>()||cells->get<std::string>().size()!=4096||!goal)return 0;
        const auto &s=cells->get<std::string>();for(char c:s)if(c!='?'&&c!='.'&&c!='#')return 0;
        std::memcpy(m->cells,s.c_str(),4097);m->width=64;m->height=64;
        if(!goal->is<picojson::null>()) {if(!vector(goal,m->goal,2))return 0;m->has_goal=1;}
        auto radius=field(o,"robot_radius_m");
        if(radius&&(!number(radius,m->robot_radius_m)||m->robot_radius_m<0||m->robot_radius_m>2))return 0;
        m->sensor_age_ms=(int64_t)sensor_age;m->loaded=1;*out=*m;return 1;
    } catch(...) { return 0; }
}
extern "C" int navigation_map_read(const char *path,struct navigation_map *out) {
    if(!path||!out)return 0;
    int fd=open(path,O_RDONLY|O_CLOEXEC|O_NONBLOCK|O_NOFOLLOW);if(fd<0)return 0;
    struct stat st{};
    if(fstat(fd,&st)||!S_ISREG(st.st_mode)||st.st_size<=0||st.st_size>(off_t)MAX_JSON_BYTES){close(fd);return 0;}
    std::string data((size_t)st.st_size,'\0');size_t at=0;
    while(at<data.size()) {ssize_t n=read(fd,&data[at],data.size()-at);if(n<=0){close(fd);return 0;}at+=(size_t)n;}
    close(fd);return navigation_map_parse(data.data(),data.size(),out);
}
struct navigation_reader {
    std::string path;std::mutex mutex;std::condition_variable wake;std::thread worker;
    std::atomic<bool> stop{false};navigation_map map{};
};
extern "C" struct navigation_reader *navigation_reader_start(const char *path) {
    if(!path||!*path)return nullptr;
    try {
        auto reader=std::make_unique<navigation_reader>();reader->path=path;
        auto *r=reader.get();r->worker=std::thread([r] {
            auto next=std::make_unique<navigation_map>();
            while(!r->stop.load()) {
                bool okay=navigation_map_read(r->path.c_str(),next.get());
                std::unique_lock<std::mutex> lock(r->mutex);
                if(okay)r->map=*next;
                else {r->map.available=0;r->map.pose_valid=0;std::snprintf(r->map.reason,sizeof(r->map.reason),"map_file_unavailable");}
                r->wake.wait_for(lock,std::chrono::milliseconds(500),[r]{return r->stop.load();});
            }
        });return reader.release();
    } catch(...) {return nullptr;}
}
extern "C" void navigation_reader_stop(struct navigation_reader *r) {
    if(!r)return;
    r->stop.store(true);r->wake.notify_all();if(r->worker.joinable())r->worker.join();delete r;
}
extern "C" int navigation_reader_snapshot(struct navigation_reader *r,struct navigation_map *out) {
    if(!r||!out)return 0;
    std::unique_lock<std::mutex> lock(r->mutex,std::try_to_lock);
    if(!lock.owns_lock())return 0;
    *out=r->map;return 1;
}
