#include "wrong_log.h"
#include "../navigation/vendor/picojson.h"
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
using Obj = picojson::object;
constexpr size_t kPayloadMax = 16384, kHistoryMax = 512, kQueueMax = 1024;
constexpr uint64_t kHistoryMs = 10000, kPostMs = 5000;
uint64_t mono_ms() { return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count(); }
std::string utc_now() {
    auto n=std::chrono::system_clock::now();
    auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(n.time_since_epoch()).count();
    time_t sec=static_cast<time_t>(ms/1000); struct tm tm{}; gmtime_r(&sec,&tm);
    char b[48]; std::strftime(b,sizeof(b),"%Y-%m-%dT%H:%M:%S",&tm);
    char out[64]; std::snprintf(out,sizeof(out),"%s.%03lldZ",b,static_cast<long long>(ms%1000)); return out;
}
picojson::value number(uint64_t x) { return picojson::value(static_cast<double>(x)); }
std::string bounded(const char *s,size_t max) { if(!s)return ""; return std::string(s,strnlen(s,max)); }
bool sensitive(const std::string &k) {
    std::string s; for(unsigned char c:k) if((c>='a'&&c<='z')||(c>='0'&&c<='9'))s+=char(c);else if(c>='A'&&c<='Z')s+=char(c-'A'+'a');
    const char *keys[]={"token","session","authorization","password","passwd","paircode","pairingcode","secret","credential","cookie","apikey","accesskey","privatekey","bearer"};
    for(auto key:keys)if(s.find(key)!=std::string::npos)return true;
    return false;
}
void scrub(picojson::value &v,unsigned depth=0) {
    if(depth>32) { v=picojson::value("[depth limit]"); return; }
    if(v.is<Obj>()) { auto &o=v.get<Obj>(); for(auto it=o.begin();it!=o.end();) { if(sensitive(it->first))it=o.erase(it);else {scrub(it->second,depth+1);++it;} } }
    else if(v.is<picojson::array>())for(auto &x:v.get<picojson::array>())scrub(x,depth+1);
}
bool too_deep(const char *json) {
    unsigned depth=0; bool quoted=false,escaped=false;
    for(const char *p=json;*p;++p) {
        if(quoted) {if(escaped)escaped=false;else if(*p=='\\')escaped=true;else if(*p=='"')quoted=false;}
        else if(*p=='"')quoted=true;
        else if(*p=='{'||*p=='[') {if(++depth>32)return true;}
        else if((*p=='}'||*p==']')&&depth)--depth;
    }return false;
}
std::shared_ptr<const picojson::value> payload(const char *json) {
    picojson::value v; Obj invalid;
    if(!json || strnlen(json,kPayloadMax+1)>kPayloadMax) { invalid["invalid_json"]=picojson::value(true); invalid["payload_too_large"]=picojson::value(json!=nullptr); v=picojson::value(invalid); }
    else if(too_deep(json)) {invalid["invalid_json"]=picojson::value(true);invalid["depth_limit"]=picojson::value(true);v=picojson::value(invalid);}
    else { std::string err=picojson::parse(v,std::string(json)); if(!err.empty()) {invalid["invalid_json"]=picojson::value(true);v=picojson::value(invalid);}else scrub(v); }
    return std::make_shared<const picojson::value>(std::move(v));
}
struct Sample {uint64_t at=0;std::string wall,component;std::shared_ptr<const picojson::value> data;uint64_t capture_seq=0;};
struct Record {Sample sample;uint64_t seq=0;std::string kind,phase,reason,incident;std::vector<std::string> related;};
class LogState {
    std::mutex mu_; std::condition_variable cv_; std::deque<Sample> history_; std::deque<Record> queue_;
    std::map<std::string,uint64_t> post_at_; std::map<std::string,uint64_t> dedup_;
    std::deque<std::pair<std::string,uint64_t>> post_incidents_;
    std::atomic<bool> stop_{false}; uint64_t seq_=0,incident_seq_=0,post_until_=0,capture_seq_=0,last_history_capture_=0;
    std::string process_,version_,path_,lock_path_,rotated1_,rotated2_,run_,incident_,boot_; pid_t pid_;
    size_t max_bytes_=5*1024*1024;
    uint64_t queue_dropped_=0,history_dropped_=0,deduplicated_=0,io_failures_=0,io_dropped_=0,producer_errors_=0;
    uint64_t reported_drops_=0,reported_dedup_=0,reported_io_=0,reported_producer_=0;
    uint64_t stderr_at_=0;
    void enqueue_locked(const Sample &s,const std::string &kind,const std::string &phase,const std::string &reason,const std::string &incident) {
        if(queue_.size()>=kQueueMax) { ++queue_dropped_;if(kind!="event")return;queue_.pop_front(); }
        Record r{s,++seq_,kind,phase,reason,incident,{}};
        if(phase=="after")for(const auto &i:post_incidents_)if(s.at<=i.second)r.related.push_back(i.first);
        queue_.push_back(std::move(r));
    }
    void trim_locked(uint64_t now) { while(!history_.empty() && (now-history_.front().at>kHistoryMs || history_.size()>kHistoryMax)) { if(history_.size()>kHistoryMax)++history_dropped_;history_.pop_front(); } }
    bool ordinary_path(const std::string &p,bool missing=true) {
        struct stat st{}; if(lstat(p.c_str(),&st)<0)return missing&&errno==ENOENT;
        return S_ISREG(st.st_mode)&&!S_ISLNK(st.st_mode);
    }
    int open_regular(const std::string &p,int flags) {
        if(!ordinary_path(p)) {errno=EINVAL;return -1;}
        int fd=open(p.c_str(),flags|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK,0600);
        if(fd<0)return -1;
        struct stat st{}; if(fstat(fd,&st)<0||!S_ISREG(st.st_mode)||st.st_nlink!=1) {close(fd);errno=EINVAL;return -1;}
        if(fchmod(fd,0600)<0) {int e=errno;close(fd);errno=e;return -1;} return fd;
    }
    bool write_line(const std::string &line) {
        int lock=open_regular(lock_path_,O_CREAT|O_RDWR);
        if(lock<0)return false;
        bool locked=false;
        for(unsigned n=0;n<20;++n) { if(flock(lock,LOCK_EX|LOCK_NB)==0) {locked=true;break;}if(errno!=EWOULDBLOCK&&errno!=EAGAIN&&errno!=EINTR)break;std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
        if(!locked) {close(lock);errno=EWOULDBLOCK;return false;}
        bool ok=false;
        do {
            if(!ordinary_path(path_)||!ordinary_path(rotated1_)||!ordinary_path(rotated2_)) {errno=EINVAL;break;}
            struct stat st{}; if(lstat(path_.c_str(),&st)==0 && uint64_t(st.st_size)+line.size()>max_bytes_) {
                if(unlink((rotated2_).c_str())<0&&errno!=ENOENT)break;
                if(rename((rotated1_).c_str(),(rotated2_).c_str())<0&&errno!=ENOENT)break;
                if(rename(path_.c_str(),(rotated1_).c_str())<0)break;
            }
            int fd=open_regular(path_,O_CREAT|O_RDWR|O_APPEND);if(fd<0)break;
            struct stat original{};if(fstat(fd,&original)<0) {close(fd);break;}
            // Recover only the incomplete final line left by an interrupted write.
            off_t initial=original.st_size;
            if(initial>0) {char tail=0;if(pread(fd,&tail,1,initial-1)!=1) {close(fd);break;}
                if(tail!='\n') {off_t pos=initial;char chunk[4096];bool found=false;
                    while(pos>0&&!found) {size_t n=static_cast<size_t>(std::min<off_t>(pos,sizeof(chunk)));pos-=n;ssize_t got=pread(fd,chunk,n,pos);if(got!=static_cast<ssize_t>(n))break;
                        for(ssize_t k=got-1;k>=0;--k)if(chunk[k]=='\n') {initial=pos+k+1;found=true;break;}}
                    if(!found)initial=0;
                    if(ftruncate(fd,initial)<0) {close(fd);break;}
                }
            }
            size_t done=0;while(done<line.size()) {ssize_t n=write(fd,line.data()+done,line.size()-done);if(n<0&&errno==EINTR)continue;if(n<=0)break;done+=size_t(n);}
            ok=done==line.size();int e=errno;if(!ok) {int ignored=ftruncate(fd,initial);(void)ignored;}close(fd);errno=e;
        }while(false);
        int e=errno;flock(lock,LOCK_UN);close(lock);errno=e;return ok;
    }
    std::string serialize(const Record &r) {
        Obj o; o["schema"]=number(1);o["mono_ms"]=number(r.sample.at);o["wall_time"]=picojson::value(r.sample.wall);
        o["boot_id"]=picojson::value(boot_);o["pid"]=number(pid_);o["run_id"]=picojson::value(run_);o["process"]=picojson::value(process_);o["version"]=picojson::value(version_);
        o["seq"]=number(r.seq);o["sample_seq"]=number(r.sample.capture_seq);
        if(r.phase=="event")o["context_start_mono_ms"]=number(r.sample.at>kHistoryMs?r.sample.at-kHistoryMs:0);
        o["incident_id"]=picojson::value(r.incident);o["component"]=picojson::value(r.sample.component);o["reason"]=picojson::value(r.reason);
        picojson::array related;for(const auto &id:r.related)related.push_back(picojson::value(id));o["related_incident_ids"]=picojson::value(related);
        o["event_kind"]=picojson::value(r.kind);o["phase"]=picojson::value(r.phase);o["data"]=*r.sample.data;
        {std::lock_guard<std::mutex> l(mu_);o["queue_dropped_total"]=number(queue_dropped_);o["history_dropped_total"]=number(history_dropped_);o["deduplicated_total"]=number(deduplicated_);o["io_failures_total"]=number(io_failures_);o["io_dropped_total"]=number(io_dropped_);o["producer_errors_total"]=number(producer_errors_);}
        return picojson::value(o).serialize()+"\n";
    }
    void failed() noexcept {
        int e=errno; try {std::lock_guard<std::mutex> l(mu_);++io_failures_;++io_dropped_;}catch(...) {}
        uint64_t now=mono_ms();if(!stderr_at_||now-stderr_at_>=5000) {stderr_at_=now;std::fprintf(stderr,"WRONG_LOG write failed errno=%d (record dropped; control continues)\n",e);}
    }
    void work() {
        char b[96]{};int boot=open("/proc/sys/kernel/random/boot_id",O_RDONLY|O_CLOEXEC|O_NONBLOCK);
        if(boot>=0) {ssize_t n=read(boot,b,sizeof(b)-1);close(boot);if(n>0)boot_=std::string(b,size_t(n));while(!boot_.empty()&&(boot_.back()=='\n'||boot_.back()=='\r'))boot_.pop_back();}
        if(boot_.empty())boot_="unavailable";
        for(;;) {
            Record r;bool have=false;
            {std::unique_lock<std::mutex> l(mu_);cv_.wait_for(l,std::chrono::milliseconds(100),[&]{return stop_||!queue_.empty();});
                if(!queue_.empty()) {r=std::move(queue_.front());queue_.pop_front();have=true;}
                else if(stop_)break;
                else if(reported_drops_!=queue_dropped_||reported_dedup_!=deduplicated_||reported_io_!=io_failures_||reported_producer_!=producer_errors_) {
                    reported_drops_=queue_dropped_;reported_dedup_=deduplicated_;reported_io_=io_failures_;reported_producer_=producer_errors_;
                    r={Sample{mono_ms(),utc_now(),"logger",payload("{}")},++seq_,"health","health","logger_counters",incident_,{}};have=true;
                }
            }
            if(have)try {if(!write_line(serialize(r)))failed();}catch(...) {errno=ENOMEM;failed();}
        }
    }
public:
    LogState(const char *p,const char *v):process_(bounded(p,96)),version_(bounded(v,160)),pid_(getpid()) {
        const char *path=getenv("ROBOT_WRONG_LOG_PATH");path_=path&&*path?path:"/root/robot-navigation/wrong.log";
        lock_path_=path_+".lock";rotated1_=path_+".1";rotated2_=path_+".2";
        const char *max=getenv("ROBOT_WRONG_LOG_MAX_BYTES");if(max) {char *end=nullptr;unsigned long long n=strtoull(max,&end,10);if(end&&!*end&&n>=4096&&n<=64*1024*1024)max_bytes_=size_t(n);}
        run_=std::to_string(pid_)+"-"+std::to_string(mono_ms())+"-"+std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count());
        // Build every allocating startup field before any worker exists.
        event("logger","process_start",payload("{}"));
    }
    void run() noexcept {
        try {work();}
        catch(...) {stop_.store(true,std::memory_order_release);producer_error();std::fprintf(stderr,"WRONG_LOG worker failed (control continues)\n");}
    }
    void snapshot(Sample s) {
        std::lock_guard<std::mutex> l(mu_);if(stop_)return;
        s.capture_seq=++capture_seq_;history_.push_back(s);trim_locked(s.at);
        if(s.at<=post_until_ && (!post_at_.count(s.component)||s.at-post_at_[s.component]>=100)) {if(post_at_.size()>=128&&!post_at_.count(s.component))post_at_.erase(post_at_.begin());post_at_[s.component]=s.at;enqueue_locked(s,"snapshot","after","",incident_);cv_.notify_one();}
    }
    void event(const std::string &component,const std::string &reason,std::shared_ptr<const picojson::value> data) {
        Sample s{mono_ms(),utc_now(),component,std::move(data)};
        const std::string key=component+"\x1f"+reason+"\x1f"+s.data->serialize();
        std::lock_guard<std::mutex> l(mu_);if(stop_)return;
        auto it=dedup_.find(key);if(it!=dedup_.end()&&s.at-it->second<1000) {++deduplicated_;cv_.notify_one();return;}
        dedup_[key]=s.at;for(auto d=dedup_.begin();d!=dedup_.end();)if(s.at-d->second>10000)d=dedup_.erase(d);else++d;
        // Bound distinct duplicate keys as well as records.
        if(dedup_.size()>256)dedup_.erase(dedup_.begin());
        trim_locked(s.at);incident_=run_+":"+std::to_string(++incident_seq_);post_until_=s.at+kPostMs;post_at_.clear();
        while(!post_incidents_.empty()&&post_incidents_.front().second<s.at)post_incidents_.pop_front();
        if(post_incidents_.size()>=32)post_incidents_.pop_front();
        post_incidents_.emplace_back(incident_,post_until_);
        for(const auto &h:history_)if(h.capture_seq>last_history_capture_) {enqueue_locked(h,"snapshot","before","",incident_);last_history_capture_=h.capture_seq;}
        enqueue_locked(s,"event","event",reason,incident_);cv_.notify_one();
    }
    void producer_error() noexcept {try {std::lock_guard<std::mutex> l(mu_);++producer_errors_;cv_.notify_one();}catch(...) {}}
    void shutdown() noexcept {
        // The stop flag must be set even if the optional final record cannot allocate.
        try {std::lock_guard<std::mutex> l(mu_);
            if(!stop_.exchange(true,std::memory_order_acq_rel))try {enqueue_locked(Sample{mono_ms(),utc_now(),"logger",payload("{}")},"lifecycle","shutdown","process_stop",incident_);}catch(...) {++producer_errors_;}
        }catch(...) {stop_.store(true,std::memory_order_release);}
        cv_.notify_all();
    }
};
// The worker owns its state independently. Even an OS join/detach failure must
// neither throw into motor control nor leave a detached thread using freed data.
class Logger {
    std::shared_ptr<LogState> state_;
    std::unique_ptr<std::thread> worker_;
public:
    Logger(const char *p,const char *v):state_(std::make_shared<LogState>(p,v)),
        worker_(std::make_unique<std::thread>([s=state_]() noexcept {s->run();})) {}
    ~Logger() noexcept {shutdown();}
    void snapshot(Sample s) {state_->snapshot(std::move(s));}
    void event(const std::string &c,const std::string &r,std::shared_ptr<const picojson::value> d) {state_->event(c,r,std::move(d));}
    void producer_error() noexcept {state_->producer_error();}
    void shutdown() noexcept {
        state_->shutdown();
        if(!worker_||!worker_->joinable())return;
        try {worker_->join();}
        catch(...) {
            std::fprintf(stderr,"WRONG_LOG worker join failed (control continues)\n");
            try {worker_->detach();}
            catch(...) { (void)worker_.release(); } // Preserve joinable handle on irrecoverable OS failure.
        }
    }
};
std::mutex instance_mu;
std::shared_ptr<Logger> instance;
std::atomic<bool> enabled{false};
std::shared_ptr<Logger> current() {if(!enabled.load(std::memory_order_relaxed))return {};std::lock_guard<std::mutex> l(instance_mu);return instance;}
}
extern "C" void wrong_log_start(const char *process,const char *version) noexcept {
    try {std::lock_guard<std::mutex> l(instance_mu);if(instance)return;instance=std::make_shared<Logger>(process,version);enabled.store(true,std::memory_order_release);}catch(...) {std::fprintf(stderr,"WRONG_LOG startup failed (control continues)\n");}
}
extern "C" void wrong_log_snapshot(const char *component,const char *json) noexcept {
    std::shared_ptr<Logger> x;
    try {x=current();if(x)x->snapshot(Sample{mono_ms(),utc_now(),bounded(component,96),payload(json)});}catch(...) {if(x)x->producer_error();}
}
extern "C" void wrong_log_event(const char *component,const char *reason,const char *json) noexcept {
    std::shared_ptr<Logger> x;
    try {x=current();if(x)x->event(bounded(component,96),bounded(reason,160),payload(json));}catch(...) {if(x)x->producer_error();}
}
extern "C" void wrong_log_shutdown(void) noexcept {
    enabled.store(false,std::memory_order_release);
    try {std::shared_ptr<Logger> x;{std::lock_guard<std::mutex> l(instance_mu);x.swap(instance);}if(x)x->shutdown();}
    catch(...) {std::fprintf(stderr,"WRONG_LOG shutdown failed (control continues)\n");}
}
