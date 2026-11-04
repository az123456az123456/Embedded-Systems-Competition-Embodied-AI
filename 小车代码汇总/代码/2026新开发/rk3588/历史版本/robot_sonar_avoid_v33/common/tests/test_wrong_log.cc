#include "../wrong_log.h"
#include "../../navigation/vendor/picojson.h"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <sys/file.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

using Obj=picojson::object;
int checks=0;
void check(bool ok,const char *what) {++checks;if(!ok){std::fprintf(stderr,"FAIL: %s\n",what);std::exit(1);}}
std::string readfile(const std::string &p) {std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}
std::vector<picojson::value> records(const std::string &path,bool rotated=false) {
    std::vector<picojson::value> out;
    std::vector<std::string> paths;if(rotated) {paths.push_back(path+".2");paths.push_back(path+".1");}paths.push_back(path);
    for(const auto &p:paths) {std::ifstream f(p);std::string line;while(std::getline(f,line)) {picojson::value v;check(picojson::parse(v,line).empty()&&v.is<Obj>(),"every JSONL record parses");out.push_back(v);}}
    return out;
}
std::string str(const picojson::value &v,const char *k) {auto i=v.get<Obj>().find(k);return i!=v.get<Obj>().end()&&i->second.is<std::string>()?i->second.get<std::string>():"";}
uint64_t num(const picojson::value &v,const char *k) {auto i=v.get<Obj>().find(k);return i!=v.get<Obj>().end()&&i->second.is<double>()?uint64_t(i->second.get<double>()):0;}
void setup(const std::string &path) {setenv("ROBOT_WRONG_LOG_PATH",path.c_str(),1);unsetenv("ROBOT_WRONG_LOG_MAX_BYTES");}
void history_case(const std::string &dir) {
    std::string p=dir+"/history.log";setup(p);wrong_log_start("nav","test");
    wrong_log_snapshot("sensor","{\"tag\":\"expired\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));wrong_log_snapshot("sensor","{\"tag\":\"before10s\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(9000));
    wrong_log_event("nav","bootstrap_wait","{\"transition\":1,\"token\":\"do-not-record-A\",\"nested\":{\"Authorization\":\"do-not-record-B\",\"safe\":3},\"array\":[{\"pair_code\":\"do-not-record-C\",\"safe\":4}]}");
    wrong_log_event("nav","bootstrap_wait","{\"transition\":1,\"token\":\"do-not-record-A\",\"nested\":{\"Authorization\":\"do-not-record-B\",\"safe\":3},\"array\":[{\"pair_code\":\"do-not-record-C\",\"safe\":4}]}");
    wrong_log_snapshot("sensor","{\"tag\":\"after\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(5100));wrong_log_snapshot("sensor","{\"tag\":\"too_late\"}");
    wrong_log_shutdown();auto all=records(p);std::string incident;unsigned event_count=0;
    for(auto &v:all)if(str(v,"reason")=="bootstrap_wait") {incident=str(v,"incident_id");++event_count;}
    check(event_count==1,"identical event duplicate suppressed");check(!incident.empty(),"incident id assigned");
    bool before=false,after=false,expired=false,late=false,dedup=false;
    for(auto &v:all) {check(!str(v,"boot_id").empty()&&!str(v,"run_id").empty()&&num(v,"pid")>0&&num(v,"mono_ms")>0,"correlation metadata present");if(num(v,"deduplicated_total")>0)dedup=true;if(str(v,"incident_id")!=incident)continue;
        auto text=v.get<Obj>().at("data").serialize();if(text.find("before10s")!=std::string::npos&&str(v,"phase")=="before")before=true;if(text.find("after\"")!=std::string::npos&&str(v,"phase")=="after")after=true;if(text.find("expired")!=std::string::npos)expired=true;if(text.find("too_late")!=std::string::npos)late=true;
    }
    check(before&&after,"10 second history and post-event samples retained");check(!expired&&!late,"outside 10s before / 5s after excluded");check(dedup,"deduplication counter visible");
    auto txt=readfile(p);check(txt.find("do-not-record")==std::string::npos,"nested secrets removed");
    struct stat st{};check(stat(p.c_str(),&st)==0&&(st.st_mode&0777)==0600,"log permission 0600");
    size_t old=all.size();wrong_log_start("nav","test");wrong_log_event("nav","second_run","{}");wrong_log_shutdown();all=records(p);check(all.size()>old,"restart appends");std::string run1=str(all.front(),"run_id"),run2=str(all.back(),"run_id");check(run1!=run2,"restart uses new run id");
}
void distinct_case(const std::string &dir) {
    std::string p=dir+"/distinct.log";setup(p);wrong_log_start("dash","test");
    wrong_log_event("nav","fault","{\"transition\":1}");wrong_log_event("nav","fault","{\"transition\":2}");wrong_log_event("imu","fault","{\"transition\":2}");wrong_log_event("nav","other","{\"transition\":2}");
    wrong_log_event("nav","invalid_payload","not-json-secret");
    std::string deep(40,'[');deep+="\"hidden\"";deep+=std::string(40,']');wrong_log_event("nav","deep_payload",deep.c_str());wrong_log_snapshot("sensor","{\"tag\":\"shared_post\"}");wrong_log_shutdown();auto all=records(p);unsigned n=0;bool invalid=false,depth=false;
    for(auto &v:all) {auto reason=str(v,"reason");if(reason=="fault"||reason=="other")++n;if(reason=="invalid_payload")invalid=v.get<Obj>().at("data").get<Obj>().at("invalid_json").get<bool>();if(reason=="deep_payload")depth=v.get<Obj>().at("data").get<Obj>().at("depth_limit").get<bool>();}
    bool shared_post=false;for(const auto &v:all)if(str(v,"phase")=="after"&&v.get<Obj>().at("related_incident_ids").get<picojson::array>().size()>=4)shared_post=true;
    check(shared_post,"overlapping incident post-history shares references");check(n==4,"distinct transitions/components/reasons not deduplicated");check(invalid&&depth,"invalid/deep JSON safely summarized");check(readfile(p).find("not-json-secret")==std::string::npos,"invalid JSON original omitted");
    {std::ofstream f(p,std::ios::app);f<<"{incomplete crash line";}
    wrong_log_start("dash","test");wrong_log_event("nav","after_crash","{}");wrong_log_shutdown();
    check(!records(p).empty()&&readfile(p).find("incomplete crash")==std::string::npos,"interrupted final JSONL write repaired on append");
}
void rotation_case(const std::string &dir) {
    std::string p=dir+"/rotate.log";setup(p);setenv("ROBOT_WRONG_LOG_MAX_BYTES","4096",1);wrong_log_start("nav","test");
    for(int i=0;i<45;++i) {std::string j="{\"index\":"+std::to_string(i)+",\"padding\":\""+std::string(100,'x')+"\"}";wrong_log_event("nav","rotate",j.c_str());}
    wrong_log_shutdown();auto all=records(p,true);check(!all.empty(),"rotation retains records");for(auto suffix:{"",".1",".2"}) {struct stat st{};check(stat((p+suffix).c_str(),&st)==0&&st.st_size<=4096,"all three rotation files bounded");}check(access((p+".3").c_str(),F_OK)<0,"no fourth rotation file");
}
void concurrent_case(const std::string &dir) {
    std::string p=dir+"/concurrent.log";setup(p);std::vector<pid_t> kids;
    for(int k=0;k<2;++k) {pid_t child=fork();check(child>=0,"fork writer");if(child==0) {wrong_log_start(k?"nav":"dashboard","test");for(int i=0;i<80;++i) {std::string j="{\"n\":"+std::to_string(i)+"}";wrong_log_event("runtime","parallel",j.c_str());}wrong_log_shutdown();_exit(0);}kids.push_back(child);}
    for(auto child:kids){int status;waitpid(child,&status,0);check(WIFEXITED(status)&&WEXITSTATUS(status)==0,"child writer exits");}
    auto all=records(p);unsigned a=0,b=0;for(auto &v:all)if(str(v,"reason")=="parallel") {if(str(v,"process")=="nav")++a;else if(str(v,"process")=="dashboard")++b;}
    check(a==80&&b==80,"both process complete events share parseable log");
}
void unsafe_case(const std::string &dir) {
    std::string target=dir+"/target";{std::ofstream f(target);f<<"unchanged";}
    std::string link=dir+"/link.log";check(symlink(target.c_str(),link.c_str())==0,"create test symlink");
    std::string fifo=dir+"/fifo.log";mkfifo(fifo.c_str(),0600);
    for(const auto &p:{link,fifo,std::string("/proc/wrong-log-test.log")}) {
        setup(p);wrong_log_start("nav","test");auto begin=std::chrono::steady_clock::now();for(int i=0;i<250;++i)wrong_log_event("nav","io_fail","{\"transition\":1}");auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-begin).count();check(elapsed<500,"file failures cannot block producer");wrong_log_shutdown();
    }check(readfile(target)=="unchanged","symlink target untouched");
    std::string recover=dir+"/recover.log";check(mkfifo(recover.c_str(),0600)==0,"create recover FIFO");setup(recover);wrong_log_start("nav","test");wrong_log_event("nav","disk_failed","{}");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));check(unlink(recover.c_str())==0,"remove failed sink");wrong_log_event("nav","disk_recovered","{}");wrong_log_shutdown();
    bool failures=false;for(const auto &v:records(recover))if(num(v,"io_failures_total")>0&&num(v,"io_dropped_total")>0)failures=true;check(failures,"subsequent successful records expose prior I/O loss");
}
void drop_case(const std::string &dir) {
    std::string p=dir+"/drop.log";setup(p);wrong_log_start("nav","test");
    for(int i=0;i<510;++i){std::string j="{\"sample\":"+std::to_string(i)+"}";wrong_log_snapshot("source",j.c_str());}
    int blocker=open((p+".lock").c_str(),O_RDWR|O_CREAT,0600);check(blocker>=0&&flock(blocker,LOCK_EX)==0,"hold writer lock for overflow test");
    for(int i=0;i<2000;++i){std::string j="{\"transition\":"+std::to_string(i)+"}";wrong_log_event("nav","burst",j.c_str());}
    flock(blocker,LOCK_UN);close(blocker);wrong_log_shutdown();auto all=records(p);bool dropped=false;for(auto &v:all)if(num(v,"queue_dropped_total")>0)dropped=true;check(dropped,"bounded queue overflow explicitly counted");
}
int main() {
    char tmp[]="/tmp/acg138-wrong-log-XXXXXX";char *dir=mkdtemp(tmp);check(dir!=nullptr,"temporary directory");std::string off=std::string(dir)+"/disabled.log";setup(off);wrong_log_snapshot("x","{}");wrong_log_event("x","x","{}");check(access(off.c_str(),F_OK)<0,"unstarted logger no-op");
    history_case(dir);distinct_case(dir);rotation_case(dir);concurrent_case(dir);unsafe_case(dir);drop_case(dir);
    std::printf("wrong_log: %d checks PASS; artifacts=%s\n",checks,dir);return 0;
}
