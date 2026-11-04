#include "../wrong_log.h"
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <pthread.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {
thread_local long fail_after=-1;
std::atomic<bool> worker_fail{false};
pthread_t main_thread;
void *allocate(size_t n) {
    if(fail_after==0) {fail_after=-1;throw std::bad_alloc();}
    if(fail_after>0)--fail_after;
    if(!pthread_equal(pthread_self(),main_thread)&&worker_fail.exchange(false))throw std::bad_alloc();
    void *p=std::malloc(n?n:1);if(!p)throw std::bad_alloc();return p;
}
}
void *operator new(size_t n) {return allocate(n);}
void *operator new[](size_t n) {return allocate(n);}
void operator delete(void *p) noexcept {std::free(p);}
void operator delete[](void *p) noexcept {std::free(p);}
void operator delete(void *p,size_t) noexcept {std::free(p);}
void operator delete[](void *p,size_t) noexcept {std::free(p);}
int main() {
    main_thread=pthread_self();char dir[]="/tmp/acg138-wrong-alloc-XXXXXX";if(!mkdtemp(dir))return 1;
    unsigned checks=0;
    for(long target=0;target<240;++target) {
        pid_t child=fork();if(child<0)return 2;
        if(child==0) {
            char path[256];std::snprintf(path,sizeof(path),"%s/%ld.log",dir,target);setenv("ROBOT_WRONG_LOG_PATH",path,1);
            fail_after=target;
            wrong_log_start("navigation","allocation-test");
            wrong_log_snapshot("sensors","{\"range_m\":[1.0,2.0,3.0],\"encoder\":[1,2,3,4]}");
            wrong_log_event("navigation","fault","{\"transition\":1,\"nested\":{\"token\":\"secret\",\"safe\":1}}");
            wrong_log_shutdown();
            fail_after=-1;
            wrong_log_start("navigation","allocation-recovery");wrong_log_event("navigation","recovered","{}");wrong_log_shutdown();
            _exit(0);
        }
        int status=0;if(waitpid(child,&status,0)<0||!WIFEXITED(status)||WEXITSTATUS(status)!=0) {std::fprintf(stderr,"FAIL allocation point %ld status=%d\n",target,status);return 3;}++checks;
    }
    for(int mode=0;mode<2;++mode) {
        pid_t child=fork();if(child<0)return 4;
        if(child==0) {
            char path[256];std::snprintf(path,sizeof(path),"%s/extra-%d.log",dir,mode);setenv("ROBOT_WRONG_LOG_PATH",path,1);
            if(mode==0)worker_fail.store(true);
            wrong_log_start("navigation","failure-boundaries");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if(mode==1)fail_after=0; // Final lifecycle record cannot allocate.
            wrong_log_shutdown();fail_after=-1;_exit(0);
        }
        int status=0;if(waitpid(child,&status,0)<0||!WIFEXITED(status)||WEXITSTATUS(status)!=0) {std::fprintf(stderr,"FAIL boundary %d status=%d\n",mode,status);return 5;}++checks;
    }
    std::printf("wrong_log allocation failures: %u child scenarios PASS; artifacts=%s\n",checks,dir);return 0;
}
