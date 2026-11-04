#include "eye_scheduler.h"
#include <array>
#include <cassert>
#include <cstdio>
#include <map>
#include <random>
using eye_scheduler::Job;
struct Work {eye_scheduler::PairState schedule;};
static size_t checks=0;
#define CHECK(x) do{++checks;assert(x);}while(0)
static std::shared_ptr<Work> make(uint64_t id,uint64_t stamp=0){
    auto p=std::make_shared<Work>();p->schedule.frame_id=id;p->schedule.source_ms=stamp;return p;
}
static void policy(){
    std::deque<Job<Work>> q;
    for(uint64_t id=1;id<1001;id++){
        auto p=make(id);eye_scheduler::enqueue(q,p,0,2,id-1);
        CHECK(q.size()==2);auto a=eye_scheduler::take(q),b=eye_scheduler::take(q);
        CHECK(a.eye==int((id-1)%2));CHECK(b.eye==1-a.eye);CHECK(a.work==b.work);
        CHECK(p->schedule.requested==3&&p->schedule.started==3);CHECK(q.empty());
    }
    // Even when a newer pair is physically at the front, finish its older
    // already-started sibling first. This exercises production selection.
    auto old=make(1),fresh=make(2);old->schedule.started=1;
    q.push_back({fresh,0});q.push_back({fresh,1});q.push_back({old,1});
    CHECK(eye_scheduler::take(q).work==old);CHECK(eye_scheduler::take(q).work==fresh);
    q.clear();auto a=make(3,100),b=make(4,101),c=make(5,900);
    eye_scheduler::enqueue(q,a,0,2,0);eye_scheduler::enqueue(q,b,0,2,1);eye_scheduler::enqueue(q,c,0,2,2);
    a->schedule.started=1; // Existing left task is in flight; finish its right.
    q.erase(q.begin());
    auto dropped=eye_scheduler::drop_unstarted_expired(q,1000,350);
    CHECK(dropped.size()==1&&dropped[0]==b);CHECK(q.size()==3);
    CHECK(eye_scheduler::take(q).work==a);CHECK(eye_scheduler::take(q).work==c);
    q.clear();auto future=make(6,2000);eye_scheduler::enqueue(q,future,1,1,0);
    CHECK(eye_scheduler::drop_unstarted_expired(q,1000,350).empty());
    CHECK(eye_scheduler::take(q).eye==1);
}
static void simulate(unsigned seed,int workers,int camera_ms,int base_ms,bool unequal){
    std::mt19937 rng(seed);std::deque<Job<Work>> q;
    struct Slot {Job<Work> job;int finish=0;bool busy=false;};std::array<Slot,3> slots{};
    std::map<uint64_t,std::shared_ptr<Work>> live;
    uint64_t captured=0,submitted=0,completed=0,last_published=0,published=0,out_of_order=0;
    uint64_t pending_id=0,pending_stamp=0,source_skipped=0,last_admitted=0;
    uint64_t eye_jobs[2]={},leading[2]={};int maximum_inflight=0;
    const int depth=workers==3?2:1;
    for(int now=0;now<120000;now++){
        if(now%camera_ms==0){pending_id=++captured;pending_stamp=now;}
        for(int i=0;i<workers;i++)if(slots[i].busy&&now>=slots[i].finish){
            auto job=slots[i].job;slots[i].busy=false;
            CHECK(!(job.work->schedule.completed&(1u<<job.eye)));
            job.work->schedule.completed|=1u<<job.eye;++eye_jobs[job.eye];
            if(job.work->schedule.completed==3){
                ++completed;auto id=job.work->schedule.frame_id;
                if(id>last_published){last_published=id;++published;}else ++out_of_order;
                CHECK(live.erase(id)==1);
            }
        }
        if(pending_id>last_admitted&&int(live.size())<depth){
            auto p=make(pending_id,pending_stamp);live[pending_id]=p;
            if(last_admitted)source_skipped+=pending_id-last_admitted-1;
            last_admitted=pending_id;++leading[submitted%2];
            eye_scheduler::enqueue(q,p,0,2,submitted++);
        }
        maximum_inflight=std::max(maximum_inflight,int(live.size()));CHECK(maximum_inflight<=depth);
        for(int i=0;i<workers;i++)if(!slots[i].busy&&!q.empty()){
            auto job=eye_scheduler::take(q);auto &slot=slots[i];slot.job=job;slot.busy=true;
            slot.finish=now+base_ms+int(rng()%13)+(unequal&&job.eye?19:0);
            CHECK(job.work->schedule.started&(1u<<job.eye));
            // Queue may never contain a completed or already-started eye.
            for(const auto &pending:q){CHECK(!(pending.work->schedule.started&(1u<<pending.eye)));}
        }
    }
    CHECK(completed==published+out_of_order);CHECK(submitted>=completed&&submitted-completed<=unsigned(depth));
    CHECK(std::abs(int64_t(leading[0])-int64_t(leading[1]))<=1);
    CHECK(std::abs(int64_t(eye_jobs[0])-int64_t(eye_jobs[1]))<=int64_t(depth));
    CHECK(published>0);
    if(base_ms<=5){CHECK(source_skipped==0);CHECK(submitted>=captured-1);}
    if(base_ms>camera_ms*2)CHECK(source_skipped>0);
    std::printf("SIM seed=%u workers=%d camera_ms=%d base_eye_ms=%d unequal=%d captured=%llu submitted=%llu complete=%llu published=%llu capture_superseded=%llu lead=(%llu,%llu) eye_jobs=(%llu,%llu)\n",
        seed,workers,camera_ms,base_ms,unequal,(unsigned long long)captured,(unsigned long long)submitted,
        (unsigned long long)completed,(unsigned long long)published,(unsigned long long)source_skipped,
        (unsigned long long)leading[0],(unsigned long long)leading[1],(unsigned long long)eye_jobs[0],(unsigned long long)eye_jobs[1]);
}
int main(){
    policy();
    for(int workers=1;workers<=3;workers++)for(unsigned seed=1;seed<=5;seed++){
        simulate(seed,workers,33,5,false);simulate(seed,workers,20,55,false);simulate(seed,workers,20,55,true);
    }
    std::printf("PASS eye scheduler checks=%zu; actual production policy; synthetic timing only\n",checks);
}
