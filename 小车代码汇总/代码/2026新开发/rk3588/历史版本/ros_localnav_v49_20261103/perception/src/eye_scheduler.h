#ifndef PERCEPTION_EYE_SCHEDULER_H
#define PERCEPTION_EYE_SCHEDULER_H
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

// This is the queue policy used by the real inference threads. Its caller owns
// the queue mutex. Started pairs always finish; only wholly unstarted stale
// pairs can be removed. No partially completed pair is presented as stereo.
namespace eye_scheduler {
struct PairState {
    uint64_t frame_id=0,source_ms=0;
    unsigned requested=0,started=0,completed=0;
};
template<class Work> struct Job {std::shared_ptr<Work> work;int eye=0;};
template<class Work>
void enqueue(std::deque<Job<Work>> &queue,const std::shared_ptr<Work> &work,int first,int eyes,uint64_t ordinal){
    const int leading=eyes==2?int(ordinal&1):first;
    for(int i=0;i<eyes;i++){
        int eye=eyes==2?(leading+i)%2:first;
        work->schedule.requested|=1u<<eye;
        queue.push_back({work,eye});
    }
}
template<class Work>
Job<Work> take(std::deque<Job<Work>> &queue){
    assert(!queue.empty());
    // Complete the oldest already-started pair before spending a free core on
    // an entirely new pair. Arrival order breaks ties, including alternating
    // first eyes, so the right eye does not systematically take the queue hit.
    auto selected=queue.begin();
    bool selected_started=selected->work->schedule.started!=0;
    for(auto it=queue.begin()+1;it!=queue.end();++it){
        bool started=it->work->schedule.started!=0;
        if((started&&!selected_started)||(started==selected_started&&
           it->work->schedule.frame_id<selected->work->schedule.frame_id)){
            selected=it;selected_started=started;
        }
    }
    auto job=std::move(*selected);queue.erase(selected);
    job.work->schedule.started|=1u<<job.eye;
    return job;
}
template<class Work>
std::vector<std::shared_ptr<Work>> drop_unstarted_expired(std::deque<Job<Work>> &queue,uint64_t now,uint64_t max_age_ms){
    std::vector<std::shared_ptr<Work>> dropped;
    for(const auto &job:queue){
        const auto &state=job.work->schedule;
        if(!state.started&&now>=state.source_ms&&now-state.source_ms>max_age_ms&&
           std::find(dropped.begin(),dropped.end(),job.work)==dropped.end())dropped.push_back(job.work);
    }
    queue.erase(std::remove_if(queue.begin(),queue.end(),[&](const Job<Work> &job){
        return std::find(dropped.begin(),dropped.end(),job.work)!=dropped.end();
    }),queue.end());
    return dropped;
}
} // namespace eye_scheduler
#endif
