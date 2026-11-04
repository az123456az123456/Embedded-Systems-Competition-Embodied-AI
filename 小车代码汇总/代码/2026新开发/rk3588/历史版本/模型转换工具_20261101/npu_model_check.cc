// File-only NPU verification: no camera, networking, ROS, FPGA or motor APIs.
#include "detector.h"
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

using Clock=std::chrono::steady_clock;
struct Image { std::string id; std::vector<unsigned char> rgb; };

int main(int argc,char **argv) try {
    if (argc!=6) throw std::runtime_error("usage: npu-model-check MODEL INPUT_LIST OUT_JSONL WORKERS CLASS_COUNT");
    const int workers=std::stoi(argv[4]), classes=std::stoi(argv[5]);
    if (workers!=1 && workers!=3) throw std::runtime_error("workers must be 1 or 3");
    std::ifstream list(argv[2]); if (!list) throw std::runtime_error("input list missing");
    std::vector<Image> images; std::string path;
    while (std::getline(list,path)) {
        if (path.empty()) continue;
        int w=0,h=0,c=0;
        auto *rgb=stbi_load(path.c_str(),&w,&h,&c,3);
        if (!rgb || w!=640 || h!=640) { stbi_image_free(rgb); throw std::runtime_error("input must be RGB640 PNG"); }
        const auto slash=path.find_last_of('/'); const auto dot=path.find_last_of('.');
        Image image{path.substr(slash==std::string::npos?0:slash+1,
                                dot-(slash==std::string::npos?0:slash+1)),
                    std::vector<unsigned char>(rgb,rgb+640*640*3)};
        stbi_image_free(rgb); images.push_back(std::move(image));
    }
    if (images.empty()) throw std::runtime_error("no inputs");
    std::ofstream output(argv[3]); if (!output) throw std::runtime_error("output not writable");
    output<<std::setprecision(9);
    std::mutex mutex; std::condition_variable ready_condition;
    int ready=0; bool start=false; std::atomic<int> failures{0}; std::atomic<unsigned> next{0};
    std::vector<std::thread> threads;
    const unsigned jobs=images.size()*(workers==3?3:1);
    for (int core=0;core<workers;++core) threads.emplace_back([&,core] {
        RknnDetector detector; std::string error;
        bool ok=detector.open(argv[1],.25f,.45f,classes,error) && detector.set_core(core,error);
        if (ok) for (int warm=0;warm<2;++warm) {
            std::vector<Detection> d;
            if (!detector.infer(images[0].rgb.data(),640,640,640*3,d,error)) {ok=false; break;}
        }
        { std::unique_lock<std::mutex> lock(mutex);
          if (!ok) { ++failures; std::cerr<<"core "<<core<<" failed: "<<error<<'\n'; }
          ++ready; ready_condition.notify_all();
          ready_condition.wait(lock,[&]{return start;}); }
        if (!ok) return;
        for (;;) {
            unsigned job=next.fetch_add(1); if (job>=jobs) break;
            const auto &image=images[job%images.size()];
            std::vector<Detection> d; DetectorTiming t;
            if (!detector.infer(image.rgb.data(),640,640,640*3,d,error,&t)) {
                ++failures; std::lock_guard<std::mutex> lock(mutex); std::cerr<<error<<'\n'; break;
            }
            std::ostringstream row; row<<std::setprecision(9);
            row<<"{\"id\":\""<<image.id<<"\",\"job\":"<<job<<",\"core\":"<<core
               <<",\"workers\":"<<workers<<",\"preprocess_ms\":"<<t.preprocess_ms
               <<",\"input_ms\":"<<t.input_ms<<",\"run_ms\":"<<t.run_ms
               <<",\"output_ms\":"<<t.output_ms<<",\"post_ms\":"<<t.postprocess_ms
               <<",\"total_ms\":"<<t.total_ms<<",\"detections\":[";
            for (std::size_t i=0;i<d.size();++i) {
                if(i) row<<',';
                const auto &v=d[i];
                row<<'['<<v.left<<','<<v.top<<','<<v.right<<','<<v.bottom<<','<<v.score<<','<<v.class_id<<']';
            }
            row<<"]}\n"; std::lock_guard<std::mutex> lock(mutex); output<<row.str(); output.flush();
        }
    });
    std::unique_lock<std::mutex> lock(mutex);
    ready_condition.wait(lock,[&]{return ready==workers;});
    const auto begin=Clock::now(); start=true; ready_condition.notify_all(); lock.unlock();
    for(auto &t:threads)t.join();
    const double seconds=std::chrono::duration<double>(Clock::now()-begin).count();
    std::cout<<"{\"workers\":"<<workers<<",\"jobs\":"<<jobs<<",\"wall_seconds\":"<<seconds
             <<",\"eye_tasks_per_second\":"<<jobs/seconds<<",\"failed\":"<<failures.load()
             <<",\"scope\":\"file-only NPU; not stereo paired FPS or camera end-to-end\"}\n";
    return failures?1:0;
} catch(const std::exception &e) {std::cerr<<e.what()<<'\n'; return 2;}
