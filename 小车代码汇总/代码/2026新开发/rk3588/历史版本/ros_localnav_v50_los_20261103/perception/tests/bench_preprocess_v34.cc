#include "preprocess_plan.h"
#include <cassert>
#include <cstdio>
#include <random>
#include <thread>
using namespace detector_math;
static void run(const char *mode,unsigned count) {
    setenv("PERCEPTION_PREPROCESS",mode,1);
    std::vector<unsigned char> rgb(1920*1080*3);
    std::mt19937 random(34);for(auto &p:rgb)p=random()%256;
    const int workers=3;std::vector<double> busy(workers);std::vector<std::thread> threads;
    auto started=StageClock::now();
    for(int i=0;i<workers;++i)threads.emplace_back([&,i] {
        RgbPreprocessor prep;Letterbox box;std::string error;
        std::vector<unsigned char> output(640*640*3),gold;
        const auto *eye=rgb.data()+(i%2)*960*3;
        assert(preprocess_rgb(eye,960,1080,1920*3,640,640,gold,box,error));
        assert(prep.run(eye,960,1080,1920*3,640,640,output.data(),output.size(),640*3,box,error));
        assert(output==gold);
        auto begin=StageClock::now();
        for(unsigned j=0;j<count;++j) {
            assert(prep.run(eye,960,1080,1920*3,640,640,output.data(),output.size(),640*3,box,error));
        }
        busy[i]=stage_elapsed(begin);
    });
    for(auto &t:threads)t.join();
    const double elapsed=stage_elapsed(started),sum=busy[0]+busy[1]+busy[2];
    std::printf("BENCH_LOCAL mode=%s workers=%d eye_jobs=%u elapsed_ms=%.4f mean_eye_ms=%.4f eye_fps=%.3f\n",
                mode,workers,count*workers,elapsed,sum/(count*workers),count*workers*1000./elapsed);
}
int main() {for(int n=0;n<3;++n){run("pixel",50);run("cpu",50);}unsetenv("PERCEPTION_PREPROCESS");}
