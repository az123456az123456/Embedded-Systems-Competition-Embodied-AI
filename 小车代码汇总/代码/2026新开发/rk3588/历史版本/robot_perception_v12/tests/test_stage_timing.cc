// Exercise the real detector and clocks with a deterministic RKNN C API stub.
#include "detector.h"
#include "rknn_api.h"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
static int releases=0,fail_stage=0,core_mask=-1;
static bool reject_core=false;
static void pause_ms(int n){std::this_thread::sleep_for(std::chrono::milliseconds(n));}
extern "C" {
int rknn_set_core_mask(rknn_context,rknn_core_mask mask){core_mask=mask;return reject_core?-1:0;}
int rknn_init(rknn_context *c,void *,uint32_t,uint32_t,rknn_init_extend *){*c=1;return 0;}
int rknn_destroy(rknn_context){return 0;}
int rknn_query(rknn_context,rknn_query_cmd cmd,void *value,uint32_t){
    if(cmd==RKNN_QUERY_IN_OUT_NUM){auto *n=static_cast<rknn_input_output_num*>(value);n->n_input=1;n->n_output=6;return 0;}
    auto *a=static_cast<rknn_tensor_attr*>(value);a->n_dims=4;a->dims[0]=1;a->type=RKNN_TENSOR_FLOAT16;
    if(cmd==RKNN_QUERY_INPUT_ATTR){a->fmt=RKNN_TENSOR_NHWC;a->dims[1]=32;a->dims[2]=32;a->dims[3]=3;}
    else {a->fmt=RKNN_TENSOR_NCHW;a->dims[1]=a->index%2?1:64;a->dims[2]=a->dims[3]=4>>(a->index/2);}
    a->n_elems=a->dims[0]*a->dims[1]*a->dims[2]*a->dims[3];return 0;
}
int rknn_inputs_set(rknn_context,uint32_t n,rknn_input in[]){assert(n==1 && in[0].type==RKNN_TENSOR_UINT8);pause_ms(3);return fail_stage==1?-1:0;}
int rknn_run(rknn_context,rknn_run_extend *){pause_ms(15);return fail_stage==2?-1:0;}
int rknn_outputs_get(rknn_context,uint32_t n,rknn_output out[],rknn_output_extend *){
    pause_ms(5);if(fail_stage==3)return -1;
    for(unsigned i=0;i<n;i++){assert(out[i].want_float==1);unsigned grid=4>>(i/2);out[i].size=grid*grid*(i%2?1:64)*sizeof(float);out[i].buf=std::calloc(1,out[i].size);}return 0;
}
int rknn_outputs_release(rknn_context,uint32_t n,rknn_output out[]){++releases;pause_ms(2);for(unsigned i=0;i<n;i++)std::free(out[i].buf);return 0;}
}
int main(){
    const char *path="build/timing-model.stub";FILE *f=std::fopen(path,"wb");assert(f);std::fputs("stub",f);std::fclose(f);
    RknnDetector detector;std::string error;assert(detector.open(path,.35f,.45f,1,error));
    for(int core=0;core<3;core++){assert(detector.set_core(core,error));assert(core_mask==(1<<core));}
    assert(detector.set_core(-1,error) && core_mask==RKNN_NPU_CORE_AUTO);
    reject_core=true;assert(!detector.set_core(2,error) && !error.empty());reject_core=false;
    assert(!detector.set_core(3,error));
    std::vector<unsigned char> rgb(64*64*3,114);std::vector<Detection> results;DetectorTiming t;
    assert(detector.infer(rgb.data(),64,64,64*3,results,error,&t));
    assert(results.empty() && releases==1);
    assert(t.input_ms>=2.5 && t.run_ms>=14.5 && t.output_ms>=4.5 && t.release_ms>=1.5);
    const double sum=t.preprocess_ms+t.input_ms+t.run_ms+t.output_ms+t.postprocess_ms+t.release_ms;
    assert(t.total_ms>=sum && t.total_ms-sum<10);
    for(int stage=1;stage<=3;stage++){
        fail_stage=stage;assert(!detector.infer(rgb.data(),64,64,64*3,results,error,&t));
        assert(t.total_ms>0 && t.postprocess_ms==0 && t.release_ms==0);
        if(stage==1)assert(t.run_ms==0 && t.output_ms==0);
        if(stage==2)assert(t.run_ms>0 && t.output_ms==0);
        if(stage==3)assert(t.output_ms>0);
    }
    assert(releases==1);std::remove(path);
    std::puts("PASS real detector stage timings: SDK input/run/output/release, stage sum, and error-path reset (stub runtime; not hardware performance)");
}
