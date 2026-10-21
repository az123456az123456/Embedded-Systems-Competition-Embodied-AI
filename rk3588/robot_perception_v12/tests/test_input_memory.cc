// Real detector + SDK stub. Verifies input bytes and clean recovery, not speed.
#include "detector.h"
#include "detector_math.h"
#include "rknn_api.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

enum Fault { NONE, QUERY, ALLOCATE, BIND, STRIDE, SIZE, SYNC, RUN, RUN_ALWAYS };
static Fault fault=NONE;
static int initialized=0,destroyed=0,allocated=0,freed=0,legacy=0,flushed=0,ran=0;
struct Context {bool bound=false;int core=0;};
static std::map<rknn_context,Context> contexts;
static std::vector<unsigned char> expected;
static constexpr unsigned width=32,height=32,padded_width=40;
static void verify_rgb(const void *data,unsigned stride) {
    auto *bytes=static_cast<const unsigned char *>(data);
    for(unsigned y=0;y<height;y++) {
        assert(std::memcmp(bytes+y*stride,expected.data()+y*width*3,width*3)==0);
        for(unsigned x=width*3;x<stride;x++)assert(bytes[y*stride+x]==114);
    }
}
extern "C" {
int rknn_set_core_mask(rknn_context c,rknn_core_mask mask){contexts.at(c).core=mask;return 0;}
int rknn_init(rknn_context *c,void *,uint32_t,uint32_t,rknn_init_extend *){
    *c=++initialized;contexts[*c]={};return 0;
}
int rknn_destroy(rknn_context c){assert(contexts.erase(c)==1);++destroyed;return 0;}
int rknn_query(rknn_context,rknn_query_cmd cmd,void *value,uint32_t){
    if(cmd==RKNN_QUERY_IN_OUT_NUM){auto *n=static_cast<rknn_input_output_num *>(value);n->n_input=1;n->n_output=6;return 0;}
    auto *a=static_cast<rknn_tensor_attr *>(value);
    a->n_dims=4;a->dims[0]=1;a->type=RKNN_TENSOR_FLOAT16;
    if(cmd==RKNN_QUERY_INPUT_ATTR || cmd==RKNN_QUERY_NATIVE_INPUT_ATTR || cmd==RKNN_QUERY_NATIVE_NHWC_INPUT_ATTR) {
        if(fault==QUERY && cmd!=RKNN_QUERY_INPUT_ATTR)return -1;
        a->fmt=RKNN_TENSOR_NHWC;a->dims[1]=height;a->dims[2]=width;a->dims[3]=3;
        a->n_elems=width*height*3;a->size=a->n_elems*2;
        a->w_stride=fault==STRIDE?0xffffffffu:padded_width;
        a->size_with_stride=fault==SIZE?1:padded_width*height*3*2;
    } else {
        a->fmt=RKNN_TENSOR_NCHW;a->dims[1]=a->index%2?1:64;a->dims[2]=a->dims[3]=4>>(a->index/2);
        a->n_elems=a->dims[1]*a->dims[2]*a->dims[3];a->size=a->n_elems*2;
    }
    return 0;
}
#ifndef NO_IO_API
rknn_tensor_mem *rknn_create_mem2(rknn_context,uint64_t bytes,uint64_t flags) {
    assert(flags==RKNN_FLAG_MEMORY_CACHEABLE);
    if(fault==ALLOCATE)return nullptr;
    auto *m=static_cast<rknn_tensor_mem *>(std::calloc(1,sizeof(rknn_tensor_mem)));
    m->virt_addr=std::malloc(bytes);m->size=bytes;++allocated;return m;
}
int rknn_destroy_mem(rknn_context,rknn_tensor_mem *m){++freed;std::free(m->virt_addr);std::free(m);return 0;}
int rknn_set_io_mem(rknn_context c,rknn_tensor_mem *,rknn_tensor_attr *a) {
    assert(a->index==0 && a->type==RKNN_TENSOR_UINT8 && a->fmt==RKNN_TENSOR_NHWC && !a->pass_through);
    assert(a->w_stride==padded_width && a->h_stride==height);
    contexts.at(c).bound=true;return fault==BIND?-1:0;
}
int rknn_mem_sync(rknn_context c,rknn_tensor_mem *m,rknn_mem_sync_mode direction) {
    assert(contexts.at(c).bound && direction==RKNN_MEMORY_SYNC_TO_DEVICE);
    verify_rgb(m->virt_addr,padded_width*3);++flushed;return fault==SYNC?-1:0;
}
#endif
int rknn_inputs_set(rknn_context c,uint32_t n,rknn_input in[]) {
    // Even a failed bind may have mutated runtime state. Require a fresh
    // context before falling back to the ordinary SDK input API.
    assert(!contexts.at(c).bound);
    assert(n==1 && in[0].type==RKNN_TENSOR_UINT8 && in[0].fmt==RKNN_TENSOR_NHWC && !in[0].pass_through);
    assert(in[0].size==expected.size());verify_rgb(in[0].buf,width*3);++legacy;return 0;
}
int rknn_run(rknn_context c,rknn_run_extend *) {
    assert(contexts.at(c).core==RKNN_NPU_CORE_2);++ran;
    return fault==RUN_ALWAYS || (fault==RUN && contexts.at(c).bound)?-1:0;
}
int rknn_outputs_get(rknn_context,uint32_t n,rknn_output out[],rknn_output_extend *) {
    for(unsigned i=0;i<n;i++){assert(out[i].want_float==1);unsigned grid=4>>(i/2);out[i].size=grid*grid*(i%2?1:64)*sizeof(float);out[i].buf=std::calloc(1,out[i].size);}return 0;
}
int rknn_outputs_release(rknn_context,uint32_t n,rknn_output out[]){for(unsigned i=0;i<n;i++)std::free(out[i].buf);return 0;}
}
static void scenario(Fault f,bool force_legacy,bool old_preprocess) {
    fault=f;initialized=destroyed=allocated=freed=legacy=flushed=ran=0;contexts.clear();
    setenv("PERCEPTION_INPUT",force_legacy?"legacy":"auto",1);
    setenv("PERCEPTION_PREPROCESS",old_preprocess?"legacy":"auto",1);
    std::vector<unsigned char> rgb(64*48*3);for(unsigned i=0;i<rgb.size();i++)rgb[i]=(i*17+i/7)%256;
    std::string error;detector_math::Letterbox box;
    assert(detector_math::preprocess_rgb(rgb.data(),64,48,64*3,width,height,expected,box,error));
    {
        RknnDetector detector;assert(detector.open("build/input-model.stub",.35f,.45f,1,error));
        assert(detector.set_core(2,error));
        std::vector<Detection> result;DetectorTiming timing;
        for(int i=0;i<3;i++) {
            assert(detector.infer(rgb.data(),64,48,64*3,result,error,&timing));
            assert(result.empty() && timing.total_ms>=0);
            assert(timing.preprocess_ms+timing.input_ms+timing.run_ms+timing.output_ms+timing.postprocess_ms+timing.release_ms<=timing.total_ms+0.01);
        }
#ifndef NO_IO_API
        const bool fast=f==NONE && !force_legacy;
        assert(ran==(f==RUN && !force_legacy?4:3));
        assert(legacy==(fast?0:3));
        assert(flushed==(fast?3:(f==SYNC||f==RUN) && !force_legacy?1:0));
        assert(initialized==((f==BIND||f==SYNC||f==RUN)&&!force_legacy?2:1));
#else
        assert(ran==3 && legacy==3 && flushed==0 && initialized==1);
#endif
    }
    assert(initialized==destroyed && allocated==freed && contexts.empty());
}
static void terminal_run_failure() {
    fault=RUN_ALWAYS;initialized=destroyed=allocated=freed=legacy=flushed=ran=0;contexts.clear();
    setenv("PERCEPTION_INPUT","auto",1);setenv("PERCEPTION_PREPROCESS","auto",1);
    std::vector<unsigned char> rgb(64*48*3,114);std::string error;detector_math::Letterbox box;
    assert(detector_math::preprocess_rgb(rgb.data(),64,48,64*3,width,height,expected,box,error));
    {
        RknnDetector detector;assert(detector.open("build/input-model.stub",.35f,.45f,1,error));
        assert(detector.set_core(2,error));std::vector<Detection> result;DetectorTiming timing;
        assert(!detector.infer(rgb.data(),64,48,64*3,result,error,&timing));
        assert(!error.empty() && result.empty() && timing.output_ms==0);
#ifndef NO_IO_API
        assert(initialized==2 && ran==2 && legacy==1 && flushed==1);
#else
        assert(initialized==1 && ran==1 && legacy==1 && flushed==0);
#endif
        const int existing_contexts=initialized,previous_runs=ran;
        assert(!detector.infer(rgb.data(),64,48,64*3,result,error,&timing));
        assert(initialized==existing_contexts && ran==previous_runs+1);
    }
    assert(initialized==destroyed && allocated==freed && contexts.empty());
}
int main() {
    FILE *f=std::fopen("build/input-model.stub","wb");assert(f);std::fputs("stub",f);std::fclose(f);
    scenario(NONE,false,false);scenario(NONE,false,true);scenario(NONE,true,false);
    for(Fault failure:{QUERY,ALLOCATE,BIND,STRIDE,SIZE,SYNC,RUN})scenario(failure,false,false);
    terminal_run_failure();
    std::remove("build/input-model.stub");
    std::puts("PASS persistent UINT8 inputs: exact RGB/letterbox bytes, padded stride, synchronization, API fallback, bind/reset/core recovery, and balanced teardown (SDK stub)");
}
