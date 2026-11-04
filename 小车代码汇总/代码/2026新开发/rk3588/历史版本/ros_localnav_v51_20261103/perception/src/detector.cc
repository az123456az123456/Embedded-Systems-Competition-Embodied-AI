#include "detector.h"
#include "detector_math.h"
#include "stage_clock.h"
#include "preprocess_plan.h"
#include "preprocess_rga.h"
#include "fpga_rgb_input.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

#ifdef HAVE_RKNN
#include "rknn_api.h"
#include <dlfcn.h>
#include "input_memory.h"
#endif

struct RknnDetector::Impl {
#ifdef HAVE_RKNN
    rknn_context context;
    std::vector<rknn_tensor_attr> output_attrs;
    std::vector<unsigned char> input;
    std::vector<unsigned char> model_bytes;
    detector_math::RgbPreprocessor preprocessor;
    detector_math::RgaPreprocessor rga;
    RknnInputMemory input_memory;
    std::string input_reason="not initialized";
    int core=-1;
    bool input_reported=false;
    bool rga_reported=false;
    int rga_status=-1;
    int width, height;
    float confidence, nms;
    Impl() : context(0), width(0), height(0), confidence(0.35f), nms(0.45f) {}
    ~Impl() { if(context){input_memory.release(context);rknn_destroy(context);} }
    bool rebuild_legacy(std::string &error) {
        input_memory.release(context);
        if(context)rknn_destroy(context);
        context=0;
        int rc=rknn_init(&context,model_bytes.data(),static_cast<uint32_t>(model_bytes.size()),0,NULL);
        if(rc<0){error="Failed to recreate legacy RKNN input context";return false;}
        if(core>=0) {
            using SetCore=int (*)(rknn_context,rknn_core_mask);
            auto set=reinterpret_cast<SetCore>(dlsym(RTLD_DEFAULT,"rknn_set_core_mask"));
            if(!set || set(context,static_cast<rknn_core_mask>(1<<core))<0) {
                error="Failed to restore NPU core after legacy input fallback";return false;
            }
        }
        model_bytes.clear();model_bytes.shrink_to_fit();
        return true;
    }
#endif
};

RknnDetector::RknnDetector() : impl_(new Impl) {}
RknnDetector::~RknnDetector() {}

bool RknnDetector::set_core(int core, std::string &error) {
    error.clear();
#ifdef HAVE_RKNN
    if(!impl_->context || core < -1 || core > 2){error="Invalid NPU context/core";return false;}
    using SetCore=int (*)(rknn_context,rknn_core_mask);
    auto set=reinterpret_cast<SetCore>(dlsym(RTLD_DEFAULT,"rknn_set_core_mask"));
    if(!set){error="Runtime has no rknn_set_core_mask; keeping automatic core selection";return false;}
    auto mask=core<0?RKNN_NPU_CORE_AUTO:static_cast<rknn_core_mask>(1<<core);
    int ret=set(impl_->context,mask);
    if(ret<0){error="Runtime rejected NPU core selection; keeping automatic selection";return false;}
    impl_->core=core;
    return true;
#else
    (void)core;error="NPU core selection unavailable in this build";return false;
#endif
}

#ifdef HAVE_RKNN
namespace {
std::string runtime_error(const char *operation, int code) {
    std::ostringstream text;
    text << operation << " failed (RKNN " << code << ")";
    return text.str();
}
struct Outputs {
    rknn_context context;
    std::vector<rknn_output> data;
    bool acquired;
    Outputs(rknn_context ctx, std::size_t count) : context(ctx), data(count), acquired(false) {
        for (std::size_t i=0; i<count; ++i) {
            data[i].index=static_cast<uint32_t>(i);
            // The SDK converts/dequantizes FP16, INT8 or UINT8 into ordinary
            // NCHW float buffers; the decoder never reinterprets raw INT8.
            data[i].want_float=1;
        }
    }
    ~Outputs() { if (acquired) rknn_outputs_release(context,data.size(),data.data()); }
    int get() {
        const int ret=rknn_outputs_get(context,data.size(),data.data(),NULL);
        acquired=ret >= 0;
        // Some runtime failures may have already allocated output buffers.
        if (!acquired) for (const rknn_output &out : data) if (out.buf) acquired=true;
        return ret;
    }
    int release() {
        if (!acquired) return 0;
        acquired=false;
        return rknn_outputs_release(context,data.size(),data.data());
    }
};
}
#endif

bool RknnDetector::open(const std::string &model, float confidence, float nms,
                        int class_count, std::string &error) {
    error.clear();
#ifndef HAVE_RKNN
    (void)model; (void)confidence; (void)nms; (void)class_count;
    error="RKNN support was not compiled into this program; rebuild with HAVE_RKNN and the RK3588 runtime";
    return false;
#else
    if (!(confidence >= 0 && confidence <= 1) || !(nms >= 0 && nms <= 1) ||
        class_count < 1 || class_count > 10000) {
        error="Invalid confidence/NMS threshold or class count";
        return false;
    }
    std::ifstream file(model.c_str(),std::ios::binary|std::ios::ate);
    if (!file) { error="Cannot open RKNN model: "+model; return false; }
    const std::streamoff length=file.tellg();
    if (length <= 0 || length > 512LL*1024*1024) {
        error="RKNN model is empty or exceeds the 512 MiB limit";
        return false;
    }
    std::vector<unsigned char> bytes(static_cast<std::size_t>(length));
    file.seekg(0);
    if (!file.read(reinterpret_cast<char *>(bytes.data()),length)) {
        error="Cannot read complete RKNN model: "+model;
        return false;
    }
    std::unique_ptr<Impl> next(new Impl);
    int ret=rknn_init(&next->context,bytes.data(),static_cast<uint32_t>(bytes.size()),0,NULL);
    if (ret < 0) { error=runtime_error("rknn_init",ret); return false; }
    rknn_input_output_num count={};
    ret=rknn_query(next->context,RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count));
    if (ret < 0) { error=runtime_error("query input/output count",ret); return false; }
    if (count.n_input != 1 || (count.n_output != 6 && count.n_output != 9)) {
        error="Expected one RGB input and 6/9 Rockchip YOLOv8 DFL/class outputs; standard Ultralytics [1,84,8400] output is incompatible";
        return false;
    }
    rknn_tensor_attr input={};
    ret=rknn_query(next->context,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input));
    if (ret < 0) { error=runtime_error("query input attributes",ret); return false; }
    if (input.n_dims != 4 || input.dims[0] != 1 ||
        (input.fmt != RKNN_TENSOR_NCHW && input.fmt != RKNN_TENSOR_NHWC)) {
        error="Model input must be batch-one NCHW/NHWC RGB";
        return false;
    }
    const uint32_t channels=input.fmt == RKNN_TENSOR_NCHW ? input.dims[1] : input.dims[3];
    const uint32_t height=input.fmt == RKNN_TENSOR_NCHW ? input.dims[2] : input.dims[1];
    const uint32_t width=input.fmt == RKNN_TENSOR_NCHW ? input.dims[3] : input.dims[2];
    if (channels != 3 || width == 0 || height == 0 || width > 16384 || height > 16384 ||
        static_cast<uint64_t>(width)*height*3 > 64u*1024u*1024u) {
        error="Model input has invalid dimensions or does not have three RGB channels";
        return false;
    }
    next->width=width; next->height=height;
    next->confidence=confidence; next->nms=nms;
    std::vector<detector_math::TensorShape> shapes;
    next->output_attrs.resize(count.n_output);
    for (uint32_t i=0; i<count.n_output; ++i) {
        rknn_tensor_attr &attr=next->output_attrs[i];
        attr.index=i;
        ret=rknn_query(next->context,RKNN_QUERY_OUTPUT_ATTR,&attr,sizeof(attr));
        if (ret < 0) { error=runtime_error("query output attributes",ret); return false; }
        if (attr.type != RKNN_TENSOR_FLOAT32 && attr.type != RKNN_TENSOR_FLOAT16 &&
            attr.type != RKNN_TENSOR_INT8 && attr.type != RKNN_TENSOR_UINT8) {
            error="Unsupported RKNN output data type";
            return false;
        }
        detector_math::TensorShape shape;
        shape.nchw=attr.fmt == RKNN_TENSOR_NCHW;
        if (attr.n_dims > RKNN_MAX_DIMS) { error="Invalid RKNN output rank"; return false; }
        for (uint32_t d=0; d<attr.n_dims; ++d) {
            if (attr.dims[d] > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
                error="Invalid RKNN output dimensions"; return false;
            }
            shape.dims.push_back(static_cast<int>(attr.dims[d]));
        }
        shapes.push_back(shape);
    }
    if (!detector_math::validate_contract(shapes,width,height,class_count,error)) return false;
    unsigned quantized_outputs=0;
    for(const auto &attr:next->output_attrs)
        if(attr.type==RKNN_TENSOR_INT8 || attr.type==RKNN_TENSOR_UINT8)++quantized_outputs;
    std::fprintf(stderr,"MODEL path=%s input=%ux%ux%u input_type=%d quantized_outputs=%u/%u\n",
                 model.c_str(),width,height,channels,static_cast<int>(input.type),quantized_outputs,count.n_output);
    for (const rknn_tensor_attr &attr : next->output_attrs) {
        uint64_t expected=1;
        for (uint32_t d=0; d<attr.n_dims; ++d) expected*=attr.dims[d];
        if (expected != attr.n_elems || expected > std::numeric_limits<uint32_t>::max()/sizeof(float)) {
            error="RKNN output element count does not match its declared shape";
            return false;
        }
    }
    const char *mode=std::getenv("PERCEPTION_INPUT");
    if(mode && std::strcmp(mode,"legacy")==0)next->input_reason="forced by PERCEPTION_INPUT=legacy";
    else if(mode && std::strcmp(mode,"auto")!=0)next->input_reason="invalid PERCEPTION_INPUT (use auto or legacy)";
    else {
        if(next->input_memory.open(next->context,width,height,next->input_reason)) {
            // Retain the already loaded bytes only while an optional binding
            // exists, allowing a clean context reset if cache synchronization
            // fails later. The fallback always uses these exact model bytes.
            next->model_bytes=std::move(bytes);
            next->input_reason="persistent UINT8 NHWC, model normalization retained";
        } else if(next->input_memory.binding_attempted) {
            next->model_bytes=std::move(bytes);
            if(!next->rebuild_legacy(error))return false;
        }
    }
    impl_.swap(next);
    return true;
#endif
}

bool RknnDetector::infer(const unsigned char *rgb, int width, int height, int stride,
                         std::vector<Detection> &results, std::string &error, DetectorTiming *timing) {
    results.clear(); error.clear();
    DetectorTiming measured;
    if(!timing)timing=&measured;
    *timing={};StageTotal total(&timing->total_ms);
#ifndef HAVE_RKNN
    (void)rgb; (void)width; (void)height; (void)stride;
    error="RKNN inference is unavailable in this build";
    return false;
#else
    if (!impl_->context) { error="RKNN model has not been opened"; return false; }
    auto tick=StageClock::now();
    detector_math::Letterbox box;
    const char *preprocess=std::getenv("PERCEPTION_PREPROCESS");
    const bool old_preprocess=preprocess && std::strcmp(preprocess,"legacy")==0;
    const bool use_rga=preprocess && (!std::strcmp(preprocess,"auto") || !std::strcmp(preprocess,"rga"));
    detector_math::PreprocessTiming prep;
    struct PrepareFinish {
        DetectorTiming *timing;detector_math::PreprocessTiming *prep;
        StageClock::time_point begin=StageClock::now();
        ~PrepareFinish() {
            timing->preprocess_ms+=stage_elapsed(begin);
            timing->prep_plan_ms=prep->plan_ms;timing->prep_fill_ms=prep->fill_ms;
            timing->prep_resize_ms=prep->resize_ms;timing->prep_copy_ms=prep->copy_ms;
            timing->prep_rga_ms=prep->rga_ms;timing->prep_sync_ms=prep->sync_ms;
            timing->prep_validate_ms=prep->validate_ms;timing->preprocess_backend=prep->backend;
        }
    };
    auto prepare=[&](bool direct)->bool {
        PrepareFinish finish{timing,&prep};
        if(width==640 && height==640 && impl_->width==640 && impl_->height==640) {
            if(!direct)impl_->input.resize(640u*640u*3u);
            auto *dst=direct?static_cast<unsigned char *>(impl_->input_memory.memory->virt_addr):impl_->input.data();
            const auto bytes=direct?impl_->input_memory.memory->size:impl_->input.size();
            const auto row=direct?impl_->input_memory.row_bytes:640u*3u;
            const auto begin=StageClock::now();prep.backend=3;
            const bool ok=detector_math::copy_identity_rgb(rgb,width,height,stride,dst,bytes,row,
                                                          impl_->width,impl_->height,box,error);
            prep.copy_ms+=stage_elapsed(begin);return ok;
        }
        if(old_preprocess) {
            const auto begin=StageClock::now();prep.backend=2;
            if(!detector_math::preprocess_rgb(rgb,width,height,stride,impl_->width,impl_->height,impl_->input,box,error))return false;
            prep.resize_ms+=stage_elapsed(begin);
            const auto copy=StageClock::now();
            if(direct)for(int y=0;y<impl_->height;y++)
                std::memcpy(static_cast<unsigned char *>(impl_->input_memory.memory->virt_addr)+y*impl_->input_memory.row_bytes,
                            impl_->input.data()+static_cast<std::size_t>(y)*impl_->width*3,impl_->width*3);
            prep.copy_ms+=stage_elapsed(copy);
            return true;
        }
        if(!direct)impl_->input.resize(static_cast<std::size_t>(impl_->width)*impl_->height*3);
        auto *destination=direct?static_cast<unsigned char *>(impl_->input_memory.memory->virt_addr):impl_->input.data();
        const auto bytes=direct?impl_->input_memory.memory->size:impl_->input.size();
        const auto row=direct?impl_->input_memory.row_bytes:static_cast<std::size_t>(impl_->width)*3;
        bool ok=false;
        if(use_rga) {
            ok=impl_->rga.run(rgb,width,height,stride,impl_->width,impl_->height,destination,bytes,row,box,prep);
            if(!impl_->rga_reported || impl_->rga_status!=static_cast<int>(ok)) {
                std::fprintf(stderr,"PREPROCESS core=%d requested=%s active=%s zero_copy=no reason=%s\n",impl_->core,
                             preprocess,ok?"rga-staged-dma":"cached-rb64",impl_->rga.reason().c_str());
                impl_->rga_reported=true;
                impl_->rga_status=static_cast<int>(ok);
            }
        }
        if(!ok) {
            prep.backend=0;
            ok=impl_->preprocessor.run(rgb,width,height,stride,impl_->width,impl_->height,
                                       destination,bytes,row,box,error,&prep);
        }
        return ok;
    };
    if(!prepare(impl_->input_memory.memory!=nullptr))return false;
    tick=StageClock::now();
    double fallback_preprocess_ms=0;
    bool synchronized=true;
    if(impl_->input_memory.memory) {
        const auto sync_start=StageClock::now();synchronized=impl_->input_memory.flush(impl_->context);
        timing->input_sync_ms+=stage_elapsed(sync_start);
    }
    if(impl_->input_memory.memory && !synchronized) {
        impl_->input_reason="cache synchronization failed; clean legacy context restored";
        if(!impl_->rebuild_legacy(error))return false;
        // The first attempt wrote directly to the old bound memory. Recreate
        // the normal RGB input after reset rather than sending an empty vector.
        auto preprocess_start=StageClock::now();
        if(!prepare(false))return false;
        fallback_preprocess_ms=stage_elapsed(preprocess_start);
        std::fprintf(stderr,"INPUT core=%d path=legacy fallback=cache-sync\n",impl_->core);
    }
    int ret=0;
    if(!impl_->input_memory.memory) {
        rknn_input input={};
        input.index=0;input.buf=impl_->input.data();
        input.size=static_cast<uint32_t>(impl_->input.size());
        input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;
        input.pass_through=0; // RKNN model contains the 1/255 normalization.
        ret=rknn_inputs_set(impl_->context,1,&input);
    }
    timing->input_ms=std::max(0.,stage_elapsed(tick)-fallback_preprocess_ms);tick=StageClock::now();
    if (ret < 0) { error=runtime_error("rknn_inputs_set",ret); return false; }
    if(!impl_->input_reported) {
        std::fprintf(stderr,"INPUT core=%d path=%s row_bytes=%zu preprocess=%s reason=%s\n",impl_->core,
                     impl_->input_memory.memory?"io-memory-u8":"legacy",
                     impl_->input_memory.memory?impl_->input_memory.row_bytes:static_cast<std::size_t>(impl_->width)*3,
                     timing->preprocess_backend==3?"identity-rgb640":old_preprocess?"legacy":timing->preprocess_backend==1?"rga-staged-dma":
                     preprocess&&std::strcmp(preprocess,"gather")==0?"gather":
                     preprocess&&std::strcmp(preprocess,"pixel")==0?"cached-pixel":"cached-rb64",impl_->input_reason.c_str());
        impl_->input_reported=true;
    }
    ret=rknn_run(impl_->context,NULL);
    timing->run_ms=stage_elapsed(tick);tick=StageClock::now();
    if(ret<0 && impl_->input_memory.memory) {
        // Some runtimes accept the input binding but reject mixed bound-input
        // /ordinary-output execution only at run time. Permanently leave the
        // optional path after one failure and retry this exact frame once.
        const int binding_run_error=ret;
        impl_->input_reason="bound-input execution failed; clean legacy context restored";
        if(!impl_->rebuild_legacy(error))return false;
        timing->input_ms+=stage_elapsed(tick);tick=StageClock::now();
        if(!prepare(false))return false;
        tick=StageClock::now();
        rknn_input input={};
        input.index=0;input.buf=impl_->input.data();
        input.size=static_cast<uint32_t>(impl_->input.size());
        input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;input.pass_through=0;
        ret=rknn_inputs_set(impl_->context,1,&input);
        timing->input_ms+=stage_elapsed(tick);tick=StageClock::now();
        if(ret<0){error=runtime_error("rknn_inputs_set after bound-input fallback",ret);return false;}
        ret=rknn_run(impl_->context,NULL);
        timing->run_ms+=stage_elapsed(tick);tick=StageClock::now();
        std::fprintf(stderr,"INPUT core=%d path=legacy fallback=bound-run-error code=%d\n",
                     impl_->core,binding_run_error);
    }
    if (ret < 0) { error=runtime_error("rknn_run",ret); return false; }
    Outputs outputs(impl_->context,impl_->output_attrs.size());
    ret=outputs.get();
    timing->output_ms=stage_elapsed(tick);tick=StageClock::now();
    if (ret < 0) { error=runtime_error("rknn_outputs_get",ret); return false; }
    for (std::size_t i=0; i<outputs.data.size(); ++i) {
        const uint64_t expected=static_cast<uint64_t>(impl_->output_attrs[i].n_elems)*sizeof(float);
        if (!outputs.data[i].buf || outputs.data[i].size < expected) {
            error="RKNN returned a missing or truncated float output buffer";
            return false;
        }
    }
    const std::size_t per_branch=outputs.data.size()/3;
    std::vector<detector_math::Head> heads;
    for (std::size_t branch=0; branch<3; ++branch) {
        const std::size_t idx=branch*per_branch;
        const rknn_tensor_attr &attr=impl_->output_attrs[idx];
        detector_math::Head head;
        head.boxes=static_cast<const float *>(outputs.data[idx].buf);
        head.classes=static_cast<const float *>(outputs.data[idx+1].buf);
        head.score_sum=per_branch == 3 ? static_cast<const float *>(outputs.data[idx+2].buf) : NULL;
        head.grid_w=attr.dims[3]; head.grid_h=attr.dims[2]; head.dfl_len=attr.dims[1]/4;
        head.class_count=impl_->output_attrs[idx+1].dims[1];
        heads.push_back(head);
    }
    const bool decoded=detector_math::decode_heads(heads,box,impl_->confidence,impl_->nms,results,error);
    timing->postprocess_ms=stage_elapsed(tick);tick=StageClock::now();
    ret=outputs.release();
    timing->release_ms=stage_elapsed(tick);
    if (ret < 0) { results.clear(); error=runtime_error("rknn_outputs_release",ret); return false; }
    return decoded;
#endif
}
