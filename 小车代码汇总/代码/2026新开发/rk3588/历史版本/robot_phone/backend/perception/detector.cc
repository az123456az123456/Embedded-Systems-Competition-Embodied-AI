#include "detector.h"
#include "detector_math.h"

#include <fstream>
#include <limits>
#include <sstream>

#ifdef HAVE_RKNN
#include "rknn_api.h"
#endif

struct RknnDetector::Impl {
#ifdef HAVE_RKNN
    rknn_context context;
    std::vector<rknn_tensor_attr> output_attrs;
    std::vector<unsigned char> input;
    int width, height;
    float confidence, nms;
    Impl() : context(0), width(0), height(0), confidence(0.35f), nms(0.45f) {}
    ~Impl() { if (context) rknn_destroy(context); }
#endif
};

RknnDetector::RknnDetector() : impl_(new Impl) {}
RknnDetector::~RknnDetector() {}

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
    for (const rknn_tensor_attr &attr : next->output_attrs) {
        uint64_t expected=1;
        for (uint32_t d=0; d<attr.n_dims; ++d) expected*=attr.dims[d];
        if (expected != attr.n_elems || expected > std::numeric_limits<uint32_t>::max()/sizeof(float)) {
            error="RKNN output element count does not match its declared shape";
            return false;
        }
    }
    impl_.swap(next);
    return true;
#endif
}

bool RknnDetector::infer(const unsigned char *rgb, int width, int height, int stride,
                         std::vector<Detection> &results, std::string &error) {
    results.clear(); error.clear();
#ifndef HAVE_RKNN
    (void)rgb; (void)width; (void)height; (void)stride;
    error="RKNN inference is unavailable in this build";
    return false;
#else
    if (!impl_->context) { error="RKNN model has not been opened"; return false; }
    detector_math::Letterbox box;
    if (!detector_math::preprocess_rgb(rgb,width,height,stride,impl_->width,impl_->height,impl_->input,box,error)) return false;
    rknn_input input={};
    input.index=0;
    input.buf=impl_->input.data();
    input.size=static_cast<uint32_t>(impl_->input.size());
    input.type=RKNN_TENSOR_UINT8;
    input.fmt=RKNN_TENSOR_NHWC;
    input.pass_through=0; // RKNN model contains the 1/255 normalization.
    int ret=rknn_inputs_set(impl_->context,1,&input);
    if (ret < 0) { error=runtime_error("rknn_inputs_set",ret); return false; }
    ret=rknn_run(impl_->context,NULL);
    if (ret < 0) { error=runtime_error("rknn_run",ret); return false; }
    Outputs outputs(impl_->context,impl_->output_attrs.size());
    ret=outputs.get();
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
    ret=outputs.release();
    if (ret < 0) { results.clear(); error=runtime_error("rknn_outputs_release",ret); return false; }
    return decoded;
#endif
}
