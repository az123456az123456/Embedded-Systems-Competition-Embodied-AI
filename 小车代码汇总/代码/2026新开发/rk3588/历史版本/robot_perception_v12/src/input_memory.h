#ifndef PERCEPTION_INPUT_MEMORY_H
#define PERCEPTION_INPUT_MEMORY_H

// Optional RKNN input binding. Kept behind dynamically resolved API calls so
// installations with an older runtime can retain rknn_inputs_set unchanged.
#include "rknn_api.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <string>

struct RknnInputMemory {
    using Create=rknn_tensor_mem *(*)(rknn_context,uint32_t);
    using Create2=rknn_tensor_mem *(*)(rknn_context,uint64_t,uint64_t);
    using Destroy=int (*)(rknn_context,rknn_tensor_mem *);
    using Bind=int (*)(rknn_context,rknn_tensor_mem *,rknn_tensor_attr *);
    using Sync=int (*)(rknn_context,rknn_tensor_mem *,rknn_mem_sync_mode);
    Create create=nullptr;Create2 create2=nullptr;Destroy destroy=nullptr;
    Bind bind=nullptr;Sync sync=nullptr;
    rknn_tensor_mem *memory=nullptr;rknn_tensor_attr attr={};
    std::size_t row_bytes=0;bool binding_attempted=false;

    void release(rknn_context context) {
        if(memory && destroy)destroy(context,memory);
        memory=nullptr;row_bytes=0;
    }
    bool open(rknn_context context,unsigned width,unsigned height,std::string &reason) {
        constexpr uint64_t limit=64u*1024u*1024u;
        create=reinterpret_cast<Create>(dlsym(RTLD_DEFAULT,"rknn_create_mem"));
        create2=reinterpret_cast<Create2>(dlsym(RTLD_DEFAULT,"rknn_create_mem2"));
        destroy=reinterpret_cast<Destroy>(dlsym(RTLD_DEFAULT,"rknn_destroy_mem"));
        bind=reinterpret_cast<Bind>(dlsym(RTLD_DEFAULT,"rknn_set_io_mem"));
        sync=reinterpret_cast<Sync>(dlsym(RTLD_DEFAULT,"rknn_mem_sync"));
        if((!create&&!create2)||!destroy||!bind||!sync) {
            reason="optional input-memory/cache API unavailable";return false;
        }
        attr={};
        int rc=rknn_query(context,RKNN_QUERY_NATIVE_INPUT_ATTR,&attr,sizeof(attr));
        if(rc<0 || attr.fmt!=RKNN_TENSOR_NHWC) {
            attr={};
            rc=rknn_query(context,RKNN_QUERY_NATIVE_NHWC_INPUT_ATTR,&attr,sizeof(attr));
        }
        if(rc<0 || attr.fmt!=RKNN_TENSOR_NHWC || attr.n_dims!=4 ||
           attr.dims[0]!=1 || attr.dims[1]!=height || attr.dims[2]!=width || attr.dims[3]!=3) {
            reason="native NHWC RGB geometry unavailable";return false;
        }
        unsigned element_bytes=0;
        if(attr.type==RKNN_TENSOR_UINT8 || attr.type==RKNN_TENSOR_INT8)element_bytes=1;
        if(attr.type==RKNN_TENSOR_FLOAT16)element_bytes=2;
        if(attr.type==RKNN_TENSOR_FLOAT32)element_bytes=4;
        const uint64_t stride=attr.w_stride?attr.w_stride:width;
        const uint64_t elements=static_cast<uint64_t>(width)*height*3;
        const uint64_t occupied=stride*height*3;
        if(!element_bytes || stride<width || occupied>limit ||
           elements!=attr.n_elems || attr.size<elements*element_bytes ||
           occupied*element_bytes>limit || attr.size_with_stride<occupied*element_bytes ||
           attr.size_with_stride>limit) {
            reason="native RGB tensor stride/size outside supported bounds";return false;
        }
        memory=create2?create2(context,attr.size_with_stride,RKNN_FLAG_MEMORY_CACHEABLE)
                      :create(context,attr.size_with_stride);
        if(!memory || !memory->virt_addr || memory->size<attr.size_with_stride) {
            reason="persistent input allocation failed";release(context);return false;
        }
        // Match Rockchip's zero-copy examples: UINT8 + pass_through=0 retains
        // model normalization. Native FP16/INT8 pass-through would require the
        // caller to duplicate model-specific normalization/quantization.
        attr.type=RKNN_TENSOR_UINT8;attr.fmt=RKNN_TENSOR_NHWC;
        attr.pass_through=0;attr.h_stride=height;
        std::memset(memory->virt_addr,114,memory->size);
        binding_attempted=true;
        if(bind(context,memory,&attr)<0) {
            reason="runtime rejected persistent UINT8 input binding";
            release(context);return false;
        }
        row_bytes=static_cast<std::size_t>(stride)*3;
        return true;
    }
    bool flush(rknn_context context)const {
        return memory && sync && sync(context,memory,RKNN_MEMORY_SYNC_TO_DEVICE)>=0;
    }
};

#endif
