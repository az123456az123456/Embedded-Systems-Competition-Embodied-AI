#define STB_IMAGE_IMPLEMENTATION
#define STBI_MAX_DIMENSIONS 4096
#include "stb_image.h"
#include "rknn_api.h"
#include "rkllm.h"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <sys/file.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

using Json = nlohmann::json;
static double milliseconds() {
    return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static void require(bool okay, const std::string& message) { if (!okay) throw std::runtime_error(message); }
static long available_kb() {
    std::ifstream f("/proc/meminfo");std::string line;
    while(std::getline(f,line))if(line.rfind("MemAvailable:",0)==0)return std::stol(line.substr(13));
    return 0;
}
static void resource_check() {
    require(available_kb() >= 1000*1024, "insufficient_memory: need at least 1000 MiB available before loading");
    DIR* directory=opendir("/proc");require(directory,"cannot inspect active NPU users");
    bool yolo=false;
    while(auto* entry=readdir(directory)) {
        if(!std::isdigit(static_cast<unsigned char>(entry->d_name[0])))continue;
        std::ifstream f(std::string("/proc/")+entry->d_name+"/cmdline");
        std::string cmd((std::istreambuf_iterator<char>(f)),std::istreambuf_iterator<char>());
        if(cmd.find("perception-dashboard")!=std::string::npos && cmd.find("--no-detect")==std::string::npos)yolo=true;
    }
    closedir(directory);
    require(!yolo,"npu_busy: stop YOLO or select the explicit --no-detect semantic mode before loading");
}
static std::vector<unsigned char> image_rgb(const std::string& path,int width,int height,bool left_eye,int& source_width) {
    int w=0,h=0,n=0;
    unsigned char* data=stbi_load(path.c_str(),&w,&h,&n,3);
    require(data,"cannot decode input image");
    const int stride=w;
    if(left_eye){
        if(w%2!=0){stbi_image_free(data);throw std::runtime_error("stereo frame width must be even");}
        w/=2;
    }
    source_width=w;
    std::vector<unsigned char> output(static_cast<size_t>(width)*height*3,128);
    const double square=std::max(w,h),left=(square-w)/2.0,top=(square-h)/2.0;
    for(int y=0;y<height;++y)for(int x=0;x<width;++x) {
        const double sx=(x+.5)*square/width-.5-left,sy=(y+.5)*square/height-.5-top;
        if(sx<-.5||sy<-.5||sx>w-.5||sy>h-.5)continue;
        const double xx=std::clamp(sx,0.,double(w-1)),yy=std::clamp(sy,0.,double(h-1));
        const int x0=static_cast<int>(xx),y0=static_cast<int>(yy),x1=std::min(x0+1,w-1),y1=std::min(y0+1,h-1);
        const double fx=xx-x0,fy=yy-y0;
        for(int c=0;c<3;++c) {
            const double a=data[(y0*stride+x0)*3+c]*(1-fx)+data[(y0*stride+x1)*3+c]*fx;
            const double b=data[(y1*stride+x0)*3+c]*(1-fx)+data[(y1*stride+x1)*3+c]*fx;
            output[(y*width+x)*3+c]=static_cast<unsigned char>(std::clamp(std::lround(a*(1-fy)+b*fy),0L,255L));
        }
    }
    stbi_image_free(data);return output;
}
struct Response {
    std::mutex mutex;std::condition_variable cv;std::string text;bool done=false,error=false;
};
static int callback(RKLLMResult* result,void* userdata,LLMCallState state) {
    if(!userdata)return -1;
    auto& r=*static_cast<Response*>(userdata);std::lock_guard<std::mutex> lock(r.mutex);
    if(state==RKLLM_RUN_NORMAL && result && result->text)r.text+=result->text;
    if(state==RKLLM_RUN_FINISH||state==RKLLM_RUN_ERROR){r.error=state==RKLLM_RUN_ERROR;r.done=true;r.cv.notify_all();}
    return 0;
}
class Model {
    rknn_context vision_=0;LLMHandle llm_=nullptr;
    rknn_tensor_attr in_{},out_{};int tokens_=0,hidden_=0;
public:
    ~Model(){if(vision_)rknn_destroy(vision_);if(llm_)rkllm_destroy(llm_);}
    void load(const std::string& vision,const std::string& language,int max_tokens) {
        resource_check();
        RKLLMParam p=rkllm_createDefaultParam();p.model_path=language.c_str();p.max_new_tokens=max_tokens;p.max_context_len=512;
        p.top_k=1;p.skip_special_token=true;p.extend_param.base_domain_id=1;
        p.img_start="<fake_token_around_image><global-img>";
        p.img_end="<fake_token_around_image>";p.img_content="<image>";
        require(rkllm_init(&llm_,&p,callback)==0,"rkllm_init failed");
        // SmolVLM uses role capitalization and end_of_utterance, unlike Qwen ChatML.
        require(rkllm_set_chat_template(llm_,"<|im_start|>","User:","<end_of_utterance>\nAssistant:")==0,"chat template failed");
        require(rknn_init(&vision_,const_cast<char*>(vision.c_str()),0,0,nullptr)==0,"rknn_init failed");
        require(rknn_set_core_mask(vision_,RKNN_NPU_CORE_0_1_2)==0,"cannot select NPU cores");
        rknn_input_output_num io{};require(rknn_query(vision_,RKNN_QUERY_IN_OUT_NUM,&io,sizeof(io))==0,"query IO failed");
        require(io.n_input==1&&io.n_output==1,"unsupported encoder IO count");
        require(rknn_query(vision_,RKNN_QUERY_INPUT_ATTR,&in_,sizeof(in_))==0,"query input failed");
        require(rknn_query(vision_,RKNN_QUERY_OUTPUT_ATTR,&out_,sizeof(out_))==0,"query output failed");
        const int height=in_.fmt==RKNN_TENSOR_NCHW?in_.dims[2]:in_.dims[1];
        const int width=in_.fmt==RKNN_TENSOR_NCHW?in_.dims[3]:in_.dims[2];
        require(width==384&&height==384,"encoder must match fixed 384x384 model");
        for(unsigned i=0;i+1<out_.n_dims;++i)if(out_.dims[i]>1){tokens_=out_.dims[i];hidden_=out_.dims[i+1];break;}
        require(tokens_>0&&hidden_>0&&uint64_t(tokens_)*hidden_==out_.n_elems,"unsupported image embedding layout");
        std::cerr<<"SMOL_READY input=384x384 image_tokens="<<tokens_<<" hidden="<<hidden_<<" available_kb="<<available_kb()<<std::endl;
    }
    Json ask(const Json& request,double load_ms) {
        require(available_kb()>=256*1024,"insufficient_memory: inference reserve below 256 MiB");
        const auto started=milliseconds();
        const auto path=request.at("image").get<std::string>();auto prompt=request.at("prompt").get<std::string>();
        require(prompt.size()<=2048,"prompt too long");
        if(prompt.find("<image>")==std::string::npos)prompt="<image>"+prompt;
        int source_width=0;
        auto pixels=image_rgb(path,384,384,request.value("left_eye",false),source_width);
        const auto before_encoder=milliseconds();
        rknn_input input{};input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;input.size=pixels.size();input.buf=pixels.data();
        require(rknn_inputs_set(vision_,1,&input)==0,"encoder input failed");
        require(rknn_run(vision_,nullptr)==0,"encoder run failed");
        rknn_output output{};output.want_float=1;
        require(rknn_outputs_get(vision_,1,&output,nullptr)==0,"encoder output failed");
        std::vector<float> embedding(static_cast<float*>(output.buf),static_cast<float*>(output.buf)+out_.n_elems);
        require(rknn_outputs_release(vision_,1,&output)==0,"encoder output release failed");
        const auto encoded=milliseconds();
        require(rkllm_clear_kv_cache(llm_,0,nullptr,nullptr)==0,"clear KV cache failed");
        RKLLMInput query{};query.input_type=RKLLM_INPUT_MULTIMODAL;query.role="user";
        query.multimodal_input.prompt=const_cast<char*>(prompt.c_str());query.multimodal_input.image_embed=embedding.data();
        query.multimodal_input.n_image_tokens=tokens_;query.multimodal_input.n_image=1;
        query.multimodal_input.image_height=query.multimodal_input.image_width=384;
        RKLLMInferParam infer{};infer.mode=RKLLM_INFER_GENERATE;infer.keep_history=0;
        Response response;
        require(rkllm_run(llm_,&query,&infer,&response)==0,"rkllm_run failed");
        {std::unique_lock<std::mutex> lock(response.mutex);
         if(!response.cv.wait_for(lock,std::chrono::seconds(30),[&]{return response.done;})){
             lock.unlock();rkllm_abort(llm_);throw std::runtime_error("language callback timeout");
         }
         require(!response.error,"language callback reported error");}
        const auto finished=milliseconds();rusage usage{};getrusage(RUSAGE_SELF,&usage);
        return Json{{"id",request.value("id",std::string())},{"text",response.text},{"load_ms",load_ms},
                    {"preprocess_ms",before_encoder-started},{"encoder_ms",encoded-before_encoder},
                    {"language_ms",finished-encoded},{"total_ms",finished-started},{"max_rss_kb",usage.ru_maxrss},
                    {"available_kb",available_kb()},{"image_size",384},{"source_width_px",source_width},{"backend","rkllm-smolvlm2-500m"}};
    }
};
static void serve(Model& model,double load_ms,const std::string& path) {
    require(path=="/run/acg138-ros-bridge/semantic/model.sock","unexpected IPC path");
    int server=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);
    require(server>=0,"socket failed");
    sockaddr_un address{};address.sun_family=AF_UNIX;
    require(path.size()<sizeof(address.sun_path),"socket path too long");
    std::copy(path.begin(),path.end(),address.sun_path);
    struct stat status{};
    if(lstat(path.c_str(),&status)==0){
        require(S_ISSOCK(status.st_mode)&&status.st_uid==getuid(),"unexpected IPC file");
        int probe=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);
        int result=connect(probe,reinterpret_cast<sockaddr*>(&address),sizeof(address));close(probe);
        require(result!=0,"IPC socket already active");
        require(unlink(path.c_str())==0,"cannot remove stale IPC socket");
    }
    require(bind(server,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,"bind failed");
    require(chmod(path.c_str(),0600)==0&&listen(server,2)==0,"listen failed");
    std::cerr<<"SMOL_SOCKET_READY "<<path<<std::endl;
    for(;;){
        int client=accept4(server,nullptr,nullptr,SOCK_CLOEXEC);
        if(client<0)continue;
        timeval limit{5,0};setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&limit,sizeof(limit));
        setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&limit,sizeof(limit));
        Json result;
        try{
            ucred peer{};socklen_t size=sizeof(peer);
            require(getsockopt(client,SOL_SOCKET,SO_PEERCRED,&peer,&size)==0&&peer.uid==getuid(),"IPC peer not allowed");
            std::string line;char c;
            while(line.size()<8192&&recv(client,&c,1,0)==1&&c!='\n')line+=c;
            require(!line.empty()&&line.size()<8192,"invalid IPC request");
            auto request=Json::parse(line);const auto file=request.at("image").get<std::string>();
            char resolved[4096];require(realpath(file.c_str(),resolved),"image not found");
            require(std::string(resolved).rfind("/run/acg138-ros-bridge/semantic/",0)==0,"image outside shared IPC");
            result=model.ask(request,load_ms);
        }catch(const std::exception& ex){result=Json{{"error",ex.what()}};}
        const auto output=result.dump()+"\n";size_t sent=0;
        while(sent<output.size()){
            auto n=send(client,output.data()+sent,output.size()-sent,MSG_NOSIGNAL);
            if(n<=0)break;
            sent+=static_cast<size_t>(n);
        }
        close(client);std::cout<<"SMOL_RESULT "<<result.dump()<<std::endl;
    }
}
int main(int argc,char** argv) {
    try {
        if(argc<3){std::cerr<<"Usage: smol-worker VISION.rknn LANGUAGE.rkllm [max_new_tokens=48]\nJSON lines on stdin: {id,image,prompt}\n";return 2;}
        const int max_tokens=argc>3?std::stoi(argv[3]):48;require(max_tokens>=4&&max_tokens<=128,"max_new_tokens outside 4..128");
        const int lock=open("/run/lock/acg138-semantic-npu.lock",O_CREAT|O_RDWR|O_CLOEXEC,0600);
        require(lock>=0&&flock(lock,LOCK_EX|LOCK_NB)==0,"another semantic model owns the NPU lease");
        Model model;const auto began=milliseconds();model.load(argv[1],argv[2],max_tokens);const auto load_ms=milliseconds()-began;
        if(argc>4){require(argc==6&&std::string(argv[4])=="--socket","expected --socket PATH");serve(model,load_ms,argv[5]);}
        std::string line;
        while(std::getline(std::cin,line)) {
            try {require(line.size()<=8192,"request too large");auto result=model.ask(Json::parse(line),load_ms);std::cout<<"SMOL_RESULT "<<result.dump()<<std::endl;}
            catch(const std::exception& ex){std::cout<<"SMOL_RESULT "<<Json{{"error",ex.what()}}.dump()<<std::endl;}
        }
        close(lock);return 0;
    }catch(const std::exception& ex){std::cerr<<"SMOL_FATAL "<<ex.what()<<std::endl;return 1;}
}
