// Development-only bridge: test the production C++ preprocessing/decoder with
// real ONNX outputs, without requiring an RK3588 or linking librknnrt.
#include "detector_math.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace dm=detector_math;
static void geometry(const dm::Letterbox &b) {
    std::cout << "\"source_w\":" << b.source_w << ",\"source_h\":" << b.source_h
              << ",\"target_w\":" << b.target_w << ",\"target_h\":" << b.target_h
              << ",\"resized_w\":" << b.resized_w << ",\"resized_h\":" << b.resized_h
              << ",\"pad_x\":" << b.pad_x << ",\"pad_y\":" << b.pad_y
              << ",\"scale_x\":" << b.scale_x << ",\"scale_y\":" << b.scale_y;
}
template<typename T>
static std::vector<T> read_exact(const std::string &path,std::size_t elements) {
    std::ifstream file(path.c_str(),std::ios::binary|std::ios::ate);
    if (!file || file.tellg()!=static_cast<std::streamoff>(elements*sizeof(T)))
        throw std::runtime_error("Missing or incorrectly sized input: "+path);
    file.seekg(0);
    std::vector<T> data(elements);
    if (!file.read(reinterpret_cast<char *>(data.data()),elements*sizeof(T)))
        throw std::runtime_error("Cannot read input: "+path);
    return data;
}
int main(int argc,char **argv) {
    try {
        std::cout << std::setprecision(9);
        std::string error;
        dm::Letterbox box;
        if (argc==8 && std::string(argv[1])=="preprocess") {
            const int sw=std::stoi(argv[4]),sh=std::stoi(argv[5]);
            const int tw=std::stoi(argv[6]),th=std::stoi(argv[7]);
            if (!dm::make_letterbox(sw,sh,tw,th,box,error)) throw std::runtime_error(error);
            const std::vector<unsigned char> rgb=read_exact<unsigned char>(argv[2],static_cast<std::size_t>(sw)*sh*3);
            std::vector<unsigned char> out;
            if (!dm::preprocess_rgb(rgb.data(),sw,sh,sw*3,tw,th,out,box,error)) throw std::runtime_error(error);
            std::ofstream file(argv[3],std::ios::binary);
            if (!file.write(reinterpret_cast<const char *>(out.data()),out.size())) throw std::runtime_error("Cannot write preprocessed pixels");
            std::cout << '{'; geometry(box); std::cout << "}\n";
            return 0;
        }
        if (argc==12 && std::string(argv[1])=="decode") {
            const int sw=std::stoi(argv[3]),sh=std::stoi(argv[4]);
            const int tw=std::stoi(argv[5]),th=std::stoi(argv[6]);
            const int classes=std::stoi(argv[7]),bins=std::stoi(argv[8]),count=std::stoi(argv[9]);
            const float confidence=std::stof(argv[10]),nms=std::stof(argv[11]);
            if ((count!=6 && count!=9) || bins<2 || bins>64 || classes<1 || classes>10000)
                throw std::runtime_error("Invalid head contract requested");
            if (!dm::make_letterbox(sw,sh,tw,th,box,error)) throw std::runtime_error(error);
            const int per=count/3;
            std::vector<std::vector<float> > tensors(count);
            std::vector<dm::TensorShape> shapes;
            for (int i=0;i<count;++i) {
                const int branch=i/per,part=i%per,stride=8<<branch;
                shapes.push_back(dm::TensorShape{1,part==0?bins*4:(part==1?classes:1),th/stride,tw/stride});
            }
            if (!dm::validate_contract(shapes,tw,th,classes,error)) throw std::runtime_error(error);
            for (int i=0;i<count;++i) {
                const std::vector<int> &s=shapes[i].dims;
                tensors[i]=read_exact<float>(std::string(argv[2])+"/output_"+std::to_string(i)+".bin",
                                             static_cast<std::size_t>(s[1])*s[2]*s[3]);
            }
            std::vector<dm::Head> heads;
            for (int branch=0;branch<3;++branch) {
                const int idx=branch*per,stride=8<<branch;
                dm::Head head={tensors[idx].data(),tensors[idx+1].data(),per==3?tensors[idx+2].data():NULL,
                               tw/stride,th/stride,bins,classes};
                heads.push_back(head);
            }
            std::vector<Detection> results;
            if (!dm::decode_heads(heads,box,confidence,nms,results,error)) throw std::runtime_error(error);
            std::cout << "{\"letterbox\":{"; geometry(box); std::cout << "},\"detections\":[";
            for (std::size_t i=0;i<results.size();++i) {
                const Detection &d=results[i];
                if (i) std::cout << ',';
                std::cout << "{\"left\":" << d.left << ",\"top\":" << d.top
                          << ",\"right\":" << d.right << ",\"bottom\":" << d.bottom
                          << ",\"score\":" << d.score << ",\"class_id\":" << d.class_id << '}';
            }
            std::cout << "]}\n";
            return 0;
        }
        throw std::runtime_error("Usage: probe preprocess RGB OUT source_w source_h target_w target_h; or probe decode DIR source_w source_h target_w target_h classes bins output_count confidence nms");
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
