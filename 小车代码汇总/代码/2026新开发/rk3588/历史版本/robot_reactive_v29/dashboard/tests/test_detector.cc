#include "detector.h"
#include "detector_math.h"
#include "model_labels.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>
#include <fstream>

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#condition); \
    std::exit(1); } } while (0)

namespace dm=detector_math;
static bool near(float a,float b,float tolerance=0.001f) { return std::fabs(a-b)<tolerance; }

struct OwnedHead {
    int w,h,bins,classes;
    std::vector<float> boxes,scores,sums;
    OwnedHead(int width,int height,int count=2,int dfl=4)
        :w(width),h(height),bins(dfl),classes(count),boxes(w*h*bins*4,0),scores(w*h*classes,0),sums(w*h,0) {}
    dm::Head view(bool sum=true) const {
        dm::Head head={boxes.data(),scores.data(),sum?sums.data():NULL,w,h,bins,classes};
        return head;
    }
    void score(int cell,int cls,float probability) { scores[cls*w*h+cell]=probability; sums[cell]=1; }
    void distance(int cell,int side,int peak) {
        for (int i=0;i<bins;++i) boxes[(side*bins+i)*w*h+cell]=(i==peak ? 0.0f : -1000.0f);
    }
};

static void test_contract() {
    std::string error;
    std::vector<dm::TensorShape> six;
    for (int size=4;size>=1;size/=2) {
        six.push_back(dm::TensorShape{1,64,size,size});
        six.push_back(dm::TensorShape{1,80,size,size});
    }
    CHECK(dm::validate_contract(six,32,32,80,error));
    CHECK(!dm::validate_contract(six,32,32,2,error));
    std::vector<dm::TensorShape> nine;
    for (int i=0;i<3;++i) {
        nine.push_back(six[i*2]); nine.push_back(six[i*2+1]);
        nine.push_back(dm::TensorShape{1,1,4>>i,4>>i});
    }
    CHECK(dm::validate_contract(nine,32,32,80,error));
    nine[2].dims[1]=80;
    CHECK(!dm::validate_contract(nine,32,32,80,error));
    six[0].nchw=false;
    CHECK(!dm::validate_contract(six,32,32,80,error));
    CHECK(!dm::validate_contract({dm::TensorShape{1,84,8400}},640,640,80,error));
    CHECK(error.find("84,8400")!=std::string::npos);
    six[0].nchw=true; six[2].dims[2]=1;
    CHECK(!dm::validate_contract(six,32,32,80,error));
}

static void test_custom_classes() {
    std::string error;
    std::vector<dm::TensorShape> shapes;
    for(int size=80;size>=20;size/=2) {
        shapes.push_back(dm::TensorShape{1,64,size,size});
        shapes.push_back(dm::TensorShape{1,10,size,size});
        shapes.push_back(dm::TensorShape{1,1,size,size});
    }
    CHECK(dm::validate_contract(shapes,640,640,10,error));
    CHECK(!dm::validate_contract(shapes,640,640,80,error));
    CHECK(error.find("80 classes")!=std::string::npos);
    CHECK(!dm::validate_contract(shapes,640,640,9,error));
    shapes[4].dims[1]=80;
    CHECK(!dm::validate_contract(shapes,640,640,10,error));

    // Decode the final custom class and class zero independently: no COCO
    // class semantics, hard-coded nc=80, or second sigmoid may leak through.
    std::vector<OwnedHead> owned;
    owned.emplace_back(4,4,10);owned.emplace_back(2,2,10);owned.emplace_back(1,1,10);
    owned[0].score(5,9,.91f);owned[1].score(1,0,.82f);
    std::vector<dm::Head> heads;for(const auto &head:owned)heads.push_back(head.view());
    dm::Letterbox box;CHECK(dm::make_letterbox(960,1080,32,32,box,error));
    std::vector<Detection> result;
    CHECK(dm::decode_heads(heads,box,.5f,.45f,result,error));
    CHECK(result.size()==2 && result[0].class_id==9 && result[1].class_id==0);
    CHECK(near(result[0].score,.91f) && near(result[1].score,.82f));

    const char *path="build/test-custom-labels.txt";
    {std::ofstream file(path);for(int i=0;i<10;i++)file<<"industrial_"<<i<<"\r\n";}
    std::vector<std::string> names;
    CHECK(load_model_labels(path,names,error) && names.size()==10 && names[9]=="industrial_9");
    {std::ofstream file(path);file<<"helmet\nhelmet\n";}
    CHECK(!load_model_labels(path,names,error) && names.empty());
    {std::ofstream file(path);file<<"helmet\n\n";}
    CHECK(!load_model_labels(path,names,error));
    {std::ofstream file(path);file<<" helmet\n";}
    CHECK(!load_model_labels(path,names,error));
    {std::ofstream file(path);file<<"helmet\tbad\n";}
    CHECK(!load_model_labels(path,names,error));
    {std::ofstream file(path);file<<"\xef\xbb\xbfhelmet\n";}
    CHECK(load_model_labels(path,names,error) && names.size()==1 && names[0]=="helmet");
    std::remove(path);
    CHECK(!load_model_labels(path,names,error));
}

static void test_letterbox() {
    std::string error;
    dm::Letterbox box;
    CHECK(dm::make_letterbox(960,1080,640,640,box,error));
    CHECK(box.resized_w==569 && box.resized_h==640 && box.pad_x==35 && box.pad_y==0);
    CHECK(!near(box.scale_x,box.scale_y,0.00001f));
    CHECK(dm::make_letterbox(1920,1080,640,640,box,error));
    CHECK(box.resized_w==640 && box.resized_h==360 && box.pad_y==140);
    CHECK(dm::make_letterbox(640,480,640,384,box,error));
    CHECK(box.resized_w==512 && box.resized_h==384 && box.pad_x==64);
    CHECK(dm::make_letterbox(1,16384,32,32,box,error));
    CHECK(box.resized_w==1 && box.resized_h==32);
    CHECK(!dm::make_letterbox(0,1080,640,640,box,error));

    // Padded input rows must never become pixels. Bilinear RGB must preserve
    // channel order and identity when scale is one.
    const unsigned char rgb[]={255,0,0,0,255,0,99,99,99,0,0,255,255,255,255,99,99,99};
    std::vector<unsigned char> output;
    CHECK(dm::preprocess_rgb(rgb,2,2,9,2,2,output,box,error));
    const unsigned char expected[]={255,0,0,0,255,0,0,0,255,255,255,255};
    CHECK(output==std::vector<unsigned char>(expected,expected+12));
    CHECK(dm::preprocess_rgb(rgb,2,2,9,1,1,output,box,error));
    CHECK(output.size()==3 && output[0]==128 && output[1]==128 && output[2]==128);
    const unsigned char red[]={255,0,0,255,0,0};
    CHECK(dm::preprocess_rgb(red,2,1,6,4,4,output,box,error));
    CHECK(box.pad_y==1 && output[0]==114 && output[1]==114 && output[2]==114);
    CHECK(output[(4+1)*3]==255 && output[(4+1)*3+1]==0 && output[(4+1)*3+2]==0);
    CHECK(output[(12+1)*3]==114);
    CHECK(!dm::preprocess_rgb(rgb,2,2,5,2,2,output,box,error));
    CHECK(!dm::preprocess_rgb(NULL,2,2,6,2,2,output,box,error));
}

static void test_dfl() {
    float output=0;
    const float uniform[]={0,0,0,0};
    CHECK(dm::dfl_expectation(uniform,4,1,output) && near(output,1.5f));
    const float large[]={10000,10001,10002,10003};
    const float small[]={0,1,2,3};
    float reference=0;
    CHECK(dm::dfl_expectation(large,4,1,output));
    CHECK(dm::dfl_expectation(small,4,1,reference) && near(output,reference));
    const float strided[]={-1000,99,-1000,99,0,99,-1000,99};
    CHECK(dm::dfl_expectation(strided,4,2,output) && near(output,2));
    const float invalid[]={0,std::numeric_limits<float>::quiet_NaN()};
    CHECK(!dm::dfl_expectation(invalid,2,1,output));
    CHECK(!dm::dfl_expectation(NULL,4,1,output));
    CHECK(!dm::dfl_expectation(uniform,1,1,output));
}

static void test_nms() {
    std::vector<Detection> candidates={
        {0,0,10,10,0.8f,0}, {1,1,11,11,0.9f,0},
        {1,1,11,11,0.7f,1}, {30,30,40,40,0.6f,0},
        {0,0,0,0,0.99f,0}, {0,0,10,10,std::numeric_limits<float>::quiet_NaN(),0}
    },results;
    dm::class_aware_nms(candidates,0.45f,128,results);
    CHECK(results.size()==3 && near(results[0].score,0.9f));
    CHECK(results[1].class_id==1 && near(results[1].score,0.7f));
    dm::class_aware_nms(candidates,0.45f,1,results);
    CHECK(results.size()==1 && near(results[0].score,0.9f));
    // Touching floating-point boxes have zero IoU, without pixel +1 bias.
    candidates={{0,0,1,1,0.9f,0},{1,0,2,1,0.8f,0}};
    dm::class_aware_nms(candidates,0,128,results);
    CHECK(results.size()==2);
}

static void test_decode() {
    std::string error;
    dm::Letterbox box;
    CHECK(dm::make_letterbox(90,301,32,32,box,error));
    std::vector<OwnedHead> owned;
    owned.reserve(3);
    owned.push_back(OwnedHead(4,4)); owned.push_back(OwnedHead(2,2)); owned.push_back(OwnedHead(1,1));
    const int cell=2*4+1;
    owned[0].score(cell,1,0.8f);
    owned[0].distance(cell,0,1); owned[0].distance(cell,1,1);
    owned[0].distance(cell,2,1); owned[0].distance(cell,3,1);
    std::vector<dm::Head> heads;
    for (const OwnedHead &h:owned) heads.push_back(h.view());
    std::vector<Detection> results;
    CHECK(dm::decode_heads(heads,box,0.8f,0.45f,results,error));
    CHECK(results.size()==1 && results[0].class_id==1 && near(results[0].score,0.8f));
    CHECK(near(results[0].left,0));
    CHECK(near(results[0].right,(20-box.pad_x)/box.scale_x));
    CHECK(near(results[0].top,12/box.scale_y));
    CHECK(near(results[0].bottom,28/box.scale_y));
    CHECK(dm::decode_heads(heads,box,0.81f,0.45f,results,error) && results.empty());
    CHECK(!dm::decode_heads(heads,box,std::numeric_limits<float>::quiet_NaN(),0.45f,results,error));
    CHECK(!dm::decode_heads(heads,box,0.5f,1.1f,results,error));
    // A NaN in one side's DFL distribution must discard the candidate.
    owned[0].boxes[cell]=std::numeric_limits<float>::quiet_NaN();
    CHECK(dm::decode_heads(heads,box,0.5f,0.45f,results,error) && results.empty());
    owned[0].boxes[cell]=-1000;
    owned[0].sums[cell]=std::numeric_limits<float>::quiet_NaN();
    CHECK(dm::decode_heads(heads,box,0.5f,0.45f,results,error) && results.empty());
    owned[0].sums[cell]=1;
    owned[0].scores[owned[0].w*owned[0].h+cell]=std::numeric_limits<float>::infinity();
    CHECK(dm::decode_heads(heads,box,0.5f,0.45f,results,error) && results.empty());
    owned[0].scores[owned[0].w*owned[0].h+cell]=0.8f;
    // Six-output models have no score-sum head.
    heads.clear(); for (const OwnedHead &h:owned) heads.push_back(h.view(false));
    CHECK(dm::decode_heads(heads,box,0.5f,0.45f,results,error) && results.size()==1);
    CHECK(!dm::decode_heads(heads,box,0.5f,0.45f,results,error,0));
    // Clamp every edge after inverse scaling, in the source eye's coordinates.
    owned[2].score(0,0,0.95f);
    CHECK(dm::decode_heads(heads,box,0.5f,0.45f,results,error));
    for (const Detection &d:results) CHECK(d.left>=0 && d.top>=0 && d.right<=90 && d.bottom<=301);

    CHECK(dm::make_letterbox(301,90,64,32,box,error));
    owned.clear();
    owned.push_back(OwnedHead(8,4)); owned.push_back(OwnedHead(4,2)); owned.push_back(OwnedHead(2,1));
    owned[0].score(2*8+3,0,0.95f);
    heads.clear(); for (const OwnedHead &h:owned) heads.push_back(h.view());
    CHECK(dm::decode_heads(heads,box,0.5f,0.45f,results,error));
    CHECK(results.size()==1);
    for (const Detection &d:results) CHECK(d.left>=0 && d.top>=0 && d.right<=301 && d.bottom<=90);
    // All cells above threshold must still be bounded before NMS. With IoU=1,
    // the highest scoring five survive the requested candidate limit.
    for (OwnedHead &h:owned) for (int i=0;i<h.w*h.h;++i) h.score(i,0,0.5f+0.005f*i);
    heads.clear(); for (const OwnedHead &h:owned) heads.push_back(h.view());
    CHECK(dm::decode_heads(heads,box,0.1f,1.0f,results,error,5,128));
    CHECK(results.size()==5);
    CHECK(near(results[0].score,0.655f));
    CHECK(near(results[4].score,0.635f));
}

int main() {
    test_contract(); test_custom_classes(); test_letterbox(); test_dfl(); test_nms(); test_decode();
#ifndef HAVE_RKNN
    RknnDetector detector;
    std::string error;
    CHECK(!detector.open("missing.rknn",0.35f,0.45f,80,error));
    CHECK(error.find("RKNN")!=std::string::npos);
    std::vector<Detection> result={{0,0,1,1,1,1}};
    CHECK(!detector.infer(NULL,0,0,0,result,error) && result.empty());
#endif
    std::puts("PASS detector: 6/9-output contract, bilinear RGB/padding/stride, stable DFL, class-aware NMS, inverse mapping, NaN/threshold handling, candidate cap, explicit no-RKNN failure");
    return 0;
}
