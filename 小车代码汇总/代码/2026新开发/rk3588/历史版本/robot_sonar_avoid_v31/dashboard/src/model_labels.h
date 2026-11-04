#ifndef MODEL_LABELS_H
#define MODEL_LABELS_H
#include <fstream>
#include <set>
#include <string>
#include <vector>

// The label order is the model's class order. Never fall back to COCO names
// when deploying a custom model; detector.open also verifies every head's nc.
inline bool load_model_labels(const std::string &path,
                              std::vector<std::string> &names, std::string &error) {
    names.clear(); error.clear();
    std::ifstream file(path); std::string name; std::set<std::string> unique;
    if (!file) { error="Cannot open model labels: "+path; return false; }
    while (std::getline(file,name)) {
        if (names.empty() && name.compare(0,3,"\xef\xbb\xbf")==0) name.erase(0,3);
        if (!name.empty() && name.back()=='\r') name.pop_back();
        bool valid=!name.empty() && name.size()<=63 && names.size()<256;
        if(valid && (name.front()==' ' || name.back()==' '))valid=false;
        for(unsigned char c:name)if(c<32 || c==127)valid=false;
        if (!valid || !unique.insert(name).second) {
            const auto line=names.size()+1;
            names.clear(); error="Invalid or duplicate model label at line "+std::to_string(line)+": "+path; return false;
        }
        names.push_back(name);
    }
    if (!file.eof() || names.empty()) { names.clear(); error="Cannot read model labels: "+path; return false; }
    return true;
}
#endif
