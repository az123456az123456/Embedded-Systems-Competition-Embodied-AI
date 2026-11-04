#pragma once
#include <map>
#include <string>
namespace robotnav {
inline std::string semantic_target(std::string text) {
 const auto first=text.find_first_not_of(" \t\r\n");
 if(first==std::string::npos||text.size()>320)return {};
 text=text.substr(first,text.find_last_not_of(" \t\r\n")-first+1);
 for(const std::string punctuation:{"。","！",".","!"})
  if(text.size()>=punctuation.size()&&text.compare(text.size()-punctuation.size(),punctuation.size(),punctuation)==0)
   text.resize(text.size()-punctuation.size());
 static const std::map<std::string,std::string> targets={
  {"找到没戴口罩的人","NO-Mask"},{"找到未戴口罩的人","NO-Mask"},
  {"找到没有戴口罩的人","NO-Mask"},{"寻找未戴口罩的人","NO-Mask"},
  {"找到未戴口罩的人并提醒","NO-Mask"},
  {"找到没戴安全帽的人","NO-Hardhat"},{"找到未戴安全帽的人","NO-Hardhat"},
  {"找到没穿反光衣的人","NO-Safety Vest"},{"找到未穿反光衣的人","NO-Safety Vest"},
  {"找到人","Person"},{"寻找人员","Person"},{"找到人员","Person"},
  {"找到安全帽","Hardhat"},{"找到口罩","Mask"},{"找到反光衣","Safety Vest"},
  {"找到机械","machinery"},{"找到车辆","vehicle"},
  {"Hardhat","Hardhat"},{"Mask","Mask"},{"NO-Hardhat","NO-Hardhat"},
  {"NO-Mask","NO-Mask"},{"NO-Safety Vest","NO-Safety Vest"},{"Person","Person"},
  {"Safety Vest","Safety Vest"},{"machinery","machinery"},{"vehicle","vehicle"}};
 const auto it=targets.find(text);return it==targets.end()?std::string():it->second;
}
}
