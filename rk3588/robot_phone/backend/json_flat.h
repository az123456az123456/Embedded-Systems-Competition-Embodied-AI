#ifndef ROBOT_PHONE_JSON_FLAT_H
#define ROBOT_PHONE_JSON_FLAT_H
#include <map>
#include <string>
namespace robotphone {
struct JsonValue { enum Type { String, Number, Boolean, Null } type=Null; std::string text; };
using JsonObject=std::map<std::string,JsonValue>;
bool parseFlatJson(const std::string &,JsonObject &,std::string &error);
std::string quoteJson(const std::string &);
bool jsonString(const JsonObject &,const char *,std::string &);
bool jsonNumber(const JsonObject &,const char *,double &);
bool jsonBool(const JsonObject &,const char *,bool &);
}
#endif
