#include "json_flat.h"
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstdio>
namespace robotphone {
namespace {
size_t utf8Length(const std::string &s,size_t at){
 const unsigned char first=static_cast<unsigned char>(s[at]);
 if(first<128)return 1;
 size_t length=first>=0xc2&&first<=0xdf?2:first>=0xe0&&first<=0xef?3:first>=0xf0&&first<=0xf4?4:0;
 if(!length||length>s.size()-at)return 0;
 for(size_t i=1;i<length;i++)if((static_cast<unsigned char>(s[at+i])&0xc0)!=0x80)return 0;
 const unsigned char second=static_cast<unsigned char>(s[at+1]);
 if((first==0xe0&&second<0xa0)||(first==0xed&&second>=0xa0)||(first==0xf0&&second<0x90)||(first==0xf4&&second>=0x90))return 0;
 return length;
}
void appendCodepoint(std::string &out,unsigned cp){
 if(cp<128)out+=char(cp);
 else if(cp<2048){out+=char(0xc0|(cp>>6));out+=char(0x80|(cp&63));}
 else if(cp<65536){out+=char(0xe0|(cp>>12));out+=char(0x80|((cp>>6)&63));out+=char(0x80|(cp&63));}
 else{out+=char(0xf0|(cp>>18));out+=char(0x80|((cp>>12)&63));out+=char(0x80|((cp>>6)&63));out+=char(0x80|(cp&63));}
}
struct Parser {
 const std::string &s;size_t p=0;
 void ws(){while(p<s.size() && (s[p]==' '||s[p]=='\t'||s[p]=='\r'||s[p]=='\n'))p++;}
 bool take(char c){ws();if(p<s.size() && s[p]==c){p++;return true;}return false;}
 bool hex(unsigned &cp){
  cp=0;if(p+4>s.size())return false;
  for(int i=0;i<4;i++){char h=s[p++];cp<<=4;if(h>='0'&&h<='9')cp|=h-'0';else if(h>='a'&&h<='f')cp|=h-'a'+10;else if(h>='A'&&h<='F')cp|=h-'A'+10;else return false;}
  return true;
 }
 bool string(std::string &out){
  if(!take('"'))return false;
  out.clear();
  while(p<s.size()){
   unsigned char c=s[p++];if(c=='"')return true;if(c<32)return false;
   if(c!='\\'){
    if(c>=128){size_t length=utf8Length(s,p-1);if(!length)return false;out.append(s,p-1,length);p+=length-1;}
    else out+=char(c);
    continue;
   }if(p==s.size())return false;c=s[p++];
   switch(c){case '"':case '\\':case '/':out+=char(c);break;case 'b':out+='\b';break;case 'f':out+='\f';break;case 'n':out+='\n';break;case 'r':out+='\r';break;case 't':out+='\t';break;
    case 'u':{unsigned cp=0;if(!hex(cp))return false;
     if(cp>=0xd800&&cp<=0xdbff){unsigned low=0;if(p+2>s.size()||s[p]!='\\'||s[p+1]!='u')return false;p+=2;if(!hex(low)||low<0xdc00||low>0xdfff)return false;cp=0x10000+((cp-0xd800)<<10)+(low-0xdc00);}
     else if(cp>=0xdc00&&cp<=0xdfff)return false;
     appendCodepoint(out,cp);break;}
    default:return false;}
  }return false;
 }
 bool value(JsonValue &v){
  ws();if(p==s.size())return false;
  if(s[p]=='"'){v.type=JsonValue::String;return string(v.text);}
  for(const char *literal:{"true","false","null"}){std::string x=literal;if(s.compare(p,x.size(),x)==0){p+=x.size();v.type=x=="null"?JsonValue::Null:JsonValue::Boolean;v.text=x;return true;}}
  size_t start=p;if(s[p]=='-')p++;if(p==s.size())return false;
  if(s[p]=='0')p++;else{if(s[p]<'1'||s[p]>'9')return false;while(p<s.size()&&std::isdigit(static_cast<unsigned char>(s[p])))p++;}
  if(p<s.size()&&s[p]=='.'){p++;size_t digits=p;while(p<s.size()&&std::isdigit(static_cast<unsigned char>(s[p])))p++;if(p==digits)return false;}
  if(p<s.size()&&(s[p]=='e'||s[p]=='E')){p++;if(p<s.size()&&(s[p]=='+'||s[p]=='-'))p++;size_t digits=p;while(p<s.size()&&std::isdigit(static_cast<unsigned char>(s[p])))p++;if(p==digits)return false;}
  v.type=JsonValue::Number;v.text=s.substr(start,p-start);char *end=nullptr;double n=std::strtod(v.text.c_str(),&end);return end && !*end && std::isfinite(n);
 }
};
}
bool parseFlatJson(const std::string &s,JsonObject &out,std::string &error){
 out.clear();error.clear();JsonObject parsed;Parser p{s};if(s.size()>8192 || !p.take('{')){error="Expected a small JSON object";return false;}
 if(!p.take('}'))while(true){std::string key;JsonValue value;if(parsed.size()>=32||!p.string(key)||key.empty()||key.size()>64||parsed.count(key)||!p.take(':')||!p.value(value)){error="Invalid JSON key/value";return false;}parsed.emplace(key,value);if(p.take('}'))break;if(!p.take(',')){error="Invalid JSON separator";return false;}}
 p.ws();if(p.p!=s.size()){error="Trailing JSON data";return false;}out.swap(parsed);return true;
}
std::string quoteJson(const std::string &s){std::string out="\"";for(unsigned char c:s){switch(c){case '"':out+="\\\"";break;case '\\':out+="\\\\";break;case '\n':out+="\\n";break;case '\r':out+="\\r";break;case '\t':out+="\\t";break;default:if(c<32){char b[7];std::snprintf(b,sizeof(b),"\\u%04x",c);out+=b;}else out+=char(c);}}return out+'"';}
bool jsonString(const JsonObject &o,const char *key,std::string &v){auto i=o.find(key);if(i==o.end()||i->second.type!=JsonValue::String)return false;v=i->second.text;return true;}
bool jsonNumber(const JsonObject &o,const char *key,double &v){auto i=o.find(key);if(i==o.end()||i->second.type!=JsonValue::Number||i->second.text.empty())return false;char *end=nullptr;v=std::strtod(i->second.text.c_str(),&end);return end&&!*end&&std::isfinite(v);}
bool jsonBool(const JsonObject &o,const char *key,bool &v){auto i=o.find(key);if(i==o.end()||i->second.type!=JsonValue::Boolean)return false;v=i->second.text=="true";return true;}
}
