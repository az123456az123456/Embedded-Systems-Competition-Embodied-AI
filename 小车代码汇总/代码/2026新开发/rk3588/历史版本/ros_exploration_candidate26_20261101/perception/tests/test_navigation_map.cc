#include "navigation_map.h"
#include "picojson.h"
#include <cassert>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>
static picojson::value array(std::initializer_list<double> list){picojson::array a;for(double d:list)a.emplace_back(d);return picojson::value(a);}
static picojson::object fixture(){return {
 {"schema",picojson::value(1.)},{"width",picojson::value(64.)},{"height",picojson::value(64.)},
 {"resolution_m",picojson::value(.1)},{"origin",array({-3.2,-3.2})},{"pose",array({0,0,.5})},
 {"cells",picojson::value(std::string(4096,'.'))},{"path",picojson::value(picojson::array{array({0,0}),array({.5,0})})},
 {"trajectory",picojson::value(picojson::array{array({0,0})})},{"goal",picojson::value()},
 {"state",picojson::value(std::string("observing"))},{"reason",picojson::value(std::string("manual"))},
 {"session_id",picojson::value(std::string("session-1"))},{"available",picojson::value(true)},
 {"pose_valid",picojson::value(true)},{"updated_monotonic_ms",picojson::value(10000.)},
 {"map_revision",picojson::value(5.)},{"sensor_age_ms",picojson::value(0.)},{"robot_radius_m",picojson::value(.18)}};}
static int parse(const picojson::object &o,navigation_map *m){auto s=picojson::value(o).serialize();return navigation_map_parse(s.data(),s.size(),m);}
int main(int argc,char **argv){
 auto m=std::make_unique<navigation_map>();auto o=fixture();unsigned tests=0;
 auto check=[&](bool okay){if(!okay)std::cerr<<"failed check "<<tests+1<<"\n";assert(okay);tests++;};
 check(parse(o,m.get()));check(m->loaded&&m->width==64&&m->path_count==2);
 check(navigation_map_fresh(m.get(),10000));check(navigation_map_fresh(m.get(),12000));
 check(!navigation_map_fresh(m.get(),12001));check(!navigation_map_fresh(m.get(),9999));
 for(const char *key:{"schema","width","height","pose","cells","updated_monotonic_ms","available","pose_valid","path","session_id","sensor_age_ms"}){auto a=o;a.erase(key);check(!parse(a,m.get()));}
 for(const char *key:{"width","height","resolution_m","updated_monotonic_ms","map_revision"}){auto a=o;a[key]=picojson::value(-1.);check(!parse(a,m.get()));}
 auto a=o;a["sensor_age_ms"]=picojson::value(-1.);check(parse(a,m.get())&&!navigation_map_fresh(m.get(),10000));
 a=o;a["sensor_age_ms"]=picojson::value(-2.);check(!parse(a,m.get()));
 a=o;a["cells"]=picojson::value(std::string(4096,'x'));check(!parse(a,m.get()));
 a=o;a["cells"]=picojson::value(std::string(4095,'.'));check(!parse(a,m.get()));
 a=o;a["path"]=picojson::value(picojson::array(4097,array({0,0})));check(!parse(a,m.get()));
 a=o;a["goal"]=array({1,2});check(parse(a,m.get())&&m->has_goal&&m->goal[1]==2);
 a=o;a["available"]=picojson::value(false);check(parse(a,m.get())&&!navigation_map_fresh(m.get(),10000));
 a=o;a["pose_valid"]=picojson::value(false);check(parse(a,m.get())&&!navigation_map_fresh(m.get(),10000));
 picojson::object detail={{"width",picojson::value(1000.)},{"height",picojson::value(1000.)},
  {"resolution_m",picojson::value(.02)},{"origin",array({-10,-10,0})},{"revision",picojson::value(123.)},
  {"cells",picojson::value(std::string(1000000,'.'))}};
 a=o;a["render_map"]=picojson::value(detail);check(parse(a,m.get())&&m->render_width==1000&&m->render_cells[999999]=='.'&&m->render_revision==123);
 auto broken=detail;broken["width"]=picojson::value(1001.);a["render_map"]=picojson::value(broken);check(!parse(a,m.get()));
 broken=detail;broken["cells"]=picojson::value(std::string(1000000,'x'));a["render_map"]=picojson::value(broken);check(!parse(a,m.get()));
 broken=detail;broken["cells"]=picojson::value(std::string(999999,'.'));a["render_map"]=picojson::value(broken);check(!parse(a,m.get()));
 broken=detail;broken["resolution_m"]=picojson::value(0.);a["render_map"]=picojson::value(broken);check(!parse(a,m.get()));
 check(parse(o,m.get())&&m->render_width==0); // old schema clears previous detail
 auto text=picojson::value(o).serialize();check(!navigation_map_parse((text+" garbage").data(),text.size()+8,m.get()));
 char dir[]="/tmp/acg138-map-reader-XXXXXX";check(mkdtemp(dir)!=nullptr);std::string path=std::string(dir)+"/map.json";
 check(!navigation_map_read(path.c_str(),m.get()));check(mkfifo(path.c_str(),0600)==0);check(!navigation_map_read(path.c_str(),m.get()));unlink(path.c_str());
 {std::ofstream f(path);f<<text;}check(navigation_map_read(path.c_str(),m.get()));
 auto r=navigation_reader_start(path.c_str());check(r!=nullptr);bool found=false;
 for(int i=0;i<100;i++){if(navigation_reader_snapshot(r,m.get())&&m->loaded){found=true;break;}std::this_thread::sleep_for(std::chrono::milliseconds(10));}check(found&&m->available);
 a=o;a["session_id"]=picojson::value(std::string("session-2"));a["path"]=picojson::value(picojson::array{});
 {std::ofstream f(path+".tmp");f<<picojson::value(a).serialize();}rename((path+".tmp").c_str(),path.c_str());found=false;
 for(int i=0;i<100;i++){navigation_reader_snapshot(r,m.get());if(!strcmp(m->session_id,"session-2")){found=true;break;}std::this_thread::sleep_for(std::chrono::milliseconds(10));}check(found&&m->path_count==0);
 unlink(path.c_str());found=false;
 for(int i=0;i<100;i++){navigation_reader_snapshot(r,m.get());if(!m->available){found=true;break;}std::this_thread::sleep_for(std::chrono::milliseconds(10));}check(found&&!m->pose_valid);
 navigation_reader_stop(r);rmdir(dir);
 if(argc>1){check(navigation_map_read(argv[1],m.get()));check(m->loaded&&m->width==64&&m->height==64);std::cout<<"PASS RK producer fixture schema/session/cells/pose/path parse\n";}
 std::cout<<"PASS navigation map "<<tests<<" checks: strict schema, freshness/future rejection, atomic session replacement, invalid/FIFO/missing file, worker snapshots\n";
}
