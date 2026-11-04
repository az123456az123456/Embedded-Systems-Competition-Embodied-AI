#include "native-reference/navigation/sonar_diagnostics.h"
#include <iostream>
#include <string>
// Feed the Android formatter with the actual native RK diagnostics producer.
void emit(const char* name,const navwire::Sensors& s,bool have=true,double now=100.01){
    std::cout<<"{\"case\":\""<<name<<"\",\"state\":{\"navigation\":{\"sonar_health\":"
             <<robot_nav::sonar_diagnostics_json(s,have,100.,now,robot_nav::Config{})<<"}}}\n";
}
int main(){
    navwire::Sensors s;s.valid=7;s.distance_mm={{1234,820,4500}};s.age_ms={{30,40,50}};emit("normal",s);
    auto a=s;a.distance_mm[0]=4000;a.distance_mm[2]=4001;emit("four_metres",a);
    a=s;a.valid=6;a.fault=1;emit("fpga_invalid",a);
    emit("packet_expired",s,true,100.251);
    a=s;a.age_ms[0]=900;emit("source_expired",a);
    emit("not_received",s,false);
    a=s;a.age_ms[0]=65535;a.distance_mm[0]=0;emit("not_observed",a);
    a=s;a.age_ms[0]=65535;emit("saturated_source",a);
    a=s;a.distance_mm[0]=10;emit("outside_range",a);
    emit("host_time_invalid",s,true,99.);
}
