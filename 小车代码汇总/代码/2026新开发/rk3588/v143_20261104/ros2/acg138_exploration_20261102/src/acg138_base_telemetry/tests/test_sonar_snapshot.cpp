#include "acg138_base_telemetry/sonar_snapshot.hpp"
#include <cassert>
#include <iostream>
int main(){
 sensor_state s{};s.received_ms=123456;s.sample.sequence=7;s.sample.uptime_ms=500;
 s.sample.valid_mask=7;s.sample.fault_mask=0;
 s.sample.distance_mm[0]=99;s.sample.distance_mm[1]=400;s.sample.distance_mm[2]=65534;
 s.sample.age_ms[0]=2;s.sample.age_ms[1]=5;s.sample.age_ms[2]=9;
 const auto value=acg138::sonar_snapshot(s);
 assert(value=="{\"received_mono_ns\":\"123456000000\",\"sequence\":7,\"uptime_ms\":500,\"valid_mask\":7,\"fault_mask\":0,\"distance_mm\":[99,400,65534],\"age_ms\":[2,5,9]}");
 const auto old=value;s.sample.sequence=8;
 // A new packet counter cannot rejuvenate the source timestamp or ages.
 const auto next=acg138::sonar_snapshot(s);
 assert(next.find("123456000000")!=std::string::npos);
 assert(next.find("[2,5,9]")!=std::string::npos);
 s.sample.valid_mask=3;s.sample.fault_mask=4;
 assert(acg138::sonar_snapshot(s).find("\"fault_mask\":4")!=std::string::npos);
 std::cout<<"PASS source timestamp, F/L/R order, ages, raw numeric and fault preservation\n";
}
