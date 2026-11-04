#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
namespace robotnav {
// The existing dashboard validates private HTTP intents on its millisecond
// clock. Floor the source timestamp so quantization can only shorten its TTL,
// never make a new request appear to originate in the future or extend a lease.
inline uint64_t dashboard_request_timestamp(uint64_t monotonic_ns){return monotonic_ns/1000000*1000000;}
inline uint64_t next_dashboard_generation(uint64_t local,uint64_t observed){
 if(observed==std::numeric_limits<uint64_t>::max())throw std::overflow_error("dashboard generation exhausted");
 return local>observed?local:observed+1;
}
struct HttpResult {int status=503;std::string body;bool transport_ok=false;};
HttpResult dashboard_request(unsigned port,const std::string&token,const std::string&method,
                             const std::string&path,const std::string&body,unsigned timeout_ms=80);
}
