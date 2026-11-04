#pragma once
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <sys/socket.h>
#include <time.h>
namespace acg138 {
struct RxStamp {std::int64_t realtime_ns;std::uint64_t mono_ms;};
// SO_TIMESTAMPNS uses kernel receive CLOCK_REALTIME, not callback execution time.
// The explicit age check prevents a queued packet becoming a new-looking TF.
inline std::optional<RxStamp> receive_stamp(msghdr& message,std::int64_t real_now,std::uint64_t mono_now_ms,std::int64_t max_age_ns=250000000LL){
 if(message.msg_flags&(MSG_CTRUNC|MSG_TRUNC))return std::nullopt;
 for(auto*c=CMSG_FIRSTHDR(&message);c;c=CMSG_NXTHDR(&message,c)){
  if(c->cmsg_level!=SOL_SOCKET||c->cmsg_type!=SCM_TIMESTAMPNS||c->cmsg_len<CMSG_LEN(sizeof(timespec)))continue;
  timespec t{};std::memcpy(&t,CMSG_DATA(c),sizeof(t));
  if(t.tv_sec<0||t.tv_nsec<0||t.tv_nsec>=1000000000L||t.tv_sec>(std::numeric_limits<std::int64_t>::max()-t.tv_nsec)/1000000000LL)return std::nullopt;
  const auto packet=std::int64_t(t.tv_sec)*1000000000LL+t.tv_nsec;
  if(packet>real_now||real_now-packet>max_age_ns)return std::nullopt;
  const auto age_ms=static_cast<std::uint64_t>((real_now-packet)/1000000LL);
  if(age_ms>mono_now_ms)return std::nullopt;
  return RxStamp{packet,mono_now_ms-age_ms};
 }
 return std::nullopt; // Missing timestamps must not silently fall back to now().
}
}
