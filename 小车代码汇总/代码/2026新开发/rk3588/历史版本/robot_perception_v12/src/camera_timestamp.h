#ifndef PERCEPTION_CAMERA_TIMESTAMP_H
#define PERCEPTION_CAMERA_TIMESTAMP_H
#include <cstdint>
#include <limits>
#include <linux/videodev2.h>

// V4L2 MONOTONIC timestamps share Linux's steady-clock epoch. Reject realtime,
// malformed, future or implausibly old timestamps and use dequeue time instead.
// A trusted driver timestamp may mean start/end of frame, not exposure time.
inline uint64_t camera_source_timestamp(const v4l2_buffer &buffer,uint64_t dequeued_ms) {
    if((buffer.flags&V4L2_BUF_FLAG_TIMESTAMP_MASK)!=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC ||
       buffer.timestamp.tv_sec<0 || buffer.timestamp.tv_usec<0 || buffer.timestamp.tv_usec>=1000000)
        return dequeued_ms;
    const auto seconds=static_cast<uint64_t>(buffer.timestamp.tv_sec);
    const auto millis=static_cast<uint64_t>(buffer.timestamp.tv_usec)/1000;
    if(seconds>(std::numeric_limits<uint64_t>::max()-millis)/1000)return dequeued_ms;
    const auto source=seconds*1000+millis;
    return source && source<=dequeued_ms && dequeued_ms-source<=5000?source:dequeued_ms;
}
#endif
