#ifndef PERCEPTION_RGB_CAMERA_H
#define PERCEPTION_RGB_CAMERA_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fpga_rgb {
constexpr uint16_t EtherType=0x88ba;
constexpr unsigned Width=640,Height=640,PartPixels=320,Parts=2;
constexpr size_t HeaderBytes=32,PayloadBytes=PartPixels*3,PacketBytes=HeaderBytes+PayloadBytes;
constexpr size_t FrameBytes=Width*Height*3*2,ChunksPerPair=Height*Parts*2;
constexpr std::array<uint8_t,6> DefaultPeer{{0x02,0xac,0x13,0x80,0x00,0x01}};
uint32_t crc32_update(uint32_t running,const uint8_t *data,size_t bytes);
uint32_t packet_crc32(const uint8_t *header,const uint8_t *payload);
std::array<uint8_t,16> start_command(uint16_t epoch);

struct FrameInfo {
    uint32_t sequence=0;uint16_t epoch=0;
    uint64_t device_uptime_ms=0,source_ms=0,first_rx_ms=0,completed_ms=0;
    unsigned complete_mask=0;
};
struct Stats {
    uint64_t packets=0,accepted=0,completed=0,superseded=0;
    uint64_t malformed=0,crc_bad=0,wrong_epoch=0,old=0,duplicates=0,conflicts=0;
    uint64_t partial_expired=0,complete_expired=0,partial_evicted=0,resets=0;
};

// Two bounded in-progress source frames plus one latest complete stereo frame.
// Never publishes a partial/mixed-time pair; socket ownership is separate.
class Assembler {
public:
    explicit Assembler(uint16_t epoch,uint64_t max_age_ms=250);
    ~Assembler();
    bool ingest(const uint8_t *payload,size_t bytes,uint64_t host_ms);
    bool take_latest(std::vector<uint8_t> &rgb,FrameInfo &info,uint64_t host_ms);
    void expire(uint64_t host_ms);
    void restart_epoch(uint16_t epoch);
    const Stats &stats()const;
private:
    struct Impl;std::unique_ptr<Impl> p;
    Assembler(const Assembler&)=delete;Assembler &operator=(const Assembler&)=delete;
};

struct ReadTiming {double wait_ms=0,receive_ms=0;FrameInfo frame;};
class RgbCamera {
public:
    RgbCamera();~RgbCamera();
    bool open(const std::string &interface,std::string &error);
    int read(std::vector<uint8_t> &rgb,std::string &error,ReadTiming *timing=nullptr);
    std::string description()const;
    // Test seam: duplicates a connected datagram socket, does not open NICs.
    bool adopt_test_socket(int fd,uint16_t epoch,std::string &error);
private:
    struct Impl;std::unique_ptr<Impl> p;
};
}
#endif
