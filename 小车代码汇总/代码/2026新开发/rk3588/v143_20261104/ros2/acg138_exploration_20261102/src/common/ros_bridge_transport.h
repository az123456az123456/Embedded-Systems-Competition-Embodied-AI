#pragma once
#include "ros_bridge_session.h"
#include <atomic>
#include <functional>
#include <memory>
namespace arb1 {
uint64_t mono_ns();
Id kernel_boot_id();
// Each lane has its own thread, queue and stream. Callbacks must be bounded.
class LocalBridge {
 struct Impl;std::unique_ptr<Impl> p_;
public:
 using Receiver=std::function<void(Channel,const AcceptedMessage&)>;
 LocalBridge(const std::string& directory,bool server,Receiver);
 ~LocalBridge();
 bool send(Channel,const Message&); // bounded, never waits for socket IO
 bool ready(Channel)const;
 uint64_t epoch()const;
 size_t queued_bytes(Channel)const;
};
}
