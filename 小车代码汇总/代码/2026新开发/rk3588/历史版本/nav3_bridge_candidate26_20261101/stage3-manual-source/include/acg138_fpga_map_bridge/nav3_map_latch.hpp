#pragma once
// Caller serializes access. A load request is explicit and single-use per bridge
// session. Only FPGA ACK establishes commit; BUSY/timeouts never auto-retry a job.
namespace acg138 {
class Nav3MapLatch {
public:
  enum class State { Preview, Requested, Uploading, Frozen, Failed };
private:
  State state_=State::Preview;
public:
  State state()const noexcept{return state_;}
  const char* name()const noexcept {
    switch(state_){case State::Preview:return "PREVIEW";case State::Requested:return "REQUESTED";
      case State::Uploading:return "UPLOADING";case State::Frozen:return "FROZEN";case State::Failed:return "FAILED";}
    return "FAILED";
  }
  bool request()noexcept {if(state_!=State::Preview)return false;state_=State::Requested;return true;}
  bool begin()noexcept {if(state_!=State::Requested)return false;state_=State::Uploading;return true;}
  bool commit()noexcept {if(state_!=State::Uploading)return false;state_=State::Frozen;return true;}
  void fail()noexcept {if(state_==State::Requested||state_==State::Uploading)state_=State::Failed;}
};
}
