#include "acg138_fpga_map_bridge/nav3_map_latch.hpp"
#include <iostream>
int main(){
  using G=acg138::Nav3MapLatch;unsigned failed=0,total=0;
  const auto check=[&](const char* id,bool ok){++total;failed+=!ok;std::cout<<"{\"case\":\""<<id<<"\",\"pass\":"<<(ok?"true":"false")<<"}\n";};
  G g;check("preview_cannot_begin_upload",!g.begin());check("preview_cannot_fake_commit",!g.commit());
  check("explicit_request",g.request());check("duplicate_request_rejected",!g.request());
  check("authorized_upload_once",g.begin());check("cannot_begin_second_upload",!g.begin());
  check("ack_establishes_frozen",g.commit()&&g.state()==G::State::Frozen);
  check("frozen_cannot_replace_map",!g.request()&&!g.begin());
  g.fail();check("later_error_cannot_unfreeze",g.state()==G::State::Frozen);
  G f;f.request();f.begin();f.fail();check("busy_or_timeout_latches_failure",f.state()==G::State::Failed);
  check("failed_job_no_automatic_retry",!f.request()&&!f.begin()&&!f.commit());
  G q;q.request();q.fail();check("expired_queued_job_fails_closed",!q.begin());
  std::cout<<"{\"total\":"<<total<<",\"failed\":"<<failed<<"}\n";return failed?1:0;
}
