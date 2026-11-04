#ifndef NAV_RUNTIME_H
#define NAV_RUNTIME_H
#include "nav_exec_mmio.h"
#include "nav_track.h"
#include "nav_avoid.h"
#include "layered_mmio.h"
/* One production executor, one shared stop proof and one RX/TX workspace.
 * No copied cell path or second authority implementation. */
struct nav_runtime {
 struct nav_exec exec;
 struct nav_track track;
 struct nav_avoid avoid;
 struct nv_stop stop;
 struct nav_hw_event adopted;
 struct nav_hw_live hardware;
 struct ne_header reply_header;
 uint8_t rx[NE_WIRE_BYTES],tx[NE_WIRE_BYTES],ack[2][NE_WIRE_BYTES];
 uint64_t tx_seq,last_state_seq,last_plan,last_ack_seq,last_request;
 uint64_t installed_run,installed_plan,release_session,release_run,release_epoch,release_event;
 uint32_t last_poll,last_state_ms,ticket;
 uint32_t token,rx_ms,max_service_gap_cycles,last_service_cycle;
 uint32_t held_token,ack_dropped,missed_events;
 uint32_t job_ms,command_cycles;
 uint32_t release_ticket;
 uint8_t lane,stage,publish_action,have_reply,state_pending,plan_pending,hw_claimed;
 uint8_t ack_lane[2],ack_used[2],held_lane,last_plan_valid,ready;
 uint8_t task_pending,claim_pending,fence_pending,hw_started;
 uint8_t fast_turn;
 uint8_t ack_newest;
 uint16_t reply_status;
};
void nav_runtime_init(struct nav_runtime *runtime,struct nav_server *server);
/* One coherent read only. Hardware owns new_epoch; adoption never adds one. */
int nav_runtime_adopt(struct nav_runtime *runtime);
/* Called at every main iteration and each bounded A* checkpoint; it never
 * invokes astar_plan recursively and never waits for RX/TX/ACK or owner busy. */
void nav_runtime_service(struct nav_runtime *runtime);
/* Only main may call plan; the installed cooperation callback calls service. */
void nav_runtime_plan(struct nav_runtime *runtime);
#endif
