#ifndef NAV_EXEC_MMIO_H
#define NAV_EXEC_MMIO_H
#include <stdint.h>
#include "nav_exec.h"
/* Native scalar CSR values; BODY/VIEW words remain big-endian wire words.
 * Source authority: docs/NAV3_MMIO_contract.txt; no new wire ABI here. */
enum nav_csr {
 NM_ID=0x400,NM_ABI=0x404,NM_CAPS=0x408,NM_GATES=0x40c,NM_OWNER=0x410,
 NM_EPOCH=0x414,NM_LIVE=0x41c,NM_NOW=0x420,
 NM_REQ_PENDING=0x424,NM_REQ_TOKEN=0x428,NM_REQ_RX_MS=0x42c,NM_REQ_RELEASE=0x430,
 NM_TX_READY=0x434,NM_TX_TICKET=0x438,NM_TX_SEND=0x43c,
 NM_POSE_ACQUIRE=0x440,NM_POSE_TOKEN=0x444,NM_POSE_RX_MS=0x448,NM_POSE_FLAGS=0x44c,NM_POSE_RELEASE=0x450,NM_POSE_COUNT=0x454,
 NM_PERMIT_ACQUIRE=0x458,NM_PERMIT_TOKEN=0x45c,NM_PERMIT_RX_MS=0x460,NM_PERMIT_FLAGS=0x464,NM_PERMIT_RELEASE=0x468,NM_PERMIT_COUNT=0x46c,
 NM_CANCEL_PENDING=0x470,NM_CANCEL_TOKEN=0x474,NM_CANCEL_RX_MS=0x478,NM_CANCEL_KIND=0x47c,NM_CANCEL_ACQUIRE=0x480,NM_CANCEL_RELEASE=0x484,NM_CANCEL_BYTES=0x488,NM_CANCEL_COUNT=0x48c,
 NM_TX_CLASS=0x490,NM_TX_BYTES=0x494,NM_REQ_DROPPED=0x498,NM_POSE_SUPERSEDED=0x49c,NM_PERMIT_SUPERSEDED=0x4a0,NM_CANCEL_REJECTED=0x4a4,NM_RX_BAD=0x4a8,
 NM_GENERATION=0x4ac,NM_AUTH_PUBLISH=0x4b0,NM_AUTH_RESULT=0x4b4,NM_CLAIM=0x4b8,NM_CLAIM_RESULT=0x4bc,NM_PWM_COMMIT=0x4c0,NM_PWM_RESULT=0x4c4,NM_SW_REVOKE=0x4c8,NM_REVOKE_RESULT=0x4cc,
 NM_EVENT_GENERATION=0x4d0,NM_EVENT_FLAGS=0x4d4,NM_EVENT_SESSION=0x4d8,NM_EVENT_ID=0x4e0,NM_EVENT_OLD_EPOCH=0x4e8,NM_EVENT_NEW_EPOCH=0x4f0,NM_EVENT_REASON=0x4f8,NM_EVENT_CONTEXT=0x4fc,
 NM_REQ_BODY=0x500,NM_TX_BODY=0x600,NM_POSE_BODY=0x700,NM_PERMIT_BODY=0x7a0,NM_CANCEL_BODY=0x840,
 NM_AUTH_SHADOW=0x900,NM_PWM_SHADOW=0xa00,NM_ACCEPTED=0xb00
};
enum nav_hw_result { NH_OK=0,NH_EMPTY=1,NH_BUSY=2,NH_BAD_STROBE=3,NH_BAD_TOKEN=4,NH_BAD_TICKET=5,NH_IDENTITY=6,NH_OLD_EPOCH=7,NH_OLD_SOURCE=8,NH_EXPIRED=9,NH_NO_START=10,NH_ACTIVE_OWNER=11,NH_BAD_RANGE=12,NH_RESERVED=13 };
enum nav_hw_action { NH_INSTALL_SESSION=1,NH_INSTALL_TASK=2,NH_ACCEPT_POSE=3,NH_ACCEPT_PERMIT=4,NH_MAP_FENCE=5 };
enum nav_hw_owner { NH_OWNER_NONE=0,NH_OWNER_MANUAL=1,NH_OWNER_NAV=2 };
enum nav_lane { NL_REQUEST=0,NL_POSE=1,NL_PERMIT=2,NL_CANCEL=3 };
enum nav_tx_class { NT_ACK=0,NT_EVENT=1,NT_STATE=2,NT_PATH=3 };
enum nav_hw_event_reason { NH_EV_MOT=1u,NH_EV_CANCEL=2u,NH_EV_REVOKE=4u,NH_EV_MOTION=8u,NH_EV_AUTH=16u,NH_EV_POSE=32u,NH_EV_TRAP=64u,NH_EV_REFERENCE=128u,NH_EV_EXHAUSTED=256u };
#define NH_LIVE_MANUAL_QUIET (UINT32_C(1)<<7)
struct nav_hw_event { uint64_t session,id,old_epoch,new_epoch;uint32_t reasons,context,flags; };
struct nav_hw_live {
 uint64_t session,epoch,pose_seq,permit_seq,auth_epoch;
 uint32_t generation,context,gates,owner,live,pose_token,permit_token;
};
/* Verification replaces only these physical bus accesses, never admission. */
uint32_t nav_mmio_read(unsigned offset);
void nav_mmio_write(unsigned offset,uint32_t value);
uint64_t nav_mmio_read64(unsigned offset);
void nav_mmio_write64(unsigned offset,uint64_t value);
int nav_mmio_event(struct nav_hw_event *event);
int nav_mmio_live(struct nav_hw_live *live);
/* No busy wait: ACQUIRE starts a crossing; a later poll inspects its reply. */
int nav_mmio_pin(unsigned lane,uint32_t *token,uint32_t *rx_ms);
void nav_mmio_release(unsigned lane,uint32_t token);
/* Reads an immutable pinned view into caller's one256B buffer. Hardware has
 * already checked full FCS/CRC/shape; compact views are not258B wire frames. */
int nav_mmio_message(unsigned lane,uint8_t body[256],struct ne_message *message);
/* TX only: payload already lives at body+96; header must not alias body.
 * Validates the frozen shape, writes an identical256B wire body. The actual
 * dedicated hardware TX block generates CRC16/FCS; no duplicate CPU CRC. */
int nav_mmio_encode_body(const struct ne_header *header,uint8_t body[256]);
#endif
