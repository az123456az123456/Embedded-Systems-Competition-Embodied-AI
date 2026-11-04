#ifndef NAV_EXEC_WIRE_H
#define NAV_EXEC_WIRE_H
#include <stddef.h>
#include <stdint.h>

/* Wire bytes are big endian. These C structures are never sent as structs. */
#define NE_BODY_BYTES 256u
#define NE_WIRE_BYTES 258u
#define NE_HEADER_BYTES 96u
#define NE_PAYLOAD_BYTES 160u
#define NE_VERSION 1u
#define NE_ETH_NAV3 0x88b9u
#define NE_ETH_NVA1 0x88b6u
#define NE_MAP_BYTES 512u
#define NE_MAP_CHUNKS 16u
#define NE_MAP_CHUNK_BYTES 32u
#define NE_PAGE_CELLS 64u
#define NE_ACK_BYTES 32u
#define NE_MOTION_MS 500u
#define NE_AUTH_MS 30000u
#define NE_POSE_MS 250u
#define NE_CPU_MS 60u

enum ne_family { NE_NAV3=0, NE_NVA1=1 };
enum ne_opcode {
 NE_INFO=0x01, NE_FRAME_BIND=0x02, NE_SESSION_BIND=0x03,
 NE_CLOCK_ANCHOR=0x04, NE_MAP_BEGIN=0x10, NE_MAP_CHUNK=0x11,
 NE_MAP_COMMIT=0x12, NE_GOAL_PLAN=0x20, NE_START=0x21,
 NE_CANCEL=0x22, NE_POSE=0x30, NE_STATE=0x40, NE_PATH_PAGE=0x41,
 NE_EVENT=0x42, NE_PLAN_RESULT=0x43
};
enum ne_permit_op { NE_PROBE=1, NE_PERMIT=2, NE_REVOKE=3 };
enum ne_status {
 NE_OK=0, NE_BAD_REQUEST=1, NE_UNBOUND=2, NE_SESSION=3,
 NE_STALE=4, NE_IDENTITY=5, NE_NO_MAP=6, NE_INCOMPLETE=7,
 NE_MAP_CRC=8, NE_NO_ROUTE=9, NE_BUSY=10, NE_POSE_STALE=11,
 NE_NO_PERMIT=12, NE_GOAL_OUTSIDE_WINDOW=13, NE_REBIND=14,
 NE_CANCELLED=15, NE_INTERNAL=16
};
enum ne_codec_error {
 NE_CODEC_OK=0, NE_E_LENGTH=1, NE_E_MAGIC=2, NE_E_VERSION=3,
 NE_E_CRC=4, NE_E_FLAGS=5, NE_E_RESERVED=6, NE_E_OPCODE=7,
 NE_E_SHAPE=8, NE_E_RANGE=9, NE_E_POINTER=10
};
enum ne_pose_quality { NE_POSE_VALID=1u, NE_TF_VALID=2u };
enum ne_source_clock { NE_CLOCK_ROS=1u, NE_CLOCK_MONOTONIC=2u };
enum ne_owner { NE_OWNER_NONE=0, NE_OWNER_NAV=1 };

/* Numeric offsets are the common wire ABI. */
enum ne_offset {
 NE_O_MAGIC=0, NE_O_VERSION=4, NE_O_OPCODE=5, NE_O_FLAGS=6,
 NE_O_TRANSPORT=8, NE_O_SESSION=16, NE_O_RUN=24, NE_O_GOAL=32,
 NE_O_MAP_EPOCH=48, NE_O_MAP_REVISION=56, NE_O_LOCALIZATION=64,
 NE_O_PLAN=72, NE_O_REQUEST=80, NE_O_PAYLOAD_BYTES=88,
 NE_O_RESERVED=90, NE_O_PAYLOAD=96
};
struct ne_header {
 uint64_t transport_seq, session_id, run_generation;
 uint8_t goal_id[16];
 uint64_t map_epoch, map_revision, localization_epoch, plan_id, request_id;
 uint16_t payload_bytes, flags;
 uint8_t family, opcode;
};
struct ne_message { struct ne_header h; const uint8_t *payload; };

uint16_t ne_get16(const uint8_t *p);
uint32_t ne_get32(const uint8_t *p);
uint64_t ne_get64(const uint8_t *p);
void ne_put16(uint8_t *p,uint16_t v);
void ne_put32(uint8_t *p,uint32_t v);
void ne_put64(uint8_t *p,uint64_t v);
uint16_t ne_crc16(const uint8_t *p,unsigned n);
/* Decode is all-or-nothing. On error *out is unchanged. The view references
 * caller storage, which must remain immutable while the view is in use. */
int ne_decode(const uint8_t *wire,size_t n,struct ne_message *out);
/* Encoder validates shape and does not change out on failure. Payload may
 * overlap out (including out+96); implementation is memmove-safe. */
int ne_encode(const struct ne_header *h,const uint8_t *payload,
              uint8_t out[NE_WIRE_BYTES]);
int ne_validate_shape(const struct ne_message *m);
/* True only for requests that can mutate task execution. POSE, maps and
 * FRAME_BIND are not execution permission; NE_PERMIT is separately gated. */
int ne_is_execution_command(const struct ne_message *m);

#endif
