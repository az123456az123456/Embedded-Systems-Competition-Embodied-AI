#include "nav_exec_mmio.h"
#include "mmio.h"
#include <string.h>
#ifndef NAV_MMIO_TEST
uint32_t nav_mmio_read(unsigned offset){return NAV_REG(offset);}
void nav_mmio_write(unsigned offset,uint32_t value){NAV_REG(offset)=value;}
#endif
uint64_t nav_mmio_read64(unsigned o){uint32_t lo=nav_mmio_read(o),hi=nav_mmio_read(o+4u);return ((uint64_t)hi<<32)|lo;}
void nav_mmio_write64(unsigned o,uint64_t v){nav_mmio_write(o,(uint32_t)v);nav_mmio_write(o+4u,(uint32_t)(v>>32));}
int nav_mmio_event(struct nav_hw_event *e)
{
 uint32_t a=nav_mmio_read(NM_EVENT_GENERATION);if(a&1u)return 0;
 e->flags=nav_mmio_read(NM_EVENT_FLAGS);e->session=nav_mmio_read64(NM_EVENT_SESSION);e->id=nav_mmio_read64(NM_EVENT_ID);e->old_epoch=nav_mmio_read64(NM_EVENT_OLD_EPOCH);e->new_epoch=nav_mmio_read64(NM_EVENT_NEW_EPOCH);e->reasons=nav_mmio_read(NM_EVENT_REASON);e->context=nav_mmio_read(NM_EVENT_CONTEXT);
 return a==nav_mmio_read(NM_EVENT_GENERATION)&&!(e->flags&~3u)&&!(e->reasons&~511u);
}
int nav_mmio_live(struct nav_hw_live *l)
{
 uint32_t a=nav_mmio_read(NM_GENERATION);if(a&1u)return 0;
 l->generation=a;l->owner=nav_mmio_read(NM_OWNER);l->live=nav_mmio_read(NM_LIVE);l->gates=nav_mmio_read(NM_GATES);l->epoch=nav_mmio_read64(0xb08);l->session=nav_mmio_read64(0xb10);l->context=nav_mmio_read(0xb30);l->pose_seq=nav_mmio_read64(0xb40);l->permit_seq=nav_mmio_read64(0xb50);l->auth_epoch=nav_mmio_read64(0xb60);l->permit_token=nav_mmio_read(0xbb8);l->pose_token=nav_mmio_read(0xbbc);
 return a==nav_mmio_read(NM_GENERATION);
}
int nav_mmio_pin(unsigned lane,uint32_t *token,uint32_t *rx)
{
 unsigned acq,tk,stamp,flags;uint32_t f;
 if(lane==NL_REQUEST){if(!(nav_mmio_read(NM_REQ_PENDING)&1u))return 0;*token=nav_mmio_read(NM_REQ_TOKEN);*rx=nav_mmio_read(NM_REQ_RX_MS);return *token!=0;}
 if(lane==NL_CANCEL){*token=nav_mmio_read(NM_CANCEL_TOKEN);if(*token){*rx=nav_mmio_read(NM_CANCEL_RX_MS);return 1;}if(nav_mmio_read(NM_CANCEL_PENDING)&1u)nav_mmio_write(NM_CANCEL_ACQUIRE,1);return 0;}
 if(lane==NL_POSE){acq=NM_POSE_ACQUIRE;tk=NM_POSE_TOKEN;stamp=NM_POSE_RX_MS;flags=NM_POSE_FLAGS;}
 else if(lane==NL_PERMIT){acq=NM_PERMIT_ACQUIRE;tk=NM_PERMIT_TOKEN;stamp=NM_PERMIT_RX_MS;flags=NM_PERMIT_FLAGS;}else return 0;
 f=nav_mmio_read(flags);if(f&1u){*token=nav_mmio_read(tk);*rx=nav_mmio_read(stamp);return *token!=0;}if(!(f&4u))nav_mmio_write(acq,1);return 0;
}
void nav_mmio_release(unsigned lane,uint32_t token)
{
 static const uint16_t offset[4]={NM_REQ_RELEASE,NM_POSE_RELEASE,NM_PERMIT_RELEASE,NM_CANCEL_RELEASE};if(lane<4u)nav_mmio_write(offset[lane],token);
}
int nav_mmio_message(unsigned lane,uint8_t b[256],struct ne_message *m)
{
 unsigned off,n,i;uint32_t magic;struct ne_header *h=&m->h;
 if(lane==NL_REQUEST){off=NM_REQ_BODY;n=256;}
 else if(lane==NL_POSE){off=NM_POSE_BODY;n=160;}
 else if(lane==NL_PERMIT){off=NM_PERMIT_BODY;n=160;}
 else if(lane==NL_CANCEL){off=NM_CANCEL_BODY;n=128;}else return NE_BAD_REQUEST;
 for(i=0;i<256u;++i)b[i]=0;
 for(i=0;i<n;i+=4u)ne_put32(b+i,nav_mmio_read(off+i));
 magic=ne_get32(b);if(magic!=UINT32_C(0x4e415633)&&magic!=UINT32_C(0x4e564131))return NE_BAD_REQUEST;
 if(b[4]!=NE_VERSION||ne_get16(b+90)||ne_get32(b+92))return NE_BAD_REQUEST;
 h->family=(uint8_t)(magic==UINT32_C(0x4e564131));h->opcode=b[5];h->flags=ne_get16(b+6);h->transport_seq=ne_get64(b+8);h->session_id=ne_get64(b+16);h->run_generation=ne_get64(b+24);
 for(i=0;i<16;++i)h->goal_id[i]=b[32+i];
 h->map_epoch=ne_get64(b+48);h->map_revision=ne_get64(b+56);h->localization_epoch=ne_get64(b+64);h->plan_id=ne_get64(b+72);h->request_id=ne_get64(b+80);h->payload_bytes=ne_get16(b+88);m->payload=b+96;
 if(h->payload_bytes>160u||h->payload_bytes+96u>n)return NE_BAD_REQUEST;
 for(i=96u+h->payload_bytes;i<n;++i)if(b[i])return NE_BAD_REQUEST;
 if(ne_validate_shape(m)!=NE_CODEC_OK||(h->opcode&0x80u))return NE_BAD_REQUEST;
 if((lane==NL_POSE&&(h->family!=NE_NAV3||(h->opcode!=NE_POSE&&h->opcode!=NE_REAR_CLEAR)))||(lane==NL_PERMIT&&(h->family!=NE_NVA1||h->opcode!=NE_PERMIT))||(lane==NL_CANCEL&&!((h->family==NE_NAV3&&h->opcode==NE_CANCEL)||(h->family==NE_NVA1&&h->opcode==NE_REVOKE))))return NE_BAD_REQUEST;
 return NE_OK;
}
int nav_mmio_encode_body(const struct ne_header *h,uint8_t b[256])
{
 struct ne_message m;unsigned i;int rc;
 if(!h||!b||((uintptr_t)h<(uintptr_t)b+256u&&(uintptr_t)h+sizeof(*h)>(uintptr_t)b))return NE_E_POINTER;
 m.h=*h;m.payload=b+96;rc=ne_validate_shape(&m);if(rc)return rc;
 for(i=0;i<96u;++i)b[i]=0;
 for(i=96u+h->payload_bytes;i<256u;++i)b[i]=0;
 ne_put32(b,h->family==NE_NAV3?UINT32_C(0x4e415633):UINT32_C(0x4e564131));b[4]=NE_VERSION;b[5]=h->opcode;
 ne_put64(b+8,h->transport_seq);ne_put64(b+16,h->session_id);ne_put64(b+24,h->run_generation);
 memcpy(b+32,h->goal_id,16);
 ne_put64(b+48,h->map_epoch);ne_put64(b+56,h->map_revision);ne_put64(b+64,h->localization_epoch);ne_put64(b+72,h->plan_id);ne_put64(b+80,h->request_id);ne_put16(b+88,h->payload_bytes);return NE_CODEC_OK;
}
