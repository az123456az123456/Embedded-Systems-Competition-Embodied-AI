#include "nav_exec_wire.h"

uint16_t ne_get16(const uint8_t *p){return (uint16_t)(((uint16_t)p[0]<<8)|p[1]);}
uint32_t ne_get32(const uint8_t *p){return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];}
uint64_t ne_get64(const uint8_t *p){return ((uint64_t)ne_get32(p)<<32)|ne_get32(p+4);}
void ne_put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)(v>>8);p[1]=(uint8_t)v;}
void ne_put32(uint8_t *p,uint32_t v){p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;}
void ne_put64(uint8_t *p,uint64_t v){ne_put32(p,(uint32_t)(v>>32));ne_put32(p+4,(uint32_t)v);}
uint16_t ne_crc16(const uint8_t *p,unsigned n){
 uint16_t c=0xffffu;unsigned i,k;
 for(i=0;i<n;++i){c^=(uint16_t)((uint16_t)p[i]<<8);for(k=0;k<8;++k)c=(uint16_t)((c<<1)^((c&0x8000u)?0x1021u:0u));}
 return c;
}
static int zero(const uint8_t *p,unsigned n){unsigned i;for(i=0;i<n;++i)if(p[i])return 0;return 1;}
static int len(const struct ne_message *m,unsigned n){return m->h.payload_bytes==n?NE_CODEC_OK:NE_E_SHAPE;}
static int task(const struct ne_header *h){return h->run_generation && !zero(h->goal_id,16) && h->map_epoch && h->map_revision && h->localization_epoch;}
static int known(unsigned op){
 switch(op){case NE_INFO:case NE_FRAME_BIND:case NE_SESSION_BIND:case NE_CLOCK_ANCHOR:
 case NE_MAP_BEGIN:case NE_MAP_CHUNK:case NE_MAP_COMMIT:case NE_GOAL_PLAN:
 case NE_START:case NE_CANCEL:case NE_POSE:case NE_STATE:case NE_PATH_PAGE:
 case NE_EVENT:case NE_PLAN_RESULT:return 1;default:return 0;}
}
int ne_validate_shape(const struct ne_message *m){
 const struct ne_header *h;const uint8_t *p;unsigned op,n,i;int reply;
 if(!m)return NE_E_POINTER;
 h=&m->h;p=m->payload;n=h->payload_bytes;
 if(n>NE_PAYLOAD_BYTES)return NE_E_LENGTH;
 if(n&&!p)return NE_E_POINTER;
 if(h->flags)return NE_E_FLAGS;
 if(h->family>NE_NVA1)return NE_E_MAGIC;
 op=h->opcode&0x7fu;reply=(h->opcode&0x80u)!=0;
 if(h->family==NE_NVA1){
  if(op<NE_PROBE||op>NE_REVOKE)return NE_E_OPCODE;
  if(reply){
   if(len(m,NE_ACK_BYTES))return NE_E_SHAPE;
   if(!zero(p+28,4))return NE_E_RESERVED;
   return ne_get16(p)<=NE_INTERNAL?NE_CODEC_OK:NE_E_RANGE;
  }
  if(op==NE_PROBE)return len(m,0);
  if(!h->session_id||!h->transport_seq||!h->request_id||!h->run_generation)return NE_E_RANGE;
  if(op==NE_REVOKE){if(len(m,16))return NE_E_SHAPE;return zero(p+12,4)?NE_CODEC_OK:NE_E_RESERVED;}
  if(len(m,64))return NE_E_SHAPE;
  if(ne_get32(p)!=NE_OWNER_NAV||!ne_get64(p+4)||!ne_get64(p+12)||!ne_get64(p+20)||!ne_get64(p+28)||!ne_get64(p+44))return NE_E_RANGE;
  return zero(p+52,12)?NE_CODEC_OK:NE_E_RESERVED;
 }
 if(!known(op))return NE_E_OPCODE;
 if(reply){
  if(op==NE_POSE||op==NE_STATE||op==NE_EVENT||op==NE_PLAN_RESULT)return NE_E_OPCODE;
  if(n<(op==NE_INFO?64u:NE_ACK_BYTES))return NE_E_SHAPE;
  if(ne_get16(p)>NE_INTERNAL)return NE_E_RANGE;
  if(!zero(p+28,4))return NE_E_RESERVED;
  if(op==NE_INFO)return len(m,64);
  if(op==NE_PATH_PAGE){
   unsigned offset=ne_get16(p+24),count=ne_get16(p+26); /* ACK value */
   if(count>NE_PAGE_CELLS||n!=NE_ACK_BYTES+count*2u)return NE_E_SHAPE;
   if(ne_get16(p)!=NE_OK&&count)return NE_E_SHAPE;
   if(offset>4096u||offset+count>4096u)return NE_E_RANGE;
   for(i=0;i<count;++i)if(ne_get16(p+NE_ACK_BYTES+2u*i)>=4096u)return NE_E_RANGE;
   return NE_CODEC_OK;
  }
  return len(m,NE_ACK_BYTES);
 }
 if(op==NE_INFO)return len(m,0);
 if(!h->transport_seq)return NE_E_RANGE;
 if(op!=NE_POSE&&op!=NE_STATE&&op!=NE_EVENT&&op!=NE_PLAN_RESULT&&!h->request_id)return NE_E_RANGE;
 if(op==NE_SESSION_BIND){
  if(len(m,16))return NE_E_SHAPE;
  return !h->session_id&&ne_get64(p)&&ne_get64(p+8)?NE_CODEC_OK:NE_E_RANGE;
 }
 if(!h->session_id)return NE_E_RANGE;
 switch(op){
 case NE_FRAME_BIND:
  if(len(m,68))return NE_E_SHAPE;
  if(!ne_get32(p)||!p[4]||p[4]>63)return NE_E_RANGE;
  for(i=0;i<p[4];++i)if(!p[5+i])return NE_E_RANGE;
  return zero(p+5+p[4],63-p[4])?NE_CODEC_OK:NE_E_RESERVED;
 case NE_CLOCK_ANCHOR:
  if(len(m,24))return NE_E_SHAPE;
  if(!zero(p+16,8))return NE_E_RESERVED;
  return ne_get64(p)&&ne_get32(p+12)<=100u?NE_CODEC_OK:NE_E_RANGE;
 case NE_MAP_BEGIN:
  if(len(m,32))return NE_E_SHAPE;
  if(!h->map_epoch||!h->map_revision||!ne_get32(p)||ne_get16(p+12)!=100u||p[14]!=64||p[15]!=64||!ne_get64(p+20))return NE_E_RANGE;
  return zero(p+28,4)?NE_CODEC_OK:NE_E_RESERVED;
 case NE_MAP_CHUNK:
  if(len(m,48))return NE_E_SHAPE;
  if(!h->map_epoch||!h->map_revision||!ne_get64(p)||ne_get16(p+8)>=NE_MAP_BYTES||(ne_get16(p+8)&31u)||ne_get16(p+10)!=32u)return NE_E_RANGE;
  return zero(p+12,4)?NE_CODEC_OK:NE_E_RESERVED;
 case NE_MAP_COMMIT:
  if(len(m,16))return NE_E_SHAPE;
  if(!h->map_epoch||!h->map_revision||!ne_get64(p))return NE_E_RANGE;
  return zero(p+12,4)?NE_CODEC_OK:NE_E_RESERVED;
 case NE_GOAL_PLAN:
  if(len(m,24))return NE_E_SHAPE;
  if(!task(h)||!ne_get32(p+8)||!ne_get64(p+12))return NE_E_RANGE;
  return zero(p+20,4)?NE_CODEC_OK:NE_E_RESERVED;
 case NE_START:
  if(len(m,24))return NE_E_SHAPE;
  return task(h)&&h->plan_id&&ne_get64(p)&&ne_get64(p+8)&&ne_get64(p+16)?NE_CODEC_OK:NE_E_RANGE;
 case NE_CANCEL:
  if(len(m,8))return NE_E_SHAPE;
  return h->run_generation&&!zero(h->goal_id,16)&&zero(p+4,4)?NE_CODEC_OK:NE_E_RANGE;
 case NE_POSE:
  if(len(m,60))return NE_E_SHAPE;
  if(!h->map_epoch||!h->map_revision||!h->localization_epoch||!ne_get32(p+12)||!ne_get64(p+16)||!ne_get64(p+48)||ne_get64(p+36)>UINT64_C(250000000))return NE_E_RANGE;
  if((int32_t)ne_get32(p+8)<-3141593||(int32_t)ne_get32(p+8)>=3141593)return NE_E_RANGE;
  return (ne_get32(p+56)&~3u)?NE_E_RESERVED:NE_CODEC_OK;
 case NE_PATH_PAGE:
  if(len(m,8))return NE_E_SHAPE;
  return task(h)&&h->plan_id&&ne_get16(p)<4096u&&ne_get16(p+2)&&ne_get16(p+2)<=NE_PAGE_CELLS?NE_CODEC_OK:NE_E_RANGE;
 case NE_STATE:
  if(len(m,160))return NE_E_SHAPE;
  if(!zero(p+130,2)||!zero(p+152,8)||(ne_get32(p+4)&~255u))return NE_E_RESERVED;
  return p[0]<=5u&&p[1]<=3u&&ne_get32(p+124)<=NE_OWNER_NAV?NE_CODEC_OK:NE_E_RANGE;
 case NE_EVENT:
  if(len(m,64))return NE_E_SHAPE;
  return zero(p+28,4)&&zero(p+60,4)?NE_CODEC_OK:NE_E_RESERVED;
 case NE_PLAN_RESULT:
  if(len(m,64))return NE_E_SHAPE;
  if(!zero(p+60,4)||(ne_get16(p+46)&~1u))return NE_E_RESERVED;
  return task(h)&&ne_get16(p)<=NE_INTERNAL&&ne_get16(p+40)<=4096u&&ne_get16(p+42)<4096u&&ne_get16(p+44)<4096u?NE_CODEC_OK:NE_E_RANGE;
 default:return NE_E_OPCODE;
 }
}
int ne_decode(const uint8_t *w,size_t n,struct ne_message *out){
 struct ne_message m;unsigned i;int e;
 if(!w||!out)return NE_E_POINTER;
 if(n!=NE_WIRE_BYTES)return NE_E_LENGTH;
 if(w[0]=='N'&&w[1]=='A'&&w[2]=='V'&&w[3]=='3')m.h.family=NE_NAV3;
 else if(w[0]=='N'&&w[1]=='V'&&w[2]=='A'&&w[3]=='1')m.h.family=NE_NVA1;
 else return NE_E_MAGIC;
 if(w[4]!=NE_VERSION)return NE_E_VERSION;
 if(ne_crc16(w,NE_BODY_BYTES)!=ne_get16(w+NE_BODY_BYTES))return NE_E_CRC;
 if(!zero(w+90,6))return NE_E_RESERVED;
 m.h.opcode=w[5];m.h.flags=ne_get16(w+6);m.h.transport_seq=ne_get64(w+8);
 m.h.session_id=ne_get64(w+16);m.h.run_generation=ne_get64(w+24);
 for(i=0;i<16;++i)m.h.goal_id[i]=w[32+i];
 m.h.map_epoch=ne_get64(w+48);m.h.map_revision=ne_get64(w+56);
 m.h.localization_epoch=ne_get64(w+64);m.h.plan_id=ne_get64(w+72);
 m.h.request_id=ne_get64(w+80);m.h.payload_bytes=ne_get16(w+88);m.payload=w+96;
 if(m.h.payload_bytes>NE_PAYLOAD_BYTES)return NE_E_LENGTH;
 if(!zero(m.payload+m.h.payload_bytes,NE_PAYLOAD_BYTES-m.h.payload_bytes))return NE_E_RESERVED;
 e=ne_validate_shape(&m);if(e)return e;*out=m;return NE_CODEC_OK;
}
int ne_encode(const struct ne_header *h,const uint8_t *p,uint8_t w[NE_WIRE_BYTES]){
 struct ne_message m;unsigned i;int e;
 if(!h||!w)return NE_E_POINTER;
 m.h=*h;m.payload=p;
 e=ne_validate_shape(&m);if(e)return e;
 /* Support in-place payloads without a 258-byte temporary on the RV32 stack.
  * Snapshot the header before writes, memmove payload first, then clear only
  * the header and unused tail. Payload may overlap either side of its target. */
 h=&m.h;
 if(h->payload_bytes){
  if((uintptr_t)(w+96)>(uintptr_t)p){
   for(i=h->payload_bytes;i>0;--i)w[95+i]=p[i-1];
  }else for(i=0;i<h->payload_bytes;++i)w[96+i]=p[i];
 }
 for(i=0;i<NE_HEADER_BYTES;++i)w[i]=0;
 for(i=NE_HEADER_BYTES+h->payload_bytes;i<NE_WIRE_BYTES;++i)w[i]=0;
 if(h->family==NE_NAV3){w[0]='N';w[1]='A';w[2]='V';w[3]='3';}
 else {w[0]='N';w[1]='V';w[2]='A';w[3]='1';}
 w[4]=NE_VERSION;w[5]=h->opcode;ne_put16(w+6,h->flags);
 ne_put64(w+8,h->transport_seq);ne_put64(w+16,h->session_id);ne_put64(w+24,h->run_generation);
 for(i=0;i<16;++i)w[32+i]=h->goal_id[i];
 ne_put64(w+48,h->map_epoch);ne_put64(w+56,h->map_revision);ne_put64(w+64,h->localization_epoch);
 ne_put64(w+72,h->plan_id);ne_put64(w+80,h->request_id);ne_put16(w+88,h->payload_bytes);
 ne_put16(w+256,ne_crc16(w,256));return NE_CODEC_OK;
}
int ne_is_execution_command(const struct ne_message *m){
 if(!m||(m->h.opcode&0x80u))return 0;
 return m->h.family==NE_NVA1?m->h.opcode==NE_PERMIT||m->h.opcode==NE_REVOKE:
  m->h.opcode==NE_START||m->h.opcode==NE_CANCEL;
}
