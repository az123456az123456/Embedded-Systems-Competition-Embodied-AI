#include "nav_protocol.h"
uint32_t nav_get32(const uint8_t *p) { return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }
void nav_put32(uint8_t *p,uint32_t v) { p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v; }
uint32_t nav_crc32(const uint8_t *p,unsigned len) {
 uint32_t c=UINT32_MAX;unsigned i,k;
 for(i=0;i<len;++i){c^=p[i];for(k=0;k<8;++k)c=(c>>1)^((c&1u)?UINT32_C(0xedb88320):0u);}
 return ~c;
}
static void copy(uint8_t *d,const uint8_t *s,unsigned n){unsigned i;for(i=0;i<n;++i)d[i]=s[i];}
static int same(const uint8_t *a,const uint8_t *b,unsigned n){unsigned i;for(i=0;i<n;++i)if(a[i]!=b[i])return 0;return 1;}
static int zeros(const uint8_t *p,unsigned n){unsigned i;for(i=0;i<n;++i)if(p[i])return 0;return 1;}
void nav_server_init(struct nav_server *s) {
 unsigned i;uint8_t *p=(uint8_t*)s;for(i=0;i<sizeof(*s);++i)p[i]=0;
}
int nav_process(struct nav_server *s,const uint8_t q[64],uint8_t r[64],uint32_t (*cycles)(void)) {
 unsigned i;uint8_t op=q[5],error=NAV_OK;uint32_t id=nav_get32(q+8),gen=nav_get32(q+12);
 uint32_t a=nav_get32(q+16),b=nav_get32(q+20),c=nav_get32(q+24);
 if(nav_get32(q)!=NAV_MAGIC||q[4]!=1||q[6]||!zeros(q+28,4)||op<1||op>6)return 0;
 for(i=0;i<64;++i)r[i]=0;
 nav_put32(r,NAV_MAGIC);r[4]=1;r[5]=op|0x80u;nav_put32(r+8,id);nav_put32(r+12,gen);
 if(op==NAV_INFO){
  if(q[7]||a||b||c||!zeros(q+32,32)){r[6]=NAV_BAD_REQUEST;return 1;}
  r[7]=(uint8_t)(s->map_valid|(s->upload_active<<1)|(s->path_valid<<2));
  nav_put32(r+16,s->map_valid?s->generation:0);nav_put32(r+20,s->have_previous?s->previous_id:0);
  nav_put32(r+24,s->path_valid?s->plan_id:0);nav_put32(r+32,UINT32_C(0x00010000));return 1;
 }
 if(s->have_previous&&id==s->previous_id){
  if(same(q,s->previous_request,64)){copy(r,s->previous_reply,64);return 1;}
  r[6]=NAV_STALE;return 1;
 }
 if(!id||(s->have_previous&&(int32_t)(id-s->previous_id)<=0)){r[6]=NAV_STALE;return 1;}
 if(!gen)error=NAV_GENERATION;
 else if(c|| (op!=NAV_CHUNK&&!zeros(q+32,32)))error=NAV_BAD_REQUEST;
 else switch(op){
 case NAV_BEGIN:
  if(q[7]||b)error=NAV_BAD_REQUEST;
  else if(s->map_valid&&gen==s->generation)error=NAV_GENERATION;
  else {s->staging_generation=gen;s->expected_crc=a;s->chunks=0;s->upload_active=1;}
  break;
 case NAV_CHUNK:
  if(q[7]!=32||b||a>=ASTAR_BITMAP_BYTES||(a&31u))error=NAV_BAD_REQUEST;
  else if(!s->upload_active||gen!=s->staging_generation)error=NAV_GENERATION;
  else {copy(s->staging+a,q+32,32);s->chunks|=(uint16_t)(1u<<(a>>5));nav_put32(r+16,a);}
  break;
 case NAV_COMMIT:
  if(q[7]||a||b)error=NAV_BAD_REQUEST;
  else if(!s->upload_active||gen!=s->staging_generation)error=NAV_GENERATION;
  else if(s->chunks!=UINT16_MAX)error=NAV_INCOMPLETE;
  else if(nav_crc32(s->staging,ASTAR_BITMAP_BYTES)!=s->expected_crc)error=NAV_MAP_CRC;
  else {copy(s->planner.occupied,s->staging,ASTAR_BITMAP_BYTES);s->generation=gen;s->map_valid=1;s->upload_active=0;s->path_valid=0;s->path_length=0;s->plan_id=0;}
  break;
 case NAV_PLAN:
  if(q[7]||a>=ASTAR_CELLS||b>=ASTAR_CELLS)error=NAV_BAD_REQUEST;
  else if(!s->map_valid)error=NAV_NO_MAP;
  else if(gen!=s->generation)error=NAV_GENERATION;
  else if(s->upload_active)error=NAV_BUSY;
  else {
   uint32_t began=cycles?cycles():0;int n;
   s->path_valid=0;s->path_length=0;s->plan_id=0;
   n=astar_plan(&s->planner,a&63u,a>>6,b&63u,b>>6);
   s->plan_cycles=cycles?cycles()-began:0;
   if(n>0){s->path_length=(uint16_t)n;s->path_valid=1;s->plan_id=id;}
   else error=n==ASTAR_NO_ROUTE?NAV_NO_ROUTE:n==ASTAR_BLOCKED_ENDPOINT?NAV_BLOCKED:NAV_INTERNAL;
   nav_put32(r+16,s->path_length);nav_put32(r+20,s->planner.expanded);nav_put32(r+24,s->plan_cycles);
  }break;
 case NAV_PAGE:
  if(!q[7]||q[7]>16)error=NAV_BAD_REQUEST;
  else if(!s->map_valid)error=NAV_NO_MAP;
  else if(gen!=s->generation||!s->path_valid||b!=s->plan_id)error=NAV_GENERATION;
  else if(a>=s->path_length)error=NAV_BAD_REQUEST;
  else {
   unsigned n=s->path_length-a;if(n>q[7])n=q[7];r[7]=(uint8_t)n;
   nav_put32(r+16,a);nav_put32(r+20,s->path_length);nav_put32(r+24,s->plan_id);
   for(i=0;i<n;++i){uint16_t cell=s->planner.path[a+i];r[32+2*i]=(uint8_t)(cell>>8);r[33+2*i]=(uint8_t)cell;}
  }break;
 default:error=NAV_BAD_REQUEST;break;
 }
 r[6]=error;s->previous_id=id;s->have_previous=1;
 copy(s->previous_request,q,64);copy(s->previous_reply,r,64);return 1;
}
