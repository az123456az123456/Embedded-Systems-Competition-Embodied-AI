#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/nav_protocol.h"
static struct nav_server s;
static uint8_t q[64],r[64],saved[64],map[512];
static uint32_t seq;
static void request(unsigned op,uint32_t gen,uint32_t a,uint32_t b,unsigned count) {
 memset(q,0,sizeof(q));nav_put32(q,NAV_MAGIC);q[4]=1;q[5]=op;q[7]=count;
 nav_put32(q+8,++seq);nav_put32(q+12,gen);nav_put32(q+16,a);nav_put32(q+20,b);
}
static void send(unsigned status){assert(nav_process(&s,q,r,0));assert(r[6]==status);assert((r[5]&127)==q[5]);assert(nav_get32(r+8)==seq);}
static void upload(unsigned gen){unsigned i;request(NAV_BEGIN,gen,nav_crc32(map,512),0,0);send(0);for(i=0;i<16;i++){request(NAV_CHUNK,gen,i*32,0,32);memcpy(q+32,map+i*32,32);send(0);}request(NAV_COMMIT,gen,0,0,0);send(0);}
int main(void){
 unsigned i,plan,n;nav_server_init(&s);
 assert(nav_crc32((const uint8_t*)"123456789",9)==0xcbf43926u);
 request(NAV_INFO,0,0,0,0);send(0);assert(r[7]==0&&nav_get32(r+20)==0);
 request(NAV_PLAN,1,0,4095,0);send(NAV_NO_MAP);
 request(NAV_BEGIN,1,nav_crc32(map,512),0,0);send(0);
 request(NAV_CHUNK,1,1,0,32);send(NAV_BAD_REQUEST);
 request(NAV_COMMIT,1,0,0,0);send(NAV_INCOMPLETE);assert(!s.map_valid);
 for(i=0;i<16;i++){request(NAV_CHUNK,1,i*32,0,32);send(0);}
 request(NAV_COMMIT,1,0,0,0);send(0);assert(s.map_valid&&s.generation==1);
 memcpy(saved,r,64);assert(nav_process(&s,q,r,0));assert(!memcmp(saved,r,64));
 q[19]=1;assert(nav_process(&s,q,r,0)&&r[6]==NAV_STALE);q[19]=0;
 request(NAV_PLAN,1,0,4095,0);plan=seq;send(0);n=nav_get32(r+16);assert(n==127);
 for(i=0;i<n;i+=16){unsigned k;request(NAV_PAGE,1,i,plan,16);send(0);assert(r[7]==(n-i>16?16:n-i));for(k=0;k<r[7];k++){unsigned cell=(r[32+2*k]<<8)|r[33+2*k];assert(cell==s.planner.path[i+k]);}}
 request(NAV_PAGE,1,127,plan,16);send(NAV_BAD_REQUEST);
 request(NAV_PAGE,1,0,plan+1,16);send(NAV_GENERATION);
 request(NAV_BEGIN,2,1,0,0);send(0);
 request(NAV_PLAN,1,0,4095,0);send(NAV_BUSY);
 for(i=0;i<16;i++){request(NAV_CHUNK,2,i*32,0,32);send(0);}
 request(NAV_COMMIT,2,0,0,0);send(NAV_MAP_CRC);assert(s.generation==1&&s.path_valid);
 memset(map,0xff,512);upload(3);assert(!s.path_valid);
 request(NAV_PLAN,3,0,4095,0);send(NAV_BLOCKED);
 memset(map,0,512);memset(map+32*8,255,8);upload(4);
 request(NAV_PLAN,4,0,4095,0);send(NAV_NO_ROUTE);
 request(NAV_PLAN,3,0,4095,0);send(NAV_GENERATION);
 request(NAV_CHUNK,4,512,0,32);send(NAV_BAD_REQUEST);
 request(NAV_INFO,0,0,0,0);send(0);assert(nav_get32(r+16)==4&&nav_get32(r+20)==seq-1);
 /* Paging must support every possible cell, not just a short fixed route. */
 s.path_valid=1;s.path_length=4096;s.plan_id=1234;for(i=0;i<4096;i++)s.planner.path[i]=i;
 for(i=0;i<4096;i+=16){request(NAV_PAGE,4,i,1234,16);send(0);assert(r[7]==16);assert((((unsigned)r[62]<<8)|r[63])==i+15);}
 request(NAV_PLAN,4,0,4095,0);nav_put32(q+8,1);assert(nav_process(&s,q,r,0)&&r[6]==NAV_STALE);
 request(NAV_INFO,0,0,0,0);q[0]=0;assert(!nav_process(&s,q,r,0));
 nav_server_init(&s);s.have_previous=1;s.previous_id=0xfffffff0u;seq=0xfffffffeu;
 request(NAV_BEGIN,5,nav_crc32(map,512),0,0);send(0);
 request(NAV_CHUNK,5,0,0,32);send(NAV_STALE); /* zero is reserved */
 request(NAV_CHUNK,5,0,0,32);send(0);assert(s.previous_id==1);
 request(NAV_INFO,0,0,0,0);nav_put32(q+8,0);assert(nav_process(&s,q,r,0)&&nav_get32(r+20)==1);
 puts("PASS navigation protocol: CRC vector, atomic map, chunk bounds, map CRC failure, retries/cache, stale/conflict/wrap, generation, plan/no-route/blocked, all4096 path paging, INFO restart discovery");
}
