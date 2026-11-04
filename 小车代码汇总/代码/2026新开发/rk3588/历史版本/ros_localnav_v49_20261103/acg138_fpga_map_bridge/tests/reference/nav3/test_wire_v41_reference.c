#include "nav_exec_wire.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static FILE *trace;static unsigned cases,failed;static const char *outdir;
static void record(const char *name,int expect,int got,const uint8_t *wire,size_t n){
 size_t i;int ok=expect==got;++cases;if(!ok)++failed;
 fprintf(trace,"{\"case\":\"%s\",\"expected\":%d,\"actual\":%d,\"pass\":%s,\"input_hex\":\"",name,expect,got,ok?"true":"false");
 for(i=0;wire&&i<n;++i)fprintf(trace,"%02x",wire[i]);
 fprintf(trace,"\"}\n");
 if(!ok&&wire){char p[1024];FILE*f;snprintf(p,sizeof p,"%s/failure-%u.bin",outdir,cases);f=fopen(p,"wb");if(f){fwrite(wire,1,n,f);fclose(f);}}
}
static void fix(uint8_t *w){ne_put16(w+256,ne_crc16(w,256));}
static struct ne_header head(unsigned family,unsigned op,unsigned n){
 struct ne_header h;unsigned i;memset(&h,0,sizeof h);h.family=(uint8_t)family;h.opcode=(uint8_t)op;h.payload_bytes=(uint16_t)n;
 h.transport_seq=UINT64_C(0x0102030405060708);h.session_id=UINT64_C(0x1112131415161718);
 h.run_generation=UINT64_C(0x2122232425262728);for(i=0;i<16;++i)h.goal_id[i]=(uint8_t)(0x30+i);
 h.map_epoch=UINT64_C(0x4142434445464748);h.map_revision=UINT64_C(0x5152535455565758);
 h.localization_epoch=UINT64_C(0x6162636465666768);h.plan_id=UINT64_C(0x7172737475767778);h.request_id=UINT64_C(0x8182838485868788);return h;
}
static void roundtrip(const char *name,struct ne_header h,uint8_t *p,unsigned index){
 uint8_t w[258],again[258];struct ne_message m;int e;char path[1024];FILE*f;
 memset(w,0xa5,sizeof w);e=ne_encode(&h,p,w);record(name,0,e,w,sizeof w);if(e)return;
 e=ne_decode(w,sizeof w,&m);record("decode_valid",0,e,w,sizeof w);if(e)return;
 e=ne_encode(&m.h,m.payload,again);record("roundtrip_byte_identical",1,e==0&&!memcmp(w,again,258),w,sizeof w);
 snprintf(path,sizeof path,"%s/golden-%02u.bin",outdir,index);f=fopen(path,"wb");if(!f)exit(2);fwrite(w,1,258,f);fclose(f);
}
static void reject(const char *name,uint8_t *w,size_t n,int expect){
 struct ne_message out,before;int e;memset(&out,0xa5,sizeof out);memcpy(&before,&out,sizeof out);
 e=ne_decode(w,n,&out);record(name,expect,e,w,n<=258?n:258);
 record("reject_does_not_mutate_view",1,e!=0&&!memcmp(&out,&before,sizeof out),w,n<=258?n:258);
}
int main(int argc,char **argv){
 uint8_t p[160],w[258],base[258],b[258];unsigned i,j,index=0;struct ne_header h;struct ne_message m;char name[128],path[1024];FILE*f;
 if(argc!=2)return 2;
 outdir=argv[1];snprintf(path,sizeof path,"%s/cases.jsonl",outdir);trace=fopen(path,"w");if(!trace)return 2;
 record("crc16_ccitt_false_known_vector",0x29b1,ne_crc16((const uint8_t*)"123456789",9),0,0);
 ne_put16(p,0xabcd);ne_put32(p+2,UINT32_C(0x12345678));ne_put64(p+6,UINT64_C(0xfedcba9876543210));
 record("big_endian_helpers",1,ne_get16(p)==0xabcd&&ne_get32(p+2)==UINT32_C(0x12345678)&&ne_get64(p+6)==UINT64_C(0xfedcba9876543210),p,14);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_INFO,0);roundtrip("info",h,p,index++);
 h=head(NE_NAV3,NE_SESSION_BIND,16);h.session_id=0;ne_put64(p,3);ne_put64(p+8,4);roundtrip("session_bind",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_CLOCK_ANCHOR,24);ne_put64(p,8);ne_put32(p+8,9);ne_put32(p+12,100);roundtrip("clock_anchor",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_FRAME_BIND,68);ne_put32(p,1);p[4]=3;memcpy(p+5,"map",3);roundtrip("frame_bind",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_MAP_BEGIN,32);ne_put32(p,1);ne_put32(p+4,(uint32_t)-3200);ne_put32(p+8,(uint32_t)-3200);ne_put16(p+12,100);p[14]=64;p[15]=64;ne_put32(p+16,0x12345678);ne_put64(p+20,2);roundtrip("map_begin",h,p,index++);
 for(j=0;j<16;++j){memset(p,0,sizeof p);h=head(NE_NAV3,NE_MAP_CHUNK,48);ne_put64(p,2);ne_put16(p+8,(uint16_t)(j*32));ne_put16(p+10,32);for(i=0;i<32;++i)p[16+i]=(uint8_t)(i+j);snprintf(name,sizeof name,"chunk_%u",j);roundtrip(name,h,p,index++);}
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_MAP_COMMIT,16);ne_put64(p,2);ne_put32(p+8,0x12345678);roundtrip("map_commit",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_GOAL_PLAN,24);ne_put32(p,200);ne_put32(p+4,(uint32_t)-300);ne_put32(p+8,1);ne_put64(p+12,0x99);roundtrip("goal_plan",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_START,24);ne_put64(p,5);ne_put64(p+8,6);ne_put64(p+16,7);roundtrip("start",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_CANCEL,8);ne_put32(p,9);roundtrip("cancel",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_POSE,60);ne_put32(p,42);ne_put32(p+4,(uint32_t)-42);ne_put32(p+8,(uint32_t)-3141593);ne_put32(p+12,1);ne_put64(p+16,2);ne_put64(p+24,1000);ne_put32(p+32,1);ne_put64(p+36,250000000);ne_put32(p+44,1234);ne_put64(p+48,3);ne_put32(p+56,3);roundtrip("pose_250ms_boundary",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_PATH_PAGE,8);ne_put16(p,4095);ne_put16(p+2,64);ne_put32(p+4,7);roundtrip("path_page",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_STATE,160);p[0]=5;p[1]=3;roundtrip("state",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_EVENT,64);roundtrip("event",h,p,index++);h.opcode=NE_PLAN_RESULT;roundtrip("plan_result",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NVA1,NE_PROBE,0);roundtrip("permit_probe",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NVA1,NE_PERMIT,64);ne_put32(p,1);ne_put64(p+4,2);ne_put64(p+12,3);ne_put64(p+20,4);ne_put64(p+28,5);ne_put32(p+36,500);ne_put32(p+40,30000);ne_put64(p+44,6);roundtrip("permit",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NVA1,NE_REVOKE,16);ne_put64(p,2);ne_put32(p+8,9);roundtrip("revoke",h,p,index++);
 for(i=0;i<=NE_INTERNAL;++i){memset(p,0,sizeof p);h=head(NE_NAV3,NE_START|0x80,32);ne_put16(p,(uint16_t)i);snprintf(name,sizeof name,"ack_status_%u",i);roundtrip(name,h,p,index++);}
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_INFO|0x80,64);roundtrip("info_ack",h,p,index++);
 for(j=0;j<=64;++j){memset(p,0,sizeof p);h=head(NE_NAV3,NE_PATH_PAGE|0x80,32+2*j);ne_put16(p+26,(uint16_t)j);for(i=0;i<j;++i)ne_put16(p+32+2*i,(uint16_t)(4095-i));snprintf(name,sizeof name,"page_reply_%u",j);roundtrip(name,h,p,index++);}
 memset(p,0,sizeof p);h=head(NE_NVA1,NE_PERMIT|0x80,32);roundtrip("permit_ack",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_INFO,0);if(ne_encode(&h,p,base))return 2;
 for(i=0;i<=300;++i)if(i!=258){snprintf(name,sizeof name,"length_%u",i);reject(name,base,i,NE_E_LENGTH);}
 memcpy(w,base,258);w[0]='X';reject("bad_magic",w,258,NE_E_MAGIC);
 for(i=0;i<256;++i)if(i!=1){memcpy(w,base,258);w[4]=(uint8_t)i;snprintf(name,sizeof name,"version_%u",i);reject(name,w,258,NE_E_VERSION);}
 for(i=0;i<258;++i){memcpy(w,base,258);w[i]^=1;snprintf(name,sizeof name,"crc_bit_corruption_%u",i);if(i<4)reject(name,w,258,NE_E_MAGIC);else if(i==4)reject(name,w,258,NE_E_VERSION);else reject(name,w,258,NE_E_CRC);}
 for(i=90;i<96;++i){memcpy(w,base,258);w[i]=1;fix(w);reject("header_reserved",w,258,NE_E_RESERVED);}
 for(i=96;i<256;++i){memcpy(w,base,258);w[i]=1;fix(w);reject("unused_payload",w,258,NE_E_RESERVED);}
 memcpy(w,base,258);ne_put16(w+6,1);fix(w);reject("unknown_flag",w,258,NE_E_FLAGS);
 memcpy(w,base,258);ne_put16(w+88,161);fix(w);reject("payload_overflow",w,258,NE_E_LENGTH);
 for(i=0;i<256;++i){memcpy(w,base,258);w[5]=(uint8_t)i;fix(w);if(i!=NE_INFO){struct ne_message v;int e=ne_decode(w,258,&v);record("other_op_not_info",1,e!=0,w,258);}}
 memcpy(w,base,258);w[0]='N';w[1]='V';w[2]='A';w[3]='1';w[5]=NE_STATE;fix(w);reject("nva1_cannot_accept_nav3_state",w,258,NE_E_OPCODE);
 memcpy(w,base,258);w[5]=NE_STATE|0x80;fix(w);reject("state_cannot_masquerade_as_ack",w,258,NE_E_OPCODE);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_FRAME_BIND,68);ne_put32(p,1);p[4]=63;memset(p+5,'m',63);roundtrip("frame_63",h,p,index++);
 p[4]=64;memset(b,0xa5,258);memcpy(w,b,258);record("frame_64_rejected",NE_E_RANGE,ne_encode(&h,p,w),0,0);record("encode_failure_unchanged",1,!memcmp(w,b,258),w,258);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_POSE,60);ne_put32(p+12,1);ne_put64(p+16,2);ne_put64(p+48,3);
 for(i=249;i<=251;++i){ne_put64(p+36,(uint64_t)i*1000000);snprintf(name,sizeof name,"pose_age_%u",i);record(name,i<=250?0:NE_E_RANGE,ne_encode(&h,p,w),0,0);}
 ne_put64(p+36,UINT64_MAX);record("pose_age_overflow_rejected",NE_E_RANGE,ne_encode(&h,p,w),0,0);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_INFO,0);ne_encode(&h,p,w);ne_decode(w,258,&m);record("info_not_execution",0,ne_is_execution_command(&m),w,258);
 h.opcode=NE_GOAL_PLAN;h.payload_bytes=24;ne_put32(p+8,1);ne_put64(p+12,2);ne_encode(&h,p,w);ne_decode(w,258,&m);record("plan_not_execution",0,ne_is_execution_command(&m),w,258);
 h.opcode=NE_START;ne_put64(p,1);ne_put64(p+8,2);ne_put64(p+16,3);ne_encode(&h,p,w);ne_decode(w,258,&m);record("start_is_execution",1,ne_is_execution_command(&m),w,258);
 record("null_decode",NE_E_POINTER,ne_decode(0,258,&m),0,0);record("null_output",NE_E_POINTER,ne_decode(w,258,0),0,0);record("null_encode",NE_E_POINTER,ne_encode(0,p,w),0,0);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_STATE,160);h.request_id=0;roundtrip("unsolicited_state_without_request",h,p,index++);
 ne_put32(p+4,256);record("state_unknown_flags",NE_E_RESERVED,ne_encode(&h,p,w),0,0);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_PLAN_RESULT,64);ne_put16(p+46,2);record("plan_result_unknown_flags",NE_E_RESERVED,ne_encode(&h,p,w),0,0);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_PATH_PAGE|0x80,34);ne_put16(p,NE_NO_ROUTE);ne_put16(p+26,1);record("error_ack_no_cached_cells",NE_E_SHAPE,ne_encode(&h,p,w),0,0);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_PATH_PAGE|0x80,34);ne_put16(p+24,65535);ne_put16(p+26,1);{int e=ne_encode(&h,p,w);record("page_reply_offset65535_rejected",NE_E_RANGE,e,w,258);}
 ne_put16(p+24,4096);{int e=ne_encode(&h,p,w);record("page_reply_offset4096_nonempty_rejected",NE_E_RANGE,e,w,258);}
 h.payload_bytes=36;ne_put16(p+24,4095);ne_put16(p+26,2);{int e=ne_encode(&h,p,w);record("page_reply_tail_overrun_rejected",NE_E_RANGE,e,w,258);}
 h.payload_bytes=34;ne_put16(p+24,4095);ne_put16(p+26,1);roundtrip("page_reply_last_cell_4095",h,p,index++);
 memset(p,0,sizeof p);h=head(NE_NAV3,NE_START,24);ne_put64(p,1);ne_put64(p+8,2);ne_put64(p+16,3);ne_encode(&h,p,w);
 for(j=0;j<3;++j){unsigned off=j==0?96:j==1?90:100;memset(b,0xa5,258);memcpy(b+off,p,24);memcpy(base,b,258);{int e=ne_encode(&h,b+off,b);snprintf(name,sizeof name,"inplace_payload_overlap_%u",off);record(name,1,e==0&&!memcmp(w,b,258),base,258);}}
 memset(p,0,sizeof p);h=head(NE_NVA1,NE_REVOKE,16);ne_put64(p,1);ne_encode(&h,p,w);ne_decode(w,258,&m);record("revoke_is_execution_mutation",1,ne_is_execution_command(&m),w,258);
 fclose(trace);snprintf(path,sizeof path,"%s/summary.json",outdir);f=fopen(path,"w");if(!f)return 2;fprintf(f,"{\"layer\":\"L1\",\"cases\":%u,\"failed\":%u,\"golden_frames\":%u,\"pass\":%s}\n",cases,failed,index,failed?"false":"true");fclose(f);
 printf("L1 cases=%u failed=%u golden=%u\n",cases,failed,index);return failed?1:0;
}
