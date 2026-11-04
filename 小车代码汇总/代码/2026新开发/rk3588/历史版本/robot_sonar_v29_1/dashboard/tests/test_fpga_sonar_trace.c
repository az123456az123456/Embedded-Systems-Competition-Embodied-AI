/* Replay actual FPGA SNS1 snapshots. A 64-byte payload trace is wrapped in the
   unchanged Ethernet/CRC16 envelope; an 80/84-byte frame trace is used verbatim.
   This checks cross-end payload/state/display compatibility, not PHY timing. */
#include "sensor_data.h"
#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,packets,over,unknown;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"FPGA trace assertion %u failed at line %u\n",checks,packets);exit(1);}}while(0)
static int digit(int c){if(c>='0'&&c<='9')return c-'0';c=tolower(c);if(c>='a'&&c<='f')return c-'a'+10;return -1;}
int main(int argc,char**argv){
    if(argc<2||argc>3||(argc==3&&strcmp(argv[2],"--cases"))){fprintf(stderr,"Usage: %s FPGA_SNS_HEX_TRACE [--cases]\n",argv[0]);return 2;}
    FILE*f=fopen(argv[1],"r");if(!f)return 2;
    struct sensor_state state;sensor_init(&state);char line[600],text[32];
    struct sensor_sample final_case[10];unsigned seen_case[10]={0};
    while(fgets(line,sizeof(line),f)){
        char*p=line;while(isspace((unsigned char)*p))++p;if(!*p||*p=='#')continue;
        unsigned label=10;
        if(argc==3){char*end;label=(unsigned)strtoul(p,&end,10);CHECK(*end=='\t'&&label<10);p=end+1;}
        size_t n=strcspn(p,"\r\n \t");CHECK(n==128||n==160||n==168);
        unsigned char b[84]={0};const size_t offset=n==128?14:0;
        for(size_t i=0;i<n/2;++i){int high=digit(p[i*2]),low=digit(p[i*2+1]);CHECK(high>=0&&low>=0);b[offset+i]=(unsigned char)((high<<4)|low);}
        size_t size=n/2;
        if(n==128){
            const unsigned char header[]={255,255,255,255,255,255,2,0xac,0x13,0x80,0,1,0x88,0xb8};
            memcpy(b,header,sizeof(header));unsigned crc=sensor_crc16(b+14,64);b[78]=crc>>8;b[79]=crc;size=80;
        }
        struct sensor_sample s;CHECK(sensor_decode(b,size,&s));++packets;
        if(label<10){final_case[label]=s;seen_case[label]=1;}
        uint64_t now=UINT64_C(1000000)+s.uptime_ms;CHECK(sensor_update(&state,&s,now));
        for(unsigned ch=0;ch<3;++ch){
            double metres=999;enum sensor_distance_status status=sensor_distance_text(&state,ch,now,text,sizeof(text));
            CHECK(sensor_distance(&state,ch,now,&metres)==status);
            if(status==SENSOR_VALID){
                CHECK(metres==s.distance_mm[ch]/1000.);
                if(s.distance_mm[ch]>4000){++over;CHECK(!strcmp(text,">4 m"));}
                else CHECK(strcmp(text,">4 m")!=0);
            }else{++unknown;CHECK(metres==0&&!strcmp(text,"-- m"));}
        }
        // A corrupted Ethernet payload must fail before changing sensor state.
        b[48]^=1;struct sensor_sample rejected;CHECK(!sensor_decode(b,size,&rejected));
        CHECK(state.received_ms==now&&state.sample.sequence==s.sequence);
    }
    CHECK(!ferror(f));fclose(f);CHECK(packets>0&&over>0&&unknown>0);
    if(argc==3){
        for(unsigned i=0;i<10;++i)CHECK(seen_case[i]);
        // Case IDs are emitted by the production UART-to-SNS FPGA testbench.
        // 1/2/3: raw 4501/6000/65534 pairs. 4: zero. 5: 4001. 6: FFFF.
        // 7: near250. 8: bad-checksum6000. 9: disconnected until timeout.
        for(unsigned i=1;i<=3;++i)CHECK((final_case[i].valid_mask&1)&&!(final_case[i].fault_mask&1)&&final_case[i].distance_mm[0]==4500);
        CHECK(!(final_case[4].valid_mask&1)&&(final_case[4].fault_mask&1));
        CHECK((final_case[5].valid_mask&1)&&final_case[5].distance_mm[0]==4001);
        CHECK(!(final_case[6].valid_mask&1)&&(final_case[6].fault_mask&1));
        CHECK((final_case[7].valid_mask&1)&&final_case[7].distance_mm[0]==250);
        CHECK(final_case[8].distance_mm[0]==250&&final_case[8].age_ms[0]>final_case[7].age_ms[0]);
        CHECK(final_case[9].valid_mask==0&&final_case[9].fault_mask==7);
    }
    printf("PASS actual FPGA SNS trace: %u checks, %u snapshots, %u valid >4m channel reports, %u unknown/fault channel reports\n",checks,packets,over,unknown);
    return 0;
}
