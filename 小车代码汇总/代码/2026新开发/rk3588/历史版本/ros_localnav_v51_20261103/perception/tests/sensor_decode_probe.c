#include "sensor_data.h"
#include <stdio.h>
int main(int argc,char **argv){
    if(argc!=2)return 2;
    FILE *f=fopen(argv[1],"rb");if(!f)return 2;
    unsigned char data[2048];size_t n=fread(data,1,sizeof(data),f);fclose(f);struct sensor_sample s;
    if(!sensor_decode(data,n,&s)){fprintf(stderr,"invalid SNS1 frame\n");return 1;}
    printf("{\"sequence\":%u,\"uptime_ms\":%u,\"valid_mask\":%u,\"fault_mask\":%u,\"invert_mask\":%u,\"illegal_mask\":%u,\"window_ms\":%u,\"position\":[%d,%d,%d,%d],\"delta\":[%d,%d,%d,%d],\"distance_mm\":[%u,%u,%u],\"age_ms\":[%u,%u,%u]}\n",s.sequence,s.uptime_ms,s.valid_mask,s.fault_mask,s.invert_mask,s.illegal_mask,s.window_ms,s.position[0],s.position[1],s.position[2],s.position[3],s.delta[0],s.delta[1],s.delta[2],s.delta[3],s.distance_mm[0],s.distance_mm[1],s.distance_mm[2],s.age_ms[0],s.age_ms[1],s.age_ms[2]);
    return 0;
}
