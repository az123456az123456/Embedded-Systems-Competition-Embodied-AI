#ifndef ACG138_WRONGLOG_STATE_H
#define ACG138_WRONGLOG_STATE_H
#include "../../common/wrong_log.h"
#include "sensor_data.h"
#include "imu_data.h"
#include "vision.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* Data is explicitly selected. No paths, environment, credentials or images. */
static inline double wronglog_number(double value){return isfinite(value)?value:0.;}
static inline void wronglog_status_event(const char *component,const char *reason,const char *status){
 char json[1200];size_t used=0;const char prefix[]="{\"status\":\"";memcpy(json,prefix,sizeof(prefix)-1);used=sizeof(prefix)-1;
 for(unsigned i=0;status&&status[i]&&i<159;i++){
  unsigned char c=(unsigned char)status[i];
  if(c=='"'||c=='\\'){json[used++]='\\';json[used++]=(char)c;}
  else if(c<32)used+=(size_t)snprintf(json+used,sizeof(json)-used,"\\u%04x",c);
  else json[used++]=(char)c;
 }
 memcpy(json+used,"\"}",3);wrong_log_event(component,reason,json);
}
static inline void wronglog_network_error(const char *component,const char *reason,int error,int fd){
 char json[160];snprintf(json,sizeof(json),"{\"errno\":%d,\"socket_open\":%s,\"category\":\"network_failure\"}",error,fd>=0?"true":"false");
 wrong_log_event(component,reason,json);
}
static inline void wronglog_sensor_snapshot(const struct sensor_state *s,uint64_t now,int fd,int error,uint64_t rx,uint64_t decoded,uint64_t accepted){
 char json[2300];const struct sensor_sample *v=&s->sample;
 long long host_age=s->have_sample&&now>=s->received_ms?(long long)(now-s->received_ms):-1;
 long long age[3];int status[3];
 for(unsigned i=0;i<3;i++){age[i]=host_age>=0&&v->age_ms[i]!=UINT16_MAX?host_age+v->age_ms[i]:-1;status[i]=(int)sensor_distance(s,i,now,NULL);}
 snprintf(json,sizeof(json),"{\"sample_mono_ms\":%llu,\"socket_open\":%s,\"errno\":%d,\"rx\":%llu,\"decoded\":%llu,\"accepted\":%llu,\"have_sample\":%s,\"fresh\":%s,\"sequence\":%u,\"uptime_ms\":%u,\"received_ms\":%llu,\"host_age_ms\":%lld,\"valid_mask\":%u,\"fault_mask\":%u,\"retained_distance_mm\":[%u,%u,%u],\"source_age_ms\":[%u,%u,%u],\"total_range_age_ms\":[%lld,%lld,%lld],\"range_status\":[%d,%d,%d],\"status_legend\":\"0=waiting,1=valid,2=stale,3=fault\",\"counts\":[%d,%d,%d,%d],\"delta\":[%d,%d,%d,%d],\"window_ms\":%u,\"invert_mask\":%u,\"illegal_mask\":%u,\"restarts\":%u,\"raw_uart_codes_available\":false}",
 (unsigned long long)now,fd>=0?"true":"false",error,(unsigned long long)rx,(unsigned long long)decoded,(unsigned long long)accepted,s->have_sample?"true":"false",sensor_link_fresh(s,now)?"true":"false",v->sequence,v->uptime_ms,(unsigned long long)s->received_ms,host_age,v->valid_mask,v->fault_mask,v->distance_mm[0],v->distance_mm[1],v->distance_mm[2],v->age_ms[0],v->age_ms[1],v->age_ms[2],age[0],age[1],age[2],status[0],status[1],status[2],v->position[0],v->position[1],v->position[2],v->position[3],v->delta[0],v->delta[1],v->delta[2],v->delta[3],v->window_ms,v->invert_mask,v->illegal_mask,s->restarts);
 wrong_log_snapshot("sensors",json);
}
static inline void wronglog_imu_snapshot(const struct imu_state *s,uint64_t now,int fd,int error,uint64_t rx,uint64_t decoded,uint64_t accepted){
 char json[2200];long long host_age=s->have_packet&&now>=s->received_ms?(long long)(now-s->received_ms):-1;
 snprintf(json,sizeof(json),"{\"sample_mono_ms\":%llu,\"socket_open\":%s,\"errno\":%d,\"rx\":%llu,\"decoded\":%llu,\"accepted\":%llu,\"sequence\":%u,\"source_timestamp_ms\":%u,\"uptime_ms\":%u,\"received_ms\":%llu,\"source_age_ms\":%u,\"host_age_ms\":%lld,\"total_age_ms\":%lld,\"flags\":%u,\"fresh\":%s,\"initialized\":%s,\"calibrating\":%s,\"calibration_status\":%d,\"calibration_percent\":%u,\"bias_valid\":%s,\"bias_dps\":[%.6f,%.6f,%.6f],\"gyro_dps\":[%.6f,%.6f,%.6f],\"rpy_deg\":[%.6f,%.6f,%.6f],\"accel_g\":[%.6f,%.6f,%.6f],\"dt_ms\":%u,\"errors\":%u,\"drops\":%u}",
 (unsigned long long)now,fd>=0?"true":"false",error,(unsigned long long)rx,(unsigned long long)decoded,(unsigned long long)accepted,s->sample.sequence,s->sample.timestamp,s->sample.uptime,(unsigned long long)s->received_ms,s->sample.ag_age,host_age,host_age>=0?host_age+s->sample.ag_age:-1,s->sample.flags,imu_fresh(s,now)?"true":"false",s->initialized?"true":"false",s->calibrating?"true":"false",(int)s->calibration_status,s->calibration/2,s->bias_valid?"true":"false",wronglog_number(s->bias[0]),wronglog_number(s->bias[1]),wronglog_number(s->bias[2]),wronglog_number(s->sample.gyro[0]),wronglog_number(s->sample.gyro[1]),wronglog_number(s->sample.gyro[2]),wronglog_number(s->roll),wronglog_number(s->pitch),wronglog_number(s->yaw),wronglog_number(s->sample.accel[0]),wronglog_number(s->sample.accel[1]),wronglog_number(s->sample.accel[2]),s->sample_interval_ms,s->sample.errors,s->sample.drops);
 wrong_log_snapshot("imu",json);
}
static inline void wronglog_vision_snapshot(const struct vision_snapshot *s,uint64_t now){
 char json[1100];
 snprintf(json,sizeof(json),"{\"sample_mono_ms\":%llu,\"frame_id\":%llu,\"result_frame_id\":%llu,\"capture_timestamp_ms\":%llu,\"detection_timestamp_ms\":%llu,\"result_timestamp_ms\":%llu,\"capture_fps\":%.4f,\"preview_fps\":%.4f,\"result_fps\":%.4f,\"inference_ms\":%.4f,\"frame_age_ms\":%.4f,\"box_age_ms\":%.4f,\"have_frame\":%s,\"detection_enabled\":%s,\"detection_available\":%s,\"objects\":%d,\"npu_workers\":%d,\"completed_pairs\":%llu,\"discarded_pairs\":%llu}",
 (unsigned long long)now,(unsigned long long)s->frame_id,(unsigned long long)s->detection_frame_id,(unsigned long long)s->timestamp_ms,(unsigned long long)s->detection_timestamp_ms,(unsigned long long)s->result_timestamp_ms,wronglog_number(s->capture_fps),wronglog_number(s->preview_fps),wronglog_number(s->fps),wronglog_number(s->inference_ms),wronglog_number(s->frame_age_ms),wronglog_number(s->detection_age_ms),s->have_frame?"true":"false",s->detection_enabled?"true":"false",s->detection_available?"true":"false",s->count,s->npu_workers,(unsigned long long)s->completed_pairs,(unsigned long long)s->discarded_pairs);
 wrong_log_snapshot("vision",json);
}
#endif
