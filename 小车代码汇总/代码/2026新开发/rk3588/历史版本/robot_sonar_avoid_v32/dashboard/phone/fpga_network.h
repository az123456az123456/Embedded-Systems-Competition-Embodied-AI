#ifndef ACG138_FPGA_NETWORK_H
#define ACG138_FPGA_NETWORK_H
/* Interface discovery is independent of either raw socket opening. */
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
static inline int fpga_device_path_matches(const char *path) {
    const char *patterns[]={"fe1c0000.ethernet","ethernet@fe1c0000"};
    for(unsigned i=0;i<2;i++) {
        const char *p=strstr(path,patterns[i]);size_t n=strlen(patterns[i]);
        if(p && (p==path||p[-1]=='/') && (!p[n]||p[n]=='/'))return 1;
    }
    return 0;
}
static inline int fpga_resolve_interface(const char *requested,char out[16],char *source,size_t source_size) {
    out[0]=0;
    if(requested&&*requested) {
        if(strlen(requested)>=IFNAMSIZ){errno=EINVAL;return -1;}
        if(!if_nametoindex(requested)){errno=ENODEV;return -1;}
        snprintf(out,16,"%s",requested);snprintf(source,source_size,"explicit");return 0;
    }
    DIR *d=opendir("/sys/class/net");
    if(d) {
        struct dirent *e;
        while((e=readdir(d))) {
            if(strlen(e->d_name)>=16||!if_nametoindex(e->d_name))continue;
            char path[512],target[4096];
            snprintf(path,sizeof(path),"/sys/class/net/%s/device",e->d_name);
            if(realpath(path,target)&&fpga_device_path_matches(target)) {
                if(out[0]&&strcmp(out,e->d_name)){closedir(d);out[0]=0;errno=EEXIST;return -1;}
                snprintf(out,16,"%s",e->d_name);
            }
        }
        closedir(d);
    }
    if(out[0]){snprintf(source,source_size,"gmac1-fe1c0000");return 0;}
    /* Preserve the established internal fixed-IP setup if its sysfs device
       alias differs. Never silently substitute the Wi-Fi/default-route device. */
    struct ifaddrs *addresses=NULL;
    if(getifaddrs(&addresses)==0) {
        for(struct ifaddrs *a=addresses;a;a=a->ifa_next) {
            if(!a->ifa_addr||a->ifa_addr->sa_family!=AF_INET||strlen(a->ifa_name)>=16)continue;
            const struct sockaddr_in *ip=(const struct sockaddr_in *)a->ifa_addr;
            if(ntohl(ip->sin_addr.s_addr)==0xc0a80003u) {
                if(out[0]&&strcmp(out,a->ifa_name)){freeifaddrs(addresses);out[0]=0;errno=EEXIST;return -1;}
                snprintf(out,16,"%s",a->ifa_name);
            }
        }
        freeifaddrs(addresses);
    }
    if(out[0]){snprintf(source,source_size,"internal-ip-192.168.0.3");return 0;}
    snprintf(source,source_size,"not-found: use --interface eth1 (verify fe1c0000 first)");errno=ENODEV;return -1;
}
#endif
