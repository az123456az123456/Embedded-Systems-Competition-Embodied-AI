#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stop;
static void on_signal(int signo) { (void)signo; stop=1; }
static uint16_t u16(const unsigned char *p) { return (uint16_t)((p[0]<<8)|p[1]); }
static uint32_t u32(const unsigned char *p) {
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}
static int s16(const unsigned char *p) { unsigned n=u16(p); return n<32768 ? (int)n : (int)n-65536; }
static uint64_t now_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC,&t);
    return (uint64_t)t.tv_sec*1000+(uint64_t)t.tv_nsec/1000000;
}
static uint32_t crc32(const unsigned char *p, size_t n) {
    uint32_t c=UINT32_MAX;
    while (n--) {
        c^=*p++;
        for (int i=0;i<8;i++) c=(c>>1)^((c&1)?0xedb88320u:0);
    }
    return ~c;
}
// Linux normally strips the 4-byte FCS. An offline capture may retain it.
static int decode(const unsigned char *frame, size_t n, unsigned char out[64]) {
    static const unsigned char header[14]={255,255,255,255,255,255,2,0xac,0x13,0x80,0,1,0x88,0xb5};
    if (n!=78 && n!=82) return 0;
    if (memcmp(frame,header,14) || memcmp(frame+14,"IMU9",4)) return 0;
    const unsigned char *p=frame+14;
    if (p[4]!=1 || p[5]!=0x92 || (u16(p+6)&~63u)) return 0;
    if ((p[41]!=0x68 && p[41]!=0x69) || u16(p+58)!=100) return 0;
    if (n==82) {
        uint32_t f=(uint32_t)frame[78]|((uint32_t)frame[79]<<8)|
                   ((uint32_t)frame[80]<<16)|((uint32_t)frame[81]<<24);
        if (crc32(frame,78)!=f) return 0;
    }
    memcpy(out,p,64);
    return 1;
}
static void show(const unsigned char *p, uint64_t elapsed, int clear) {
    unsigned flags=u16(p+6);
    int ag=(flags&1) && p[39]==0x71 && u16(p+56)+elapsed<500;
    int mag=(flags&2) && p[40]==0x48 && !(flags&32) && u16(p+42)+elapsed<500;
    if (clear) printf("\033[H\033[2J");
    printf("MPU9250 / AK8963 | FPGA -> RGMII -> RK3588\n");
    printf("Link: %s | packet age: %llu ms | FPGA uptime: %u ms\n",
           elapsed<2000?"RECEIVING":"TIMEOUT",(unsigned long long)elapsed,u32(p+52));
    printf("I2C: 0x%02X | WHO_AM_I: 0x%02X | AK8963 WIA: 0x%02X\n",p[41],p[39],p[40]);
    printf("AG: %s, age %u ms, seq %u | MAG: %s, age %u ms, seq %u\n",
           ag?"VALID":"STALE/UNAVAILABLE",u16(p+56),u32(p+8),
           mag?"VALID":"STALE/UNAVAILABLE",u16(p+42),u32(p+60));
    if (ag) {
        printf("Accel [g]     X=%+10.5f Y=%+10.5f Z=%+10.5f\n",
               s16(p+16)/16384.0,s16(p+18)/16384.0,s16(p+20)/16384.0);
        printf("Gyro  [deg/s] X=%+10.5f Y=%+10.5f Z=%+10.5f\n",
               s16(p+24)/131.0,s16(p+26)/131.0,s16(p+28)/131.0);
        printf("Chip temperature [C] = %.2f\n",s16(p+22)/333.87+21.0);
    } else printf("Accel / gyro / temperature: waiting for fresh data\n");
    if (mag) {
        double m[3];
        for (int i=0;i<3;i++) m[i]=s16(p+30+2*i)*0.15*(1.0+((int)p[36+i]-128)/256.0);
        printf("Mag   [uT]    X=%+10.5f Y=%+10.5f Z=%+10.5f\n",m[0],m[1],m[2]);
    } else printf("Magnetometer: waiting for fresh data (overflow=%u)\n",!!(flags&32));
    printf("ASA: %u/%u/%u | I2C errors: %u | FPGA snapshot drops: %u | fault: %u\n",
           p[36],p[37],p[38],u32(p+44),u32(p+48),!!(flags&16));
    printf("Native sensor axes; no attitude fusion or hard/soft-iron calibration.\n");
    fflush(stdout);
}
static int discover(char name[IFNAMSIZ]) {
    DIR *d=opendir("/sys/class/net");
    if (!d) return 0;
    struct dirent *entry;
    int found=0;
    while ((entry=readdir(d))) {
        char path[512], resolved[4096];
        if (entry->d_name[0]=='.') continue;
        snprintf(path,sizeof(path),"/sys/class/net/%s/device",entry->d_name);
        if (!realpath(path,resolved)) continue;
        const char *base=strrchr(resolved,'/');
        if (base && strcmp(base+1,"fe1c0000.ethernet")==0 && strlen(entry->d_name)<IFNAMSIZ) {
            strcpy(name,entry->d_name); found++; 
        }
    }
    closedir(d);
    return found==1;
}
int main(int argc, char **argv) {
    char iface[IFNAMSIZ]={0};
    const char *offline=NULL;
    int once=0, clear=isatty(STDOUT_FILENO);
    for (int i=1;i<argc;i++) {
        if (!strcmp(argv[i],"--interface") && i+1<argc && strlen(argv[i+1])<IFNAMSIZ)
            strcpy(iface,argv[++i]);
        else if (!strcmp(argv[i],"--decode") && i+1<argc) offline=argv[++i];
        else if (!strcmp(argv[i],"--once")) once=1;
        else if (!strcmp(argv[i],"--no-clear")) clear=0;
        else {
            fprintf(stderr,"Usage: %s [--interface NAME] [--once] [--no-clear]\n       %s --decode FRAME.bin\n",argv[0],argv[0]);
            return !strcmp(argv[i],"--help")?0:2;
        }
    }
    unsigned char frame[2048], latest[64];
    if (offline) {
        FILE *f=fopen(offline,"rb");
        if (!f) { perror(offline); return 1; }
        size_t n=fread(frame,1,sizeof(frame),f);
        int io_error=ferror(f); fclose(f);
        if (io_error || !decode(frame,n,latest)) { fprintf(stderr,"Invalid IMU Ethernet frame\n"); return 1; }
        show(latest,0,0); return 0;
    }
    if (!*iface && !discover(iface)) {
        fprintf(stderr,"Cannot find GMAC1 (fe1c0000.ethernet). Check RGMII device tree or use --interface NAME.\n"); return 1;
    }
    unsigned index=if_nametoindex(iface);
    if (!index) { perror("if_nametoindex"); return 1; }
    int fd=socket(AF_PACKET,SOCK_RAW,htons(0x88b5));
    if (fd<0) { perror("socket (run as root or grant CAP_NET_RAW)"); return 1; }
    struct sockaddr_ll bind_addr={0};
    bind_addr.sll_family=AF_PACKET; bind_addr.sll_protocol=htons(0x88b5); bind_addr.sll_ifindex=(int)index;
    if (bind(fd,(struct sockaddr *)&bind_addr,sizeof(bind_addr))<0) { perror("bind"); close(fd); return 1; }
    signal(SIGINT,on_signal); signal(SIGTERM,on_signal);
    fprintf(stderr,"Listening on %s, EtherType 0x88b5. Ctrl+C to exit.\n",iface);
    uint64_t start=now_ms(), received=0, displayed=0;
    int have=0, result=0;
    while (!stop) {
        struct pollfd pfd={.fd=fd,.events=POLLIN};
        int r=poll(&pfd,1,100);
        if (r<0) { if (errno==EINTR) continue; perror("poll"); result=1; break; }
        if (r>0 && (pfd.revents&(POLLERR|POLLHUP|POLLNVAL))) {
            fprintf(stderr,"Network interface error\n"); result=1; break;
        }
        if (r>0 && (pfd.revents&POLLIN)) {
            struct sockaddr_ll from={0}; socklen_t len=sizeof(from);
            ssize_t n=recvfrom(fd,frame,sizeof(frame),0,(struct sockaddr *)&from,&len);
            if (n<0 && errno!=EINTR) { perror("recvfrom"); result=1; break; }
            if (n>0 && from.sll_pkttype!=PACKET_OUTGOING && decode(frame,(size_t)n,latest)) {
                received=now_ms(); have=1;
                if (once) { show(latest,0,0); break; }
            }
        }
        uint64_t now=now_ms();
        if (now-displayed>=(clear?100u:1000u)) {
            if (have) show(latest,now-received,clear);
            else fprintf(stderr,"Waiting for IMU9 frames (%llu s); check FPGA image, GMAC1 clock and interface UP.\n",
                         (unsigned long long)((now-start)/1000));
            displayed=now;
        }
        if (once && now-start>=10000) { fprintf(stderr,"No IMU frame within 10 s\n"); result=1; break; }
    }
    close(fd);
    return result;
}
