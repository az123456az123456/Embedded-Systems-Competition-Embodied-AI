#ifndef CAMERA_RAW_ROWS_H
#define CAMERA_RAW_ROWS_H
#include <stddef.h>
#include <string.h>
/* One RAW8 row per UDP datagram. No frame ID is present on this legacy wire
 * format: only contiguous rows starting at zero form an accepted frame. */
struct raw_rows { unsigned next; int collecting, swap_pairs; };
static int raw_rows_push(struct raw_rows *s, unsigned char *dst,
                         unsigned stride, unsigned width, unsigned height,
                         const unsigned char *packet, size_t n) {
    if(!width || !height || stride<width || (s->swap_pairs && (width&1)) || n!=width+2u) {
        s->collecting=0;return 0;
    }
    unsigned row=packet[0]|((unsigned)packet[1]<<8);
    if(!row){s->next=0;s->collecting=1;}
    if(!s->collecting)return 0;
    if(row!=s->next || row>=height){s->collecting=0;return 0;}
    unsigned char *out=dst+(size_t)stride*row;
    if(s->swap_pairs)for(unsigned x=0;x<width;x+=2){out[x]=packet[x+3];out[x+1]=packet[x+2];}
    else memcpy(out,packet+2,width);
    if(++s->next==height){s->collecting=0;return 1;}
    return 0;
}
#endif
