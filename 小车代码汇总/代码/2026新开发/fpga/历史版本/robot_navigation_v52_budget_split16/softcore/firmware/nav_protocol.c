#include "nav_protocol.h"
/* NAV3 shared map storage only; no legacy NAV2 codec or dispatcher. */
uint32_t nav_crc32(const uint8_t *p,unsigned len) {
 /* Same reflected IEEE polynomial/init/final XOR, four bits per lookup.
  * The512-byte atomic map check stays bounded without a per-bit branch. */
 static const uint32_t table[16]={0,0x1db71064,0x3b6e20c8,0x26d930ac,
  0x76dc4190,0x6b6b51f4,0x4db26158,0x5005713c,0xedb88320,0xf00f9344,
  0xd6d6a3e8,0xcb61b38c,0x9b64c2b0,0x86d3d2d4,0xa00ae278,0xbdbdf21c};
 uint32_t c=UINT32_MAX;unsigned i;
 for(i=0;i<len;++i){c^=p[i];c=(c>>4)^table[c&15u];c=(c>>4)^table[c&15u];}
 return ~c;
}
void nav_server_init(struct nav_server *s) {
 unsigned i;uint8_t *p=(uint8_t*)s;for(i=0;i<sizeof(*s);++i)p[i]=0;
}
