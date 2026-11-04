#define _POSIX_C_SOURCE 200809L
#include "jpeg_codec.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
struct output { unsigned char *bytes; size_t size, capacity; };
static void collect(void *context,void *bytes,int length) {
    struct output *out=context;
    assert(length>0 && out->size+(size_t)length<=out->capacity);
    memcpy(out->bytes+out->size,bytes,(size_t)length);out->size+=(size_t)length;
}
static double milliseconds(void) {
    struct timespec t;assert(!clock_gettime(CLOCK_MONOTONIC,&t));
    return t.tv_sec*1000.+t.tv_nsec/1000000.;
}
int main(int argc,char **argv) {
    if(argc!=2)return 2;
    const int width=864,height=486,iterations=24;
    uint32_t *pixels=malloc((size_t)width*height*4);assert(pixels);
    struct output out={malloc((size_t)width*height*4),0,(size_t)width*height*4};assert(out.bytes);
    for(int y=0;y<height;y++)for(int x=0;x<width;x++) {
        /* Natural-image-like gradients/texture, with solid colour checks. */
        unsigned noise=(unsigned)(x*1103515245u+y*12345u)>>26;
        unsigned r=(x*160/width+noise),g=(y*170/height+noise),b=((x+y)*130/(width+height)+noise);
        pixels[y*width+x]=0xff000000u|(r<<16)|(g<<8)|b;
        if(y<120)pixels[y*width+x]=x<width/2?0xffe02030u:0xff2040e0u;
    }
    assert(!jpeg_from_argb(NULL,width,height,72,collect,&out));
    assert(!jpeg_from_argb(pixels,0,height,72,collect,&out));
    assert(!jpeg_from_argb(pixels,width,height,0,collect,&out));
    double start=milliseconds();
    for(int i=0;i<iterations;i++) {
        out.size=0;assert(jpeg_from_argb(pixels,width,height,72,collect,&out));
        assert(out.size>10 && out.bytes[0]==255 && out.bytes[1]==216);
        assert(out.bytes[out.size-2]==255 && out.bytes[out.size-1]==217);
    }
    double duration=(milliseconds()-start)/iterations;
    FILE *file=fopen(argv[1],"wb");assert(file);
    assert(fwrite(out.bytes,1,out.size,file)==out.size);assert(!fclose(file));
    printf("{\"codec\":\"%s\",\"mean_ms\":%.4f,\"bytes\":%zu,\"iterations\":%d}\n",jpeg_codec_name(),duration,out.size,iterations);
    free(out.bytes);free(pixels);return 0;
}
