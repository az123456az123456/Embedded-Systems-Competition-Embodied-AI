#include "jpeg_codec.h"
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#ifdef HAVE_TURBOJPEG
#include <turbojpeg.h>
const char *jpeg_codec_name(void) { return "turbojpeg-simd"; }
#else
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include "vendor/stb_image_write.h"
#pragma GCC diagnostic pop
const char *jpeg_codec_name(void) { return "stb-scalar"; }
#endif
int jpeg_from_argb(const uint32_t *argb,int width,int height,int quality,void (*write)(void *,void *,int),void *context){
 if(!argb || !write || width<1 || height<1 || width>1920 || height>1080)return 0;
 if(quality<1 || quality>100)return 0;
#ifdef HAVE_TURBOJPEG
 /* Native 0xAARRGGBB pixels are BGRA bytes on RK3588's little-endian CPU.
    Avoid the RGB conversion/allocation and use the SDK's SIMD DCT/colour path.
    Each caller owns its compressor, so concurrent streams cannot share state. */
 tjhandle encoder=tjInitCompress();if(!encoder)return 0;
 unsigned char *encoded=NULL;unsigned long length=0;
 const unsigned short endian=1;
 int format=*(const unsigned char *)&endian?TJPF_BGRA:TJPF_ARGB;
 int status=tjCompress2(encoder,(const unsigned char *)argb,width,width*4,height,
                       format,&encoded,&length,TJSAMP_420,quality,TJFLAG_FASTDCT);
 int okay=status==0 && encoded && length>0 && length<=INT_MAX;
 if(okay)write(context,encoded,(int)length);
 tjFree(encoded);tjDestroy(encoder);return okay;
#else
 unsigned char *rgb=malloc((size_t)width*height*3);if(!rgb)return 0;
 for(size_t i=0;i<(size_t)width*height;i++){rgb[i*3]=argb[i]>>16;rgb[i*3+1]=argb[i]>>8;rgb[i*3+2]=argb[i];}
 int result=stbi_write_jpg_to_func(write,context,width,height,3,rgb,quality);free(rgb);return result;
#endif
}
