#include "jpeg_codec.h"
#include <stdlib.h>
#include <stdio.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include "vendor/stb_image_write.h"
#pragma GCC diagnostic pop
int jpeg_from_argb(const uint32_t *argb,int width,int height,int quality,void (*write)(void *,void *,int),void *context){
 if(!argb || !write || width<1 || height<1 || width>1920 || height>1080)return 0;
 unsigned char *rgb=malloc((size_t)width*height*3);if(!rgb)return 0;
 for(size_t i=0;i<(size_t)width*height;i++){rgb[i*3]=argb[i]>>16;rgb[i*3+1]=argb[i]>>8;rgb[i*3+2]=argb[i];}
 int result=stbi_write_jpg_to_func(write,context,width,height,3,rgb,quality);free(rgb);return result;
}
