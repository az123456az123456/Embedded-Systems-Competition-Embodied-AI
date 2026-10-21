#ifndef PERCEPTION_CAMERA_COLOR_MATH_H
#define PERCEPTION_CAMERA_COLOR_MATH_H
#include <algorithm>
#include <cmath>
#include <cstdint>

// Cached V4L2 matrix/range arithmetic shared by the reference and SIMD paths.
// LUTs preserve the original independently rounded 16.16 conversion exactly.
namespace camera_color {
struct Matrix {
    int luma[256],red_v[256],green_u[256],green_v[256],blue_u[256];
    int cy,crv,cgu,cgv,cbu,y_offset;
    Matrix(bool bt709,bool full_range) {
        const double kr=bt709?.2126:.299,kb=bt709?.0722:.114,kg=1-kr-kb;
        const double ys=full_range?1.:255./219,cs=full_range?1.:255./224;
        const double rv=2*(1-kr)*cs,gu=-2*kb*(1-kb)/kg*cs;
        const double gv=-2*kr*(1-kr)/kg*cs,bu=2*(1-kb)*cs;
        y_offset=full_range?0:16;
        cy=std::lround(ys*65536);crv=std::lround(rv*65536);
        cgu=std::lround(gu*65536);cgv=std::lround(gv*65536);cbu=std::lround(bu*65536);
        for(int i=0;i<256;i++) {
            luma[i]=std::lround((i-y_offset)*ys*65536);
            red_v[i]=std::lround(2*(1-kr)*(i-128)*cs*65536);
            green_u[i]=std::lround(-2*kb*(1-kb)/kg*(i-128)*cs*65536);
            green_v[i]=std::lround(-2*kr*(1-kr)/kg*(i-128)*cs*65536);
            blue_u[i]=std::lround(2*(1-kb)*(i-128)*cs*65536);
        }
    }
};
inline const Matrix &matrix(bool bt709,bool full_range) {
    static const Matrix matrices[4]={{false,false},{false,true},{true,false},{true,true}};
    return matrices[(bt709?2:0)+(full_range?1:0)];
}
inline uint8_t channel(int v) {
    return static_cast<uint8_t>(std::max(0,std::min(255,(v+32768)>>16)));
}
inline void reference_pixel(const Matrix &m,int y,int u,int v,uint8_t *dst) {
    dst[0]=channel(m.luma[y]+m.red_v[v]);
    dst[1]=channel(m.luma[y]+m.green_u[u]+m.green_v[v]);
    dst[2]=channel(m.luma[y]+m.blue_u[u]);
}
// Portable equivalent of the NEON arithmetic, used for exhaustive verification
// on development machines without ARM execution. The rounded coefficients can
// differ from the legacy LUT by at most one final 8-bit colour level.
inline void fixed_pixel(const Matrix &m,int y,int u,int v,uint8_t *dst) {
    const int yy=(y-m.y_offset)*m.cy,uu=u-128,vv=v-128;
    dst[0]=channel(yy+vv*m.crv);
    dst[1]=channel(yy+uu*m.cgu+vv*m.cgv);
    dst[2]=channel(yy+uu*m.cbu);
}
}
#endif
