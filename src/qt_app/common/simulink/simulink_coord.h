/**
 * @file simulink_coord.h
 * @brief 坐标变换: LLA↔ENU, 旋转矩阵, 齐次变换
 */
#ifndef SIMULINK_COORD_H
#define SIMULINK_COORD_H

#include <cmath>
#include <array>

namespace simulink {

// 3x3/4x4矩阵、3D/4D向量类型
using Mat3 = std::array<std::array<double, 3>, 3>;
using Mat4 = std::array<std::array<double, 4>, 4>;
using Vec3 = std::array<double, 3>;
using Vec4 = std::array<double, 4>;

inline Vec3 mat3_mul_vec3(const Mat3& m, const Vec3& v) {
    Vec3 r; for (int i=0;i<3;i++) r[i]=m[i][0]*v[0]+m[i][1]*v[1]+m[i][2]*v[2]; return r;
}
inline Vec4 mat4_mul_vec4(const Mat4& m, const Vec4& v) {
    Vec4 r; for (int i=0;i<4;i++) r[i]=m[i][0]*v[0]+m[i][1]*v[1]+m[i][2]*v[2]+m[i][3]*v[3]; return r;
}
inline Mat4 mat4_mul_mat4(const Mat4& a, const Mat4& b) {
    Mat4 r={{{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}}};
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) r[i][j]+=a[i][k]*b[k][j];
    return r;
}

/// 欧拉角(ZYX)→旋转矩阵 eul=[yaw,pitch,roll] in radians
inline Mat3 eul2rotm_ZYX(const Vec3& eul) {
    double cz=std::cos(eul[0]),sz=std::sin(eul[0]);
    double cy=std::cos(eul[1]),sy=std::sin(eul[1]);
    double cx=std::cos(eul[2]),sx=std::sin(eul[2]);
    Mat3 R;
    R[0]={cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx};
    R[1]={sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx};
    R[2]={-sy,   cy*sx,          cy*cx};
    return R;
}
inline Mat4 eul2tform_ZYX(const Vec3& eul) {
    Mat3 R=eul2rotm_ZYX(eul);
    Mat4 T={{{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,1}}};
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) T[i][j]=R[i][j];
    return T;
}
inline Mat4 trvec2tform(const Vec3& t) {
    return {{{1,0,0,t[0]},{0,1,0,t[1]},{0,0,1,t[2]},{0,0,0,1}}};
}

/// LLA→ENU (WGS84)
inline Vec3 lla2enu_internal(const Vec3& lla, const Vec3& lla0) {
    constexpr double a=6378137.0, f=1.0/298.257223563, e2=2*f-f*f;
    double lat=lla[0]*M_PI/180, lon=lla[1]*M_PI/180, alt=lla[2];
    double lat0=lla0[0]*M_PI/180, lon0=lla0[1]*M_PI/180, alt0=lla0[2];
    auto ecef=[&](double la,double lo,double al)->Vec3{
        double sl=std::sin(la),cl=std::cos(la),slo=std::sin(lo),clo=std::cos(lo);
        double N=a/std::sqrt(1-e2*sl*sl);
        return {(N+al)*cl*clo,(N+al)*cl*slo,(N*(1-e2)+al)*sl};};
    auto p=ecef(lat,lon,alt), p0=ecef(lat0,lon0,alt0);
    double dx=p[0]-p0[0],dy=p[1]-p0[1],dz=p[2]-p0[2];
    double sl0=std::sin(lat0),cl0=std::cos(lat0),slo0=std::sin(lon0),clo0=std::cos(lon0);
    return { -slo0*dx+clo0*dy,
             -sl0*clo0*dx-sl0*slo0*dy+cl0*dz,
              cl0*clo0*dx+cl0*slo0*dy+sl0*dz };
}

/// ENU→LLA (WGS84, 迭代法)
inline Vec3 enu2lla_internal(const Vec3& enu, const Vec3& lla0) {
    constexpr double a=6378137.0, f=1.0/298.257223563, e2=2*f-f*f;
    double lat0=lla0[0]*M_PI/180, lon0=lla0[1]*M_PI/180, alt0=lla0[2];
    double sl0=std::sin(lat0),cl0=std::cos(lat0),slo0=std::sin(lon0),clo0=std::cos(lon0);
    double dx=-slo0*enu[0]-sl0*clo0*enu[1]+cl0*clo0*enu[2];
    double dy= clo0*enu[0]-sl0*slo0*enu[1]+cl0*slo0*enu[2];
    double dz=              cl0*enu[1]     +sl0*enu[2];
    double N0=a/std::sqrt(1-e2*sl0*sl0);
    double x0=(N0+alt0)*cl0*clo0, y0=(N0+alt0)*cl0*slo0, z0=(N0*(1-e2)+alt0)*sl0;
    double x=x0+dx, y=y0+dy, z=z0+dz;
    double lon=std::atan2(y,x), p=std::sqrt(x*x+y*y);
    double lat=std::atan2(z,p*(1-e2));
    for(int i=0;i<10;i++){double sl=std::sin(lat);double N=a/std::sqrt(1-e2*sl*sl);lat=std::atan2(z+e2*N*sl,p);}
    double sl=std::sin(lat); double N=a/std::sqrt(1-e2*sl*sl);
    double alt_out=p/std::cos(lat)-N;
    return {lat*180/M_PI, lon*180/M_PI, alt_out};
}

inline double rad2deg(double u) { return u * 180.0 / M_PI; }

} // namespace simulink
#endif // SIMULINK_COORD_H
