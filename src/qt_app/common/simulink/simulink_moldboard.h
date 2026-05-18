/**
 * @file simulink_moldboard.h
 * @brief 铲刀位姿计算: RTK天线→杆臂变换→铲刀端点LLA
 */
#ifndef SIMULINK_MOLDBOARD_H
#define SIMULINK_MOLDBOARD_H

#include "simulink_coord.h"
#include <algorithm>

namespace simulink {

// 标定常量
constexpr double LEVER_A_B[3]     = {0.2281, 0.1924, -0.1068};
constexpr double LEVER_B_D_0[3]   = {0.2613, 3.0947, -0.2294};
constexpr double LEVER_D_M_0[3]   = {-4.1814, 0.0047, -0.0632};
constexpr double INIT_ORIGIN[3]   = {34.400685539, 117.470578425, 45.256};
constexpr double ANGLE_MAX_THRESHOLD[3] = {0.0, 1.1620, 18.4927};
constexpr double ANGLE_MIN_THRESHOLD[3] = {0.0, 0.1691, -5.4196};

struct MoldboardPoseOutput {
    double actual_height_left_moldboard_enu;
    double actual_height_right_moldboard_enu;
    std::array<double,3> PointD_lla, PointM_lla, PointB_lla;
    double PointD_enu_maxThreshold_height, PointD_enu_minThreshold_height;
    double PointM_enu_maxThreshold_height, PointM_enu_minThreshold_height;
};

/**
 * @brief 铲刀位姿计算
 * @param PointA_lla  RTK天线 LLA [lat, lon, alt]
 * @param vehicle_angle 车体姿态角 [yaw, pitch, roll] (度)
 * @param imu_angle     铲刀IMU姿态角 [yaw, pitch, roll] (度)
 * @param work_origin   铲刀高度基准点 LLA [lat, lon, alt]，默认使用 INIT_ORIGIN
 */
inline MoldboardPoseOutput Moldboard_Pose_Calc(
    const std::array<double,3>& PointA_lla,
    std::array<double,3> vehicle_angle,
    std::array<double,3> imu_angle,
    const Vec3& work_origin = {INIT_ORIGIN[0], INIT_ORIGIN[1], INIT_ORIGIN[2]})
{
    MoldboardPoseOutput output;
    // 角度范围标准化
    if (vehicle_angle[0] < 180) vehicle_angle[0] = -vehicle_angle[0];
    else                        vehicle_angle[0] = 360 - vehicle_angle[0];
    imu_angle[0] = vehicle_angle[0];
    std::swap(vehicle_angle[1], vehicle_angle[2]);
    std::swap(imu_angle[1], imu_angle[2]);

    const Vec3 PointA_enu = {0,0,0};
    const Vec3 lever_A_B = {LEVER_A_B[0], LEVER_A_B[1], LEVER_A_B[2]};
    const Vec3 lever_B_D_0 = {LEVER_B_D_0[0], LEVER_B_D_0[1], LEVER_B_D_0[2]};
    const Vec3 lever_D_M_0 = {LEVER_D_M_0[0], LEVER_D_M_0[1], LEVER_D_M_0[2]};
    const Vec3& InIt_origin = work_origin;  // [Issue#7] 使用传入的基准点
    const Vec3 angle_max = {ANGLE_MAX_THRESHOLD[0]*M_PI/180, ANGLE_MAX_THRESHOLD[1]*M_PI/180, ANGLE_MAX_THRESHOLD[2]*M_PI/180};
    const Vec3 angle_min = {ANGLE_MIN_THRESHOLD[0]*M_PI/180, ANGLE_MIN_THRESHOLD[1]*M_PI/180, ANGLE_MIN_THRESHOLD[2]*M_PI/180};

    // 阈值计算 (2次取平均)
    Vec3 Dd_max={0,0,0},Dd_min={0,0,0},Mm_max={0,0,0},Mm_min={0,0,0};
    for(int i=0;i<2;i++){
        Mat3 Rpm=eul2rotm_ZYX({0,angle_max[1],0}), Rpn=eul2rotm_ZYX({0,angle_min[1],0});
        Mat3 Rrm=eul2rotm_ZYX({0,0,angle_max[2]}), Rrn=eul2rotm_ZYX({0,0,angle_min[2]});
        Vec3 va={vehicle_angle[0]*M_PI/180, vehicle_angle[1]*M_PI/180, vehicle_angle[2]*M_PI/180};
        Mat4 vr=eul2tform_ZYX(va);
        Mat4 TWA=mat4_mul_mat4(trvec2tform(PointA_enu),vr);
        Vec4 abh={lever_A_B[0],lever_A_B[1],lever_A_B[2],1};
        Vec4 Bh=mat4_mul_vec4(TWA,abh); Vec3 Be={Bh[0],Bh[1],Bh[2]};
        Vec3 bdm=mat3_mul_vec3(Rpm,lever_B_D_0), bdn=mat3_mul_vec3(Rpn,lever_B_D_0);
        Mat4 TWB=mat4_mul_mat4(trvec2tform(Be),vr);
        Vec4 Dmh=mat4_mul_vec4(TWB,{bdm[0],bdm[1],bdm[2],1}), Dnh=mat4_mul_vec4(TWB,{bdn[0],bdn[1],bdn[2],1});
        Vec3 Dm={Dmh[0],Dmh[1],Dmh[2]}, Dn={Dnh[0],Dnh[1],Dnh[2]};
        Vec3 dmm=mat3_mul_vec3(Rrm,lever_D_M_0), dmn=mat3_mul_vec3(Rrn,lever_D_M_0);
        Mat4 TWDm=mat4_mul_mat4(trvec2tform(Dm),vr), TWDn=mat4_mul_mat4(trvec2tform(Dn),vr);
        Vec4 Mmh=mat4_mul_vec4(TWDm,{dmm[0],dmm[1],dmm[2],1}), Mnh=mat4_mul_vec4(TWDn,{dmn[0],dmn[1],dmn[2],1});
        Vec3 Dl_max=enu2lla_internal({Dmh[0],Dmh[1],Dmh[2]},PointA_lla);
        Vec3 Dl_min=enu2lla_internal({Dnh[0],Dnh[1],Dnh[2]},PointA_lla);
        Vec3 Ml_max=enu2lla_internal({Mmh[0],Mmh[1],Mmh[2]},PointA_lla);
        Vec3 Ml_min=enu2lla_internal({Mnh[0],Mnh[1],Mnh[2]},PointA_lla);
        for(int j=0;j<3;j++){Dd_max[j]+=Dl_max[j]/2;Dd_min[j]+=Dl_min[j]/2;Mm_max[j]+=Ml_max[j]/2;Mm_min[j]+=Ml_min[j]/2;}
    }
    output.PointD_enu_maxThreshold_height=lla2enu_internal(Dd_max,InIt_origin)[2];
    output.PointD_enu_minThreshold_height=lla2enu_internal(Dd_min,InIt_origin)[2];
    output.PointM_enu_maxThreshold_height=lla2enu_internal(Mm_max,InIt_origin)[2];
    output.PointM_enu_minThreshold_height=lla2enu_internal(Mm_min,InIt_origin)[2];

    // 实际铲刀位置
    Vec3 avi={(imu_angle[0]-vehicle_angle[0])*M_PI/180,(imu_angle[1]-vehicle_angle[1])*M_PI/180,(imu_angle[2]-vehicle_angle[2])*M_PI/180};
    Mat3 Rp=eul2rotm_ZYX({0,avi[1],0}), Rr=eul2rotm_ZYX({0,0,avi[2]});
    Vec3 va={vehicle_angle[0]*M_PI/180, vehicle_angle[1]*M_PI/180, vehicle_angle[2]*M_PI/180};
    Mat4 vr=eul2tform_ZYX(va);
    Mat4 TWA=mat4_mul_mat4(trvec2tform(PointA_enu),vr);
    Vec4 Bh=mat4_mul_vec4(TWA,{lever_A_B[0],lever_A_B[1],lever_A_B[2],1}); Vec3 Be={Bh[0],Bh[1],Bh[2]};
    Vec3 bd=mat3_mul_vec3(Rp,lever_B_D_0);
    Mat4 TWB=mat4_mul_mat4(trvec2tform(Be),vr);
    Vec4 Dh=mat4_mul_vec4(TWB,{bd[0],bd[1],bd[2],1}); Vec3 De={Dh[0],Dh[1],Dh[2]};
    Vec3 dm=mat3_mul_vec3(Rr,lever_D_M_0);
    Mat4 TWD=mat4_mul_mat4(trvec2tform(De),vr);
    Vec4 Mh=mat4_mul_vec4(TWD,{dm[0],dm[1],dm[2],1}); Vec3 Me={Mh[0],Mh[1],Mh[2]};

    output.PointB_lla=enu2lla_internal(Be,PointA_lla);
    output.PointD_lla=enu2lla_internal(De,PointA_lla);
    output.PointM_lla=enu2lla_internal(Me,PointA_lla);
    Vec3 De_init=lla2enu_internal(output.PointD_lla,InIt_origin);
    Vec3 Me_init=lla2enu_internal(output.PointM_lla,InIt_origin);
    output.actual_height_right_moldboard_enu=De_init[2];
    output.actual_height_left_moldboard_enu=Me_init[2];
    return output;
}

inline double Avg_Height_Actual_Calc(double left, double right) { return (left+right)/2.0; }

} // namespace simulink
#endif // SIMULINK_MOLDBOARD_H
