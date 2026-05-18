/**
 * @file control_kinematics.h
 * @brief 运动学计算 + LLA↔ENU坐标变换 + 速度坐标变换
 */
#ifndef CONTROL_KINEMATICS_H
#define CONTROL_KINEMATICS_H

#include <cmath>

namespace control {

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// WGS84 椭球参数
constexpr double WGS84_A  = 6378137.0;
constexpr double WGS84_F  = 1.0 / 298.257223563;
constexpr double WGS84_B  = WGS84_A * (1 - WGS84_F);
constexpr double WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;

//------------------------------------------------------------------------------
// 运动学
//------------------------------------------------------------------------------
struct KinematicsOutput { double V_right_reference; double V_left_reference; };

/// 运动学规划 (Plan版本 D=0)
inline KinematicsOutput KinematicsForPlan(double x_velocity, double theta_velocity, double gama) {
    const double D = 0;
    double W_c = theta_velocity * DEG2RAD;
    return { x_velocity + 0.5 * W_c * D * gama,
             x_velocity - 0.5 * W_c * D * gama };
}

struct KinematicsRealOutput { double V_right_real; double V_left_real; };

/// 运动学实际 (D=3.42)
inline KinematicsRealOutput KinematicsForReal(double theta_velocity, double V_c, double gama) {
    const double D = 3.42;
    double W_c = theta_velocity * DEG2RAD;
    return { V_c + 0.5 * W_c * D * gama,
             V_c - 0.5 * W_c * D * gama };
}

//------------------------------------------------------------------------------
// LLA → ENU
//------------------------------------------------------------------------------
struct ENUCoord { double X; double Y; double UP; };

inline ENUCoord lla2enu(double lat, double lon, double alt,
                        double lat0, double lon0, double alt0) {
    double lat_rad  = lat  * DEG2RAD, lon_rad  = lon  * DEG2RAD;
    double lat0_rad = lat0 * DEG2RAD, lon0_rad = lon0 * DEG2RAD;

    double sl0 = std::sin(lat0_rad), cl0 = std::cos(lat0_rad);
    double N0 = WGS84_A / std::sqrt(1 - WGS84_E2 * sl0 * sl0);
    double X0 = (N0 + alt0) * cl0 * std::cos(lon0_rad);
    double Y0 = (N0 + alt0) * cl0 * std::sin(lon0_rad);
    double Z0 = (N0 * (1 - WGS84_E2) + alt0) * sl0;

    double sl = std::sin(lat_rad), cl = std::cos(lat_rad);
    double N  = WGS84_A / std::sqrt(1 - WGS84_E2 * sl * sl);
    double X  = (N + alt) * cl * std::cos(lon_rad);
    double Y  = (N + alt) * cl * std::sin(lon_rad);
    double Z  = (N * (1 - WGS84_E2) + alt) * sl;

    double dX = X - X0, dY = Y - Y0, dZ = Z - Z0;
    ENUCoord enu;
    enu.X  = -std::sin(lon0_rad) * dX + std::cos(lon0_rad) * dY;
    enu.Y  = -sl0 * std::cos(lon0_rad) * dX - sl0 * std::sin(lon0_rad) * dY + cl0 * dZ;
    enu.UP =  cl0 * std::cos(lon0_rad) * dX + cl0 * std::sin(lon0_rad) * dY + sl0 * dZ;
    return enu;
}

//------------------------------------------------------------------------------
// 速度坐标变换 (RTK东北 → 车体)
//------------------------------------------------------------------------------
struct VelocityTransformOutput { double v_x; double v_y; };

inline VelocityTransformOutput VelocityTransform(double chassis_theta_rtk_east_0,
                                                  double v_east_rtk, double v_north_rtk) {
    double a = -chassis_theta_rtk_east_0 * DEG2RAD;
    return { std::cos(a) * v_east_rtk - std::sin(a) * v_north_rtk,
             std::sin(a) * v_east_rtk + std::cos(a) * v_north_rtk };
}

//------------------------------------------------------------------------------
// 位置坐标变换
//------------------------------------------------------------------------------
inline double cosd(double deg) { return std::cos(deg * DEG2RAD); }
inline double sind(double deg) { return std::sin(deg * DEG2RAD); }

struct PositionTransformOutput { double theta; double x; double y; };

inline PositionTransformOutput PositionTransform(
    double chassis_theta_rtk_east, double chassis_theta_rtk_east_0,
    double x_rtk, double y_rtk)
{
    double t = chassis_theta_rtk_east - chassis_theta_rtk_east_0;
    double x = cosd(-chassis_theta_rtk_east_0) * x_rtk - sind(-chassis_theta_rtk_east_0) * y_rtk;
    double y = sind(-chassis_theta_rtk_east_0) * x_rtk + cosd(-chassis_theta_rtk_east_0) * y_rtk;
    return {t, x, y};
}

/// 北向角转东向角
inline double NorthToEastAngle(double chassis_theta_rtk_north) {
    double e = 90 - chassis_theta_rtk_north;
    if (e >= -269.99 && e < -180) e += 360;
    return e;
}

/// 导航坐标变换
inline PositionTransformOutput NaviPositionTransform(
    double x_navi_real, double y_navi_real, double theta_navi_real,
    double x_real0, double y_real0, double theta_navi_real0)
{
    double t = theta_navi_real - theta_navi_real0;
    double x = cosd(-theta_navi_real0) * (x_navi_real - x_real0) - sind(-theta_navi_real0) * (y_navi_real - y_real0);
    double y = sind(-theta_navi_real0) * (x_navi_real - x_real0) + cosd(-theta_navi_real0) * (y_navi_real - y_real0);
    return {t, x, y};
}

/// RTK角度修正
inline double RTKAngleCorrection(double Cabin_Theta_rtk, double Correction_Angle) {
    return Cabin_Theta_rtk + (Correction_Angle - 180);
}

/// 角度锁存
inline double ThetaLatch(double THETA, double BDP_Counter, double theta_) {
    return (BDP_Counter == 3) ? theta_ : THETA;
}

/// LLA锁存
struct LLALatchOutput { double lat; double lon; double alt; };
inline LLALatchOutput LLALatch(double LAT, double LON, double ALT,
                               double BDP_Counter, double lat_, double lon_, double alt_) {
    if (BDP_Counter == 3) return {lat_, lon_, alt_};
    return {LAT, LON, ALT};
}

/// 旋转标志位置锁存
inline PositionTransformOutput RotatingFlagLatch(
    double x, double y, double t, double flag,
    double x_, double y_, double t_)
{
    if (flag == 2) return {t_, x_, y_};
    return {t, x, y};
}

/// 参考位置选择
inline double RefPositionSelect(double Ref_X, double BDP_Counter, double X_Real) {
    return (BDP_Counter < 3) ? X_Real : Ref_X;
}

/// 参考差值计算
inline double RefDeltaCalc(double Ref_X, double Ref_X_, double y_) {
    return (Ref_X != Ref_X_) ? (Ref_X - Ref_X_) : y_;
}

} // namespace control
#endif // CONTROL_KINEMATICS_H
