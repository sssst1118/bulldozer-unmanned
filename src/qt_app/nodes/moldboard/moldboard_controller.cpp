/**
 * @file moldboard_controller.cpp
 * @brief 铲刀控制器实现
 * @author dozer-dev
 * @date 2026-03-15
 */

#include "moldboard_controller.h"
#include "simulink_functions.h"
#include <ros/console.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

MoldboardController::MoldboardController(ros::NodeHandle& nh) : nh_(nh)
{
    ROS_INFO("Initializing MoldboardController...");
    
    initPublishers();
    initSubscribers();
    
    // 创建定时器 (50Hz，铲刀控制不需要太高频率)
    update_timer_ = nh_.createTimer(ros::Duration(0.02),
                                     &MoldboardController::timerCallback, this);
    
    ROS_INFO("MoldboardController initialized at 50Hz");
}

void MoldboardController::initPublishers()
{
    // 发送到CAN的话题（与ros_to_can_ros_adapter.cpp对应）
    pub_moldboard_target_ = nh_.advertise<std_msgs::Int16MultiArray>(
        "/Bulldozer_Moldboard_Target", 1);
    pub_moldboard_actual_ = nh_.advertise<std_msgs::Int16MultiArray>(
        "/Bulldozer_Moldboard_Actual", 1);
    pub_moldboard_right_dval_ = nh_.advertise<std_msgs::Int16>(
        "/Bulldozer_Moldboard_Right_Dvalue", 1);
    pub_moldboard_left_dval_ = nh_.advertise<std_msgs::Int16>(
        "/Bulldozer_Moldboard_Left_Dvalue", 1);
    pub_mode_switch_3d_ = nh_.advertise<std_msgs::Int16>(
        "/Mode_Switch_3D", 1);
    
    // 发送到决策模块的话题
    pub_avg_height_actual_ = nh_.advertise<std_msgs::Float64>(
        "/decision/avg_height_actual", 1);
    pub_overload_flag_ = nh_.advertise<std_msgs::Float64>(
        "/decision/overload_flag", 1);
    
    // 发送到控制模块 (Float64, 单位米)
    pub_height_left_float_ = nh_.advertise<std_msgs::Float64>(
        "/moldboard/actual_height_left", 1);
    pub_height_right_float_ = nh_.advertise<std_msgs::Float64>(
        "/moldboard/actual_height_right", 1);
    
    // [Fix-v19] 发送铲刀IMU横滚角 (度) 给 control 的铲刀角度PID
    //   来源: /AHRS_IMU 的 orientation_covariance[0] (铲刀 IMU 绝对横滚)
    //   消费: control_node 订阅 /moldboard/imu_roll → inputs_.IMU_Roll
    //        用于 e_Theta = Ref_Angle - IMU_Roll 做 3D 绝对找平的横滚 PID
    pub_imu_roll_ = nh_.advertise<std_msgs::Float64>("/moldboard/imu_roll", 1);
    
    // 发布铲刀端点位置 (用于可视化和调试)
    pub_point_b_lla_ = nh_.advertise<geometry_msgs::Point>("/PointB_lla", 1);
    pub_point_d_lla_ = nh_.advertise<geometry_msgs::Point>("/PointD_lla", 1);
    pub_point_m_lla_ = nh_.advertise<geometry_msgs::Point>("/PointM_lla", 1);

    ROS_INFO("MoldboardController publishers initialized");
}

void MoldboardController::initSubscribers()
{
    // RTK位置数据 [lat, lon, alt] - 支持两种消息类型
    sub_rtk_ = nh_.subscribe("/LLA", 1,
                              &MoldboardController::rtkCallback, this);
    
    // RTK姿态数据 [yaw, pitch, roll]
    sub_rtk_angle_ = nh_.subscribe("/Angle_Heading", 1,
                                    &MoldboardController::rtkAngleCallback, this);
    
    // IMU姿态数据 [yaw, pitch, roll] - 铲刀上的IMU
    sub_imu_angle_ = nh_.subscribe("/AHRS_IMU", 1,
                                    &MoldboardController::imuAngleCallback, this);
    
    // 来自决策模块的规划数据
    sub_avg_height_plan_ = nh_.subscribe("/decision/avg_height_plan", 1,
                                          &MoldboardController::avgHeightPlanCallback, this);
    sub_ref_mold_theta_plan_ = nh_.subscribe("/decision/ref_mold_theta_plan", 1,
                                              &MoldboardController::refMoldThetaPlanCallback, this);
    sub_moldboard_control_flag_ = nh_.subscribe("/decision/moldboard_control_flag", 1,
                                                 &MoldboardController::moldboardControlFlagCallback, this);
    
    // 3D找平模式开关
    sub_mode_switch_3d_ = nh_.subscribe("/Mode_Switch_3D_Input", 1,
                                         &MoldboardController::modeSwitchCallback, this);
    
    // 输出电流（用于过载检测）
    sub_output_current_ = nh_.subscribe("/OUTPUT_CURRENT", 1,
                                         &MoldboardController::outputCurrentCallback, this);
    
    // [Issue#7] 铲刀高度基准点 (来自 decision_node, 由 GUI 设置)
    sub_blade_origin_ = nh_.subscribe("/decision/moldboard_origin", 1,
                                       &MoldboardController::bladeOriginCallback, this);
    
    ROS_INFO("MoldboardController subscribers initialized");
}

void MoldboardController::rtkCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // [Fix-v18] can_to_ros 发布约定: point.x=经度, point.y=纬度, point.z=高度
    //   原代码 lat=msg->x / lon=msg->y 把经纬度搞反了,导致铲刀位姿计算基准完全错位。
    //   现按 can_to_ros.cpp:517-519 的实际发布方向修正。
    inputs_.rtk_latitude  = msg->point.y;
    inputs_.rtk_longitude = msg->point.x;
    inputs_.rtk_altitude  = msg->point.z;
    
    ROS_DEBUG_THROTTLE(1.0, "RTK LLA: lat=%.7f, lon=%.7f, alt=%.2f", 
        inputs_.rtk_latitude, inputs_.rtk_longitude, inputs_.rtk_altitude);
}

void MoldboardController::rtkAngleCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // [Fix-v18] can_to_ros 发布 /Angle_Heading 的单位是弧度 (can_to_ros.cpp:532-534 显式乘 M_PI/180)。
    //   而 simulink::Moldboard_Pose_Calc 要求输入是度 (内部会 deg2rad)。
    //   原代码直接存弧度导致姿态补偿完全失效 (yaw=0.5rad 被当成 0.5°)。
    // 车体姿态: x=yaw, y=pitch, z=roll, 单位:弧度 → 存为度
    constexpr double RAD2DEG = 180.0 / M_PI;
    inputs_.rtk_yaw   = msg->point.x * RAD2DEG;
    inputs_.rtk_pitch = msg->point.y * RAD2DEG;
    inputs_.rtk_roll  = msg->point.z * RAD2DEG;
    
    ROS_DEBUG_THROTTLE(1.0, "RTK Angle (deg): yaw=%.2f, pitch=%.2f, roll=%.2f",
        inputs_.rtk_yaw, inputs_.rtk_pitch, inputs_.rtk_roll);
}

void MoldboardController::imuAngleCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // [Fix-v18] /AHRS_IMU 发布类型是 sensor_msgs/Imu (can_to_ros.cpp:100)。
    //   原代码签名写成 PointStamped, md5sum 不匹配 → 订阅建不起来, 铲刀IMU永远是0。
    //   can_to_ros 把欧拉角(度)塞进 orientation_covariance[0..2] = [roll, pitch, yaw]。
    //   见 can_to_ros.cpp:337-345 和 691-693。
    inputs_.imu_roll  = msg->orientation_covariance[0];  // 度
    inputs_.imu_pitch = msg->orientation_covariance[1];  // 度
    inputs_.imu_yaw   = msg->orientation_covariance[2];  // 度 (下游Moldboard_Pose_Calc会被车体yaw覆盖)
    
    ROS_DEBUG_THROTTLE(1.0, "IMU Angle (deg): yaw=%.2f, pitch=%.2f, roll=%.2f",
        inputs_.imu_yaw, inputs_.imu_pitch, inputs_.imu_roll);
}

void MoldboardController::avgHeightPlanCallback(const std_msgs::Float64::ConstPtr& msg)
{
    inputs_.avg_height_plan = msg->data;
    ROS_DEBUG_THROTTLE(1.0, "Avg height plan: %.3f m", inputs_.avg_height_plan);
}

void MoldboardController::refMoldThetaPlanCallback(const std_msgs::Float64::ConstPtr& msg)
{
    inputs_.ref_mold_theta_plan = msg->data;
    ROS_DEBUG_THROTTLE(1.0, "Ref mold theta plan: %.2f deg", inputs_.ref_mold_theta_plan);
}

void MoldboardController::moldboardControlFlagCallback(const std_msgs::Float64::ConstPtr& msg)
{
    inputs_.moldboard_control_flag = msg->data;
    ROS_DEBUG_THROTTLE(1.0, "Moldboard control flag: %.0f", inputs_.moldboard_control_flag);
}

void MoldboardController::modeSwitchCallback(const std_msgs::Int16::ConstPtr& msg)
{
    inputs_.mode_switch_3d = msg->data;
    ROS_INFO_THROTTLE(5.0, "3D mode switch: %d", msg->data);
}

void MoldboardController::outputCurrentCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
    // [Fix-v18] can_to_ros 发布 /OUTPUT_CURRENT 类型是 UInt16MultiArray (can_to_ros.cpp:103)。
    //   原签名 Float64MultiArray md5sum 不匹配, 订阅失败, checkOverload() 永远拿到 0 → 过载保护失效。
    if (msg->data.size() >= 4) {
        for (int i = 0; i < 4; ++i) {
            inputs_.output_current[i] = static_cast<double>(msg->data[i]);
        }
    }
}

void MoldboardController::bladeOriginCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    // [Issue#7] 更新铲刀高度基准点 (x=lat, y=lon, z=alt)
    work_origin_ = {msg->x, msg->y, msg->z};
}

void MoldboardController::timerCallback(const ros::TimerEvent& event)
{
    update();
    publishOutputs();
}

void MoldboardController::update()
{
    // 计算实际高度
    computeActualHeight();
    
    // 计算目标高度
    computeTargetHeight();
    
    // 计算高度差
    computeHeightDiff();
    
    // 检查过载
    checkOverload();
}

void MoldboardController::computeActualHeight()
{
    // 检查输入有效性
    if (inputs_.rtk_latitude == 0 && inputs_.rtk_longitude == 0) {
        ROS_WARN_THROTTLE(5.0, "RTK position not available, skipping moldboard height calculation");
        return;
    }
    
    // 计算铲刀实际高度
    std::array<double, 3> PointA_lla = {
        inputs_.rtk_latitude,
        inputs_.rtk_longitude,
        inputs_.rtk_altitude
    };
    
    std::array<double, 3> vehicle_angle = {
        inputs_.rtk_yaw,
        inputs_.rtk_pitch,
        inputs_.rtk_roll
    };
    
    std::array<double, 3> imu_angle = {
        inputs_.imu_yaw,
        inputs_.imu_pitch,
        inputs_.imu_roll
    };
    
    simulink::MoldboardPoseOutput pose_result = simulink::Moldboard_Pose_Calc(
        PointA_lla, vehicle_angle, imu_angle, work_origin_);  // [Issue#7] 使用运行时基准点
    
    // 保存所有输出
    outputs_.actual_height_left_moldboard_enu = pose_result.actual_height_left_moldboard_enu;
    outputs_.actual_height_right_moldboard_enu = pose_result.actual_height_right_moldboard_enu;
    outputs_.PointD_lla = pose_result.PointD_lla;
    outputs_.PointM_lla = pose_result.PointM_lla;
    outputs_.PointB_lla = pose_result.PointB_lla;
    outputs_.PointD_enu_maxThreshold_height = pose_result.PointD_enu_maxThreshold_height;
    outputs_.PointD_enu_minThreshold_height = pose_result.PointD_enu_minThreshold_height;
    outputs_.PointM_enu_maxThreshold_height = pose_result.PointM_enu_maxThreshold_height;
    outputs_.PointM_enu_minThreshold_height = pose_result.PointM_enu_minThreshold_height;
    
    // 转换为mm发送到CAN (ENU高度 * 1000)
    outputs_.actual_height_right = static_cast<int16_t>(
        outputs_.actual_height_right_moldboard_enu * 1000);
    outputs_.actual_height_left = static_cast<int16_t>(
        outputs_.actual_height_left_moldboard_enu * 1000);
    
    // 计算平均实际高度 
    outputs_.avg_height_actual = simulink::Avg_Height_Actual_Calc(
        outputs_.actual_height_left_moldboard_enu,
        outputs_.actual_height_right_moldboard_enu);
    
    ROS_DEBUG_THROTTLE(1.0, "Moldboard height: left=%.3f m, right=%.3f m, avg=%.3f m",
        outputs_.actual_height_left_moldboard_enu,
        outputs_.actual_height_right_moldboard_enu,
        outputs_.avg_height_actual);
}

void MoldboardController::computeTargetHeight()
{
    // 只在铲刀控制标志激活时计算目标高度
    if (inputs_.moldboard_control_flag != 1) {
        // 控制关闭时，目标等于实际
        outputs_.target_height_right = outputs_.actual_height_right;
        outputs_.target_height_left = outputs_.actual_height_left;
        return;
    }
    
    // 基于规划高度和规划角度计算目标高度
    // 铲刀宽度使用lever_D_M_0[0] = -4.1814
    double half_width = 4.1814 / 2.0;
    double plan_roll_rad = inputs_.ref_mold_theta_plan * M_PI / 180.0;
    
    // 计算左右铲刀目标高度 (mm)
    double height_offset = half_width * std::sin(plan_roll_rad);
    outputs_.target_height_right = static_cast<int16_t>(
        (inputs_.avg_height_plan - height_offset) * 1000);
    outputs_.target_height_left = static_cast<int16_t>(
        (inputs_.avg_height_plan + height_offset) * 1000);
    
    ROS_DEBUG_THROTTLE(1.0, "Target height: left=%d mm, right=%d mm",
        outputs_.target_height_left, outputs_.target_height_right);
}

void MoldboardController::computeHeightDiff()
{
    // 计算目标与实际的高度差
    outputs_.height_diff_right = outputs_.target_height_right - outputs_.actual_height_right;
    outputs_.height_diff_left = outputs_.target_height_left - outputs_.actual_height_left;
    
    ROS_DEBUG_THROTTLE(1.0, "Height diff: left=%d mm, right=%d mm",
        outputs_.height_diff_left, outputs_.height_diff_right);
}

void MoldboardController::checkOverload()
{
    // 检查输出电流是否超过阈值
    double max_current = 0;
    for (int i = 0; i < 4; ++i) {
        if (inputs_.output_current[i] > max_current) {
            max_current = inputs_.output_current[i];
        }
    }
    
    if (max_current > OVERLOAD_CURRENT_THRESHOLD) {
        outputs_.overload_flag = 1;
        ROS_WARN_THROTTLE(1.0, "Moldboard overload detected! Current: %.1f A", max_current);
    } else {
        outputs_.overload_flag = 0;
    }
}

void MoldboardController::publishOutputs()
{
    // 发布铲刀目标高程
    std_msgs::Int16MultiArray target_msg;
    target_msg.data.resize(2);
    target_msg.data[0] = outputs_.target_height_right;
    target_msg.data[1] = outputs_.target_height_left;
    pub_moldboard_target_.publish(target_msg);
    
    // 发布铲刀实际高程
    std_msgs::Int16MultiArray actual_msg;
    actual_msg.data.resize(2);
    actual_msg.data[0] = outputs_.actual_height_right;
    actual_msg.data[1] = outputs_.actual_height_left;
    pub_moldboard_actual_.publish(actual_msg);
    
    // 发布高程差
    std_msgs::Int16 right_dval_msg;
    right_dval_msg.data = outputs_.height_diff_right;
    pub_moldboard_right_dval_.publish(right_dval_msg);
    
    std_msgs::Int16 left_dval_msg;
    left_dval_msg.data = outputs_.height_diff_left;
    pub_moldboard_left_dval_.publish(left_dval_msg);
    
    // 发布3D找平模式开关
    std_msgs::Int16 mode_msg;
    mode_msg.data = static_cast<int16_t>(inputs_.mode_switch_3d);
    pub_mode_switch_3d_.publish(mode_msg);
    
    // 发布平均实际高度
    std_msgs::Float64 avg_height_msg;
    avg_height_msg.data = outputs_.avg_height_actual;
    pub_avg_height_actual_.publish(avg_height_msg);
    
    // 发布左右铲刀端点ENU高度 (Float64, 单位米, 给 control_node 铲刀PID 用)
    std_msgs::Float64 hl_msg, hr_msg;
    hl_msg.data = outputs_.actual_height_left_moldboard_enu;
    hr_msg.data = outputs_.actual_height_right_moldboard_enu;
    pub_height_left_float_.publish(hl_msg);
    pub_height_right_float_.publish(hr_msg);
    
    // [Fix-v19] 发布铲刀 IMU 绝对横滚角 (给 control 的铲刀角度 PID 做反馈)
    //   没有这个发布之前, control.IMU_Roll 永远是 0, e_Theta = Ref_Angle - 0 = Ref_Angle,
    //   PID 跟不闭环。修复后横滚 PID 才能真正工作。
    std_msgs::Float64 imu_roll_msg;
    imu_roll_msg.data = inputs_.imu_roll;  // 度, 相对地平线的绝对横滚
    pub_imu_roll_.publish(imu_roll_msg);
    
    // 发布过载标志
    std_msgs::Float64 overload_msg;
    overload_msg.data = outputs_.overload_flag;
    pub_overload_flag_.publish(overload_msg);
    
    // 发布铲刀端点位置
    geometry_msgs::Point point_b, point_d, point_m;
    point_b.x = outputs_.PointB_lla[0];
    point_b.y = outputs_.PointB_lla[1];
    point_b.z = outputs_.PointB_lla[2];
    pub_point_b_lla_.publish(point_b);
    
    point_d.x = outputs_.PointD_lla[0];
    point_d.y = outputs_.PointD_lla[1];
    point_d.z = outputs_.PointD_lla[2];
    pub_point_d_lla_.publish(point_d);
    
    point_m.x = outputs_.PointM_lla[0];
    point_m.y = outputs_.PointM_lla[1];
    point_m.z = outputs_.PointM_lla[2];
    pub_point_m_lla_.publish(point_m);
}
