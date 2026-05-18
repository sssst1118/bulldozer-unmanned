/**
 * @file control_node.h
 * @brief 运动控制节点头文件
 * @details 该文件定义了推土机运动控制模块，实现以下功能：
 *          - 卡尔曼滤波速度融合 (IMU + RTK)
 *          - 跟踪微分器 (TD) 信号平滑
 *          - PID速度控制
 *          - 运动学计算 (差速转向)
 *          - 坐标转换 (LLA ↔ ENU)
 * 
 * @version V1.0
 * @date 2026-03-15
 * @author dozer-dev
 * 
 * @note 以100Hz频率运行完整的控制环路
 * 
 * @par 控制架构:
 *      决策层输入 → 规划层 → 运动学层 → TD平滑 → PID控制 → 执行层
 * 
 * @par 运行频率: 100Hz
 */

#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H

//==============================================================================
// 系统头文件
//==============================================================================
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/function.hpp>
#include "control_functions.h"

// 控制周期
constexpr double CONTROL_PERIOD_SEC = 0.01;  ///< 控制周期 (100Hz)

//==============================================================================
// 控制节点输入数据结构
//==============================================================================

/**
 * @brief 控制节点输入数据结构
 * @details 包含运动控制所需的所有传感器数据和控制参数
 */
struct ControlInputs {
    //--------------------------------------------------------------------------
    // RTK数据 (来自 /RTK 或 /LLA)
    //--------------------------------------------------------------------------
    double rtk_lat = 0;  ///< RTK纬度 (度，WGS84)
    double rtk_lon = 0;  ///< RTK经度 (度，WGS84)
    double rtk_alt = 0;  ///< RTK高度 (米，椭球高)
    double rtk_yaw = 0;  ///< RTK航向角 (度，北偏东为正)
    
    //--------------------------------------------------------------------------
    // IMU数据 (来自 /IMU)
    //--------------------------------------------------------------------------
    double imu_velocity = 0;   ///< IMU测量速度 (m/s，前进为正)
    double imu_yaw_rate = 0;   ///< IMU角速度 (deg/s，逆时针为正)
    
    //--------------------------------------------------------------------------
    // 终点参考值 (来自 decision_node)
    //--------------------------------------------------------------------------
    double X_terminal = 0;      ///< X方向终点距离 (米，相对于当前位置)
    double Y_terminal = 0;      ///< Y方向终点距离 (米，通常为0)
    double Theta_terminal = 0;  ///< 终点角度 (度，相对于初始航向)
    double v_reference = 0;     ///< 参考线速度 (m/s)
    double theta_velocity_reference = 0; ///< 参考角速度 (deg/s)
    
    //--------------------------------------------------------------------------
    // 状态 (来自 decision_node)
    //--------------------------------------------------------------------------
    double walk_state = 0;   ///< 行走状态 (0=停止, 1=旋转, 2=直行)
    double main_switch = 0;  ///< 主开关 (0=关闭, 1=开启)
    
    //--------------------------------------------------------------------------
    // 容差参数
    //--------------------------------------------------------------------------
    double X_Tolerance = 0.1;      ///< X方向到达容差 (米)
    double Y_Tolerance = 0.1;      ///< Y方向到达容差 (米)
    double Theta_Tolerance = 2.0;  ///< 角度到达容差 (度)
    
    //--------------------------------------------------------------------------
    // 控制参数
    //--------------------------------------------------------------------------
    double hold_x = 1.0;       ///< X方向保持距离 (米，用于减速)
    double hold_theta = 5.0;   ///< 角度保持距离 (度，用于减速)
    double gama = 1.0;         ///< 运动学增益系数
    double track_width = 2.5;  ///< 履带宽度 (米，用于差速转向)
    
    //--------------------------------------------------------------------------
    // PID参数 - X方向 (纵向速度控制)
    //--------------------------------------------------------------------------
    double Kp_x = 1.0;   ///< X方向比例增益
    double Ki_x = 0.1;   ///< X方向积分增益
    double Kd_x = 0.01;  ///< X方向微分增益
    
    //--------------------------------------------------------------------------
    // PID参数 - Theta方向 (角速度控制)
    //--------------------------------------------------------------------------
    double Kp_theta = 1.0;   ///< Theta方向比例增益
    double Ki_theta = 0.1;   ///< Theta方向积分增益
    double Kd_theta = 0.01;  ///< Theta方向微分增益
    
    //--------------------------------------------------------------------------
    // 跟踪微分器 (TD) 参数
    //--------------------------------------------------------------------------
    double td_r = 100;   ///< TD跟踪速度因子 (越大跟踪越快)
    double td_h = 0.01;  ///< TD滤波因子 (越大滤波效果越好)
    
    //--------------------------------------------------------------------------
    // 非线性层参数 (用于速度限制和死区处理)
    //--------------------------------------------------------------------------
    double nl_param1 = 200;  ///< 非线性参数1 (死区阈值)
    double nl_param2 = 400;  ///< 非线性参数2 (线性区斜率)
    double nl_param3 = 800;  ///< 非线性参数3 (饱和阈值)
    
    //--------------------------------------------------------------------------
    // V_right/V_left → /U_Lever1 桥接参数
    //--------------------------------------------------------------------------
    double gear_deadzone = 10.0;   ///< 挡位切换死区 (低于此值为空挡)
    double steer_deadzone = 20.0;  ///< 转向触发死区 (速度差低于此值不转向)
    double v_max_limit = 500.0;    ///< 全局速度上限 (0~1000, 调试阶段建议设低)
    
    //--------------------------------------------------------------------------
    // 铲刀控制输入 (来自 decision_node 和 moldboard_node)
    //--------------------------------------------------------------------------
    double actual_height_left_moldboard = 0;   ///< 左铲刀实际高度 (m, 来自moldboard_node)
    double actual_height_right_moldboard = 0;  ///< 右铲刀实际高度 (m, 来自moldboard_node)
    double ref_height_middle_moldboard = 0;    ///< 铲刀中心参考高度 (m, 来自decision_node)
    double Ref_Angle = 0;                      ///< 铲刀参考角度 (deg, 来自decision_node)
    double IMU_Roll = 0;                       ///< 铲刀IMU横滚角 (deg, 来自IMU)
    double Moldboard_Control_Flag = 0;         ///< 铲刀控制标志 (0=关闭, 1=开启)
    
    //--------------------------------------------------------------------------
    // 铲刀高度PID参数
    //--------------------------------------------------------------------------
    double Kp_Height_Up = 1.0;     ///< 高度上升P增益
    double Kp_Height_Down = 1.0;   ///< 高度下降P增益
    double Ki_Height_Up = 0.1;     ///< 高度上升I增益
    double Ki_Height_Down = 0.1;   ///< 高度下降I增益
    double Height_Param1 = 200;    ///< 高度非线性层参数1
    double Height_Param2 = 400;    ///< 高度非线性层参数2
    double Height_Param3 = 800;    ///< 高度非线性层参数3
    double Deadzone_Height = 3.0;  ///< 高度死区 (mm)
    double Dead_Value_Height = 100; ///< 高度死区阀值输出
    double I_MAX = 500;            ///< 高度积分限幅
    double I_angle_threshold = 10; ///< 高度积分衰减角度阈值
    
    //--------------------------------------------------------------------------
    // 铲刀角度PID参数
    //--------------------------------------------------------------------------
    double Kp_Theta_Up = 1.0;     ///< 角度上升P增益
    double Kp_Theta_Down = 1.0;   ///< 角度下降P增益
    double Ki_Theta_Up = 0.1;     ///< 角度上升I增益
    double Ki_Theta_Down = 0.1;   ///< 角度下降I增益
    double Theta_Param1 = 200;    ///< 角度非线性层参数1
    double Theta_Param2 = 400;    ///< 角度非线性层参数2
    double Theta_Param3 = 800;    ///< 角度非线性层参数3
    double Deadzone_Theta = 3.0;  ///< 角度死区 (deg)
    double Dead_Value_Theta = 100; ///< 角度死区阀值输出
    double I_MAX_Theta = 500;     ///< 角度积分限幅
    double I_angle_threshold_Theta = 10; ///< 角度积分衰减角度阈值
};

//==============================================================================
// 控制节点输出数据结构
//==============================================================================

/**
 * @brief 控制节点输出数据结构
 * @details 包含运动控制的所有输出数据
 */
struct ControlOutputs {
    //--------------------------------------------------------------------------
    // 速度指令输出 (发送到 ros_to_can_node)
    //--------------------------------------------------------------------------
    double V_right_cmd = 0;  ///< 右履带速度指令 (内部单位)
    double V_left_cmd = 0;   ///< 左履带速度指令 (内部单位)
    
    //--------------------------------------------------------------------------
    // 融合速度 (卡尔曼滤波后)
    //--------------------------------------------------------------------------
    double fusion_velocity = 0;  ///< 融合后的车辆速度 (m/s)
    
    //--------------------------------------------------------------------------
    // ENU位置 (相对于主开关开启时的位置)
    //--------------------------------------------------------------------------
    double X_enu = 0;   ///< 东向位置 (米)
    double Y_enu = 0;   ///< 北向位置 (米)
    double UP_enu = 0;  ///< 天向位置 (米)
    
    //--------------------------------------------------------------------------
    // 终止标志
    //--------------------------------------------------------------------------
    double terminal_flag = 0;          ///< 位置到达标志 (0=未到达, 1=到达)
    double terminal_flag_rotation = 0; ///< 角度到达标志 (0=未到达, 1=到达)
    
    //--------------------------------------------------------------------------
    // 规划速度 (规划层输出)
    //--------------------------------------------------------------------------
    double v_x_plan = 0;      ///< X方向规划速度 (m/s)
    double v_theta_plan = 0;  ///< 角速度规划 (deg/s)
    
    //--------------------------------------------------------------------------
    // 参考速度 (运动学层输出)
    //--------------------------------------------------------------------------
    double V_right_reference = 0;  ///< 右履带参考速度
    double V_left_reference = 0;   ///< 左履带参考速度
    
    //--------------------------------------------------------------------------
    // 误差 (PID输入)
    //--------------------------------------------------------------------------
    double error_x = 0;      ///< X方向速度误差
    double error_theta = 0;  ///< 角速度误差
    
    //--------------------------------------------------------------------------
    // 铲刀控制输出 (发送到 /U_Lever_Moldboard 经 ros_to_can)
    //--------------------------------------------------------------------------
    double actual_height_middle_moldboard = 0; ///< 铲刀中心实际高度 (m)
    double e_Height = 0;           ///< 高度误差 (ref - actual, m)
    double e_Theta = 0;            ///< 角度误差 (ref - actual, deg)
    double e_Height_Actual = 0;    ///< 高度误差 (经非线性层+死区处理后)
    double e_Theta_Actual = 0;     ///< 角度误差 (经非线性层+死区处理后)
    double u_Height = 0;           ///< 高度控制量 (P+I, 经Reset和限幅)
    double u_Theta = 0;            ///< 角度控制量 (P+I, 经Reset和限幅)
    double u_Height_Up = 0;        ///< 高度控制-升 (正值有效)
    double u_Height_Dn = 0;        ///< 高度控制-降 (正值有效)
    double u_Theta_Up = 0;         ///< 角度控制-升 (正值有效)
    double u_Theta_Dn = 0;         ///< 角度控制-降 (正值有效)
    double u_Height_Actual = 0;    ///< 高度P项输出 (经非线性层+死区+阀门死区)
    double u_Theta_Actual = 0;     ///< 角度P项输出 (经非线性层+死区+阀门死区)
    double Control_Flag = 0;       ///< 铲刀控制有效标志 (Moldboard_Control_Flag && Main_Switch)
};

//==============================================================================
// 控制节点类
//==============================================================================

/**
 * @brief 控制节点类
 * @details 实现推土机的运动控制功能，包括速度融合、规划、PID控制等
 * 
 * @par 主要功能:
 *      1. 卡尔曼滤波 - 融合IMU和RTK速度
 *      2. 坐标转换 - LLA转ENU，计算相对位置
 *      3. 轨迹规划 - 根据终点计算速度规划
 *      4. 运动学 - 差速转向模型
 *      5. TD滤波 - 平滑参考信号
 *      6. PID控制 - 跟踪参考速度
 * 
 * @par 运行频率: 100Hz
 * 
 * @par ROS接口:
 *      - 订阅: /RTK, /IMU, /decision/walk_state, /decision/main_switch等
 *      - 发布: /control/V_right, /control/V_left, /control/cmd_vel等
 */
class ControlNode {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit ControlNode(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~ControlNode() = default;
    
    // 禁止拷贝和赋值
    ControlNode(const ControlNode&) = delete;
    ControlNode& operator=(const ControlNode&) = delete;
    
    /**
     * @brief 获取输入数据 (常量引用)
     */
    const ControlInputs& getInputs() const { return inputs_; }
    
    /**
     * @brief 获取输出数据 (常量引用)
     */
    const ControlOutputs& getOutputs() const { return outputs_; }
    
    /**
     * @brief 获取输入数据 (可修改引用)
     */
    ControlInputs& getInputs() { return inputs_; }

private:
    //==========================================================================
    // ROS成员
    //==========================================================================
    ros::NodeHandle& nh_;  ///< ROS节点句柄引用
    ros::Timer timer_;     ///< 定时器 (100Hz)
    
    //==========================================================================
    // 输入输出数据
    //==========================================================================
    ControlInputs inputs_;   ///< 输入数据
    ControlOutputs outputs_; ///< 输出数据
    
    //==========================================================================
    // 内部状态变量
    //==========================================================================
    
    // 上一周期值 (用于边沿检测和微分)
    double main_switch_prev_ = 0;          ///< 上一周期主开关状态
    double terminal_flag_prev_ = 0;        ///< 上一周期位置到达标志
    double terminal_flag_rotation_prev_ = 0; ///< 上一周期角度到达标志
    double X_terminal_prev_ = 0;           ///< 上一周期X终点
    double Theta_terminal_prev_ = 0;       ///< 上一周期角度终点
    
    // LLA参考点 (主开关开启时锁定)
    double lat0_ = 0;   ///< 参考纬度 (度)
    double lon0_ = 0;   ///< 参考经度 (度)
    double alt0_ = 0;   ///< 参考高度 (米)
    double yaw0_ = 0;   ///< 参考航向 (度)
    
    // ENU初始位置 (用于轨迹跟踪)
    double x0_ = 0;      ///< 初始X位置 (米)
    double y0_ = 0;      ///< 初始Y位置 (米)
    double theta0_ = 0;  ///< 初始航向 (度)
    
    //==========================================================================
    // PID积分项
    //==========================================================================
    double integral_x_ = 0;       ///< X方向积分累计
    double integral_theta_ = 0;   ///< Theta方向积分累计
    double error_x_prev_ = 0;     ///< 上一周期X误差 (用于微分)
    double error_theta_prev_ = 0; ///< 上一周期Theta误差 (用于微分)
    
    //==========================================================================
    // 跟踪微分器 (TD) 状态
    //==========================================================================
    double td_x1_ = 0;      ///< TD X方向状态1
    double td_x2_ = 0;      ///< TD X方向状态2
    double td_theta1_ = 0;  ///< TD Theta方向状态1
    double td_theta2_ = 0;  ///< TD Theta方向状态2
    
    //==========================================================================
    // 卡尔曼滤波器
    //==========================================================================
    control::KalmanFilter kf_velocity_;  ///< 速度卡尔曼滤波器
    
    //==========================================================================
    // 铲刀控制内部状态
    //==========================================================================
    double moldboard_integral_height_ = 0;   ///< 高度积分项
    double moldboard_integral_theta_ = 0;    ///< 角度积分项
    double ref_height_prev_ = 0;             ///< 上一周期参考高度 (用于积分清零)
    double Ref_Angle_prev_ = 0;              ///< 上一周期参考角度 (用于积分清零)
    
    //==========================================================================
    // ROS订阅者
    //==========================================================================
    ros::Subscriber sub_rtk_;         ///< RTK位置订阅
    ros::Subscriber sub_imu_;         ///< IMU数据订阅
    ros::Subscriber sub_heading_;     ///< 航向角订阅
    ros::Subscriber sub_vehicle_speed_; ///< 车速订阅
    ros::Subscriber sub_walk_state_;  ///< 行走状态订阅
    ros::Subscriber sub_main_switch_; ///< 主开关订阅
    ros::Subscriber sub_terminal_;    ///< 终点参数订阅
    ros::Subscriber sub_reference_;   ///< 参考速度订阅
    ros::Subscriber sub_params_;      ///< 控制参数订阅
    ros::Subscriber sub_moldboard_height_left_;  ///< 铲刀左侧高度订阅
    ros::Subscriber sub_moldboard_height_right_; ///< 铲刀右侧高度订阅
    ros::Subscriber sub_ref_height_;             ///< 铲刀参考高度订阅
    ros::Subscriber sub_ref_angle_;              ///< 铲刀参考角度订阅
    ros::Subscriber sub_imu_roll_;               ///< 铲刀IMU横滚角订阅
    ros::Subscriber sub_moldboard_control_flag_; ///< 铲刀控制标志订阅
    ros::Subscriber sub_moldboard_params_;       ///< 铲刀控制参数订阅
    ros::Subscriber sub_speed_gain_;             ///< 风险减速增益订阅
    
    //==========================================================================
    // ROS发布者
    //==========================================================================
    ros::Publisher pub_cmd_vel_;          ///< 速度指令 (Twist格式)
    ros::Publisher pub_V_right_;          ///< 右履带速度
    ros::Publisher pub_V_left_;           ///< 左履带速度
    ros::Publisher pub_fusion_velocity_;  ///< 融合速度
    ros::Publisher pub_enu_position_;     ///< ENU位置
    ros::Publisher pub_terminal_flag_;    ///< 到达标志
    ros::Publisher pub_error_;            ///< 误差
    ros::Publisher pub_output_;           ///< 综合输出
    ros::Publisher pub_lever_moldboard_;  ///< 铲刀手柄控制
    ros::Publisher pub_terminal_flag_rot_; ///< /control/terminal_flag_rotation [u_Height_Up, u_Height_Dn, u_Theta_Up, u_Theta_Dn]
    ros::Publisher pub_moldboard_debug_;  ///< 铲刀调试信息
    ros::Publisher pub_lever1_;           ///< /U_Lever1 — V_right/V_left → 行走CAN指令
    
    //==========================================================================
    // 初始化函数
    //==========================================================================
    
    /**
     * @brief 初始化ROS订阅者
     */
    void initSubscribers();
    
    /**
     * @brief 初始化ROS发布者
     */
    void initPublishers();
    
    //==========================================================================
    // 定时器回调
    //==========================================================================
    
    /**
     * @brief 定时器回调函数 (100Hz)
     * @param event 定时器事件
     */
    void timerCallback(const ros::TimerEvent& event);
    
    //==========================================================================
    // 控制计算函数
    //==========================================================================
    
    /**
     * @brief 更新初始位置
     * @details 在terminal_flag上升沿时更新ENU初始位置
     */
    void updateInitialPosition();
    
    /**
     * @brief 更新ENU坐标
     * @details LLA转ENU，并进行速度融合
     */
    void updateENU();
    
    /**
     * @brief 更新规划
     * @details 根据行走状态和终点计算速度规划
     */
    void updatePlanning();
    
    /**
     * @brief 更新运动学
     * @details 差速转向模型，计算左右履带参考速度
     */
    void updateKinematics();
    
    /**
     * @brief 更新PID控制
     * @details 计算速度误差并输出控制量
     */
    void updatePID();
    
    /**
     * @brief 更新跟踪微分器
     * @details 平滑参考信号
     */
    void updateTD();
    
    /**
     * @brief 更新铲刀控制
     * @details 对应Simulink Control子系统中的铲刀PID控制环路
     *          计算高度/角度误差 → P控制 → 非线性层 → 死区 → I控制
     *          → 积分衰减 → 限幅 → Reset → Split Up/Dn → 发布
     */
    void updateMoldboardControl();
    
    //==========================================================================
    // 发布函数
    //==========================================================================
    
    /**
     * @brief 发布所有输出
     */
    void publishOutputs();
    
    //==========================================================================
    // ROS回调函数
    //==========================================================================
    
    /**
     * @brief RTK数据回调
     * @param msg NavSatFix消息
     */
    void rtkCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    
    /**
     * @brief IMU数据回调
     * @param msg Imu消息
     */
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
    /**
     * @brief 行走状态回调
     * @param msg Float64消息
     */
    void walkStateCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 主开关回调
     * @param msg Float64消息
     */
    void mainSwitchCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 终点参数回调
     * @param msg Float64MultiArray消息 [X_terminal, Y_terminal, Theta_terminal]
     */
    void terminalCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    
    /**
     * @brief 参考速度回调
     * @param msg Float64MultiArray消息 [v_reference, theta_velocity_reference]
     */
    void referenceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    
    /**
     * @brief 控制参数回调
     * @param msg Float64MultiArray消息 [PID参数, TD参数等]
     */
    void paramsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    
    /**
     * @brief 铲刀左侧高度回调
     * @param msg Float64消息 (米)
     */
    void moldboardHeightLeftCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 铲刀右侧高度回调
     * @param msg Float64消息 (米)
     */
    void moldboardHeightRightCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 铲刀参考高度回调
     * @param msg Float64消息 (米)
     */
    void refHeightCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 铲刀参考角度回调
     * @param msg Float64消息 (度)
     */
    void refAngleCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 铲刀IMU横滚角回调
     * @param msg Float64消息 (度)
     */
    void imuRollCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 铲刀控制标志回调
     * @param msg Float64消息 (0=关闭, 1=开启)
     */
    void moldboardControlFlagCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 铲刀控制参数回调
     * @param msg Float64MultiArray消息
     */
    void moldboardParamsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    //==========================================================================
    // 风险减速增益
    //==========================================================================
    double speed_gain_ = 1.0;  ///< 来自 /control_speed_gain, 风险减速时 <1
};

#endif // CONTROL_NODE_H
