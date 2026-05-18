/**
 * @file moldboard_controller.h
 * @brief 铲刀控制器头文件
 * @details 该文件定义了推土机铲刀控制模块，实现以下功能：
 *          - 铲刀位姿计算 (基于RTK和IMU)
 *          - 铲刀高度控制 (左右独立控制)
 *          - 3D找平控制
 *          - 过载保护
 * 
 * @version V1.0
 * @date 2026-03-15
 * @author dozer-dev
 * 
 * @par 坐标系说明:
 *      - LLA: WGS84经纬高坐标 (lat, lon, alt)
 *      - ENU: 东北天坐标 (East, North, Up)
 *      - 车体坐标系: X前进，Y左，Z上
 * 
 * @par 铲刀几何参数 (杆臂值):
 *      - PointA: RTK天线位置
 *      - PointB: 右大臂铰接点
 *      - PointD: 铲刀右端点
 *      - PointM: 铲刀左端点 (中点)
 * 
 * @par 运行频率: 50Hz
 */

#ifndef MOLDBOARD_CONTROLLER_H
#define MOLDBOARD_CONTROLLER_H

//==============================================================================
// 系统头文件
//==============================================================================
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>   // [Fix-v18] /OUTPUT_CURRENT 类型对齐
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>              // [Fix-v18] /AHRS_IMU 类型对齐
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <array>

#include "simulink_functions.h"  // [Issue#7] for simulink::Vec3, INIT_ORIGIN

//==============================================================================
// 铲刀控制输入数据结构
//==============================================================================

/**
 * @brief 铲刀控制输入数据结构
 * @details 包含铲刀控制所需的所有传感器数据和规划数据
 */
struct MoldboardInputs {
    //--------------------------------------------------------------------------
    // RTK位置数据 (PointA_lla)
    // 来自话题: /LLA
    //--------------------------------------------------------------------------
    double rtk_latitude = 0;   ///< RTK纬度 (度，WGS84)
    double rtk_longitude = 0;  ///< RTK经度 (度，WGS84)
    double rtk_altitude = 0;   ///< RTK高度 (米，椭球高)
    
    //--------------------------------------------------------------------------
    // RTK车体姿态角 (vehicle_angle)
    // 来自话题: /Angle_Heading
    // 由RTK双天线或组合导航提供
    //--------------------------------------------------------------------------
    double rtk_yaw = 0;    ///< 车体航向角 (度，北偏东为正，范围0~360)
    double rtk_pitch = 0;  ///< 车体俯仰角 (度，抬头为正，范围±90)
    double rtk_roll = 0;   ///< 车体横滚角 (度，右倾为正，范围±180)
    
    //--------------------------------------------------------------------------
    // IMU铲刀姿态角 (imu_angle)
    // 来自话题: /AHRS_IMU
    // 安装在铲刀上的IMU，用于测量铲刀相对于车体的姿态
    //--------------------------------------------------------------------------
    double imu_yaw = 0;    ///< 铲刀IMU航向角 (度)
    double imu_pitch = 0;  ///< 铲刀IMU俯仰角 (度)
    double imu_roll = 0;   ///< 铲刀IMU横滚角 (度，用于计算铲刀倾斜)
    
    //--------------------------------------------------------------------------
    // 规划数据 (来自decision_node)
    //--------------------------------------------------------------------------
    double avg_height_plan = 0;        ///< 规划的平均高度 (米，ENU坐标系)
    double ref_mold_theta_plan = 0;    ///< 规划的铲刀角度 (度，横滚方向)
    double moldboard_control_flag = 0; ///< 铲刀控制标志 (0=禁止控制, 1=允许控制)
    
    //--------------------------------------------------------------------------
    // 3D找平模式开关
    // 来自话题: /Mode_Switch_3D_Input
    //--------------------------------------------------------------------------
    double mode_switch_3d = 0;  ///< 3D找平模式 (0=关闭, 1=开启)
    
    //--------------------------------------------------------------------------
    // 输出电流 (用于过载检测)
    // 来自话题: /OUTPUT_CURRENT
    // 4路液压阀电流
    //--------------------------------------------------------------------------
    std::array<double, 4> output_current = {0, 0, 0, 0}; ///< 输出电流 [ch0, ch1, ch2, ch3] (安培)
};

//==============================================================================
// 铲刀控制输出数据结构
//==============================================================================

/**
 * @brief 铲刀控制输出数据结构
 * @details 包含铲刀控制的所有输出数据，发送到CAN总线和决策模块
 */
struct MoldboardOutputs {
    //--------------------------------------------------------------------------
    // 铲刀实际高度 (ENU坐标系，单位: 米)
    // 由RTK+IMU计算得出
    //--------------------------------------------------------------------------
    double actual_height_left_moldboard_enu = 0;   ///< 左铲刀端点实际高度 (米)
    double actual_height_right_moldboard_enu = 0;  ///< 右铲刀端点实际高度 (米)
    
    //--------------------------------------------------------------------------
    // 铲刀端点位置 (LLA坐标，用于可视化)
    //--------------------------------------------------------------------------
    std::array<double, 3> PointD_lla = {0, 0, 0};  ///< 铲刀右端点位置 [lat(度), lon(度), alt(米)]
    std::array<double, 3> PointM_lla = {0, 0, 0};  ///< 铲刀左端点位置 [lat(度), lon(度), alt(米)]
    std::array<double, 3> PointB_lla = {0, 0, 0};  ///< 右大臂铰接点位置 [lat(度), lon(度), alt(米)]
    
    //--------------------------------------------------------------------------
    // 高度阈值 (ENU坐标系，用于安全限制)
    //--------------------------------------------------------------------------
    double PointD_enu_maxThreshold_height = 0;  ///< 右端点最大高度阈值 (米)
    double PointD_enu_minThreshold_height = 0;  ///< 右端点最小高度阈值 (米)
    double PointM_enu_maxThreshold_height = 0;  ///< 左端点最大高度阈值 (米)
    double PointM_enu_minThreshold_height = 0;  ///< 左端点最小高度阈值 (米)
    
    //--------------------------------------------------------------------------
    // 铲刀目标高程 (单位: 毫米，发送到CAN)
    // 话题: /Bulldozer_Moldboard_Target
    //--------------------------------------------------------------------------
    int16_t target_height_right = 0;  ///< 右铲刀目标高度 (mm)
    int16_t target_height_left = 0;   ///< 左铲刀目标高度 (mm)
    
    //--------------------------------------------------------------------------
    // 铲刀实际高程 (单位: 毫米，发送到CAN)
    // 话题: /Bulldozer_Moldboard_Actual
    //--------------------------------------------------------------------------
    int16_t actual_height_right = 0;  ///< 右铲刀实际高度 (mm)
    int16_t actual_height_left = 0;   ///< 左铲刀实际高度 (mm)
    
    //--------------------------------------------------------------------------
    // 铲刀高程差 (单位: 毫米，目标-实际)
    // 话题: /Bulldozer_Moldboard_Right_Dvalue, /Bulldozer_Moldboard_Left_Dvalue
    //--------------------------------------------------------------------------
    int16_t height_diff_right = 0;  ///< 右铲刀高度差 (mm，正=需要下降)
    int16_t height_diff_left = 0;   ///< 左铲刀高度差 (mm，正=需要下降)
    
    //--------------------------------------------------------------------------
    // 反馈到决策模块
    //--------------------------------------------------------------------------
    double avg_height_actual = 0;  ///< 平均实际高度 (米)，发布到 /decision/avg_height_actual
    double overload_flag = 0;      ///< 过载标志 (0=正常, 1=过载)，发布到 /decision/overload_flag
};

//==============================================================================
// 铲刀控制器类
//==============================================================================

/**
 * @brief 铲刀控制器类
 * @details 实现铲刀位姿计算和高度控制功能
 * 
 * @par 主要功能:
 *      1. 计算铲刀端点在ENU坐标系下的实际高度
 *      2. 根据规划高度和角度计算目标高度
 *      3. 计算高度差并发送到CAN总线
 *      4. 过载检测和保护
 * 
 * @par 计算流程:
 *      RTK(LLA) + 车体姿态 + 铲刀IMU → 杆臂变换 → 铲刀端点ENU坐标 → 高度控制
 * 
 * @par 运行频率: 50Hz (铲刀控制不需要太高频率)
 */
class MoldboardController {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit MoldboardController(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~MoldboardController() = default;
    
    /**
     * @brief 控制更新（在定时器回调中调用）
     * @details 执行一次完整的铲刀控制计算
     */
    void update();
    
    /**
     * @brief 获取输出数据
     * @return 输出数据结构的常量引用
     */
    const MoldboardOutputs& getOutputs() const { return outputs_; }
    
    /**
     * @brief 设置输入数据
     * @param inputs 输入数据结构
     */
    void setInputs(const MoldboardInputs& inputs) { inputs_ = inputs; }
    
    /**
     * @brief 获取输入数据引用（用于直接修改）
     * @return 输入数据结构的引用
     */
    MoldboardInputs& getInputsRef() { return inputs_; }

private:
    //==========================================================================
    // ROS成员
    //==========================================================================
    ros::NodeHandle nh_;       ///< ROS节点句柄
    ros::Timer update_timer_;  ///< 定时器 (50Hz)
    
    //==========================================================================
    // 输入输出数据
    //==========================================================================
    MoldboardInputs inputs_;   ///< 输入数据
    MoldboardOutputs outputs_; ///< 输出数据
    
    //==========================================================================
    // 常量参数
    //==========================================================================
    
    /** @brief 过载电流阈值 (安培) */
    static constexpr double OVERLOAD_CURRENT_THRESHOLD = 100.0;
    
    //==========================================================================
    // ROS发布者
    //==========================================================================
    
    // 发送到CAN (通过ros_to_can_node)
    ros::Publisher pub_moldboard_target_;      ///< 铲刀目标高度 [right, left] (mm)
    ros::Publisher pub_moldboard_actual_;      ///< 铲刀实际高度 [right, left] (mm)
    ros::Publisher pub_moldboard_right_dval_;  ///< 右铲刀高度差 (mm)
    ros::Publisher pub_moldboard_left_dval_;   ///< 左铲刀高度差 (mm)
    ros::Publisher pub_mode_switch_3d_;        ///< 3D找平模式开关
    
    // 发送到决策模块
    ros::Publisher pub_avg_height_actual_;     ///< 平均实际高度 (m)
    ros::Publisher pub_overload_flag_;         ///< 过载标志
    
    // 发送到控制模块 (Float64, 单位米)
    ros::Publisher pub_height_left_float_;     ///< 左铲刀端点ENU高度 (m)
    ros::Publisher pub_height_right_float_;    ///< 右铲刀端点ENU高度 (m)
    
    // [Fix-v19] 发送铲刀IMU横滚角 (度) 给 control_node 的铲刀角度PID 用
    ros::Publisher pub_imu_roll_;              ///< /moldboard/imu_roll (度, 绝对横滚)
    
    // 发送位置信息 (用于可视化)
    ros::Publisher pub_point_b_lla_;           ///< 铰接点B位置 (LLA)
    ros::Publisher pub_point_d_lla_;           ///< 右端点D位置 (LLA)
    ros::Publisher pub_point_m_lla_;           ///< 左端点M位置 (LLA)
    
    //==========================================================================
    // ROS订阅者
    //==========================================================================
    ros::Subscriber sub_rtk_;                  ///< RTK位置 /LLA
    ros::Subscriber sub_rtk_angle_;            ///< RTK姿态 /Angle_Heading
    ros::Subscriber sub_imu_angle_;            ///< IMU姿态 /AHRS_IMU
    ros::Subscriber sub_avg_height_plan_;      ///< 规划高度 /decision/avg_height_plan
    ros::Subscriber sub_ref_mold_theta_plan_;  ///< 规划角度 /decision/ref_mold_theta_plan
    ros::Subscriber sub_moldboard_control_flag_; ///< 控制标志 /decision/moldboard_control_flag
    ros::Subscriber sub_mode_switch_3d_;       ///< 3D模式开关 /Mode_Switch_3D_Input
    ros::Subscriber sub_output_current_;       ///< 输出电流 /OUTPUT_CURRENT
    ros::Subscriber sub_blade_origin_;         ///< [Issue#7] 铲刀高度基准点 /decision/moldboard_origin
    
    //==========================================================================
    // ROS回调函数
    //==========================================================================
    
    /**
     * @brief RTK位置回调
     * @param msg PointStamped消息 (x=lat, y=lon, z=alt)
     */
    void rtkCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    
    /**
     * @brief RTK姿态回调
     * @param msg PointStamped消息 (x=yaw, y=pitch, z=roll)
     */
    void rtkAngleCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    
    /**
     * @brief IMU姿态回调
     * @param msg Imu消息 (欧拉角塞在 orientation_covariance[0..2] = roll,pitch,yaw, 单位:度)
     * [Fix-v18] 原本签名误写为 PointStamped, 与 can_to_ros 发布的 sensor_msgs/Imu 不匹配,
     *           订阅建不起来,回调永远不触发。改为 sensor_msgs::Imu 对齐。
     */
    void imuAngleCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
    /**
     * @brief 规划高度回调
     * @param msg Float64消息
     */
    void avgHeightPlanCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 规划角度回调
     * @param msg Float64消息
     */
    void refMoldThetaPlanCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 铲刀控制标志回调
     * @param msg Float64消息
     */
    void moldboardControlFlagCallback(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 3D模式开关回调
     * @param msg Int16消息
     */
    void modeSwitchCallback(const std_msgs::Int16::ConstPtr& msg);
    
    /**
     * @brief 输出电流回调
     * @param msg UInt16MultiArray消息
     * [Fix-v18] can_to_ros 发布的是 UInt16MultiArray, 原 Float64MultiArray 不匹配。
     */
    void outputCurrentCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg);
    
    /**
     * @brief [Issue#7] 铲刀高度基准点回调
     * @param msg Point消息 (x=lat, y=lon, z=alt)
     */
    void bladeOriginCallback(const geometry_msgs::Point::ConstPtr& msg);
    
    //==========================================================================
    // 铲刀高度基准点 [Issue#7]
    // 默认值 = INIT_ORIGIN (标定场地), 运行时可通过话题更新
    //==========================================================================
    simulink::Vec3 work_origin_ = {
        simulink::INIT_ORIGIN[0],
        simulink::INIT_ORIGIN[1],
        simulink::INIT_ORIGIN[2]
    };
    
    //==========================================================================
    // 初始化函数
    //==========================================================================
    
    /**
     * @brief 初始化ROS发布者
     */
    void initPublishers();
    
    /**
     * @brief 初始化ROS订阅者
     */
    void initSubscribers();
    
    //==========================================================================
    // 定时器回调
    //==========================================================================
    
    /**
     * @brief 定时器回调函数 (50Hz)
     * @param event 定时器事件
     */
    void timerCallback(const ros::TimerEvent& event);
    
    //==========================================================================
    // 控制算法函数
    //==========================================================================
    
    /**
     * @brief 计算铲刀实际高度
     * @details 基于RTK位置、车体姿态和铲刀IMU，通过杆臂变换计算铲刀端点的ENU高度
     * 
     * @par 计算步骤:
     *      1. LLA转ENU得到RTK天线位置
     *      2. 计算车体到铲刀的旋转矩阵
     *      3. 应用杆臂值得到铲刀端点位置
     *      4. 提取高度分量
     */
    void computeActualHeight();
    
    /**
     * @brief 计算铲刀目标高度
     * @details 根据规划的平均高度和铲刀角度，计算左右铲刀的目标高度
     * 
     * @par 计算公式:
     *      height_offset = (width/2) * sin(ref_mold_theta)
     *      target_right = avg_height_plan - height_offset
     *      target_left = avg_height_plan + height_offset
     */
    void computeTargetHeight();
    
    /**
     * @brief 计算高度差
     * @details 计算目标高度与实际高度的差值，用于液压控制
     */
    void computeHeightDiff();
    
    /**
     * @brief 检查过载状态
     * @details 检测液压系统电流是否超过阈值
     */
    void checkOverload();
    
    //==========================================================================
    // 发布函数
    //==========================================================================
    
    /**
     * @brief 发布所有输出
     * @details 将计算结果发布到对应的ROS话题
     */
    void publishOutputs();
};

#endif // MOLDBOARD_CONTROLLER_H
