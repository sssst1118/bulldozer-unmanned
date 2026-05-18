/**
 * @file decision_system.h
 * @brief 推土机决策系统 — 路径生成 + 通用执行器 + 辅助模块
 * @author dozer-dev
 * @date 2026-03-15
 *
 * 架构:
 *   界面/外部 → PathMode + PathGenParams
 *       ↓
 *   路径生成器 → vector<WayPoint>
 *       ↓
 *   通用路径执行器 (状态机: IDLE→ROTATING→DRIVING→WAYPOINT_DONE→...→FINISHED)
 *       ↓
 *   control_node (walk_state + terminal + reference + blade_cmd)
 *
 * 辅助模块 (保留):
 *   - 过载检测: 检测堵转工况
 *   - 决策逻辑: 风险/速度增益/蜂鸣
 *   - 铲刀高度计算: RTK杆臂→铲刀端点坐标
 */
#ifndef DECISION_SYSTEM_H
#define DECISION_SYSTEM_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include "path_types.h"
#include "path_generator.h"
#include "simulink_functions.h"

#include <vector>
#include <cmath>

//==============================================================================
// 顶层状态 — 遥控接管 vs 自动作业
//==============================================================================
enum class TopState {
    REMOTE_TAKEOVER = 0,  ///< 遥控接管 (安全态, 不输出任何指令)
    AUTO_OPERATION  = 1   ///< 自动作业 (执行器运行)
};

//==============================================================================
// 辅助模块的输入/输出结构 (从原代码保留, 略作精简)
//==============================================================================

/** @brief 决策逻辑输入 */
struct DecisionLogicInputs {
    double risk_state[4] = {};
    double slow_flag = 0;
    int Direction = 1;
    double mold_state = 0;
    double mold_height_d = 0;
    double control_speed_gain_qt_risk = 0.5;
    double control_speed_gain_qt_mold = 0.7;
    double mold_limit = 0.1;
    int16_t Uneven_Flag_Plan = 0;
};

/** @brief 决策逻辑输出 */
struct DecisionLogicOutputs {
    double Decision_Status = 0;
    double buzz_flag = 0;
    double control_speed_gain = 1.0;
    double Moldboard_Control_Flag = 0;
    int16_t Uneven_Flag = 0;
};

/** @brief 过载检测输入 */
struct OverloadDetectInputs {
    int16_t Walking_State = 0;
    double Trans_Speed = 0;
    double Vehicle_Speed = 0;
    double Angular_Speed = 0;
    double Trans_Speed_Limit = 1000;
    double Vehicle_Speed_Limit = 0.5;
    double Angular_Speed_Limit = 10;
    int time_num = 50;
};

/** @brief 过载检测输出 */
struct OverloadDetectOutputs {
    int16_t overload_status = 0;
    int overload_counter = 0;
};

/** @brief 铲刀高度计算输入 */
struct MoldboardHeightInputs {
    double latitude = 0, longitude = 0, altitude = 0;
    double vehicle_yaw = 0, vehicle_pitch = 0, vehicle_roll = 0;
    double imu_pitch = 0, imu_roll = 0;
};

/** @brief 铲刀高度计算输出 */
struct MoldboardHeightOutputs {
    double PointD_lla[3] = {};   ///< 铲刀右端 LLA
    double PointM_lla[3] = {};   ///< 铲刀左端 LLA
    double Avg_Height_Actual = 0;
};

//==============================================================================
// 决策系统类
//==============================================================================
class DecisionSystem {
public:
    explicit DecisionSystem(ros::NodeHandle& nh);
    ~DecisionSystem() = default;

private:
    //==========================================================================
    // 定时回调 — 主循环 100Hz
    //==========================================================================
    void timerCallback(const ros::TimerEvent& event);

    //==========================================================================
    // 顶层状态机: REMOTE_TAKEOVER ↔ AUTO_OPERATION
    //==========================================================================
    void processTopState();

    //==========================================================================
    // 路径生成 — 收到地图或界面指令时触发 (非每帧)
    //==========================================================================
    void generateNewPath();

    //==========================================================================
    // 通用路径执行器 — 每帧运行
    //==========================================================================
    void processPathExecutor();
    void execIdle();
    void execRotating();
    void execDriving();
    void execWaypointDone();
    void execOverloadBack();  ///< 过载处理: 提刀后退, 完成后回到DRIVING继续推

    /// 计算当前位置到目标点的航向角 (度)
    double calcTargetHeading(const WayPoint& wp) const;
    /// 计算当前位置到目标点的距离 (米)
    double calcTargetDistance(const WayPoint& wp) const;
    /// 发送 control_node 指令
    void sendWalkCommand(int walk_state, double x_terminal, double theta_terminal,
                         double v_ref, double omega_ref);
    /// 驶向目标 (含减速+航向纠偏, 持续调用)
    void sendDriveToTarget(const WayPoint& wp, double dist, double v_max);

    /// 发送铲刀指令 (简化接口, RAISE 用 RAISE_HEIGHT, LEVEL 用 target_level_height, 3D=on)
    /// 用于紧急停车/异常等不依赖具体 waypoint 的场景。
    void sendBladeCommand(BladeCmd cmd);

    /// 发送铲刀指令 (从 waypoint 取 target_height, 3D=on)
    /// [3D-Level] 执行器每帧按当前 WayPoint 下发规划目标, 下游 moldboard_controller
    /// 依据 target_height / blade_angle_deg_ / mode_switch_3d 算左右目标高程。
    void sendBladeCommand(const WayPoint& wp);

    //==========================================================================
    // 辅助模块 (保留, 每帧运行)
    //==========================================================================
    void processDecisionLogic();
    void processOverloadDetect();
    void processMoldboardHeight();

    //==========================================================================
    // 发布输出
    //==========================================================================
    void publishOutputs();

    //==========================================================================
    // 初始化
    //==========================================================================
    void initSubscribers();
    void initPublishers();

    //==========================================================================
    // ROS 回调
    //==========================================================================
    void mainSwitchCallback(const std_msgs::Float64::ConstPtr& msg);
    void detectionCompletedCallback(const std_msgs::Float64::ConstPtr& msg);
    void rtkCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void llaCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void angleHeadingCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void ahrsImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void occupancyLocationCallback(const geometry_msgs::Point::ConstPtr& msg);
    void navigationCoordinateCallback(const std_msgs::Float64::ConstPtr& msg);
    // [Issue#4] 注释掉 — /Location_real 当前无发布者, 保留以备恢复
    // void locationRealCallback(const geometry_msgs::Point::ConstPtr& msg);
    void riskStateCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
    void moldOverloadCallback(const std_msgs::Int16::ConstPtr& msg);
    // 路径模式回调 (从界面设置)
    void pathModeCallback(const std_msgs::Float64::ConstPtr& msg);

    /// 发布栅格坐标路径给地图可视化
    void publishGridPath();

    /// 无地图时: 从 ENU path_ 合成虚拟栅格坐标, 发布虚拟地图+路径
    void synthesizeVirtualGrid();

    /// 栅格坐标路径 (row, col, dir) — 和 ENU path_ 一一对应
    struct GridPt { double x, y, dir; };  // x=row, y=col, dir=1/-1
    std::vector<GridPt> grid_path_;

    //==========================================================================
    // ROS 成员
    //==========================================================================
    ros::NodeHandle& nh_;
    ros::Timer timer_;  ///< 100Hz

    // 订阅者
    ros::Subscriber sub_main_switch_, sub_detection_, sub_rtk_, sub_lla_;
    ros::Subscriber sub_angle_, sub_imu_, sub_grid_, sub_grid_loc_;
    ros::Subscriber sub_nav_coord_;
    // [Issue#4] ros::Subscriber sub_loc_real_;  // 注释掉, 保留以备恢复
    ros::Subscriber sub_risk_, sub_overload_;
    ros::Subscriber sub_vehicle_speed_, sub_trans_speed_;  ///< 过载检测输入
    ros::Subscriber sub_path_mode_;
    ros::Subscriber sub_test_config_;    ///< 测试模式配置
    ros::Subscriber sub_test_script_;    ///< 测试脚本
    ros::Subscriber sub_regen_path_;     ///< 手动触发路径生成
    ros::Subscriber sub_path_params_;    ///< 运行时路径参数更新
    ros::Subscriber sub_emergency_stop_; ///< 紧急停止
    ros::Subscriber sub_set_blade_origin_; ///< [Issue#7] GUI设置铲刀基准点

    // 发布者 — 直接对应 control_node 的订阅话题
    ros::Publisher pub_walk_state_;          ///< /decision/walk_state
    ros::Publisher pub_main_switch_out_;     ///< /decision/main_switch (给control)
    ros::Publisher pub_terminal_;            ///< /control/terminal [X, Y, Theta]
    ros::Publisher pub_reference_;           ///< /control/reference [v, omega]
    ros::Publisher pub_mold_ctrl_flag_;      ///< /decision/moldboard_control_flag
    ros::Publisher pub_ref_height_;          ///< /decision/ref_height_middle_moldboard
    ros::Publisher pub_ref_angle_;           ///< /decision/ref_angle
    // 状态/调试输出
    ros::Publisher pub_decision_status_;     ///< /Decision_Status
    ros::Publisher pub_exec_state_;          ///< /decision/exec_state (新增)
    ros::Publisher pub_waypoint_index_;      ///< /decision/waypoint_index (新增)
    ros::Publisher pub_path_viz_;            ///< /Path (兼容地图可视化)
    ros::Publisher pub_direction_;           ///< /Direction
    ros::Publisher pub_buzz_flag_;           ///< /buzz_flag
    ros::Publisher pub_speed_gain_;          ///< /control_speed_gain
    ros::Publisher pub_point_d_;             ///< /PointD_lla
    ros::Publisher pub_point_m_;             ///< /PointM_lla
    ros::Publisher pub_avg_height_;          ///< /decision/avg_height_actual
    ros::Publisher pub_full_path_;           ///< /decision/full_path (规划路径点序列)
    ros::Publisher pub_virtual_grid_;        ///< /occupancy_grid (无地图时发布虚拟地图)
    ros::Publisher pub_blade_enable_;        ///< /Moldboard_Control_Flag_Plan (自动模式铲刀硬件使能)
    ros::Publisher pub_blade_origin_;        ///< [Issue#7] /decision/moldboard_origin (铲刀高度基准点)
    // [3D-Level] 铲刀规划下发 (绑定到路径点的 target_height)
    ros::Publisher pub_avg_height_plan_;       ///< /decision/avg_height_plan (LEVEL 段目标高程, 给 moldboard_controller)
    ros::Publisher pub_ref_mold_theta_plan_;   ///< /decision/ref_mold_theta_plan (铲刀横滚角, 给 moldboard_controller)
    ros::Publisher pub_mode_switch_3d_in_;     ///< /Mode_Switch_3D_Input (3D找平模式标志, 给 moldboard_controller)

    //==========================================================================
    // 顶层状态
    //==========================================================================
    TopState top_state_ = TopState::REMOTE_TAKEOVER;
    double main_switch_ = 0;
    double detection_completed_ = 0;
    int rtk_status_ = 1;  ///< 当前固定为1, 预留

    // [Fix-BUG-G] 感知就绪超时: 感知节点必须持续发布, 否则自动降级
    ros::Time detection_last_time_;
    static constexpr double DETECTION_TIMEOUT_SEC = 3.0;  ///< 3秒没收到 → 感知离线

    //==========================================================================
    // 路径
    //==========================================================================
    PathMode path_mode_ = PathMode::ZIGZAG;
    PathGenParams path_params_;
    std::vector<WayPoint> path_;       ///< 当前路径点队列
    int waypoint_index_ = 0;           ///< 当前执行到第几个点
    bool path_ready_ = false;          ///< 路径是否已生成
    bool map_received_ = false;        ///< 是否收到过地图

    // 测试模式
    bool test_mode_ = false;
    double test_length_ = 10.0;        ///< 测试推土长度(m)
    double test_width_ = 10.0;         ///< 测试推土宽度(m)
    int test_passes_ = 3;              ///< 测试遍数(0=自动,-1=无限)
    int test_start_pos_ = 0;           ///< 起点: 0左下 1右下 2左上 3右上 4中心

    //==========================================================================
    // 执行器
    //==========================================================================
    ExecState exec_state_ = ExecState::IDLE;
    double theta_tolerance_ = 5.0;     ///< 旋转到位角度容差 (度)
    double position_tolerance_ = 0.3;  ///< 直行到位距离容差 (米)

    // 过载恢复
    WayPoint overload_saved_wp_;         ///< 过载时保存的原目标点
    int overload_retry_count_ = 0;       ///< 当前路径点的过载重试次数
    double overload_back_start_x_ = 0;   ///< 过载后退起始 ENU_X
    double overload_back_start_y_ = 0;   ///< 过载后退起始 ENU_Y
    static constexpr int MAX_OVERLOAD_RETRY = 3;  ///< 最大重试次数

    // 障碍物暂停
    bool obstacle_paused_ = false;       ///< risk>=2时停车, risk<2时恢复

    //==========================================================================
    // 当前位姿 (来自传感器)
    //==========================================================================
    double enu_x_ = 0, enu_y_ = 0;     ///< 当前 ENU 位置 (相对于作业起点)
    double heading_deg_ = 0;            ///< 当前航向 (度)
    double heading_rad_ = 0;            ///< 当前航向 (弧度)

    // /Navigate_location 优先级机制: 有地图场景下优先使用感知ENU, 超时回退到LLA自算
    bool use_nav_loc_enu_ = false;
    ros::Time nav_loc_last_time_;
    static constexpr double NAV_LOC_TIMEOUT_SEC = 0.5;

    // LLA 参考点 (主开关开启时锁定)
    double lat0_ = 0, lon0_ = 0, alt0_ = 0;
    double heading0_ = 0;
    bool origin_locked_ = false;

    //==========================================================================
    // 辅助模块数据
    //==========================================================================
    DecisionLogicInputs decision_logic_inputs_;
    DecisionLogicOutputs decision_logic_outputs_;
    OverloadDetectInputs overload_inputs_;
    OverloadDetectOutputs overload_outputs_;
    MoldboardHeightInputs moldboard_height_inputs_;
    MoldboardHeightOutputs moldboard_height_outputs_;

    // 地图数据 (保留, 用于将来地图感知规划)
    int map_rows_ = 0, map_cols_ = 0;
    double map_resolution_ = 0;
    double map_origin_x_ = 0, map_origin_y_ = 0;  ///< 栅格(0,0)对应的ENU坐标
    std::vector<int8_t> map_data_;
    int cur_row_ = 0, cur_col_ = 0;

    // 有地图模式参数
    int map_start_corner_ = 0;          ///< 起点: 0=左下 1=右下 2=左上 3=右上
    double map_push_heading_ = 0;       ///< 推土方向 (度, 0=北/row增大方向)

    // IMU
    double imu_roll_ = 0;

    // [Issue#7] 铲刀高度基准点 (默认=INIT_ORIGIN, 运行时可通过GUI设置)
    double blade_origin_lat_ = simulink::INIT_ORIGIN[0];
    double blade_origin_lon_ = simulink::INIT_ORIGIN[1];
    double blade_origin_alt_ = simulink::INIT_ORIGIN[2];

    // [3D-Level] 铲刀横滚角 (度, 全路径共用一个, 从 /decision/path_params 读入)
    // 将来若改成"每个点独立" 可挪进 WayPoint。暂时放这里因为是路径级全局参数。
    double blade_angle_deg_ = 0.0;

    // [SLOPE_3D] 纵坡实时追踪: 记录推土段起点位置, 用于计算行驶距离
    double slope_push_start_x_ = 0.0;  ///< 推土段起点 ENU X
    double slope_push_start_y_ = 0.0;  ///< 推土段起点 ENU Y

    // 计数器
    int loop_count_ = 0;
};

#endif // DECISION_SYSTEM_H
