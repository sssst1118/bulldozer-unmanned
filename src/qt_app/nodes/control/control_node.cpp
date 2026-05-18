/**
 * @file control_node.cpp
 * @brief 推土机运动控制节点实现
 * @author dozer-dev
 * @date 2026-03-15
 */

#include "control_node.h"
#include "qt_app/log_helper.h"
#include <cmath>  // [Fix-v19] std::abs

//==============================================================================
// 构造函数和初始化
//==============================================================================

ControlNode::ControlNode(ros::NodeHandle& nh) : nh_(nh) {
    initSubscribers();
    initPublishers();
    
    // 启动100Hz定时器
    timer_ = nh_.createTimer(ros::Duration(0.01), &ControlNode::timerCallback, this);
    
    LOG_INFO("ControlNode initialized at 100Hz");
}

void ControlNode::initSubscribers() {
    sub_rtk_ = nh_.subscribe("/RTK", 1, &ControlNode::rtkCallback, this);
    sub_imu_ = nh_.subscribe("/AHRS_IMU", 1, &ControlNode::imuCallback, this);
    sub_heading_ = nh_.subscribe<geometry_msgs::PointStamped>("/Angle_Heading", 1,
        boost::function<void(const geometry_msgs::PointStamped::ConstPtr&)>(
            [this](const geometry_msgs::PointStamped::ConstPtr& msg) {
                inputs_.rtk_yaw = msg->point.x * 180.0 / M_PI;  // rad → deg
            }));
    sub_vehicle_speed_ = nh_.subscribe<std_msgs::Float64>("/Vehicle_Speed_Vel", 1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>(
            [this](const std_msgs::Float64::ConstPtr& msg) {
                inputs_.imu_velocity = msg->data;  // 实际车速 m/s
            }));
    sub_walk_state_ = nh_.subscribe("/decision/walk_state", 1, 
        &ControlNode::walkStateCallback, this);
    sub_main_switch_ = nh_.subscribe("/decision/main_switch", 1,
        &ControlNode::mainSwitchCallback, this);
    sub_terminal_ = nh_.subscribe("/control/terminal", 1,
        &ControlNode::terminalCallback, this);
    sub_reference_ = nh_.subscribe("/control/reference", 1,
        &ControlNode::referenceCallback, this);
    sub_params_ = nh_.subscribe("/control/params", 1,
        &ControlNode::paramsCallback, this);
    sub_moldboard_height_left_ = nh_.subscribe("/moldboard/actual_height_left", 1,
        &ControlNode::moldboardHeightLeftCallback, this);
    sub_moldboard_height_right_ = nh_.subscribe("/moldboard/actual_height_right", 1,
        &ControlNode::moldboardHeightRightCallback, this);
    sub_ref_height_ = nh_.subscribe("/decision/ref_height_middle_moldboard", 1,
        &ControlNode::refHeightCallback, this);
    sub_ref_angle_ = nh_.subscribe("/decision/ref_angle", 1,
        &ControlNode::refAngleCallback, this);
    sub_imu_roll_ = nh_.subscribe("/moldboard/imu_roll", 1,
        &ControlNode::imuRollCallback, this);
    sub_moldboard_control_flag_ = nh_.subscribe("/decision/moldboard_control_flag", 1,
        &ControlNode::moldboardControlFlagCallback, this);
    sub_moldboard_params_ = nh_.subscribe("/control/moldboard_params", 1,
        &ControlNode::moldboardParamsCallback, this);
    sub_speed_gain_ = nh_.subscribe<std_msgs::Float64>("/control_speed_gain", 1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>(
            [this](const std_msgs::Float64::ConstPtr& msg) { speed_gain_ = msg->data; }));
}

void ControlNode::initPublishers() {
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/control/cmd_vel", 1);
    pub_V_right_ = nh_.advertise<std_msgs::Float64>("/control/V_right", 1);
    pub_V_left_ = nh_.advertise<std_msgs::Float64>("/control/V_left", 1);
    pub_fusion_velocity_ = nh_.advertise<std_msgs::Float64>("/control/fusion_velocity", 1);
    pub_enu_position_ = nh_.advertise<geometry_msgs::Point>("/control/enu_position", 1);
    pub_terminal_flag_ = nh_.advertise<std_msgs::Float64>("/control/terminal_flag", 1);
    pub_error_ = nh_.advertise<geometry_msgs::Point>("/control/error", 1);
    pub_output_ = nh_.advertise<std_msgs::Float64MultiArray>("/control/output", 1);
    pub_lever_moldboard_ = nh_.advertise<std_msgs::Int16MultiArray>("/U_Lever_Moldboard", 1);
    pub_moldboard_debug_ = nh_.advertise<std_msgs::Float64MultiArray>("/control/moldboard_debug", 1);
    pub_lever1_ = nh_.advertise<std_msgs::Int16MultiArray>("/U_Lever1", 1);
    pub_terminal_flag_rot_ = nh_.advertise<std_msgs::Float64>("/control/terminal_flag_rotation", 1);
}

//==============================================================================
// 定时器回调 - 主控制循环
//==============================================================================

void ControlNode::timerCallback(const ros::TimerEvent& /*event*/) {
    // 1. 更新初始位置 (主开关上升沿)
    updateInitialPosition();
    
    // 2. 计算ENU坐标
    updateENU();
    
    // 3. 规划
    updatePlanning();
    
    // 4. 运动学计算
    updateKinematics();
    
    // 5. TD跟踪微分器
    updateTD();
    
    // 6. PID控制
    updatePID();
    
    // 7. 铲刀控制 (对应Control子系统中的铲刀PID环路)
    updateMoldboardControl();
    
    // 8. 发布输出
    publishOutputs();
    
    // 保存上一次状态
    main_switch_prev_ = inputs_.main_switch;
    terminal_flag_prev_ = outputs_.terminal_flag;
    terminal_flag_rotation_prev_ = outputs_.terminal_flag_rotation;
    X_terminal_prev_ = inputs_.X_terminal;
    Theta_terminal_prev_ = inputs_.Theta_terminal;
    ref_height_prev_ = inputs_.ref_height_middle_moldboard;
    Ref_Angle_prev_ = inputs_.Ref_Angle;
}

//==============================================================================
// 控制计算
//==============================================================================

void ControlNode::updateInitialPosition() {
    // [Issue 10] terminal_flag 不再由 control 生成, 此函数保留接口但简化逻辑.
    // 在主开关上升沿时更新初始位置.
    if (main_switch_prev_ == 0 && inputs_.main_switch == 1) {
        x0_ = outputs_.X_enu;
        y0_ = outputs_.Y_enu;
        theta0_ = inputs_.rtk_yaw;
    }
}

void ControlNode::updateENU() {
    // 检测主开关上升沿，更新LLA参考点
    if (main_switch_prev_ == 0 && inputs_.main_switch == 1) {
        lat0_ = inputs_.rtk_lat;
        lon0_ = inputs_.rtk_lon;
        alt0_ = inputs_.rtk_alt;
        yaw0_ = inputs_.rtk_yaw;
    }
    
    // LLA转ENU
    if (lat0_ != 0 && lon0_ != 0) {
        auto enu = control::lla2enu(
            inputs_.rtk_lat, inputs_.rtk_lon, inputs_.rtk_alt,
            lat0_, lon0_, alt0_);
        
        outputs_.X_enu = enu.X;
        outputs_.Y_enu = enu.Y;
        outputs_.UP_enu = enu.UP;
    }
    
    // [Fix-BUG-E] 速度融合
    //
    // 原代码问题: lla_velocity 写死 0, 但 KF 每隔一帧用 lla_velocity 做观测,
    // 导致速度估计被 0 系统性拖拽, 越快偏差越大。
    //
    // 当前硬件实际情况:
    //   inputs_.imu_velocity 来自 /Vehicle_Speed_Vel (CAN 0x327),
    //   这是 RTK/INS 组合导航模块输出的地速, 已经是融合后的值,
    //   不需要再做卡尔曼融合。
    //
    // 修复方案: 直接使用 RTK 地速, 加一阶低通滤波去毛刺。
    // 后续如果有独立的 IMU 积分速度源, 再恢复 KF 双源融合。
    {
        constexpr double alpha = 0.3;  // 低通系数, 0.3 = 较平滑, 1.0 = 不滤波
        outputs_.fusion_velocity = alpha * inputs_.imu_velocity
                                 + (1.0 - alpha) * outputs_.fusion_velocity;
    }
}

void ControlNode::updatePlanning() {
    // [Issue 10] decision 是到位判断的唯一权威.
    // control 只做速度跟踪: decision 发 walk_state=0 时停车, 发 walk_state>0 时跟踪速度.
    // terminal_flag 保留发布(兼容), 但始终为 0, 不参与控制.

    if (inputs_.walk_state == 0 || inputs_.walk_state == 3) {
        // 停车: decision 已判定到位或未启动
        outputs_.v_x_plan = 0;
        outputs_.v_theta_plan = 0;
    } else if (inputs_.walk_state == 1) {
        // 旋转: 只跟踪角速度
        outputs_.v_x_plan = 0;
        outputs_.v_theta_plan = inputs_.theta_velocity_reference;
    } else {
        // 直行 (walk_state==2 或 6): 跟踪速度 + 航向纠偏
        outputs_.v_x_plan = inputs_.v_reference;
        outputs_.v_theta_plan = inputs_.theta_velocity_reference;
    }

    // terminal_flag 固定为 0 — decision 侧做到位判断
    outputs_.terminal_flag = 0;
    outputs_.terminal_flag_rotation = 0;

    // 风险减速增益 (来自 /control_speed_gain, decision 根据 risk_state 计算)
    outputs_.v_x_plan *= speed_gain_;
}

void ControlNode::updateKinematics() {
    // 计算左右履带参考速度 (Plan版本D=0)
    auto kin = control::KinematicsForPlan(
        outputs_.v_x_plan, outputs_.v_theta_plan, inputs_.gama);
    
    outputs_.V_right_reference = kin.V_right_reference;
    outputs_.V_left_reference = kin.V_left_reference;
}

void ControlNode::updateTD() {
    // [Fix-BUG-F] 跟踪微分器 (韩京清 TD) 修复
    //
    // 原代码两层问题:
    //   1. td_x1_ 在积分更新后立刻被覆盖为原始误差 → TD 跟踪状态每帧归零
    //   2. PID 没有使用 TD 的输出 → TD 是完全失效的死代码
    //
    // 正确用法: TD 平滑参考信号 V_right/left_reference,
    //   td_x1_ 跟踪 reference (平滑后的值), td_x2_ 是其导数。
    //   PID 用 td_x1_ 代替原始 reference, 避免阶跃冲击。
    //
    // fhan 函数签名: control::TD(x1, x2, r, h)
    //   这里 x1 = 跟踪误差 = td_state - input
    //   返回值是加速度 fh, 用于更新 x2

    constexpr double dt = CONTROL_PERIOD_SEC;  // 100Hz = 0.01s

    // 右履带参考平滑
    {
        double e = td_x1_ - outputs_.V_right_reference;  // 跟踪误差
        double fh = control::TD(e, td_x2_, inputs_.td_r, inputs_.td_h);
        td_x1_ += td_x2_ * dt;
        td_x2_ += fh * dt;
    }

    // 左履带参考平滑
    {
        double e = td_theta1_ - outputs_.V_left_reference;
        double fh = control::TD(e, td_theta2_, inputs_.td_r, inputs_.td_h);
        td_theta1_ += td_theta2_ * dt;
        td_theta2_ += fh * dt;
    }

    // 主开关关闭时重置 TD 状态, 避免开机时从旧值跳变
    if (inputs_.main_switch < 0.5) {
        td_x1_ = 0; td_x2_ = 0;
        td_theta1_ = 0; td_theta2_ = 0;
    }
}

void ControlNode::updatePID() {
    constexpr double dt = CONTROL_PERIOD_SEC;  // 100Hz = 0.01s
    
    // [Fix-BUG-F] PID 使用 TD 平滑后的参考值, 而非原始 reference
    // td_x1_     = 平滑后的 V_right_reference
    // td_theta1_ = 平滑后的 V_left_reference
    outputs_.error_x = td_x1_ - outputs_.fusion_velocity;
    outputs_.error_theta = td_theta1_ - outputs_.fusion_velocity;
    
    // PID for right track (X方向)
    double P_right = inputs_.Kp_x * outputs_.error_x;
    integral_x_ += outputs_.error_x * dt;
    integral_x_ = control::PID_IntegralLimit_X(integral_x_, outputs_.error_x);
    double I_right = inputs_.Ki_x * integral_x_;
    double D_right = inputs_.Kd_x * (outputs_.error_x - error_x_prev_) / dt;
    
    outputs_.V_right_cmd = P_right + I_right + D_right;
    
    // PID for left track (Theta方向)
    double P_left = inputs_.Kp_theta * outputs_.error_theta;
    integral_theta_ += outputs_.error_theta * dt;
    integral_theta_ = control::PID_IntegralLimit_Theta(integral_theta_, outputs_.error_theta);
    double I_left = inputs_.Ki_theta * integral_theta_;
    double D_left = inputs_.Kd_theta * (outputs_.error_theta - error_theta_prev_) / dt;
    
    outputs_.V_left_cmd = P_left + I_left + D_left;
    
    // 行走非线性层处理 (带terminal_flag)
    outputs_.V_right_cmd = control::WalkNonlinearLayer(
        outputs_.V_right_cmd, inputs_.nl_param1, inputs_.nl_param2, inputs_.nl_param3,
        outputs_.terminal_flag);
    outputs_.V_left_cmd = control::WalkNonlinearLayer(
        outputs_.V_left_cmd, inputs_.nl_param1, inputs_.nl_param2, inputs_.nl_param3,
        outputs_.terminal_flag);
    
    // 限幅
    outputs_.V_right_cmd = control::saturate(outputs_.V_right_cmd, -1000, 1000);
    outputs_.V_left_cmd = control::saturate(outputs_.V_left_cmd, -1000, 1000);
    
    // 保存上一次误差
    error_x_prev_ = outputs_.error_x;
    error_theta_prev_ = outputs_.error_theta;
    
    // 主开关关闭时清零 (Reset函数)
    outputs_.V_right_cmd = control::Reset(outputs_.V_right_cmd, inputs_.main_switch);
    outputs_.V_left_cmd = control::Reset(outputs_.V_left_cmd, inputs_.main_switch);
}

//==============================================================================
// 铲刀控制 - 对应Control子系统(system_3680)中的铲刀PID环路
//==============================================================================

void ControlNode::updateMoldboardControl() {
    constexpr double dt = CONTROL_PERIOD_SEC;  // 100Hz = 0.01s
    
    //--------------------------------------------------------------------------
    // 1. 计算铲刀中心实际高度 (MoldboardMiddleHeight)
    //--------------------------------------------------------------------------
    outputs_.actual_height_middle_moldboard = control::MoldboardMiddleHeight(
        inputs_.actual_height_left_moldboard,
        inputs_.actual_height_right_moldboard);
    
    //--------------------------------------------------------------------------
    // 2. 计算误差
    //    e_Height_m = ref_height_m - actual_height_m   (米, 调试用)
    //    e_Theta    = Ref_Angle - IMU_Roll             (度)
    //
    // [Fix-v19] 单位对齐:
    //   上游参考 ref_height_middle_moldboard 和 actual_height 都是米,
    //   但下游 HeightNonlinearLayer 的 X 表 {-50,-30,-15,-8,-3,0,3,8,15,30,50}
    //   以及 Deadzone_Height (默认 3.0) 注释都是 mm。
    //   原代码 e_Height 以米做 PID 输入, Kp=1.0 时 1 米误差只产出 1.0 的数值,
    //   走 X 表落在 0~3 档 → 内插接近 0 → 过死区被吃掉 → PID 输出恒 0。
    //   修复: e 计算完毕后统一换算到 mm, P+I 全流程在 mm 单位下工作。
    //   角度通路单位(度)本来就和 X 表(假设也是 °级)一致, 不换算。
    //--------------------------------------------------------------------------
    double e_Height_m = inputs_.ref_height_middle_moldboard - outputs_.actual_height_middle_moldboard;
    outputs_.e_Height = e_Height_m;                         // 调试输出保留米单位
    double e_Height_mm = e_Height_m * 1000.0;               // [Fix-v19] 换算到 mm 给 PID 用
    outputs_.e_Theta = inputs_.Ref_Angle - inputs_.IMU_Roll;
    
    //--------------------------------------------------------------------------
    // 3. 控制标志 (MoldboardControlFlag)
    //--------------------------------------------------------------------------
    outputs_.Control_Flag = control::MoldboardControlFlag(
        inputs_.Moldboard_Control_Flag, inputs_.main_switch);
    
    //--------------------------------------------------------------------------
    // 4. 高度P控制通路  (全程 mm 单位)
    //    HeightPControl → HeightNonlinearLayer → Deadzone → ValveDeadzone
    //--------------------------------------------------------------------------
    // 4a. 不对称P控制
    double height_p_out = control::HeightPControl(
        e_Height_mm, inputs_.Kp_Height_Up, inputs_.Kp_Height_Down);
    
    // 4b. 非线性层
    double height_nl_out = control::HeightNonlinearLayer(
        height_p_out, inputs_.Height_Param1, inputs_.Height_Param2, inputs_.Height_Param3);
    
    // 4c. 死区 (Deadzone_Height 单位 mm)
    double height_dz_out = control::Deadzone(height_nl_out, inputs_.Deadzone_Height);
    
    // 4d. 阀门死区 (产生阶跃输出)
    outputs_.e_Height_Actual = control::ValveDeadzone(
        height_dz_out, inputs_.Deadzone_Height, inputs_.Dead_Value_Height);
    
    //--------------------------------------------------------------------------
    // 5. 角度P控制通路
    //    ThetaPControl → ThetaNonlinearLayer → Deadzone → ThetaValveDeadzone
    //--------------------------------------------------------------------------
    // 5a. 不对称P控制
    double theta_p_out = control::ThetaPControl(
        outputs_.e_Theta, inputs_.Kp_Theta_Up, inputs_.Kp_Theta_Down);
    
    // 5b. 非线性层
    double theta_nl_out = control::ThetaNonlinearLayer(
        theta_p_out, inputs_.Theta_Param1, inputs_.Theta_Param2, inputs_.Theta_Param3);
    
    // 5c. 死区
    double theta_dz_out = control::Deadzone(theta_nl_out, inputs_.Deadzone_Theta);
    
    // 5d. 阀门死区 (产生阶跃输出)
    outputs_.e_Theta_Actual = control::ThetaValveDeadzone(
        theta_dz_out, inputs_.Deadzone_Theta, inputs_.Dead_Value_Theta);
    
    //--------------------------------------------------------------------------
    // 6. 高度I控制通路  (全程 mm 单位)
    //    VelocityISelect → 积分 → IntegralAngleDecay → Saturation
    //
    // [Fix-v19] 清零阈值:
    //   原 ClearIntegral 只要 ref != ref_prev 就清零, 但路径规划模式下
    //   每个 WayPoint 的 target_height 可能有微小浮点差异,
    //   导致每个路径点切换都被当作"参考跳变"清零, I 项实际从未累积。
    //   修复: 只在跳变 > 0.05m (50mm) 时才清零, 认为是真正的参考高度切换。
    //--------------------------------------------------------------------------
    // 6a. 积分清零 (参考值发生显著跳变时)
    if (std::abs(inputs_.ref_height_middle_moldboard - ref_height_prev_) > 0.05) {
        moldboard_integral_height_ = 0;
    }
    
    // 6b. 选择I增益 (根据误差方向)
    double Ki_Height = control::VelocityISelect(
        e_Height_mm, inputs_.Ki_Height_Up, inputs_.Ki_Height_Down);
    
    // 6c. 积分累加 (mm·s 单位)
    moldboard_integral_height_ += e_Height_mm * Ki_Height * dt;
    
    // 6d. 积分角度衰减 (此处 "angle" 是 Simulink 遗留命名, 实际是"误差幅度阈值",
    //     I_angle_threshold 单位和 e 一致, 即 mm)
    moldboard_integral_height_ = control::IntegralAngleDecay(
        moldboard_integral_height_, e_Height_mm, inputs_.I_angle_threshold);
    
    // 6e. 积分限幅
    moldboard_integral_height_ = control::Saturation(
        moldboard_integral_height_, inputs_.I_MAX);
    
    //--------------------------------------------------------------------------
    // 7. 角度I控制通路
    //--------------------------------------------------------------------------
    // 7a. 积分清零 (参考值显著跳变时, 阈值 1 度)
    if (std::abs(inputs_.Ref_Angle - Ref_Angle_prev_) > 1.0) {
        moldboard_integral_theta_ = 0;
    }
    
    // 7b. 选择I增益
    double Ki_Theta = control::VelocityISelect(
        outputs_.e_Theta, inputs_.Ki_Theta_Up, inputs_.Ki_Theta_Down);
    
    // 7c. 积分累加
    moldboard_integral_theta_ += outputs_.e_Theta * Ki_Theta * dt;
    
    // 7d. 积分角度衰减
    moldboard_integral_theta_ = control::IntegralAngleDecay(
        moldboard_integral_theta_, outputs_.e_Theta, inputs_.I_angle_threshold_Theta);
    
    // 7e. 积分限幅
    moldboard_integral_theta_ = control::Saturation(
        moldboard_integral_theta_, inputs_.I_MAX_Theta);
    
    //--------------------------------------------------------------------------
    // 8. P+I合成
    //--------------------------------------------------------------------------
    double u_Height_raw = (outputs_.e_Height_Actual + moldboard_integral_height_) * outputs_.Control_Flag;
    double u_Theta_raw = (outputs_.e_Theta_Actual + moldboard_integral_theta_) * outputs_.Control_Flag;
    
    //--------------------------------------------------------------------------
    // 9. Reset + Saturation
    //    Main_Switch==0时清零, 然后限幅±1000
    //--------------------------------------------------------------------------
    // 9a. Reset (Main_Switch==0时清零)
    u_Height_raw = control::Reset(u_Height_raw, inputs_.main_switch);
    u_Theta_raw = control::Reset(u_Theta_raw, inputs_.main_switch);
    
    // 9b. Saturation ±1000  (注意: ros_to_can 会再除以 2, 最终到 CAN 是 ±500)
    outputs_.u_Height = control::saturate(u_Height_raw, -1000, 1000);
    outputs_.u_Theta = control::saturate(u_Theta_raw, -1000, 1000);
    
    //--------------------------------------------------------------------------
    // 10. 分离Up/Dn
    //--------------------------------------------------------------------------
    auto height_split = control::HeightUpDnSplit(outputs_.u_Height);
    outputs_.u_Height_Up = height_split.u_Height_Up;
    outputs_.u_Height_Dn = height_split.u_Height_Dn;
    
    auto theta_split = control::ThetaUpDnSplit(outputs_.u_Theta);
    outputs_.u_Theta_Up = theta_split.u_Theta_Up;
    outputs_.u_Theta_Dn = theta_split.u_Theta_Dn;
    
    // 保存实际P项用于调试输出
    outputs_.u_Height_Actual = outputs_.e_Height_Actual;
    outputs_.u_Theta_Actual = outputs_.e_Theta_Actual;
    
    // Main_Switch==0时清零积分
    if (inputs_.main_switch == 0) {
        moldboard_integral_height_ = 0;
        moldboard_integral_theta_ = 0;
    }
}

//==============================================================================
// 发布输出
//==============================================================================

void ControlNode::publishOutputs() {
    // cmd_vel
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = (outputs_.V_right_cmd + outputs_.V_left_cmd) / 2;
    cmd_vel.angular.z = (outputs_.V_right_cmd - outputs_.V_left_cmd) / inputs_.track_width;
    pub_cmd_vel_.publish(cmd_vel);
    
    // V_right
    std_msgs::Float64 msg_v_right;
    msg_v_right.data = outputs_.V_right_cmd;
    pub_V_right_.publish(msg_v_right);
    
    // V_left
    std_msgs::Float64 msg_v_left;
    msg_v_left.data = outputs_.V_left_cmd;
    pub_V_left_.publish(msg_v_left);
    
    // fusion_velocity
    std_msgs::Float64 msg_fusion;
    msg_fusion.data = outputs_.fusion_velocity;
    pub_fusion_velocity_.publish(msg_fusion);
    
    // enu_position
    geometry_msgs::Point msg_enu;
    msg_enu.x = outputs_.X_enu;
    msg_enu.y = outputs_.Y_enu;
    msg_enu.z = outputs_.UP_enu;
    pub_enu_position_.publish(msg_enu);
    
    // terminal_flag
    std_msgs::Float64 msg_terminal;
    msg_terminal.data = outputs_.terminal_flag;
    pub_terminal_flag_.publish(msg_terminal);
    
    // [BUG1/4] 旋转到位标志
    std_msgs::Float64 msg_terminal_rot;
    msg_terminal_rot.data = outputs_.terminal_flag_rotation;
    pub_terminal_flag_rot_.publish(msg_terminal_rot);
    
    // error
    geometry_msgs::Point msg_error;
    msg_error.x = outputs_.error_x;
    msg_error.y = outputs_.error_theta;
    msg_error.z = 0;
    pub_error_.publish(msg_error);
    
    // 综合输出
    std_msgs::Float64MultiArray msg_output;
    msg_output.data = {
        outputs_.V_right_cmd,
        outputs_.V_left_cmd,
        outputs_.fusion_velocity,
        outputs_.X_enu,
        outputs_.Y_enu,
        outputs_.UP_enu,
        outputs_.terminal_flag,
        outputs_.terminal_flag_rotation,
        outputs_.v_x_plan,
        outputs_.v_theta_plan,
        outputs_.V_right_reference,
        outputs_.V_left_reference,
        outputs_.error_x,
        outputs_.error_theta
    };
    pub_output_.publish(msg_output);
    
    // 铲刀手柄控制 - 发送到 /U_Lever_Moldboard
    // 格式: [u_Height_Up, u_Height_Dn, u_Theta_Up, u_Theta_Dn]
    // 仅主开关开启时发布, 避免覆盖 Tab1 直接 CAN 控制
    if (inputs_.main_switch > 0.5) {
        std_msgs::Int16MultiArray msg_lever_moldboard;
        msg_lever_moldboard.data.resize(4);
        // D0-D1=斜拉缩(Theta_Up), D2-D3=斜拉伸(Theta_Dn), D4-D5=铲刀升(Height_Up), D6-D7=铲刀降(Height_Dn)
        msg_lever_moldboard.data[0] = static_cast<int16_t>(outputs_.u_Theta_Up);
        msg_lever_moldboard.data[1] = static_cast<int16_t>(outputs_.u_Theta_Dn);
        msg_lever_moldboard.data[2] = static_cast<int16_t>(outputs_.u_Height_Up);
        msg_lever_moldboard.data[3] = static_cast<int16_t>(outputs_.u_Height_Dn);
        pub_lever_moldboard_.publish(msg_lever_moldboard);
    }
    
    // 铲刀调试信息
    std_msgs::Float64MultiArray msg_moldboard_debug;
    msg_moldboard_debug.data = {
        outputs_.actual_height_middle_moldboard,
        outputs_.e_Height,
        outputs_.e_Theta,
        outputs_.e_Height_Actual,
        outputs_.e_Theta_Actual,
        outputs_.u_Height,
        outputs_.u_Theta,
        outputs_.u_Height_Up,
        outputs_.u_Height_Dn,
        outputs_.u_Theta_Up,
        outputs_.u_Theta_Dn,
        outputs_.Control_Flag,
        moldboard_integral_height_,
        moldboard_integral_theta_
    };
    pub_moldboard_debug_.publish(msg_moldboard_debug);
    
    //--------------------------------------------------------------------------
    // 行走指令桥接: V_right_cmd/V_left_cmd → /U_Lever1
    //
    // V_right_cmd, V_left_cmd 范围 [-1000, 1000]
    // /U_Lever1 格式: Int16MultiArray[7]
    //   [0] 左转位, [1] 右转位, [2] 转向值(0~255),
    //   [3] 升挡, [4] 降挡, [5] 挡位(0空/1前/2后), [6] 解锁
    //--------------------------------------------------------------------------
    std_msgs::Int16MultiArray msg_lever1;
    msg_lever1.data.resize(7, 0);
    
    double vr = std::max(-inputs_.v_max_limit, std::min(inputs_.v_max_limit, outputs_.V_right_cmd));
    double vl = std::max(-inputs_.v_max_limit, std::min(inputs_.v_max_limit, outputs_.V_left_cmd));
    double v_avg  = (vr + vl) / 2.0;
    double v_diff = vr - vl;
    
    // 主开关关闭时不发指令, 让 Tab1 直接 CAN 控制不被覆盖
    if (inputs_.main_switch > 0.5) {
        // 挡位: 平均速度方向决定前进/后退, 死区内为空挡
        
        if (v_avg > inputs_.gear_deadzone) {
            msg_lever1.data[5] = 1;   // 前进挡
        } else if (v_avg < -inputs_.gear_deadzone) {
            msg_lever1.data[5] = 2;   // 后退挡
        } else {
            msg_lever1.data[5] = 0;   // 空挡
        }
        
        // 转向: 速度差决定方向和力度
        // V_right > V_left → 右快左慢 → 车往左转
        // 转向值映射: |v_diff| / 1000 * 255 → [0, 255]
        
        if (v_diff > inputs_.steer_deadzone) {
            msg_lever1.data[0] = 1;   // 左转位
            double steer = std::min(std::abs(v_diff) / 1000.0 * 255.0, 255.0);
            msg_lever1.data[2] = static_cast<int16_t>(steer);
        } else if (v_diff < -inputs_.steer_deadzone) {
            msg_lever1.data[1] = 1;   // 右转位
            double steer = std::min(std::abs(v_diff) / 1000.0 * 255.0, 255.0);
            msg_lever1.data[2] = static_cast<int16_t>(steer);
        }
        
        // 解锁: 自动模式下始终解锁
        msg_lever1.data[6] = 1;

        pub_lever1_.publish(msg_lever1);
    }
    // 主开关关闭时不发, 让 Tab1 直接 CAN 控制不被覆盖
}

//==============================================================================
// ROS回调
//==============================================================================

void ControlNode::rtkCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    inputs_.rtk_lat = msg->latitude;
    inputs_.rtk_lon = msg->longitude;
    inputs_.rtk_alt = msg->altitude;
}

void ControlNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    inputs_.imu_yaw_rate = msg->angular_velocity.z;
    // IMU速度需要从线性加速度积分或从其他来源获取
}

void ControlNode::walkStateCallback(const std_msgs::Float64::ConstPtr& msg) {
    inputs_.walk_state = msg->data;
}

void ControlNode::mainSwitchCallback(const std_msgs::Float64::ConstPtr& msg) {
    inputs_.main_switch = msg->data;
}

void ControlNode::terminalCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 3) {
        inputs_.X_terminal = msg->data[0];
        inputs_.Y_terminal = msg->data[1];
        inputs_.Theta_terminal = msg->data[2];
    }
}

void ControlNode::referenceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        inputs_.v_reference = msg->data[0];
        inputs_.theta_velocity_reference = msg->data[1];
    }
}

void ControlNode::paramsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 15) {
        inputs_.X_Tolerance = msg->data[0];
        inputs_.Y_Tolerance = msg->data[1];
        inputs_.Theta_Tolerance = msg->data[2];
        inputs_.hold_x = msg->data[3];
        inputs_.hold_theta = msg->data[4];
        inputs_.gama = msg->data[5];
        inputs_.track_width = msg->data[6];
        inputs_.Kp_x = msg->data[7];
        inputs_.Ki_x = msg->data[8];
        inputs_.Kd_x = msg->data[9];
        inputs_.Kp_theta = msg->data[10];
        inputs_.Ki_theta = msg->data[11];
        inputs_.Kd_theta = msg->data[12];
        inputs_.td_r = msg->data[13];
        inputs_.td_h = msg->data[14];
    }
    // 桥接死区参数 (可选, 兼容旧版15个参数)
    if (msg->data.size() >= 17) {
        inputs_.gear_deadzone  = msg->data[15];
        inputs_.steer_deadzone = msg->data[16];
    }
}

//==============================================================================
// 铲刀相关ROS回调
//==============================================================================

void ControlNode::moldboardHeightLeftCallback(const std_msgs::Float64::ConstPtr& msg) {
    inputs_.actual_height_left_moldboard = msg->data;
}

void ControlNode::moldboardHeightRightCallback(const std_msgs::Float64::ConstPtr& msg) {
    inputs_.actual_height_right_moldboard = msg->data;
}

void ControlNode::refHeightCallback(const std_msgs::Float64::ConstPtr& msg) {
    inputs_.ref_height_middle_moldboard = msg->data;
}

void ControlNode::refAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
    inputs_.Ref_Angle = msg->data;
}

void ControlNode::imuRollCallback(const std_msgs::Float64::ConstPtr& msg) {
    inputs_.IMU_Roll = msg->data;
}

void ControlNode::moldboardControlFlagCallback(const std_msgs::Float64::ConstPtr& msg) {
    inputs_.Moldboard_Control_Flag = msg->data;
}

void ControlNode::moldboardParamsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // 铲刀控制参数:
    // [0]  Kp_Height_Up
    // [1]  Kp_Height_Down
    // [2]  Ki_Height_Up
    // [3]  Ki_Height_Down
    // [4]  Height_Param1
    // [5]  Height_Param2
    // [6]  Height_Param3
    // [7]  Deadzone_Height
    // [8]  Dead_Value_Height
    // [9]  I_MAX
    // [10] I_angle_threshold
    // [11] Kp_Theta_Up
    // [12] Kp_Theta_Down
    // [13] Ki_Theta_Up
    // [14] Ki_Theta_Down
    // [15] Theta_Param1
    // [16] Theta_Param2
    // [17] Theta_Param3
    // [18] Deadzone_Theta
    // [19] Dead_Value_Theta
    // [20] I_MAX_Theta
    // [21] I_angle_threshold_Theta
    if (msg->data.size() >= 22) {
        inputs_.Kp_Height_Up = msg->data[0];
        inputs_.Kp_Height_Down = msg->data[1];
        inputs_.Ki_Height_Up = msg->data[2];
        inputs_.Ki_Height_Down = msg->data[3];
        inputs_.Height_Param1 = msg->data[4];
        inputs_.Height_Param2 = msg->data[5];
        inputs_.Height_Param3 = msg->data[6];
        inputs_.Deadzone_Height = msg->data[7];
        inputs_.Dead_Value_Height = msg->data[8];
        inputs_.I_MAX = msg->data[9];
        inputs_.I_angle_threshold = msg->data[10];
        inputs_.Kp_Theta_Up = msg->data[11];
        inputs_.Kp_Theta_Down = msg->data[12];
        inputs_.Ki_Theta_Up = msg->data[13];
        inputs_.Ki_Theta_Down = msg->data[14];
        inputs_.Theta_Param1 = msg->data[15];
        inputs_.Theta_Param2 = msg->data[16];
        inputs_.Theta_Param3 = msg->data[17];
        inputs_.Deadzone_Theta = msg->data[18];
        inputs_.Dead_Value_Theta = msg->data[19];
        inputs_.I_MAX_Theta = msg->data[20];
        inputs_.I_angle_threshold_Theta = msg->data[21];
    }
}
