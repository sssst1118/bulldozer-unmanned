/**
 * @file decision_system.cpp
 * @brief 推土机决策系统实现 — 路径生成 + 通用执行器
 * @author dozer-dev
 * @date 2026-03-15
 */
#include "decision_system.h"
#include "qt_app/log_helper.h"
#include <boost/function.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cctype>
#include <algorithm>

//==============================================================================
// 构造
//==============================================================================
DecisionSystem::DecisionSystem(ros::NodeHandle& nh) : nh_(nh) {
    initSubscribers();
    initPublishers();

    // ===== 从 rosparam 读取路径参数默认值 =====
    nh_.param<double>("path/push_length",  path_params_.push_length,  20.0);
    nh_.param<double>("path/map_width",    path_params_.map_width,    20.0);
    nh_.param<double>("path/blade_width",  path_params_.blade_width,  4.2);
    nh_.param<double>("path/heading",      path_params_.heading,      0.0);
    nh_.param<double>("path/v_push",       path_params_.v_push,       0.5);
    nh_.param<double>("path/v_reverse",    path_params_.v_reverse,    0.8);
    nh_.param<double>("path/omega_rotate", path_params_.omega_rotate, 10.0);
    nh_.param<double>("path/x_back_set",   path_params_.x_back_set,   3.0);
    nh_.param<double>("path/shift_angle",  path_params_.shift_angle,  12.0);
    nh_.param<int>("path/num_columns",     path_params_.num_columns,  5);
    nh_.param<int>("path/passes_per_col",  path_params_.passes_per_col, 1);
    int pm = 0;
    nh_.param<int>("path/mode", pm, 0);
    path_mode_ = static_cast<PathMode>(std::clamp(pm, 0, 2));
    nh_.param<double>("path/position_tolerance", position_tolerance_, 0.3);
    nh_.param<double>("path/theta_tolerance",    theta_tolerance_,    5.0);

    LOG_INFO("Path params: push=%.1fm width=%.1fm blade=%.1fm heading=%.1f mode=%d",
             path_params_.push_length, path_params_.map_width,
             path_params_.blade_width, path_params_.heading, static_cast<int>(path_mode_));

    timer_ = nh_.createTimer(ros::Duration(0.01), &DecisionSystem::timerCallback, this);
    LOG_INFO("DecisionSystem initialized (new architecture)");
}

//==============================================================================
// 初始化
//==============================================================================
void DecisionSystem::initSubscribers() {
    sub_main_switch_ = nh_.subscribe("/Main_Switch", 1, &DecisionSystem::mainSwitchCallback, this);
    sub_detection_   = nh_.subscribe("/Detection_Altitude_Completed", 1, &DecisionSystem::detectionCompletedCallback, this);
    sub_rtk_         = nh_.subscribe("/RTK", 1, &DecisionSystem::rtkCallback, this);
    sub_lla_         = nh_.subscribe("/LLA", 1, &DecisionSystem::llaCallback, this);
    sub_angle_       = nh_.subscribe("/Angle_Heading", 1, &DecisionSystem::angleHeadingCallback, this);
    sub_imu_         = nh_.subscribe("/AHRS_IMU", 1, &DecisionSystem::ahrsImuCallback, this);
    sub_grid_        = nh_.subscribe("/occupancy_grid", 1, &DecisionSystem::occupancyGridCallback, this);
    sub_grid_loc_    = nh_.subscribe("/Navigate_location", 1, &DecisionSystem::occupancyLocationCallback, this);
    sub_nav_coord_   = nh_.subscribe("/Navigation_Coordinate", 1, &DecisionSystem::navigationCoordinateCallback, this);
    // [Issue#4] /Location_real 当前无节点发布, 回调会无条件覆盖 enu_x_/enu_y_,
    // 没有像 /Navigate_location 那样的优先级/超时机制, 留着是隐患。注释保留以备恢复。
    // sub_loc_real_    = nh_.subscribe("/Location_real", 1, &DecisionSystem::locationRealCallback, this);
    sub_risk_        = nh_.subscribe("/risk_state", 1, &DecisionSystem::riskStateCallback, this);
    sub_overload_    = nh_.subscribe("/Mold_OverLoad_Status", 1, &DecisionSystem::moldOverloadCallback, this);

    // 路径模式
    sub_path_mode_ = nh_.subscribe("/decision/path_mode", 1, &DecisionSystem::pathModeCallback, this);
    sub_test_config_ = nh_.subscribe<std_msgs::Float64MultiArray>("/decision/test_config", 1,
        boost::function<void(const std_msgs::Float64MultiArray::ConstPtr&)>(
            [this](const std_msgs::Float64MultiArray::ConstPtr& msg) {
                if (msg->data.size() < 5) return;
                test_mode_     = (msg->data[0] > 0.5);
                test_length_   = msg->data[1];
                test_width_    = msg->data[2];
                test_passes_   = static_cast<int>(msg->data[3]);
                test_start_pos_= static_cast<int>(msg->data[4]);
                if (msg->data.size() >= 6 && std::abs(msg->data[5]) > 0.01) {
                    path_params_.heading = msg->data[5];
                }
                LOG_INFO("Test config: mode=%d len=%.1f wid=%.1f passes=%d start=%d heading=%.1f",
                         test_mode_, test_length_, test_width_, test_passes_, test_start_pos_, path_params_.heading);
            }));
    sub_test_script_ = nh_.subscribe<std_msgs::String>("/decision/test_script", 1,
        boost::function<void(const std_msgs::String::ConstPtr&)>(
            [this](const std_msgs::String::ConstPtr& msg) {
                if (msg->data == "STOP") {
                    // 紧急停止脚本
                    path_ready_ = false;
                    path_.clear();
                    grid_path_.clear();
                    exec_state_ = ExecState::IDLE;
                    sendWalkCommand(0, 0, 0, 0, 0);
                    sendBladeCommand(BladeCmd::RAISE);
                    LOG_INFO("Script STOPPED");
                    return;
                }

                LOG_INFO("Script received, parsing...");

                // 解析脚本 → ENU 路径点
                path_.clear();
                grid_path_.clear();

                // 当前位置作为起点
                double cx = enu_x_, cy = enu_y_;
                double ch = heading_deg_ * M_PI / 180.0;  // 当前航向(弧度)

                // 解析
                std::istringstream iss(msg->data);
                std::string line;
                std::vector<std::string> commands;
                int loop_count = 1;

                while (std::getline(iss, line)) {
                    // 去掉注释和空格
                    auto pos = line.find('#');
                    if (pos != std::string::npos) line = line.substr(0, pos);
                    // trim
                    while (!line.empty() && (line.front() == ' ' || line.front() == '\t')) line.erase(line.begin());
                    while (!line.empty() && (line.back() == ' ' || line.back() == '\t')) line.pop_back();
                    if (line.empty()) continue;

                    // LOOP 指令
                    if (line.size() > 4 && (line.substr(0,4) == "LOOP" || line.substr(0,4) == "loop")) {
                        loop_count = std::atoi(line.c_str() + 5);
                        if (loop_count <= 0) loop_count = 100;  // -1 = 大量循环
                        continue;
                    }
                    commands.push_back(line);
                }

                auto execLine = [&](const std::string& cmd) {
                    std::istringstream cs(cmd);
                    std::string op; double val = 0;
                    cs >> op >> val;
                    // 转大写
                    for (auto& c : op) c = std::toupper(c);

                    if (op == "FORWARD") {
                        // 沿当前航向前进 val 米
                        cx += val * std::sin(ch);  // ENU: X=东, sin(heading)
                        cy += val * std::cos(ch);  // ENU: Y=北, cos(heading)
                        // [3D-Level] 脚本 FORWARD 视为推土: 目标高度取当前路径参数
                        path_.emplace_back(cx, cy, BladeCmd::LEVEL, DriveDir::FORWARD,
                                           path_params_.target_level_height);
                    } else if (op == "BACKWARD") {
                        cx -= val * std::sin(ch);
                        cy -= val * std::cos(ch);
                        // [3D-Level] 倒车提刀
                        path_.emplace_back(cx, cy, BladeCmd::RAISE, DriveDir::BACKWARD,
                                           RAISE_HEIGHT);
                    } else if (op == "ROTATE") {
                        ch += val * M_PI / 180.0;
                        // 原地旋转: 加一个当前位置的点, 执行器会先旋转对准下一个点
                    } else if (op == "BLADE") {
                        // 不生成路径点, 只影响下一段的铲刀状态
                    } else if (op == "WAIT") {
                        // 加一个当前位置的点, 执行器到位后会短暂停留
                        // [3D-Level] HOLD 不写 target_height (执行器在 HOLD 时不下发, 保持原值)
                        path_.emplace_back(cx, cy, BladeCmd::HOLD, DriveDir::FORWARD);
                    }
                };

                // 执行指令 loop_count 次
                for (int rep = 0; rep < loop_count; ++rep)
                    for (const auto& cmd : commands) execLine(cmd);

                // 同步生成栅格路径 (用于地图显示)
                if (map_resolution_ > 0) {
                    // 有地图: 用真实坐标系转换
                    for (const auto& wp : path_) {
                        double gr = (wp.y - map_origin_y_) / map_resolution_;
                        double gc = (wp.x - map_origin_x_) / map_resolution_;
                        double dir = (wp.drive_dir == DriveDir::FORWARD) ? 1.0 : -1.0;
                        grid_path_.push_back({gr, gc, dir});
                    }
                }

                waypoint_index_ = 0;
                path_ready_ = !path_.empty();
                if (path_ready_) {
                    exec_state_ = ExecState::IDLE;
                    if (map_resolution_ > 0) {
                        publishGridPath();
                    } else {
                        // 无地图: 合成虚拟栅格并发布
                        synthesizeVirtualGrid();
                    }
                    LOG_INFO("Script path: %d points, %d loops", (int)path_.size(), loop_count);
                    for (int k = 0; k < std::min(4, (int)path_.size()); ++k)
                        LOG_INFO("  WP[%d] enu(%.1f,%.1f) %s", k, path_[k].x, path_[k].y,
                                 path_[k].drive_dir == DriveDir::FORWARD ? "FWD" : "BWD");
                }
            }));
    sub_regen_path_ = nh_.subscribe<std_msgs::Float64>("/decision/regenerate_path", 1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>(
            [this](const std_msgs::Float64::ConstPtr& msg) {
                if (msg->data > 0.5) {
                    LOG_INFO("Manual path regenerate triggered");
                    // 没有实车RTK时, 用默认朝向
                    if (!origin_locked_) {
                        heading0_ = path_params_.heading;  // 用界面设置的方向
                        origin_locked_ = true;
                        LOG_INFO("Origin locked (manual): heading=%.1f", heading0_);
                    }
                    path_ready_ = false;
                    generateNewPath();
                }
            }));

    // 运行时路径参数更新 (从控制台)
    // [push_length, blade_width, heading, v_push, v_reverse, omega_rotate, x_back_set, pos_tol, theta_tol, num_columns, passes_per_col, shift_angle]
    sub_path_params_ = nh_.subscribe<std_msgs::Float64MultiArray>("/decision/path_params", 1,
        boost::function<void(const std_msgs::Float64MultiArray::ConstPtr&)>(
            [this](const std_msgs::Float64MultiArray::ConstPtr& msg) {
                if (msg->data.size() < 9) return;
                path_params_.push_length  = msg->data[0];
                path_params_.blade_width  = msg->data[1];
                path_params_.heading      = msg->data[2];
                path_params_.v_push       = msg->data[3];
                path_params_.v_reverse    = msg->data[4];
                path_params_.omega_rotate = msg->data[5];
                path_params_.x_back_set   = msg->data[6];
                position_tolerance_       = msg->data[7];
                theta_tolerance_          = msg->data[8];
                if (msg->data.size() >= 11) {
                    path_params_.num_columns    = static_cast<int>(msg->data[9]);
                    path_params_.passes_per_col = static_cast<int>(msg->data[10]);
                }
                if (msg->data.size() >= 12) {
                    path_params_.shift_angle = msg->data[11];
                }
                if (msg->data.size() >= 13) {
                    path_params_.map_scale = msg->data[12];
                }
                // 辅助模块参数 (原 /RLS6, /RLS10)
                if (msg->data.size() >= 16) {
                    decision_logic_inputs_.control_speed_gain_qt_risk = msg->data[13];
                    decision_logic_inputs_.control_speed_gain_qt_mold = msg->data[14];
                    decision_logic_inputs_.mold_limit = msg->data[15];
                }
                if (msg->data.size() >= 19) {
                    overload_inputs_.Trans_Speed_Limit    = msg->data[16];
                    overload_inputs_.Vehicle_Speed_Limit  = msg->data[17];
                    overload_inputs_.Angular_Speed_Limit  = msg->data[18];
                }
                // 有地图起点/推土方向
                if (msg->data.size() >= 21) {
                    map_start_corner_ = static_cast<int>(msg->data[19]);
                    map_push_heading_ = msg->data[20];
                }
                // [3D-Level] 铲刀找平参数 (从 GUI Tab5 下发)
                //   [21] level_mode          (0=FLAT_3D 找平+横坡, 1=SLOPE_3D 纵坡+横坡)
                //   [22] target_level_height (米, ENU-Up 绝对高程, FLAT_3D 用)
                //   [23] blade_angle_deg     (度, 横坡角度, 两个模式共用)
                //   [24] slope_start_height  (米, ENU-Up 绝对高程, SLOPE_3D 用)
                //   [25] slope_gradient      (%, SLOPE_3D 用, 正值上坡负值下坡)
                // 没传的话保持上次/默认值, 向后兼容。
                if (msg->data.size() >= 22) {
                    int lm = static_cast<int>(msg->data[21]);
                    path_params_.level_mode = static_cast<LevelMode>(std::clamp(lm, 0, 1));
                }
                if (msg->data.size() >= 23) {
                    path_params_.target_level_height = msg->data[22];
                }
                if (msg->data.size() >= 24) {
                    blade_angle_deg_ = msg->data[23];
                }
                if (msg->data.size() >= 25) {
                    path_params_.slope_start_height = msg->data[24];
                }
                if (msg->data.size() >= 26) {
                    path_params_.slope_gradient = msg->data[25];
                }
                LOG_INFO("Path params: push=%.1f blade=%.1f angle=%.1f hdg=%.1f v=%.2f/%.2f cols=%d ppc=%d corner=%d map_hdg=%.0f "
                         "level_mode=%d target_h=%.3f blade_deg=%.2f slope_start=%.3f slope_grad=%.2f%%",
                         path_params_.push_length, path_params_.blade_width, path_params_.shift_angle,
                         path_params_.heading, path_params_.v_push, path_params_.v_reverse,
                         path_params_.num_columns, path_params_.passes_per_col,
                         map_start_corner_, map_push_heading_,
                         static_cast<int>(path_params_.level_mode),
                         path_params_.target_level_height, blade_angle_deg_,
                         path_params_.slope_start_height, path_params_.slope_gradient);
            }));

    // 紧急停止 (立即清除路径, 停车提刀)
    sub_emergency_stop_ = nh_.subscribe<std_msgs::Float64>("/decision/emergency_stop", 1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>(
            [this](const std_msgs::Float64::ConstPtr& msg) {
                if (msg->data > 0.5) {
                    path_ready_ = false;
                    path_.clear();
                    grid_path_.clear();
                    exec_state_ = ExecState::IDLE;
                    sendWalkCommand(0, 0, 0, 0, 0);
                    sendBladeCommand(BladeCmd::RAISE);
                    LOG_WARN("EMERGENCY STOP triggered from console");
                }
            }));

    // [BUG2] 过载检测需要的车辆数据
    sub_vehicle_speed_ = nh_.subscribe<std_msgs::Float64>("/Vehicle_Speed_Vel", 1,
        boost::function<void(const std_msgs::Float64::ConstPtr&)>(
            [this](const std_msgs::Float64::ConstPtr& msg) {
                overload_inputs_.Vehicle_Speed = msg->data;
            }));
    sub_trans_speed_ = nh_.subscribe<std_msgs::Int16>("/Transmission_Speed_Actual", 1,
        boost::function<void(const std_msgs::Int16::ConstPtr&)>(
            [this](const std_msgs::Int16::ConstPtr& msg) {
                overload_inputs_.Trans_Speed = static_cast<double>(msg->data);
            }));

    // [Issue#7] GUI 设置铲刀高度基准点
    sub_set_blade_origin_ = nh_.subscribe<geometry_msgs::Point>("/decision/set_blade_origin", 1,
        boost::function<void(const geometry_msgs::Point::ConstPtr&)>(
            [this](const geometry_msgs::Point::ConstPtr& msg) {
                blade_origin_lat_ = msg->x;
                blade_origin_lon_ = msg->y;
                blade_origin_alt_ = msg->z;
                LOG_INFO("Blade origin set: lat=%.9f, lon=%.9f, alt=%.3f",
                         blade_origin_lat_, blade_origin_lon_, blade_origin_alt_);
            }));
}

void DecisionSystem::initPublishers() {
    // 直接对应 control_node 的订阅话题名
    pub_walk_state_       = nh_.advertise<std_msgs::Float64>("/decision/walk_state", 1);
    pub_main_switch_out_  = nh_.advertise<std_msgs::Float64>("/decision/main_switch", 1);
    pub_terminal_         = nh_.advertise<std_msgs::Float64MultiArray>("/control/terminal", 1);
    pub_reference_        = nh_.advertise<std_msgs::Float64MultiArray>("/control/reference", 1);
    pub_mold_ctrl_flag_   = nh_.advertise<std_msgs::Float64>("/decision/moldboard_control_flag", 1);
    pub_ref_height_       = nh_.advertise<std_msgs::Float64>("/decision/ref_height_middle_moldboard", 1);
    pub_ref_angle_        = nh_.advertise<std_msgs::Float64>("/decision/ref_angle", 1);

    // 状态/调试
    pub_decision_status_  = nh_.advertise<std_msgs::Float64>("/Decision_Status", 1);
    pub_exec_state_       = nh_.advertise<std_msgs::Float64>("/decision/exec_state", 1);
    pub_waypoint_index_   = nh_.advertise<std_msgs::Float64>("/decision/waypoint_index", 1);
    pub_path_viz_         = nh_.advertise<geometry_msgs::Point>("/Path", 1);
    pub_direction_        = nh_.advertise<std_msgs::Float64>("/Direction", 1);
    pub_buzz_flag_        = nh_.advertise<std_msgs::Float64>("/buzz_flag", 1);
    pub_speed_gain_       = nh_.advertise<std_msgs::Float64>("/control_speed_gain", 1);
    pub_point_d_          = nh_.advertise<geometry_msgs::Point>("/PointD_lla", 1);
    pub_point_m_          = nh_.advertise<geometry_msgs::Point>("/PointM_lla", 1);
    pub_avg_height_       = nh_.advertise<std_msgs::Float64>("/decision/avg_height_actual", 1);
    pub_full_path_        = nh_.advertise<std_msgs::Float64MultiArray>("/decision/full_path", 1, true);  // latched
    pub_virtual_grid_     = nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1, true);  // latched
    pub_blade_enable_     = nh_.advertise<std_msgs::Int16>("/Moldboard_Control_Flag_Plan", 1);
    // [Issue#7] 铲刀高度基准点 → moldboard_controller
    pub_blade_origin_     = nh_.advertise<geometry_msgs::Point>("/decision/moldboard_origin", 1, true);  // latched

    // [3D-Level] 铲刀规划下发 → moldboard_controller
    // 把路径点上的 target_height / blade_angle_deg / 3D找平标志 持续下发给 moldboard_controller,
    // 让 computeTargetHeight() 能拿到正确的规划目标, 不再需要 GUI 直接给它。
    pub_avg_height_plan_     = nh_.advertise<std_msgs::Float64>("/decision/avg_height_plan", 1);
    pub_ref_mold_theta_plan_ = nh_.advertise<std_msgs::Float64>("/decision/ref_mold_theta_plan", 1);
    pub_mode_switch_3d_in_   = nh_.advertise<std_msgs::Int16>("/Mode_Switch_3D_Input", 1);
}

//==============================================================================
// 主循环 100Hz
//==============================================================================
void DecisionSystem::timerCallback(const ros::TimerEvent& /*event*/) {
    loop_count_++;

    // 顶层状态机
    processTopState();

    // 路径执行器 (仅在自动作业时运行)
    if (top_state_ == TopState::AUTO_OPERATION) {
        processPathExecutor();
    }

    // 辅助模块 (始终运行, 提供监控数据)
    processMoldboardHeight();
    processOverloadDetect();
    processDecisionLogic();

    // 发布
    publishOutputs();

    // 每秒打印一次摘要
    if (loop_count_ % 100 == 0) {
        loop_count_ = 0;
        LOG_INFO("TopState=%d ExecState=%d WP=%d/%d pos=(%.1f,%.1f) hdg=%.1f",
            static_cast<int>(top_state_), static_cast<int>(exec_state_),
            waypoint_index_, static_cast<int>(path_.size()),
            enu_x_, enu_y_, heading_deg_);
    }
}

//==============================================================================
// 顶层状态: REMOTE_TAKEOVER ↔ AUTO_OPERATION
//==============================================================================
void DecisionSystem::processTopState() {
    // [Fix-BUG-G] 感知超时检查: 即使 detection_completed_ 曾经为 1,
    // 如果感知节点挂了(超时没收到新消息), 也不能继续自动作业。
    bool detection_alive = (detection_completed_ > 0.5) &&
        (!detection_last_time_.isZero()) &&
        ((ros::Time::now() - detection_last_time_).toSec() < DETECTION_TIMEOUT_SEC);

    if (top_state_ == TopState::REMOTE_TAKEOVER) {
        // 进入自动作业的条件: 主开关ON + 感知就绪(且持续在线)
        if (main_switch_ > 0.5 && detection_alive) {
            top_state_ = TopState::AUTO_OPERATION;

            // 锁定 ENU 原点
            if (!origin_locked_) {
                lat0_ = moldboard_height_inputs_.latitude;
                lon0_ = moldboard_height_inputs_.longitude;
                alt0_ = moldboard_height_inputs_.altitude;
                heading0_ = heading_deg_;
                origin_locked_ = true;
                LOG_INFO("ENU origin locked: lat=%f lon=%f heading=%.1f",
                         lat0_, lon0_, heading0_);
            }

            // 没生成过路径则自动生成 (有地图用地图, 无地图用参数化)
            if (!path_ready_) {
                generateNewPath();
            }

            LOG_INFO("→ AUTO_OPERATION");
        }
    } else {
        // 退回遥控的条件: 主开关OFF 或 RTK失效 或 感知超时
        if (main_switch_ < 0.5 || rtk_status_ == 0 || !detection_alive) {
            top_state_ = TopState::REMOTE_TAKEOVER;
            exec_state_ = ExecState::IDLE;
            // 停车指令
            sendWalkCommand(0, 0, 0, 0, 0);
            sendBladeCommand(BladeCmd::RAISE);
            if (!detection_alive && main_switch_ > 0.5) {
                LOG_WARN("→ REMOTE_TAKEOVER (detection timeout! last=%.1fs ago)",
                         detection_last_time_.isZero() ? -1.0 :
                         (ros::Time::now() - detection_last_time_).toSec());
            } else {
                LOG_INFO("→ REMOTE_TAKEOVER (switch=%.0f rtk=%d)", main_switch_, rtk_status_);
            }
        }
    }
}

//==============================================================================
// 路径生成
//==============================================================================
void DecisionSystem::generateNewPath() {
    grid_path_.clear();
    path_.clear();

    auto isPassable = [&](int r, int c) -> bool {
        if (r < 0 || r >= map_rows_ || c < 0 || c >= map_cols_) return false;
        int idx = r * map_cols_ + c;
        return (idx < static_cast<int>(map_data_.size()) && map_data_[idx] == 0);
    };

    bool has_map = (map_received_ && map_rows_ > 0 && map_cols_ > 0 && !map_data_.empty());

    int blade_cells = has_map ? std::max(1, static_cast<int>(std::round(
        path_params_.blade_width / std::max(map_resolution_, 0.01)))) : 0;

    if (!has_map) {
        // ===== 无地图: 参数化 ENU 路径 =====
        // 所有参数通过 /decision/path_params 话题设置, 直接使用 path_params_

        // 没设过 heading 就用当前车头方向
        if (std::abs(path_params_.heading) < 0.01) {
            path_params_.heading = heading0_;
        }
        // 以当前 ENU 位置为起点
        path_params_.start_x = enu_x_;
        path_params_.start_y = enu_y_;

        path_ = path_gen::generatePath(path_mode_, path_params_, true);  // 无地图: 用 num_columns

        const char* mode_name = (path_mode_ == PathMode::ZIGZAG) ? "ZIGZAG(top-shift)" :
                                (path_mode_ == PathMode::UNIDIRECTIONAL) ? "UNIDI(bot-fwd-shift)" : "UNIDI(bot-back-shift)";
        LOG_INFO("NO-MAP %s: push=%.1fm blade=%.1fm heading=%.1f cols=%d ppc=%d start=(%.1f,%.1f) → %d ENU points",
                 mode_name, path_params_.push_length, path_params_.blade_width,
                 path_params_.heading, path_params_.num_columns, path_params_.passes_per_col,
                 path_params_.start_x, path_params_.start_y, (int)path_.size());
        for (int k = 0; k < std::min(8, (int)path_.size()); ++k) {
            const char* tag = (path_[k].blade_cmd == BladeCmd::LEVEL) ? "PUSH" :
                              (path_[k].drive_dir == DriveDir::BACKWARD) ? "REV" : "REPO";
            LOG_INFO("  WP[%d] enu(%.1f,%.1f) %s", k, path_[k].x, path_[k].y, tag);
        }

        // 合成虚拟栅格 + 发布地图和路径 (让 GridMapWidget 能显示)
        synthesizeVirtualGrid();
    } else {
        // === 有地图: 逐条带扫描白色区域 ===

        // 车辆当前栅格位置 (如果没有, 用地图中心)
        int veh_r = (cur_row_ > 0) ? cur_row_ : map_rows_ / 2;
        int veh_c = (cur_col_ > 0) ? cur_col_ : map_cols_ / 2;
        LOG_INFO("Vehicle at grid (%d, %d)", veh_r, veh_c);

        if (test_mode_ && test_passes_ > 0) {
            int min_c = map_cols_, max_c = 0;
            for (int c = 0; c < map_cols_; ++c)
                for (int r = 0; r < map_rows_; ++r)
                    if (isPassable(r, c)) { if (c < min_c) min_c = c; if (c > max_c) max_c = c; break; }
            if (max_c > min_c)
                blade_cells = std::max(1, (max_c - min_c) / test_passes_);
        }

        // ===== 推土方向抽象 =====
        // map_push_heading_: 0=北(row↑), 90=东(col→), 180=南(row↓), 270=西(col←)
        int push_dir_idx = static_cast<int>(map_push_heading_) / 90;
        push_dir_idx = std::max(0, std::min(3, push_dir_idx));
        bool push_along_row = (push_dir_idx == 0 || push_dir_idx == 2);
        bool push_positive  = (push_dir_idx == 0 || push_dir_idx == 1);

        // map_start_corner_: 0=左下 1=右下 2=左上 3=右上
        bool shift_positive;
        if (push_along_row) {
            shift_positive = (map_start_corner_ == 0 || map_start_corner_ == 2);
        } else {
            shift_positive = (map_start_corner_ == 0 || map_start_corner_ == 1);
        }

        LOG_INFO("Push dir=%d along_%s %s, shift_%s, corner=%d",
                 push_dir_idx * 90,
                 push_along_row ? "row" : "col",
                 push_positive ? "+" : "-",
                 shift_positive ? "+" : "-",
                 map_start_corner_);

        // ===== 统一用 Stripe: shift_pos(换列轴坐标), push_lo/push_hi(推土轴范围) =====
        struct Stripe { int shift_pos; int push_lo; int push_hi; };
        std::vector<Stripe> stripes;

        if (push_along_row) {
            // 推沿row, 换列沿col
            int col_min = map_cols_, col_max = 0;
            for (int c = 0; c < map_cols_; ++c)
                for (int r = 0; r < map_rows_; ++r)
                    if (isPassable(r, c)) { if (c < col_min) col_min = c; if (c > col_max) col_max = c; break; }

            auto addCol = [&](int c) {
                int lo = -1, hi = -1;
                for (int r = 0; r < map_rows_; ++r)
                    if (isPassable(r, c)) { if (lo < 0) lo = r; hi = r; }
                if (lo >= 0 && hi > lo) {
                    double scale = std::max(0.1, path_params_.map_scale);
                    if (std::abs(scale - 1.0) > 0.01) {
                        int center = veh_r;
                        lo = std::max(0, std::min(center + static_cast<int>((lo - center) * scale), map_rows_ - 1));
                        hi = std::max(lo + 1, std::min(center + static_cast<int>((hi - center) * scale), map_rows_ - 1));
                    }
                    stripes.push_back({c, lo, hi});
                }
            };
            if (shift_positive)
                for (int c = col_min; c <= col_max; c += blade_cells) addCol(c);
            else
                for (int c = col_max; c >= col_min; c -= blade_cells) addCol(c);
        } else {
            // 推沿col, 换列沿row
            int row_min = map_rows_, row_max = 0;
            for (int r = 0; r < map_rows_; ++r)
                for (int c = 0; c < map_cols_; ++c)
                    if (isPassable(r, c)) { if (r < row_min) row_min = r; if (r > row_max) row_max = r; break; }

            auto addRow = [&](int r) {
                int lo = -1, hi = -1;
                for (int c = 0; c < map_cols_; ++c)
                    if (isPassable(r, c)) { if (lo < 0) lo = c; hi = c; }
                if (lo >= 0 && hi > lo) {
                    double scale = std::max(0.1, path_params_.map_scale);
                    if (std::abs(scale - 1.0) > 0.01) {
                        int center = veh_c;
                        lo = std::max(0, std::min(center + static_cast<int>((lo - center) * scale), map_cols_ - 1));
                        hi = std::max(lo + 1, std::min(center + static_cast<int>((hi - center) * scale), map_cols_ - 1));
                    }
                    stripes.push_back({r, lo, hi});
                }
            };
            if (shift_positive)
                for (int r = row_min; r <= row_max; r += blade_cells) addRow(r);
            else
                for (int r = row_max; r >= row_min; r -= blade_cells) addRow(r);
        }

        if (stripes.empty()) {
            LOG_WARN("No passable stripes found");
            path_ready_ = false;
            return;
        }

        // (push_val, shift_val) → (row, col)
        auto toGrid = [push_along_row](int push_val, int shift_val) -> std::pair<int,int> {
            if (push_along_row) return {push_val, shift_val};
            else                return {shift_val, push_val};
        };

        // grid_path_ 编码: 1.0=推土, -1.0=倒车, 2.0=空驶
        grid_path_.push_back({static_cast<double>(veh_r), static_cast<double>(veh_c), 2.0});

        // 空驶到起点
        {
            int ps = push_positive ? stripes[0].push_lo : stripes[0].push_hi;
            auto g = toGrid(ps, stripes[0].shift_pos);
            if (veh_r != g.first || veh_c != g.second)
                grid_path_.push_back({static_cast<double>(g.first), static_cast<double>(g.second), 2.0});
        }

        if (path_mode_ == PathMode::ZIGZAG) {
            for (size_t i = 0; i < stripes.size(); ++i) {
                auto& s = stripes[i];
                int ps = push_positive ? s.push_lo : s.push_hi;
                int pe = push_positive ? s.push_hi : s.push_lo;
                auto gs = toGrid(ps, s.shift_pos);
                grid_path_.push_back({static_cast<double>(gs.first), static_cast<double>(gs.second), 2.0});
                auto ge = toGrid(pe, s.shift_pos);
                grid_path_.push_back({static_cast<double>(ge.first), static_cast<double>(ge.second), 1.0});
                if (i + 1 < stripes.size()) {
                    auto& ns = stripes[i+1];
                    int nps = push_positive ? ns.push_lo : ns.push_hi;
                    auto gn = toGrid(nps, ns.shift_pos);
                    grid_path_.push_back({static_cast<double>(gn.first), static_cast<double>(gn.second), 2.0});
                }
            }
        } else if (path_mode_ == PathMode::UNIDIRECTIONAL) {
            double angle_rad = std::max(path_params_.shift_angle, 1.0) * M_PI / 180.0;
            int shift_fwd = std::max(1, static_cast<int>(std::round(blade_cells / std::tan(angle_rad))));
            for (size_t i = 0; i < stripes.size(); ++i) {
                auto& s = stripes[i];
                int ps = push_positive ? s.push_lo : s.push_hi;
                int pe = push_positive ? s.push_hi : s.push_lo;
                if (i == 0) {
                    auto gs = toGrid(ps, s.shift_pos);
                    grid_path_.push_back({static_cast<double>(gs.first), static_cast<double>(gs.second), 2.0});
                }
                auto ge = toGrid(pe, s.shift_pos);
                grid_path_.push_back({static_cast<double>(ge.first), static_cast<double>(ge.second), 1.0});
                if (i + 1 < stripes.size()) {
                    auto gb = toGrid(ps, s.shift_pos);
                    grid_path_.push_back({static_cast<double>(gb.first), static_cast<double>(gb.second), -1.0});
                    auto& ns = stripes[i+1];
                    int nps = push_positive ? ns.push_lo : ns.push_hi;
                    int npe = push_positive ? ns.push_hi : ns.push_lo;
                    int dp = push_positive ? std::min(s.push_lo + shift_fwd, npe) : std::max(s.push_hi - shift_fwd, npe);
                    auto gd = toGrid(dp, ns.shift_pos);
                    grid_path_.push_back({static_cast<double>(gd.first), static_cast<double>(gd.second), 2.0});
                    if (dp != nps) {
                        auto gnb = toGrid(nps, ns.shift_pos);
                        grid_path_.push_back({static_cast<double>(gnb.first), static_cast<double>(gnb.second), -1.0});
                    }
                }
            }
        } else {
            // UNIDI_BACK_SHIFT: 倒车回底端 → 斜着倒车到下一列 (底端后方)
            double angle_rad = std::max(path_params_.shift_angle, 1.0) * M_PI / 180.0;
            int shift_fwd = std::max(1, static_cast<int>(std::round(blade_cells / std::tan(angle_rad))));
            for (size_t i = 0; i < stripes.size(); ++i) {
                auto& s = stripes[i];
                int ps = push_positive ? s.push_lo : s.push_hi;
                int pe = push_positive ? s.push_hi : s.push_lo;

                // 第一列起始定位; 后续列从斜倒后的位置开始推
                if (i == 0) {
                    auto gs = toGrid(ps, s.shift_pos);
                    grid_path_.push_back({static_cast<double>(gs.first), static_cast<double>(gs.second), 2.0});
                }

                // 推土: start → end
                auto ge = toGrid(pe, s.shift_pos);
                grid_path_.push_back({static_cast<double>(ge.first), static_cast<double>(ge.second), 1.0});

                if (i + 1 < stripes.size()) {
                    // 倒车回本列底端
                    auto gb = toGrid(ps, s.shift_pos);
                    grid_path_.push_back({static_cast<double>(gb.first), static_cast<double>(gb.second), -1.0});

                    // 斜着倒车到下一列底端后方 (ps - shift_fwd, next_col)
                    auto& ns = stripes[i+1];
                    int back_pos = push_positive ? std::max(ns.push_lo, s.push_lo - shift_fwd)
                                                 : std::min(ns.push_hi, s.push_hi + shift_fwd);
                    auto gback = toGrid(back_pos, ns.shift_pos);
                    grid_path_.push_back({static_cast<double>(gback.first), static_cast<double>(gback.second), -1.0});
                }
            }
        }

        LOG_INFO("Map path: push=%d° %d stripes, %d grid points",
                 push_dir_idx * 90, static_cast<int>(stripes.size()), static_cast<int>(grid_path_.size()));

        // 同步生成 ENU 版本 (给 control_node 执行)
        // 栅格(row,col) → ENU: enu_x = origin_x + col * res, enu_y = origin_y + row * res
        //
        // [Fix-BUG-H] SLOPE_3D 修复:
        //   原代码对所有 LEVEL 点传 local_fwd=0.0 → 全部得到 slope_start_height,
        //   纵坡在路径层面完全丢失。修复后追踪每个推土段的起点, 用实际推进距离算高度。
        double push_seg_start_x = 0, push_seg_start_y = 0;
        bool in_push_seg = false;

        for (const auto& gp : grid_path_) {
            double enu_x = map_origin_x_ + gp.y * map_resolution_;  // col → ENU_x
            double enu_y = map_origin_y_ + gp.x * map_resolution_;  // row → ENU_y
            DriveDir dir;
            BladeCmd blade;
            double h;
            if (gp.dir < 0) {
                dir = DriveDir::BACKWARD;  blade = BladeCmd::RAISE;  // 倒车
                h = RAISE_HEIGHT;
                in_push_seg = false;
            } else if (gp.dir > 1.5) {
                dir = DriveDir::FORWARD;   blade = BladeCmd::RAISE;  // 空驶换列
                h = RAISE_HEIGHT;
                in_push_seg = false;
            } else {
                dir = DriveDir::FORWARD;   blade = BladeCmd::LEVEL;  // 推土
                // 推土段起点 = 进入 LEVEL 前的最后一个非推土点的位置
                if (!in_push_seg) {
                    if (!path_.empty()) {
                        push_seg_start_x = path_.back().x;
                        push_seg_start_y = path_.back().y;
                    } else {
                        push_seg_start_x = enu_x;
                        push_seg_start_y = enu_y;
                    }
                    in_push_seg = true;
                }
                double dx = enu_x - push_seg_start_x;
                double dy = enu_y - push_seg_start_y;
                double local_fwd = std::sqrt(dx * dx + dy * dy);
                h = path_gen::computeLevelHeight(path_params_, local_fwd);
            }
            path_.emplace_back(enu_x, enu_y, blade, dir, h);
        }

        const char* mode_name = (path_mode_ == PathMode::ZIGZAG) ? "ZIGZAG(top-shift)" :
                                (path_mode_ == PathMode::UNIDIRECTIONAL) ? "UNIDI(bot-fwd-shift)" : "UNIDI(bot-back-shift)";
        LOG_INFO("MAP %s: start(%d,%d) %d stripes, blade=%d cells, %d points",
                 mode_name, veh_r, veh_c, (int)stripes.size(), blade_cells, (int)grid_path_.size());
        for (int k = 0; k < std::min(8, (int)grid_path_.size()); ++k) {
            const char* tag = (grid_path_[k].dir < 0) ? "REV" :
                              (grid_path_[k].dir > 1.5) ? "REPO" : "PUSH";
            LOG_INFO("  WP[%d] grid(%.0f,%.0f) → enu(%.1f,%.1f) %s",
                     k, grid_path_[k].x, grid_path_[k].y,
                     path_[k].x, path_[k].y, tag);
        }
    }

    waypoint_index_ = 0;
    path_ready_ = !path_.empty() || !grid_path_.empty();

    if (path_ready_) {
        exec_state_ = ExecState::IDLE;
        publishGridPath();
    } else {
        LOG_WARN("Path generation failed");
    }
}

//==============================================================================
// 通用路径执行器
//==============================================================================
void DecisionSystem::processPathExecutor() {
    switch (exec_state_) {
        case ExecState::IDLE:          execIdle();         break;
        case ExecState::ROTATING:      execRotating();     break;
        case ExecState::DRIVING:       execDriving();      break;
        case ExecState::WAYPOINT_DONE: execWaypointDone();  break;
        case ExecState::OVERLOAD_BACK: execOverloadBack();  break;
        case ExecState::FINISHED:      /* 等待新路径 */      break;
        case ExecState::EMERGENCY:     /* 等待恢复 */        break;
    }
}

void DecisionSystem::execIdle() {
    if (!path_ready_ || path_.empty()) return;
    if (waypoint_index_ >= static_cast<int>(path_.size())) {
        exec_state_ = ExecState::FINISHED;
        sendWalkCommand(0, 0, 0, 0, 0);
        LOG_INFO("All waypoints completed");
        return;
    }

    const auto& wp = path_[waypoint_index_];

    // 计算需要旋转到的航向
    double target_heading = calcTargetHeading(wp);
    double heading_error = target_heading - heading_deg_;
    while (heading_error > 180) heading_error -= 360;
    while (heading_error < -180) heading_error += 360;

    if (std::abs(heading_error) > theta_tolerance_) {
        double omega = path_params_.omega_rotate;
        if (heading_error < 0) omega = -omega;  // 根据误差方向决定旋转方向
        sendWalkCommand(1, 0, target_heading, 0, omega);
        exec_state_ = ExecState::ROTATING;
        LOG_INFO("WP[%d] ROTATING: heading_err=%.1f → target=%.1f omega=%.1f",
                 waypoint_index_, heading_error, target_heading, omega);
    } else {
        // 航向OK, 直接直行
        double dist = calcTargetDistance(wp);
        double v = (wp.drive_dir == DriveDir::FORWARD) ? path_params_.v_push : path_params_.v_reverse;
        sendDriveToTarget(wp, dist, v);
        // [Fix-BUG-H] SLOPE_3D 初始帧修复:
        //   进入 DRIVING 时行驶距离=0, 铲刀应在 slope_start_height,
        //   而不是 wp.target_height (那是终点高度)。
        //   下一帧 execDriving 的实时跟踪会接管。
        if (path_params_.level_mode == LevelMode::SLOPE_3D &&
            wp.blade_cmd == BladeCmd::LEVEL) {
            WayPoint init_wp = wp;
            init_wp.target_height = path_params_.slope_start_height;
            sendBladeCommand(init_wp);
        } else {
            sendBladeCommand(wp);
        }
        // [SLOPE_3D] 记录推土段起点 (纵坡模式下用于实时计算行驶距离)
        if (wp.blade_cmd == BladeCmd::LEVEL) {
            slope_push_start_x_ = enu_x_;
            slope_push_start_y_ = enu_y_;
        }
        exec_state_ = ExecState::DRIVING;
        LOG_INFO("WP[%d] DRIVING: dist=%.1f v=%.2f blade=%d",
                 waypoint_index_, dist, v, static_cast<int>(wp.blade_cmd));
    }
}

void DecisionSystem::execRotating() {
    // 障碍物检查
    double max_risk_rot = 0;
    for (int i = 0; i < 4; i++) max_risk_rot = std::max(max_risk_rot, decision_logic_inputs_.risk_state[i]);
    if (max_risk_rot >= 2) {
        if (!obstacle_paused_) { sendWalkCommand(0, 0, 0, 0, 0); obstacle_paused_ = true;
            LOG_WARN("WP[%d] OBSTACLE during ROTATE → paused", waypoint_index_); }
        return;
    }
    if (obstacle_paused_) {
        obstacle_paused_ = false;
        double th = calcTargetHeading(path_[waypoint_index_]);
        double herr = th - heading_deg_;
        while (herr > 180) herr -= 360;
        while (herr < -180) herr += 360;
        double omega = path_params_.omega_rotate;
        if (herr < 0) omega = -omega;
        sendWalkCommand(1, 0, th, 0, omega);
        LOG_INFO("WP[%d] OBSTACLE cleared → resume rotating", waypoint_index_);
    }

    // decision 自己判断旋转到位 (用 RTK 航向)
    const auto& wp = path_[waypoint_index_];
    double target_heading = calcTargetHeading(wp);
    double heading_error = target_heading - heading_deg_;
    while (heading_error > 180) heading_error -= 360;
    while (heading_error < -180) heading_error += 360;

    if (std::abs(heading_error) < theta_tolerance_) {
        // 旋转完成, 开始直行
        double dist = calcTargetDistance(wp);
        double v = (wp.drive_dir == DriveDir::FORWARD) ? path_params_.v_push : path_params_.v_reverse;
        sendDriveToTarget(wp, dist, v);
        // [Fix-BUG-H] SLOPE_3D 初始帧: 同 execIdle
        if (path_params_.level_mode == LevelMode::SLOPE_3D &&
            wp.blade_cmd == BladeCmd::LEVEL) {
            WayPoint init_wp = wp;
            init_wp.target_height = path_params_.slope_start_height;
            sendBladeCommand(init_wp);
        } else {
            sendBladeCommand(wp);
        }
        // [SLOPE_3D] 记录推土段起点
        if (wp.blade_cmd == BladeCmd::LEVEL) {
            slope_push_start_x_ = enu_x_;
            slope_push_start_y_ = enu_y_;
        }
        exec_state_ = ExecState::DRIVING;
        LOG_INFO("WP[%d] ROTATE done (hdg_err=%.1f) → DRIVING dist=%.1f",
                 waypoint_index_, heading_error, dist);
    }
}

void DecisionSystem::execDriving() {
    // 1. 障碍物检查
    double max_risk_drv = 0;
    for (int i = 0; i < 4; i++) max_risk_drv = std::max(max_risk_drv, decision_logic_inputs_.risk_state[i]);
    if (max_risk_drv >= 2) {
        if (!obstacle_paused_) { sendWalkCommand(0, 0, 0, 0, 0); obstacle_paused_ = true;
            LOG_WARN("WP[%d] OBSTACLE detected → paused", waypoint_index_); }
        return;
    }
    if (obstacle_paused_) {
        obstacle_paused_ = false;
        LOG_INFO("WP[%d] OBSTACLE cleared → resume driving", waypoint_index_);
    }

    // 2. 过载检查 (仅推土时)
    if (waypoint_index_ < static_cast<int>(path_.size()) &&
        path_[waypoint_index_].blade_cmd == BladeCmd::LEVEL &&
        overload_outputs_.overload_status == 1) {
        overload_saved_wp_ = path_[waypoint_index_];
        overload_retry_count_++;
        if (overload_retry_count_ > MAX_OVERLOAD_RETRY) {
            LOG_WARN("WP[%d] OVERLOAD max retry exceeded, skip", waypoint_index_);
            overload_retry_count_ = 0;
            overload_outputs_.overload_status = 0; overload_outputs_.overload_counter = 0;
            exec_state_ = ExecState::WAYPOINT_DONE; return;
        }
        sendBladeCommand(BladeCmd::RAISE);
        // 保存后退起始位置
        overload_back_start_x_ = enu_x_;
        overload_back_start_y_ = enu_y_;
        sendWalkCommand(2, -path_params_.x_back_set, 0, path_params_.v_reverse, 0);
        exec_state_ = ExecState::OVERLOAD_BACK;
        LOG_WARN("WP[%d] OVERLOAD → reverse %.1fm (retry %d/%d)",
                 waypoint_index_, path_params_.x_back_set, overload_retry_count_, MAX_OVERLOAD_RETRY);
        overload_outputs_.overload_status = 0; overload_outputs_.overload_counter = 0;
        return;
    }

    // 3. decision 自己做 2D 到位判断 (用 RTK 位置)
    const auto& wp = path_[waypoint_index_];
    double dist = calcTargetDistance(wp);

    if (dist < position_tolerance_) {
        // 到位!
        sendWalkCommand(0, 0, 0, 0, 0);
        overload_retry_count_ = 0;
        obstacle_paused_ = false;
        exec_state_ = ExecState::WAYPOINT_DONE;
        LOG_INFO("WP[%d] ARRIVED (dist=%.2f < tol=%.2f) → WAYPOINT_DONE",
                 waypoint_index_, dist, position_tolerance_);
        return;
    }

    // 4. [SLOPE_3D] 纵坡模式: 实时更新铲刀目标高度 (每帧)
    //    高度 = slope_start_height + 行驶距离 × slope_gradient / 100
    if (path_params_.level_mode == LevelMode::SLOPE_3D &&
        wp.blade_cmd == BladeCmd::LEVEL) {
        double dx = enu_x_ - slope_push_start_x_;
        double dy = enu_y_ - slope_push_start_y_;
        double traveled = std::sqrt(dx * dx + dy * dy);
        double slope_h = path_params_.slope_start_height +
                         traveled * path_params_.slope_gradient / 100.0;

        // 构造临时 waypoint 发送实时高度
        WayPoint slope_wp = wp;
        slope_wp.target_height = slope_h;
        sendBladeCommand(slope_wp);
    }

    // 5. 还没到, 持续更新速度指令 (近目标减速)
    double v_max = (wp.drive_dir == DriveDir::FORWARD) ? path_params_.v_push : path_params_.v_reverse;
    sendDriveToTarget(wp, dist, v_max);
}

// 过载后退完成 → 重新推向原目标
void DecisionSystem::execOverloadBack() {
    // decision 自己判断后退距离
    double dx = enu_x_ - overload_back_start_x_;
    double dy = enu_y_ - overload_back_start_y_;
    double backed = std::sqrt(dx * dx + dy * dy);

    if (backed >= path_params_.x_back_set * 0.9) {  // 后退了 90% 就算到位
        const auto& wp = overload_saved_wp_;
        double dist = calcTargetDistance(wp);
        double v = path_params_.v_push;
        sendDriveToTarget(wp, dist, v);
        sendBladeCommand(wp);  // [3D-Level] 恢复推土, 用原 waypoint 的 target_height
        // [SLOPE_3D] 不重置推土起点: 过载后退恢复时, 保持原始起点, 坡度高度按总行驶距离算
        exec_state_ = ExecState::DRIVING;
        LOG_INFO("WP[%d] OVERLOAD_BACK done (backed=%.1f) → resume (dist=%.1f)",
                 waypoint_index_, backed, dist);
    }
}

void DecisionSystem::execWaypointDone() {
    // 到位后: 停车, 清状态, 推进到下一个点
    sendWalkCommand(0, 0, 0, 0, 0);
    obstacle_paused_ = false;
    overload_retry_count_ = 0;
    waypoint_index_++;

    if (waypoint_index_ >= static_cast<int>(path_.size())) {
        exec_state_ = ExecState::FINISHED;
        sendBladeCommand(BladeCmd::RAISE);
        LOG_INFO("Path finished (%d points)", static_cast<int>(path_.size()));
    } else {
        // 立即进入 IDLE 处理下一个点
        exec_state_ = ExecState::IDLE;
    }
}

//==============================================================================
// 辅助计算
//==============================================================================
double DecisionSystem::calcTargetHeading(const WayPoint& wp) const {
    double dx = wp.x - enu_x_;
    double dy = wp.y - enu_y_;

    if (wp.drive_dir == DriveDir::BACKWARD) {
        // 后退: 目标在身后, 航向指向反方向
        dx = -dx;
        dy = -dy;
    }

    // ENU: X=东, Y=北, 航向=北偏东
    double heading = std::atan2(dx, dy) * 180.0 / M_PI;
    if (heading < 0) heading += 360;
    return heading;
}

double DecisionSystem::calcTargetDistance(const WayPoint& wp) const {
    double dx = wp.x - enu_x_;
    double dy = wp.y - enu_y_;
    return std::sqrt(dx * dx + dy * dy);
}

//==============================================================================
// 发送指令给 control_node
//==============================================================================
void DecisionSystem::sendWalkCommand(int walk_state, double x_terminal,
                                     double theta_terminal,
                                     double v_ref, double omega_ref) {
    std_msgs::Float64 ws;
    ws.data = static_cast<double>(walk_state);
    pub_walk_state_.publish(ws);

    std_msgs::Float64MultiArray term;
    term.data = {x_terminal, 0.0, theta_terminal};
    pub_terminal_.publish(term);

    std_msgs::Float64MultiArray ref;
    ref.data = {v_ref, omega_ref};
    pub_reference_.publish(ref);
}

void DecisionSystem::sendDriveToTarget(const WayPoint& wp, double dist, double v_max) {
    // 近目标减速: hold_distance 内线性减速, 最低 0.1m/s
    const double hold_distance = 2.0;  // 2m 内开始减速
    double v = v_max;
    if (dist < hold_distance) {
        v = std::max(0.1, v_max * dist / hold_distance);
    }

    // 实时航向纠偏: 算当前位置到目标的航向, 作为参考
    double target_heading = calcTargetHeading(wp);
    double heading_error = target_heading - heading_deg_;
    while (heading_error > 180) heading_error -= 360;
    while (heading_error < -180) heading_error += 360;

    // walk_state=2 (直行), x_terminal=剩余距离, theta=航向纠偏
    double x_term = (wp.drive_dir == DriveDir::FORWARD) ? dist : -dist;

    // 航向纠偏: 将航向误差转换为角速度修正量
    const double Kp_heading = 0.5;  // 纠偏增益 (deg/s per deg), 可通过参数调整
    double omega_correction = Kp_heading * heading_error;
    // 限幅: 不超过最大旋转角速度
    omega_correction = std::max(-path_params_.omega_rotate,
                       std::min(path_params_.omega_rotate, omega_correction));
    sendWalkCommand(2, x_term, heading_error, v, omega_correction);
}

//==============================================================================
// 铲刀指令下发
//
// [3D-Level] 规划层持续下发给 moldboard_controller 的三个量:
//   /decision/moldboard_control_flag   → 铲刀是否跟踪目标 (0=提刀, 1=跟踪)
//   /decision/avg_height_plan          → 目标高度 (ENU-Up 绝对高程, 米)
//   /decision/ref_mold_theta_plan      → 铲刀横滚角 (度)
//   /Mode_Switch_3D_Input              → 3D模式标志 (=1, 本系统永远按3D绝对高程控制)
//
// 两个重载:
//   sendBladeCommand(cmd)        — 紧急/异常场景: RAISE 按 RAISE_HEIGHT 发,
//                                   LEVEL 按路径参数 target_level_height 发。
//   sendBladeCommand(const WayPoint&) — 正常执行: 从 waypoint 读 target_height。
//==============================================================================
void DecisionSystem::sendBladeCommand(BladeCmd cmd) {
    // [Fix-v19] 话题对齐修复:
    //   原代码只发 /decision/avg_height_plan 和 /decision/ref_mold_theta_plan (通路X,给moldboard_controller),
    //   但 control_node 订阅的是 /decision/ref_height_middle_moldboard 和 /decision/ref_angle,
    //   话题名不对齐 → control 的 ref 永远是 0 → 铲刀 PID 无法跟踪。
    //   修复后双发: 同时发两组话题, 既保持 moldboard 端兼容, 又让 control 的 PID 收到目标。
    std_msgs::Float64 flag, h, angle;
    std_msgs::Int16 m3d;

    switch (cmd) {
        case BladeCmd::RAISE:
            flag.data  = 0.0;  // PID 关 (提刀, 液压压到最高)
            h.data     = RAISE_HEIGHT;
            break;
        case BladeCmd::LEVEL:
            flag.data  = 1.0;  // PID 开, 跟踪目标
            // [SLOPE_3D] 纵坡模式用起始高度, 找平模式用目标高程
            h.data = (path_params_.level_mode == LevelMode::SLOPE_3D)
                     ? path_params_.slope_start_height
                     : path_params_.target_level_height;
            break;
        case BladeCmd::HOLD:
        default:
            // 保持当前, 不改 flag 也不改目标 (直接返回)
            return;
    }
    angle.data = blade_angle_deg_;
    m3d.data   = 1;  // 本系统的铲刀控制本质就是 3D 绝对高程跟踪

    pub_mold_ctrl_flag_.publish(flag);
    // [Fix-v19] 双发: 新话题 (给 control 的 PID) + 老话题 (给 moldboard_controller 兼容)
    pub_ref_height_.publish(h);              // → /decision/ref_height_middle_moldboard (control 用)
    pub_ref_angle_.publish(angle);           // → /decision/ref_angle (control 用)
    pub_avg_height_plan_.publish(h);         // → /decision/avg_height_plan (moldboard 兼容保留)
    pub_ref_mold_theta_plan_.publish(angle); // → /decision/ref_mold_theta_plan (moldboard 兼容保留)
    pub_mode_switch_3d_in_.publish(m3d);
}

void DecisionSystem::sendBladeCommand(const WayPoint& wp) {
    // [Fix-v19] 同上, 双发
    std_msgs::Float64 flag, h, angle;
    std_msgs::Int16 m3d;

    switch (wp.blade_cmd) {
        case BladeCmd::RAISE:
            flag.data = 0.0;
            break;
        case BladeCmd::LEVEL:
            flag.data = 1.0;
            break;
        case BladeCmd::HOLD:
        default:
            return;  // 保持当前
    }
    h.data     = wp.target_height;   // 从 waypoint 取路径生成器写好的目标高度
    angle.data = blade_angle_deg_;
    m3d.data   = 1;

    pub_mold_ctrl_flag_.publish(flag);
    // [Fix-v19] 双发: 新话题 (给 control 的 PID) + 老话题 (给 moldboard_controller 兼容)
    pub_ref_height_.publish(h);              // → /decision/ref_height_middle_moldboard (control 用)
    pub_ref_angle_.publish(angle);           // → /decision/ref_angle (control 用)
    pub_avg_height_plan_.publish(h);         // → /decision/avg_height_plan (moldboard 兼容保留)
    pub_ref_mold_theta_plan_.publish(angle); // → /decision/ref_mold_theta_plan (moldboard 兼容保留)
    pub_mode_switch_3d_in_.publish(m3d);
}

//==============================================================================
// 辅助模块 — 决策逻辑
//==============================================================================
void DecisionSystem::processDecisionLogic() {
    // 风险状态处理
    double max_risk = 0;
    for (int i = 0; i < 4; i++) {
        max_risk = std::max(max_risk, decision_logic_inputs_.risk_state[i]);
    }

    if (max_risk >= 2) {
        decision_logic_outputs_.Decision_Status = 1;  // 紧急制动
        decision_logic_outputs_.control_speed_gain = 0;
        decision_logic_outputs_.buzz_flag = 1;
    } else if (max_risk >= 1) {
        decision_logic_outputs_.Decision_Status = 2;  // 减速
        decision_logic_outputs_.control_speed_gain = decision_logic_inputs_.control_speed_gain_qt_risk;
        decision_logic_outputs_.buzz_flag = 1;
    } else {
        decision_logic_outputs_.Decision_Status = 0;  // 正常
        decision_logic_outputs_.control_speed_gain = 1.0;
        decision_logic_outputs_.buzz_flag = 0;
    }

    // 铲刀控制标志 (传递给 control_node)
    decision_logic_outputs_.Moldboard_Control_Flag =
        (exec_state_ == ExecState::DRIVING && 
         waypoint_index_ < static_cast<int>(path_.size()) &&
         path_[waypoint_index_].blade_cmd == BladeCmd::LEVEL) ? 1.0 : 0.0;
}

//==============================================================================
// 辅助模块 — 过载检测
//==============================================================================
void DecisionSystem::processOverloadDetect() {
    if (exec_state_ == ExecState::DRIVING) {
        bool overloaded =
            overload_inputs_.Trans_Speed > overload_inputs_.Trans_Speed_Limit &&
            overload_inputs_.Vehicle_Speed < overload_inputs_.Vehicle_Speed_Limit;
        if (overloaded) {
            overload_outputs_.overload_counter++;
            if (overload_outputs_.overload_counter > overload_inputs_.time_num) {
                overload_outputs_.overload_status = 1;
                LOG_WARN("Overload detected! counter=%d", overload_outputs_.overload_counter);
            }
        } else {
            overload_outputs_.overload_counter = 0;
            overload_outputs_.overload_status = 0;
        }
    } else {
        overload_outputs_.overload_counter = 0;
        overload_outputs_.overload_status = 0;
    }
}

//==============================================================================
// 辅助模块 — 铲刀高度计算
//==============================================================================
void DecisionSystem::processMoldboardHeight() {
    if (!origin_locked_) return;

    std::array<double, 3> pointA_lla = {
        moldboard_height_inputs_.latitude,
        moldboard_height_inputs_.longitude,
        moldboard_height_inputs_.altitude
    };
    std::array<double, 3> vehicle_angle = {
        moldboard_height_inputs_.vehicle_yaw,
        moldboard_height_inputs_.vehicle_pitch,
        moldboard_height_inputs_.vehicle_roll
    };
    std::array<double, 3> imu_angle = {
        0,  // yaw (not used)
        moldboard_height_inputs_.imu_pitch,
        moldboard_height_inputs_.imu_roll
    };

    // [Issue#7] 使用运行时基准点
    simulink::Vec3 work_origin = {blade_origin_lat_, blade_origin_lon_, blade_origin_alt_};
    auto result = simulink::Moldboard_Pose_Calc(pointA_lla, vehicle_angle, imu_angle, work_origin);

    moldboard_height_outputs_.PointD_lla[0] = result.PointD_lla[0];
    moldboard_height_outputs_.PointD_lla[1] = result.PointD_lla[1];
    moldboard_height_outputs_.PointD_lla[2] = result.PointD_lla[2];
    moldboard_height_outputs_.PointM_lla[0] = result.PointM_lla[0];
    moldboard_height_outputs_.PointM_lla[1] = result.PointM_lla[1];
    moldboard_height_outputs_.PointM_lla[2] = result.PointM_lla[2];

    // 平均高度 (两端 ENU 高度取平均)
    simulink::Vec3 lla_d = {result.PointD_lla[0], result.PointD_lla[1], result.PointD_lla[2]};
    simulink::Vec3 lla_m = {result.PointM_lla[0], result.PointM_lla[1], result.PointM_lla[2]};
    // [Issue#7] 使用运行时基准点 (不再硬编码 INIT_ORIGIN)
    auto enu_d = simulink::lla2enu_internal(lla_d, work_origin);
    auto enu_m = simulink::lla2enu_internal(lla_m, work_origin);
    moldboard_height_outputs_.Avg_Height_Actual = (enu_d[2] + enu_m[2]) / 2.0;  // [2] = UP
}

//==============================================================================
// 发布输出
//==============================================================================
void DecisionSystem::publishOutputs() {
    std_msgs::Float64 f;

    // 主开关透传给 control_node
    f.data = (top_state_ == TopState::AUTO_OPERATION) ? 1.0 : 0.0;
    pub_main_switch_out_.publish(f);

    // 执行器状态
    f.data = static_cast<double>(exec_state_);
    pub_exec_state_.publish(f);

    f.data = static_cast<double>(waypoint_index_);
    pub_waypoint_index_.publish(f);

    // 决策逻辑输出
    f.data = decision_logic_outputs_.Decision_Status;
    pub_decision_status_.publish(f);

    f.data = decision_logic_outputs_.buzz_flag;
    pub_buzz_flag_.publish(f);

    f.data = decision_logic_outputs_.control_speed_gain;
    pub_speed_gain_.publish(f);

    // 当前路径点可视化 (兼容 GridMapWidget)
    if (waypoint_index_ < static_cast<int>(path_.size())) {
        const auto& wp = path_[waypoint_index_];
        geometry_msgs::Point p;
        p.x = calcTargetHeading(wp);      // theta (用于地图箭头方向)
        p.y = calcTargetDistance(wp);      // distance (用于箭头长度)
        pub_path_viz_.publish(p);

        f.data = (wp.drive_dir == DriveDir::FORWARD) ? 1.0 : -1.0;
        pub_direction_.publish(f);
    }

    // 铲刀高度
    geometry_msgs::Point pd, pm;
    pd.x = moldboard_height_outputs_.PointD_lla[0];
    pd.y = moldboard_height_outputs_.PointD_lla[1];
    pd.z = moldboard_height_outputs_.PointD_lla[2];
    pub_point_d_.publish(pd);
    pm.x = moldboard_height_outputs_.PointM_lla[0];
    pm.y = moldboard_height_outputs_.PointM_lla[1];
    pm.z = moldboard_height_outputs_.PointM_lla[2];
    pub_point_m_.publish(pm);

    f.data = moldboard_height_outputs_.Avg_Height_Actual;
    pub_avg_height_.publish(f);

    // 自动模式铲刀硬件使能: 推土中(LEVEL)自动开启, 其他状态关闭
    // 主开关关闭时不发, 由 GUI 的手动使能控制
    if (top_state_ == TopState::AUTO_OPERATION) {
        std_msgs::Int16 blade_en;
        blade_en.data = (exec_state_ == ExecState::DRIVING &&
                         waypoint_index_ < static_cast<int>(path_.size()) &&
                         path_[waypoint_index_].blade_cmd == BladeCmd::LEVEL) ? 1 : 0;
        pub_blade_enable_.publish(blade_en);
    }

    // [Issue#7] 持续发布铲刀高度基准点给 moldboard_controller
    {
        geometry_msgs::Point bo;
        bo.x = blade_origin_lat_;
        bo.y = blade_origin_lon_;
        bo.z = blade_origin_alt_;
        pub_blade_origin_.publish(bo);
    }
}

//==============================================================================
// ROS 回调
//==============================================================================
void DecisionSystem::mainSwitchCallback(const std_msgs::Float64::ConstPtr& msg) {
    main_switch_ = msg->data;
}

void DecisionSystem::detectionCompletedCallback(const std_msgs::Float64::ConstPtr& msg) {
    detection_completed_ = msg->data;
    // [Fix-BUG-G] 记录最后收到感知就绪的时间
    if (msg->data > 0.5) {
        detection_last_time_ = ros::Time::now();
    }
}

void DecisionSystem::rtkCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // 预留 RTK 状态判断
    (void)msg;
}

void DecisionSystem::llaCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    moldboard_height_inputs_.latitude  = msg->point.y;  // LLA格式: x=lon, y=lat
    moldboard_height_inputs_.longitude = msg->point.x;
    moldboard_height_inputs_.altitude  = msg->point.z;

    // 更新 ENU 位置 (仅在 /Navigate_location 不活跃时才自算)
    if (origin_locked_) {
        if (use_nav_loc_enu_ &&
            (ros::Time::now() - nav_loc_last_time_).toSec() < NAV_LOC_TIMEOUT_SEC) {
            return;  // 感知ENU在线, 跳过自算
        }
        use_nav_loc_enu_ = false;

        simulink::Vec3 lla = {msg->point.y, msg->point.x, msg->point.z};
        simulink::Vec3 lla0 = {lat0_, lon0_, alt0_};
        auto enu = simulink::lla2enu_internal(lla, lla0);
        enu_x_ = enu[0];  // East
        enu_y_ = enu[1];  // North
    }
}

void DecisionSystem::angleHeadingCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    heading_rad_ = msg->point.x;
    heading_deg_ = msg->point.x * 180.0 / M_PI;
    // [Fix-v18] can_to_ros 发布单位=弧度, 但 simulink::Moldboard_Pose_Calc 要求度 (内部 deg2rad)。
    //   原代码直接存弧度导致姿态补偿完全失效 (例:yaw=0.5rad 被当成 0.5°)。
    constexpr double RAD2DEG = 180.0 / M_PI;
    moldboard_height_inputs_.vehicle_yaw   = msg->point.x * RAD2DEG;
    moldboard_height_inputs_.vehicle_pitch = msg->point.y * RAD2DEG;
    moldboard_height_inputs_.vehicle_roll  = msg->point.z * RAD2DEG;
}

void DecisionSystem::ahrsImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // [注释澄清] can_to_ros.cpp:337-345 约定: orientation_covariance[0]=roll, [1]=pitch, [2]=yaw (度)。
    // imu_yaw 在下游 Moldboard_Pose_Calc 里会被车体 yaw 直接覆盖, 所以这里不读 [2]。
    imu_roll_ = msg->orientation_covariance[0];  // decision 自用
    moldboard_height_inputs_.imu_pitch = msg->orientation_covariance[1];
    moldboard_height_inputs_.imu_roll  = msg->orientation_covariance[0];
}

void DecisionSystem::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_rows_ = static_cast<int>(msg->info.height);
    map_cols_ = static_cast<int>(msg->info.width);
    map_resolution_ = msg->info.resolution;
    map_origin_x_ = msg->info.origin.position.x;  // 栅格(0,0)的ENU_x
    map_origin_y_ = msg->info.origin.position.y;  // 栅格(0,0)的ENU_y
    map_data_.assign(msg->data.begin(), msg->data.end());
    map_received_ = true;

    LOG_INFO("Map received: %dx%d res=%.1f origin=(%.1f, %.1f)",
             map_rows_, map_cols_, map_resolution_, map_origin_x_, map_origin_y_);

    // 收到地图后, 如果在自动模式且没有路径, 自动生成
    if (top_state_ == TopState::AUTO_OPERATION && !path_ready_) {
        generateNewPath();
    }
}

void DecisionSystem::occupancyLocationCallback(const geometry_msgs::Point::ConstPtr& msg) {
    // /Navigate_location 给的是 ENU 坐标(东北天), 同时用于:
    // 1) 转成栅格行列供路径生成
    // 2) 作为有地图场景下 enu_x_/enu_y_ 的数据源(供路径执行器算距离)
    enu_x_ = msg->x;
    enu_y_ = msg->y;
    nav_loc_last_time_ = ros::Time::now();
    use_nav_loc_enu_ = true;

    if (map_resolution_ > 0) {
        cur_col_ = static_cast<int>((msg->x - map_origin_x_) / map_resolution_);
        cur_row_ = static_cast<int>((msg->y - map_origin_y_) / map_resolution_);
    }
}

void DecisionSystem::navigationCoordinateCallback(const std_msgs::Float64::ConstPtr& msg) {
    (void)msg;  // 废弃话题, 保留回调避免话题断开
}

// [Issue#4] 注释掉 — /Location_real 当前无发布者, 回调无条件覆盖 enu_x_/enu_y_ 是隐患。
// 保留代码以备将来恢复。
// void DecisionSystem::locationRealCallback(const geometry_msgs::Point::ConstPtr& msg) {
//     enu_x_ = msg->x;
//     enu_y_ = msg->y;
// }

void DecisionSystem::riskStateCallback(const std_msgs::Int8MultiArray::ConstPtr& msg) {
    for (int i = 0; i < 4 && i < static_cast<int>(msg->data.size()); i++) {
        decision_logic_inputs_.risk_state[i] = static_cast<double>(msg->data[i]);
    }
}

void DecisionSystem::moldOverloadCallback(const std_msgs::Int16::ConstPtr& msg) {
    overload_outputs_.overload_status = msg->data;
}

void DecisionSystem::pathModeCallback(const std_msgs::Float64::ConstPtr& msg) {
    int mode = static_cast<int>(msg->data);
    if (mode >= 0 && mode <= 2) {
        path_mode_ = static_cast<PathMode>(mode);
        LOG_INFO("Path mode changed to: %d", mode);
    }
}

void DecisionSystem::synthesizeVirtualGrid() {
    if (path_.empty()) return;

    // 1. 计算 ENU 路径的边界
    double min_x = path_[0].x, max_x = path_[0].x;
    double min_y = path_[0].y, max_y = path_[0].y;
    for (const auto& wp : path_) {
        min_x = std::min(min_x, wp.x); max_x = std::max(max_x, wp.x);
        min_y = std::min(min_y, wp.y); max_y = std::max(max_y, wp.y);
    }

    // 2. 扩展边界留出边距
    double margin = 5.0;  // 5m 边距
    min_x -= margin; min_y -= margin;
    max_x += margin; max_y += margin;

    // 3. 设置虚拟地图参数 (分辨率 1m)
    double vres = 1.0;
    int vcols = static_cast<int>(std::ceil((max_x - min_x) / vres));
    int vrows = static_cast<int>(std::ceil((max_y - min_y) / vres));
    vcols = std::max(vcols, 10);
    vrows = std::max(vrows, 10);

    // 更新内部地图参数 (给后续逻辑用)
    map_resolution_ = vres;
    map_origin_x_ = min_x;
    map_origin_y_ = min_y;
    map_rows_ = vrows;
    map_cols_ = vcols;

    // 4. 发布虚拟 OccupancyGrid (全白/可通行)
    nav_msgs::OccupancyGrid vgrid;
    vgrid.header.stamp = ros::Time::now();
    vgrid.header.frame_id = "map";
    vgrid.info.resolution = vres;
    vgrid.info.width = vcols;
    vgrid.info.height = vrows;
    vgrid.info.origin.position.x = min_x;
    vgrid.info.origin.position.y = min_y;
    vgrid.data.assign(vrows * vcols, 0);  // 全部可通行
    pub_virtual_grid_.publish(vgrid);

    // 5. 将 ENU 路径点转为虚拟栅格坐标
    grid_path_.clear();
    for (const auto& wp : path_) {
        double gr = (wp.y - min_y) / vres;
        double gc = (wp.x - min_x) / vres;
        double dir;
        if (wp.blade_cmd == BladeCmd::LEVEL) dir = 1.0;       // 推土
        else if (wp.drive_dir == DriveDir::BACKWARD) dir = -1.0;  // 倒车
        else dir = 2.0;                                          // 空驶
        grid_path_.push_back({gr, gc, dir});
    }

    // 6. 发布栅格路径
    publishGridPath();

    LOG_INFO("Virtual grid: %dx%d res=%.1f origin=(%.1f,%.1f) → %d grid points",
             vrows, vcols, vres, min_x, min_y, static_cast<int>(grid_path_.size()));
}

void DecisionSystem::publishGridPath() {
    // 直接发布栅格坐标路径: [row0, col0, dir0, row1, col1, dir1, ...]
    std_msgs::Float64MultiArray msg;
    msg.data.reserve(grid_path_.size() * 3);
    for (const auto& gp : grid_path_) {
        msg.data.push_back(gp.x);    // row
        msg.data.push_back(gp.y);    // col
        msg.data.push_back(gp.dir);  // 1前进/-1后退
    }
    pub_full_path_.publish(msg);
    LOG_INFO("Published grid path: %d points", static_cast<int>(grid_path_.size()));
}
