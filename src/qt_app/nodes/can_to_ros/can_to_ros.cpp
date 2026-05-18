/**
 * @file can_to_ros.cpp
 * @brief Can To Ros
 * @author dozer-dev
 * @date 2026-03-15
 */
#include "can_to_ros.h"
#include <chrono>
#include <ros/console.h>
#include <errno.h>
#include <string.h>

can_to_ros::can_to_ros(ros::NodeHandle &nodeHandle) : nh(nodeHandle)
{
    // 初始化CAN配置
    init_can_configs();
    
    // 初始化发布者
    init_publishers();
    
    // 初始化所有CAN设备
    bool all_success = true;
    for (int i = 0; i < DEVICE_COUNT; ++i) {
        if (!init_can_device(static_cast<CanDevice>(i))) {
            ROS_ERROR("Failed to initialize CAN device: %d", i);
            all_success = false;
        }
    }
    
    if (!all_success) {
        ROS_WARN("Some CAN devices failed to initialize");
    }
}

// 初始化CAN配置
void can_to_ros::init_can_configs()
{
    // 注意：以下 CAN ID 均为示例占位值，实际使用时应按目标车辆的
    // CAN 总线协议文档逐一替换（can0: RTK/IMU 传感器总线，can1: 整车控制总线）
    can_configs_[DEVICE_IMU_GYRO_RAW] = {"can0", 0x101, false};    // IMU 角速度原始值
    can_configs_[DEVICE_IMU_ACCEL_RAW] = {"can0", 0x102, false};    // IMU 加速度原始值
    can_configs_[DEVICE_GPS_LONGITUDE] = {"can0", 0x103, false};    // 经度
    can_configs_[DEVICE_GPS_LATITUDE] = {"can0", 0x104, false};    // 纬度
    can_configs_[DEVICE_GPS_ALTITUDE] = {"can0", 0x105, false};    // 高度
    can_configs_[DEVICE_RTK_STATUS] = {"can0", 0x106, false};    // RTK状态
    can_configs_[DEVICE_GPS_HEADING] = {"can0", 0x107, false};    // 航向角
    can_configs_[DEVICE_POS_SIGMA] = {"can0", 0x108, false};    // 位置标准差pos_sigma
    can_configs_[DEVICE_VEHICLE_SPEED_ENU] = {"can0", 0x109, false};    // 车速 Vel_ENU
    can_configs_[DEVICE_VEHICLE_ACCEL] = {"can0", 0x10A, false};    // 车辆坐标系加速度
    can_configs_[DEVICE_HEADING_VELOCITY] = {"can0", 0x10B, false};    // 航向角速度
    can_configs_[DEVICE_ENGINE_TORQUE_SPEED] = {"can1", 0x201, true}; // 发动机实际扭矩百分比、转速
    can_configs_[DEVICE_TRANSMISSION_SPEED] = {"can1", 0x202, true}; // 变速箱转速
    can_configs_[DEVICE_OUTCURR] = {"can1", 0x203, true}; // 输出电流
    can_configs_[DEVICE_IMU_ACC] = {"can0", 0x00, false};  // IMU设备(特殊处理)
    can_configs_[DEVICE_IMU_AHRS] = {"can0", 0x00, false};// IMU设备(特殊处理)

        // 新增配置
    can_configs_[DEVICE_FUEL_TOTAL] = {"can1", 0x204, true};        // 燃料使用总量
    can_configs_[DEVICE_ENGINE_TOTAL_HOURS] = {"can1", 0x205, true}; // 发动机工作时间
    can_configs_[DEVICE_VEHICLE_SPEED_DISPLAY] = {"can1", 0x206, true}; // 车速显示
    can_configs_[DEVICE_COOLANT_TEMP] = {"can1", 0x207, true};      // 发动机冷却液温度
    can_configs_[DEVICE_WARNING_LIGHTS] = {"can1", 0x208, true};    // 警告灯状态和故障信息
    can_configs_[DEVICE_WARNING_LIGHTS2] = {"can1", 0x209, true};   // 警告灯状态2
    can_configs_[DEVICE_SPEED_LIMIT] = {"can1", 0x20A, true};       // 速度限制状态
    can_configs_[DEVICE_GEAR_MODE] = {"can1", 0x20B, true};         // 档位模式
    can_configs_[DEVICE_LOCK_STATUS] = {"can1", 0x20C, true};       // 锁车状态
    can_configs_[DEVICE_HANDLE_INFO] = {"can1", 0x20D, true};       // 手柄信息
    can_configs_[DEVICE_FUEL_LEVEL] = {"can1", 0x20E, true};        // 燃油油位
    can_configs_[DEVICE_CONTROL_INPUTS] = {"can1", 0x20F, true};    // 控制输入
    can_configs_[DEVICE_TRANSMISSION_VALVE] = {"can1", 0x210, true}; // 变速箱档位阀
    can_configs_[DEVICE_ALARM_STATUS] = {"can1", 0x211, true};      // 报警状态
    can_configs_[DEVICE_UREA_CONCENTRATION] = {"can1", 0x212, true}; // 尿素浓度
}

// 初始化所有发布者
void can_to_ros::init_publishers()
{
    // RTK相关
    pub_LLA = nh.advertise<geometry_msgs::PointStamped>("/LLA", 1);
    pub_Angle_Heading = nh.advertise<geometry_msgs::PointStamped>("/Angle_Heading", 1);
    pub_Vehicle_Speed = nh.advertise<geometry_msgs::TwistStamped>("/Vehicle_Speed", 1);
    pub_Vehicle_Speed_Vel = nh.advertise<std_msgs::Float64>("/Vehicle_Speed_Vel", 1);
    pub_Angle_Heading_Velocity = nh.advertise<std_msgs::Float64>("/Angle_Heading_Velocity", 1);
    pub_RTK = nh.advertise<sensor_msgs::NavSatFix>("/RTK", 1);
    pub_IMU_Accel = nh.advertise<geometry_msgs::PointStamped>("/RTK_IMU_Accel_V", 1);
    pub_RTK_IMU = nh.advertise<sensor_msgs::Imu>("/RTK_IMU", 1);
    pub_PosSigma = nh.advertise<geometry_msgs::PointStamped>("/PosSigma", 1);

    // 发动机相关
    pub_Engine_Torque_Actual = nh.advertise<std_msgs::Int8>("/Engine_Torque_Actual", 1);
    pub_Engine_Speed_Actual = nh.advertise<std_msgs::Float64>("/Engine_Speed_Actual", 1);
    
    // 变速箱相关
    pub_Transmission_Speed_Actual = nh.advertise<std_msgs::Int16>("/Transmission_Speed_Actual", 1);
    
    // 旧IMU相关
    pub_IMU_Angular_Velocity = nh.advertise<geometry_msgs::PointStamped>("/IMU_Angular_Velocity", 1);
    pub_IMU_Pitch = nh.advertise<std_msgs::Float64>("/IMU_Pitch", 1);
    pub_IMU_Roll = nh.advertise<std_msgs::Float64>("/IMU_Roll", 1);
    
    // 新增AHRS IMU（s
    pub_ahrs_imu = nh.advertise<sensor_msgs::Imu>("/AHRS_IMU", 10); 

    // 输出电流
    pub_OutCurr = nh.advertise<std_msgs::UInt16MultiArray>("/OUTPUT_CURRENT", 1);

        // 新增发布者
    pub_Fuel_Total = nh.advertise<std_msgs::Float64>("/Fuel_Total", 1);
    pub_Engine_Total_Hours = nh.advertise<std_msgs::Float64>("/Engine_Total_Hours", 1);
    pub_Vehicle_Speed_Display = nh.advertise<std_msgs::Float64>("/Vehicle_Speed_Display", 1);
    pub_Coolant_Temperature = nh.advertise<std_msgs::Float64>("/Coolant_Temperature", 1);
    pub_Protection_Light = nh.advertise<std_msgs::UInt8>("/Protection_Light", 1);
    pub_Environment_Warning = nh.advertise<std_msgs::UInt8>("/Environment_Warning", 1);
    pub_Red_Stop_Light = nh.advertise<std_msgs::UInt8>("/Red_Stop_Light", 1);
    pub_MIL_Light = nh.advertise<std_msgs::UInt8>("/MIL_Light", 1);
    pub_SPN = nh.advertise<std_msgs::UInt16>("/SPN", 1);
    pub_FMI = nh.advertise<std_msgs::UInt8>("/FMI", 1);
    pub_Fault_Count = nh.advertise<std_msgs::UInt8>("/Fault_Count", 1);
    pub_Protection_Light2 = nh.advertise<std_msgs::UInt8>("/Protection_Light2", 1);
    pub_Environment_Warning2 = nh.advertise<std_msgs::UInt8>("/Environment_Warning2", 1);
    pub_Red_Stop_Light2 = nh.advertise<std_msgs::UInt8>("/Red_Stop_Light2", 1);
    pub_MIL_Light2 = nh.advertise<std_msgs::UInt8>("/MIL_Light2", 1);
    pub_SPN2 = nh.advertise<std_msgs::UInt16>("/SPN2", 1);
    pub_FMI2 = nh.advertise<std_msgs::UInt8>("/FMI2", 1);
    pub_Fault_Count2 = nh.advertise<std_msgs::UInt8>("/Fault_Count2", 1);
    pub_Speed_Limit_Status = nh.advertise<std_msgs::UInt8>("/Speed_Limit_Status", 1);
    pub_Gear_Mode = nh.advertise<std_msgs::UInt8>("/Gear_Mode", 1);
    pub_Vehicle_Lock = nh.advertise<std_msgs::UInt8>("/Vehicle_Lock", 1);
    pub_Anti_Theft = nh.advertise<std_msgs::UInt8>("/Anti_Theft", 1);
    pub_About_To_Lock = nh.advertise<std_msgs::UInt8>("/About_To_Lock", 1);
    pub_Neutral_Start = nh.advertise<std_msgs::UInt8>("/Neutral_Start", 1);
    pub_Gear_Position = nh.advertise<std_msgs::UInt8>("/Gear_Position", 1);
    pub_Gear_Direction = nh.advertise<std_msgs::UInt8>("/Gear_Direction", 1);
    pub_Manual_Auto_Switch = nh.advertise<std_msgs::UInt8>("/Manual_Auto_Switch", 1);
    pub_OverSpeed_Alarm = nh.advertise<std_msgs::UInt8>("/OverSpeed_Alarm", 1);
    pub_Handle_Left_Turn = nh.advertise<std_msgs::UInt8>("/Handle_Left_Turn", 1);
    pub_Handle_Right_Turn = nh.advertise<std_msgs::UInt8>("/Handle_Right_Turn", 1);
    pub_Wheel_Gear_Up = nh.advertise<std_msgs::UInt8>("/Wheel_Gear_Up", 1);
    pub_Wheel_Gear_Down = nh.advertise<std_msgs::UInt8>("/Wheel_Gear_Down", 1);
    pub_Wheel_Gear_Value = nh.advertise<std_msgs::UInt8>("/Wheel_Gear_Value", 1);
    pub_Handle_Turn_Value = nh.advertise<std_msgs::UInt8>("/Handle_Turn_Value", 1);
    pub_Wheel_Gear_Turn_Value = nh.advertise<std_msgs::UInt8>("/Wheel_Gear_Turn_Value", 1);
    pub_Fuel_Level = nh.advertise<std_msgs::UInt8>("/Fuel_Level", 1);
    pub_Fuel_Low_Alarm = nh.advertise<std_msgs::UInt8>("/Fuel_Low_Alarm", 1);
    pub_Power_Cutoff = nh.advertise<std_msgs::UInt8>("/Power_Cutoff", 1);
    pub_Hydraulic_Lock = nh.advertise<std_msgs::UInt8>("/Hydraulic_Lock", 1);
    pub_Low_Oil_Pressure_Inlet = nh.advertise<std_msgs::UInt8>("/Low_Oil_Pressure_Inlet", 1);
    pub_Low_Oil_Pressure_Return = nh.advertise<std_msgs::UInt8>("/Low_Oil_Pressure_Return", 1);
    pub_Transmission_Oil_Filter = nh.advertise<std_msgs::UInt8>("/Transmission_Oil_Filter", 1);
    pub_Diesel_Filter = nh.advertise<std_msgs::UInt8>("/Diesel_Filter", 1);
    pub_Hand_Brake = nh.advertise<std_msgs::UInt8>("/Hand_Brake", 1);
    pub_Gear_Select_Input1 = nh.advertise<std_msgs::UInt8>("/Gear_Select_Input1", 1);
    pub_Gear_Select_Input2 = nh.advertise<std_msgs::UInt8>("/Gear_Select_Input2", 1);
    pub_Gear_Select_Input3 = nh.advertise<std_msgs::UInt8>("/Gear_Select_Input3", 1);
    pub_Gear_Select_Input4 = nh.advertise<std_msgs::UInt8>("/Gear_Select_Input4", 1);
    pub_Transmission_Valve1 = nh.advertise<std_msgs::UInt8>("/Transmission_Valve1", 1);
    pub_Transmission_Valve2 = nh.advertise<std_msgs::UInt8>("/Transmission_Valve2", 1);
    pub_Transmission_Valve3 = nh.advertise<std_msgs::UInt8>("/Transmission_Valve3", 1);
    pub_Transmission_Valve4 = nh.advertise<std_msgs::UInt8>("/Transmission_Valve4", 1);
    pub_Brake_Valve_Short = nh.advertise<std_msgs::UInt8>("/Brake_Valve_Short", 1);
    pub_Brake_Valve_Open = nh.advertise<std_msgs::UInt8>("/Brake_Valve_Open", 1);
    pub_Brake_Valve_Overcurrent = nh.advertise<std_msgs::UInt8>("/Brake_Valve_Overcurrent", 1);
    pub_Left_Turn_Valve_Short = nh.advertise<std_msgs::UInt8>("/Left_Turn_Valve_Short", 1);
    pub_Left_Turn_Valve_Open = nh.advertise<std_msgs::UInt8>("/Left_Turn_Valve_Open", 1);
    pub_Left_Turn_Valve_Overcurrent = nh.advertise<std_msgs::UInt8>("/Left_Turn_Valve_Overcurrent", 1);
    pub_Right_Turn_Valve_Short = nh.advertise<std_msgs::UInt8>("/Right_Turn_Valve_Short", 1);
    pub_Right_Turn_Valve_Open = nh.advertise<std_msgs::UInt8>("/Right_Turn_Valve_Open", 1);
    pub_Right_Turn_Valve_Overcurrent = nh.advertise<std_msgs::UInt8>("/Right_Turn_Valve_Overcurrent", 1);
    pub_Engine_Water_Temp_High = nh.advertise<std_msgs::UInt8>("/Engine_Water_Temp_High", 1);
    pub_Transmission_Temp_High = nh.advertise<std_msgs::UInt8>("/Transmission_Temp_High", 1);
    pub_Transmission_Pressure_High = nh.advertise<std_msgs::UInt8>("/Transmission_Pressure_High", 1);
    pub_Transmission_Pressure_Low = nh.advertise<std_msgs::UInt8>("/Transmission_Pressure_Low", 1);
    pub_Oil_Pressure_High = nh.advertise<std_msgs::UInt8>("/Oil_Pressure_High", 1);
    pub_Oil_Pressure_Low = nh.advertise<std_msgs::UInt8>("/Oil_Pressure_Low", 1);
    pub_Shift_Alarm_Brake = nh.advertise<std_msgs::UInt8>("/Shift_Alarm_Brake", 1);
    pub_Fuel_Level_Low = nh.advertise<std_msgs::UInt8>("/Fuel_Level_Low", 1);
    pub_Urea_Level_Low = nh.advertise<std_msgs::UInt8>("/Urea_Level_Low", 1);
    pub_Transmission_Suction_Alarm = nh.advertise<std_msgs::UInt8>("/Transmission_Suction_Alarm", 1);
    pub_OverSpeed_Shift_Alarm = nh.advertise<std_msgs::UInt8>("/OverSpeed_Shift_Alarm", 1);
    pub_GPS_Locked = nh.advertise<std_msgs::UInt8>("/GPS_Locked", 1);
    pub_Handle_Fault = nh.advertise<std_msgs::UInt8>("/Handle_Fault", 1);
    pub_GPS_Signal_Lost = nh.advertise<std_msgs::UInt8>("/GPS_Signal_Lost", 1);
    pub_Hydraulic_Return_Alarm = nh.advertise<std_msgs::UInt8>("/Hydraulic_Return_Alarm", 1);
    pub_Steering_Suction_Filter = nh.advertise<std_msgs::UInt8>("/Steering_Suction_Filter", 1);
    pub_Handbrake_Not_Released = nh.advertise<std_msgs::UInt8>("/Handbrake_Not_Released", 1);
    pub_Brake_Pedal_Fault = nh.advertise<std_msgs::UInt8>("/Brake_Pedal_Fault", 1);
    pub_Fuel_Level_Fault = nh.advertise<std_msgs::UInt8>("/Fuel_Level_Fault", 1);
    pub_Transmission_Pressure_Fault = nh.advertise<std_msgs::UInt8>("/Transmission_Pressure_Fault", 1);
    pub_Engine_Water_Level_Low = nh.advertise<std_msgs::UInt8>("/Engine_Water_Level_Low", 1);
    pub_Air_Filter_Alarm = nh.advertise<std_msgs::UInt8>("/Air_Filter_Alarm", 1);
    pub_Hydraulic_Oil_Level_Low = nh.advertise<std_msgs::UInt8>("/Hydraulic_Oil_Level_Low", 1);
    pub_Operation_Error_Brake = nh.advertise<std_msgs::UInt8>("/Operation_Error_Brake", 1);
    pub_Turbo_Speed_Signal_Lost = nh.advertise<std_msgs::UInt8>("/Turbo_Speed_Signal_Lost", 1);
    pub_Transmission_Temp_Fault = nh.advertise<std_msgs::UInt8>("/Transmission_Temp_Fault", 1);
    pub_Transmission_Pressure_Fault2 = nh.advertise<std_msgs::UInt8>("/Transmission_Pressure_Fault2", 1);
    pub_StartStop_Button_Fault = nh.advertise<std_msgs::UInt8>("/StartStop_Button_Fault", 1);
    pub_Urea_Concentration = nh.advertise<std_msgs::Float64>("/Urea_Concentration", 1);

    //pub_systemErrorCode = nh.advertise<std_msgs::UInt32>("/systemErrorCode", 10);

    pub_Light_Status = nh.advertise<std_msgs::UInt32>("/Light_Status", 10);

    // 初始化 VCU 原始错误码发布者
    pub_VCU_Error_Code = nh.advertise<std_msgs::UInt8MultiArray>("/VCU_Error_Code", 1);
    // 预分配空间 (报警状态帧原始字节，CAN 帧通常为 8 字节)
    VCU_Error_Code.data.resize(8); 


}

// 初始化单个CAN设备
bool can_to_ros::init_can_device(CanDevice device)
{
    auto& config = can_configs_[device];
    int can_fd;
    struct sockaddr_can addr;
    struct ifreq ifr;
    
    // 创建CAN套接字
    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd < 0) {
        ROS_ERROR("Failed to create socket for CAN device %d: %s", device, strerror(errno));
        return false;
    }
    
    // 设置接口名称
    strcpy(ifr.ifr_name, config.if_name.c_str());
    if (ioctl(can_fd, SIOCGIFINDEX, &ifr) < 0) {
        ROS_ERROR("Failed to ioctl for CAN device %d (%s): %s", device, config.if_name.c_str(), strerror(errno));
        close(can_fd);
        return false;
    }
    
    // 绑定套接字
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("Failed to bind socket for CAN device %d (%s): %s", device, config.if_name.c_str(), strerror(errno));
        close(can_fd);
        return false;
    }
    
    // 配置过滤器（重点修复AHRS过滤器）
    if (device == DEVICE_IMU_ACC) { 
        struct can_filter rfilter_imu[4] = {
            {ACC_ANGULAR_X, CAN_SFF_MASK},
            {ACC_ANGULAR_Y, CAN_SFF_MASK},
            {ACC_ANGULAR_Z, CAN_SFF_MASK},
            {PITCH_ROLL, CAN_SFF_MASK}
        };
        setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter_imu, sizeof(rfilter_imu));
        ROS_INFO("IMU_ACC filter configured: 4 IDs");
    } else if (device == DEVICE_IMU_AHRS) { 
        struct can_filter rfilter_imu[5] = {
            {ROLL_PITCH, CAN_SFF_MASK},
            {YAW_GX, CAN_SFF_MASK},
            {GY_GZ, CAN_SFF_MASK},
            {AX_AY, CAN_SFF_MASK},
            {AZ_TEMP, CAN_SFF_MASK}
        };
        setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter_imu, sizeof(rfilter_imu));
        ROS_INFO("IMU_AHRS filter configured: 5 IDs (ROLL_PITCH/YAW_GX/GY_GZ/AX_AY/AZ_TEMP)");
    } else {
        struct can_filter rfilter[1];
        rfilter[0].can_id = config.can_id;
        rfilter[0].can_mask = config.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK;
        setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    }
    
    // 保存文件描述符并打印日志
    can_fds_[device] = can_fd;
    ROS_INFO("CAN device %d initialized successfully, fd: %d", device, can_fd);
    return true;
}

// 读取单个CAN设备数据（仅用于非IMU设备）
void can_to_ros::read_can_device(CanDevice device)
{
    if (device == DEVICE_IMU_ACC || device == DEVICE_IMU_AHRS) {
        return;
    }
    struct can_frame frame_temp;
    int fd = can_fds_[device];
    
    while (recv(fd, &frame_temp, sizeof(frame_temp), MSG_DONTWAIT) != -1) {
        can_frames_[device] = frame_temp;
    }
}

// 24位有符号整数转32位
int32_t can_to_ros::int24_to_int32(int32_t num)
{
    if (num & 0x800000) {
        num = 0x1000000 - num;
        num = -num;
    }
    return num;
}

// 20位有符号整数转32位
int32_t can_to_ros::int20_to_int32(int32_t num)
{
    if(num & 0x80000){
        num = 0x100000 - num;
        num = -num;
    }
    return num;
}

bool can_to_ros::parse_imu_ahrs_frame(const struct can_frame& frame)
{
    int can_id = frame.can_id & CAN_SFF_MASK; 
    // [优化] 每帧日志从 ROS_INFO 降级到 ROS_DEBUG, 避免 100Hz 日志淹掉控制台
    // 需要调试时用: rosservice call /can_to_ros/set_logger_level "logger: 'ros' level: 'debug'"
    ROS_DEBUG("IMU_AHRS frame, can_id: 0x%X, dlc: %d", can_id, frame.can_dlc);
    std::string data_str;
    for (int i = 0; i < frame.can_dlc; ++i) {
        data_str += string_format("data[%d]: 0x%02X (%d)  ", i, frame.data[i], frame.data[i]);
    }
    ROS_DEBUG("%s", data_str.c_str());
    // 消息头部初始化
    ahrs400_data.header.seq = seq_new++;
    ahrs400_data.header.stamp = ros::Time::now();
    ahrs400_data.header.frame_id = "base_link";

    start_flag_new = true;

    float ahrs_roll, ahrs_pitch, ahrs_yaw;
    Eigen::Quaterniond quaternion;
    Eigen::AngleAxisd rollAngle, pitchAngle, yawAngle;

    // memset(&ahrs400_data.orientation_covariance, 0, sizeof(ahrs400_data.orientation_covariance));
    // memset(&ahrs400_data.angular_velocity_covariance, 0, sizeof(ahrs400_data.angular_velocity_covariance));
    // memset(&ahrs400_data.linear_acceleration_covariance, 0, sizeof(ahrs400_data.linear_acceleration_covariance));

    // 根据CAN ID解析数据
    switch (can_id) {
        case ROLL_PITCH:
            ahrs_roll = *(float*)(&frame.data[0]);
            ahrs_pitch = *(float*)(&frame.data[4]);
            ahrs400_data.orientation_covariance[0] = ahrs_roll;
            ahrs400_data.orientation_covariance[1] = ahrs_pitch;
            can_output_temp_new |= 0x01;
            ROS_DEBUG("Received ROLL_PITCH: roll=%.2f, pitch=%.2f, flag=0x%X", ahrs_roll, ahrs_pitch, can_output_temp_new);
            break;
        case YAW_GX:
            ahrs_yaw = *(float*)(&frame.data[0]);
            ahrs400_data.angular_velocity.x = *(float*)(&frame.data[4]);
            ahrs400_data.orientation_covariance[2] = ahrs_yaw;

            // 计算四元数
            ahrs_roll = ahrs400_data.orientation_covariance[0];
            ahrs_pitch = ahrs400_data.orientation_covariance[1];
            rollAngle = Eigen::AngleAxisd(ahrs_roll / 180.0 * M_PI, Eigen::Vector3d::UnitX());
            pitchAngle = Eigen::AngleAxisd(ahrs_pitch / 180.0 * M_PI, Eigen::Vector3d::UnitY());
            yawAngle = Eigen::AngleAxisd(ahrs_yaw / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
            quaternion = yawAngle * pitchAngle * rollAngle;
            quaternion.normalized();
            ahrs400_data.orientation.x = quaternion.x();
            ahrs400_data.orientation.y = quaternion.y();
            ahrs400_data.orientation.z = quaternion.z();
            ahrs400_data.orientation.w = quaternion.w();

            can_output_temp_new |= 0x02;
            ROS_DEBUG("Received YAW_GX: yaw=%.2f, Gx=%.2f, flag=0x%X", ahrs_yaw, ahrs400_data.angular_velocity.x, can_output_temp_new);
            break;
        case GY_GZ:
            ahrs400_data.angular_velocity.y = *(float*)(&frame.data[0]);
            ahrs400_data.angular_velocity.z = *(float*)(&frame.data[4]);
            can_output_temp_new |= 0x04;
            ROS_DEBUG("Received GY_GZ: Gy=%.2f, Gz=%.2f, flag=0x%X", ahrs400_data.angular_velocity.y, ahrs400_data.angular_velocity.z, can_output_temp_new);
            break;
        case AX_AY:
            ahrs400_data.linear_acceleration.x = *(float*)(&frame.data[0]);
            ahrs400_data.linear_acceleration.y = *(float*)(&frame.data[4]);
            can_output_temp_new |= 0x08;
            ROS_DEBUG("Received AX_AY: Ax=%.2f, Ay=%.2f, flag=0x%X", ahrs400_data.linear_acceleration.x, ahrs400_data.linear_acceleration.y, can_output_temp_new);
            break;
        case AZ_TEMP:
            ahrs400_data.linear_acceleration.z = *(float*)(&frame.data[0]);
            ahrs400_data.orientation_covariance[3] = (*(short*)(&frame.data[4])) * 100;
            ahrs400_data.orientation_covariance[4] = frame.data[6];
            ahrs400_data.orientation_covariance[5] = frame.data[7];
            can_output_temp_new |= 0x10;
            ROS_DEBUG("Received AZ_TEMP: Az=%.2f, flag=0x%X", ahrs400_data.linear_acceleration.z, can_output_temp_new);
            break;
        default:
            ROS_DEBUG("IMU_AHRS unknown can_id: 0x%X", can_id);
            return false;
    }

    // 5帧数据完整（0x1F = 0x01|0x02|0x04|0x08|0x10）
    if (can_output_temp_new == 0x1F) {
        ROS_DEBUG("IMU_AHRS data complete, publishing to /ahrs_imu/data");
        can_output_temp_new = 0; // 重置标志
        return true;
    }

    return false;
}
// 解析IMU的单个CAN帧并非阻塞发布
void can_to_ros::parse_imu_acc_frame(const struct can_frame& frame)
{
    int can_id = frame.can_id;
    int int_temp = 0;
    IMU_Angular_Velocity.header.stamp = ros::Time::now();  // 统一时间戳

    switch (can_id) {
        case ACC_ANGULAR_X:  // X轴角速度（data[4-6]）
            // 从data[4]、data[5]、data[6]拼接24位整数
            for (int i = 0; i < 3; i++) {
                int_temp = (int_temp << 8) | frame.data[4 + i];
            }
            // 转换为角速度（单位：rad/s，系数根据 IMU 传感器手册标定）
            IMU_Angular_Velocity.point.x = int24_to_int32(int_temp) * 0.00057295;
            pub_IMU_Angular_Velocity.publish(IMU_Angular_Velocity);  // 立即发布
            break;

        case ACC_ANGULAR_Y:  // Y轴角速度（data[4-6]）
            int_temp = 0;
            for (int i = 0; i < 3; i++) {
                int_temp = (int_temp << 8) | frame.data[4 + i];
            }
            IMU_Angular_Velocity.point.y = int24_to_int32(int_temp) * 0.00057295;
            pub_IMU_Angular_Velocity.publish(IMU_Angular_Velocity);  // 立即发布
            break;

        case ACC_ANGULAR_Z:  // Z轴角速度（data[4-6]）
            int_temp = 0;
            for (int i = 0; i < 3; i++) {
                int_temp = (int_temp << 8) | frame.data[4 + i];
            }
            IMU_Angular_Velocity.point.z = int24_to_int32(int_temp) * 0.00057295;
            pub_IMU_Angular_Velocity.publish(IMU_Angular_Velocity);  // 立即发布
            break;

        case PITCH_ROLL:  // 俯仰角（data[0-2]）和横滚角（data[4-6]）
            // 解析俯仰角
            int_temp = 0;
            for (int i = 0; i < 3; i++) {
                int_temp = (int_temp << 8) | frame.data[0 + i];
            }
            IMU_Pitch.data = int24_to_int32(int_temp) * 5.7295e-5;  // 转换为度
            pub_IMU_Pitch.publish(IMU_Pitch);  // 立即发布

            // 解析横滚角
            int_temp = 0;
            for (int i = 0; i < 3; i++) {
                int_temp = (int_temp << 8) | frame.data[4 + i];
            }
            IMU_Roll.data = int24_to_int32(int_temp) * 5.7295e-5;  // 转换为度
            pub_IMU_Roll.publish(IMU_Roll);  // 立即发布
            break;

        default:
            break;
    }
}

// 读取CAN数据（整合IMU非阻塞发布逻辑）
void can_to_ros::read_can()
{
    ros::Rate loop_rate1(100);
    int32_t int32_temp   = 0;
    int16_t int16_temp   = 0;
    uint32_t uint32_temp = 0;
    static double roll = 0.0;
    static double pitch = 0.0;
    static double yaw = 0.0;

    while (ros::ok()) {
        auto begin = std::chrono::high_resolution_clock::now();
        struct can_frame frame_temp;

        // 读取所有CAN设备数据
        for (int i = 0; i < DEVICE_COUNT; ++i) {
            read_can_device(static_cast<CanDevice>(i));
        }
        
        // 处理IMU数据
        while (recv(can_fds_[DEVICE_IMU_ACC], &frame_temp, sizeof(frame_temp), MSG_DONTWAIT) != -1) {
            parse_imu_acc_frame(frame_temp);  // 解析并立即发布（非阻塞）
        }

        while (recv(can_fds_[DEVICE_IMU_AHRS], &frame_temp, sizeof(frame_temp), MSG_DONTWAIT) != -1) {
            // 处理新IMU帧，若返回true表示数据完整，发布消息
            if (parse_imu_ahrs_frame(frame_temp)) {
                pub_ahrs_imu.publish(ahrs400_data);
            }
        }

        // 数据处理及发布
        int64_t Longitude_int64_t = 0;
        auto& frame_813 = can_frames_[DEVICE_GPS_LONGITUDE];
        Longitude_int64_t |= static_cast<int64_t>(frame_813.data[7]) << 56;
        Longitude_int64_t |= static_cast<int64_t>(frame_813.data[6]) << 48;
        Longitude_int64_t |= static_cast<int64_t>(frame_813.data[5]) << 40;
        Longitude_int64_t |= static_cast<int64_t>(frame_813.data[4]) << 32;
        Longitude_int64_t |= static_cast<int64_t>(frame_813.data[3]) << 24;
        Longitude_int64_t |= static_cast<int64_t>(frame_813.data[2]) << 16;
        Longitude_int64_t |= static_cast<int64_t>(frame_813.data[1]) << 8;
        Longitude_int64_t |= static_cast<int64_t>(frame_813.data[0]);
       
        int64_t Latitude_int64_t =0;
        auto& frame_814 = can_frames_[DEVICE_GPS_LATITUDE];
        Latitude_int64_t |= static_cast<int64_t>(frame_814.data[7]) << 56;
        Latitude_int64_t |= static_cast<int64_t>(frame_814.data[6]) << 48;
        Latitude_int64_t |= static_cast<int64_t>(frame_814.data[5]) << 40;
        Latitude_int64_t |= static_cast<int64_t>(frame_814.data[4]) << 32;
        Latitude_int64_t |= static_cast<int64_t>(frame_814.data[3]) << 24;
        Latitude_int64_t |= static_cast<int64_t>(frame_814.data[2]) << 16;
        Latitude_int64_t |= static_cast<int64_t>(frame_814.data[1]) << 8;
        Latitude_int64_t |= static_cast<int64_t>(frame_814.data[0]);
     
        int32_t Altitude_int32_t = 0;
        auto& frame_805 = can_frames_[DEVICE_GPS_ALTITUDE];
        Altitude_int32_t = (frame_805.data[3] << 24) | (frame_805.data[2] << 16) | 
                          (frame_805.data[1] << 8) | frame_805.data[0];
        
        LLA.header.stamp = ros::Time::now();
        LLA.point.x = Longitude_int64_t * 0.00000001;
        LLA.point.y = Latitude_int64_t * 0.00000001;
        LLA.point.z = Altitude_int32_t * 0.001;
        pub_LLA.publish(LLA);
        
        // 810
        uint16_t Angle_Heading_uint16_t = 0;
        int16_t Angle_Pitch_int16_t = 0;
        int16_t Angle_Roll_int16_t = 0;

        auto& frame_810 = can_frames_[DEVICE_GPS_HEADING];
        Angle_Heading_uint16_t = (frame_810.data[1] << 8) | frame_810.data[0];
        Angle_Pitch_int16_t =  (frame_810.data[3] << 8) | frame_810.data[2];
        Angle_Roll_int16_t =  (frame_810.data[5] << 8) | frame_810.data[4];

        Angle_Heading.point.x = (Angle_Heading_uint16_t*0.01)/180.0*M_PI;
        Angle_Heading.point.y = (Angle_Pitch_int16_t*0.01)/180.0*M_PI;
        Angle_Heading.point.z = (Angle_Roll_int16_t*0.01)/180.0*M_PI;
        Angle_Heading.header.stamp = LLA.header.stamp;
        pub_Angle_Heading.publish(Angle_Heading);

        // 807
        int16_t Vehicle_Speed_E_int16_t = 0;
        int16_t Vehicle_Speed_N_int16_t = 0;
        int16_t Vehicle_Speed_U_int16_t = 0;
        int16_t Vehicle_Speed_int16_t = 0;
        auto& frame_807 = can_frames_[DEVICE_VEHICLE_SPEED_ENU];
        Vehicle_Speed_E_int16_t = (frame_807.data[1] << 8) | frame_807.data[0];
        Vehicle_Speed_N_int16_t = (frame_807.data[3] << 8) | frame_807.data[2];
        Vehicle_Speed_U_int16_t = (frame_807.data[5] << 8) | frame_807.data[4];
        Vehicle_Speed_int16_t = (frame_807.data[7] << 8) | frame_807.data[6];

        Vehicle_Speed.header.stamp = LLA.header.stamp;
        Vehicle_Speed.twist.linear.x = Vehicle_Speed_E_int16_t * 0.01;
        Vehicle_Speed.twist.linear.y = Vehicle_Speed_N_int16_t * 0.01;
        Vehicle_Speed.twist.linear.z = Vehicle_Speed_U_int16_t * 0.01;
        Vehicle_Speed_Vel.data = Vehicle_Speed_int16_t * 0.01;
        pub_Vehicle_Speed.publish(Vehicle_Speed);
        pub_Vehicle_Speed_Vel.publish(Vehicle_Speed_Vel);

        // 806
        uint64_t frame_806_data = 0;
        auto& frame_806 = can_frames_[DEVICE_POS_SIGMA];
        for (int i = 0; i < 8; ++i) {
            frame_806_data |= static_cast<uint64_t>(frame_806.data[i]) << (i * 8);
        } 
        pos_sigma.header.stamp = LLA.header.stamp;
        pos_sigma.point.x = 0.0001 * ((frame_806_data & 0x00000000000fffff) >> 0);
        pos_sigma.point.y = 0.0001 * ((frame_806_data & 0x000000fffff00000) >> 20);
        pos_sigma.point.z = 0.0001 * ((frame_806_data & 0x0fffff0000000000) >> 40);
        pub_PosSigma.publish(pos_sigma);

        // RTK状态
        RTK.header.stamp = LLA.header.stamp;
        RTK.position_covariance[0] = Angle_Heading_uint16_t * 0.01;  //yaw
        RTK.position_covariance[1] = Angle_Pitch_int16_t * 0.01;     //pitch
        RTK.position_covariance[2] = Angle_Roll_int16_t * 0.01;      //roll
      
        uint8_t system_state_uint8_t = 0;
        uint8_t satellite_state_uint8_t = 0;
        uint16_t gps_age_uint16_t = 0;
        uint8_t gps_num1_uint8_t = 0;
        uint8_t gps_num2_uint8_t = 0;
        auto& frame_803 = can_frames_[DEVICE_RTK_STATUS];
        system_state_uint8_t = frame_803.data[0];
        satellite_state_uint8_t = frame_803.data[2];
        gps_age_uint16_t = (frame_803.data[5] << 8) | frame_803.data[4];
        gps_num1_uint8_t = frame_803.data[1];
        gps_num2_uint8_t = frame_803.data[3];

        RTK.position_covariance[3] = system_state_uint8_t * 1.0;
        RTK.position_covariance[4] = satellite_state_uint8_t * 1.0;
        RTK.position_covariance[5] = gps_age_uint16_t * 0.01;
        RTK.position_covariance[6] = gps_num1_uint8_t * 1.0;
        RTK.position_covariance[7] = gps_num2_uint8_t * 1.0;
        RTK.longitude = LLA.point.x;
        RTK.latitude = LLA.point.y;
        RTK.altitude = LLA.point.z;
        pub_RTK.publish(RTK);

        //发动机实际扭矩百分比、转速 400
        int temp = 0;
        auto& frame_400 = can_frames_[DEVICE_ENGINE_TORQUE_SPEED];
        temp = frame_400.data[2] - 125;
        if (temp < -125) {
            temp = -125;
        } else if (temp > 125) {
            temp = 125;
        }
        Engine_Torque_Actual.data = static_cast<int8_t>(temp);
        pub_Engine_Torque_Actual.publish(Engine_Torque_Actual);

        int16_t temp_engine_speed_actual = 0;
        temp_engine_speed_actual = frame_400.data[4] << 8 | frame_400.data[3];
        Engine_Speed_Actual.data = static_cast<float_t>(temp_engine_speed_actual * 0.125);
        pub_Engine_Speed_Actual.publish(Engine_Speed_Actual);

        //变速箱转速
        int16_t temp_transmission_speed_actual = 0;
        auto& frame_011 = can_frames_[DEVICE_TRANSMISSION_SPEED];
        temp_transmission_speed_actual = frame_011.data[4] << 8 | frame_011.data[3];
        Transmission_Speed_Actual.data = temp_transmission_speed_actual;
        pub_Transmission_Speed_Actual.publish(Transmission_Speed_Actual);  

        // IMU_Accelerated_Velocity  frame_809
        uint64_t frame_809_data = 0;
        auto& frame_809 = can_frames_[DEVICE_VEHICLE_ACCEL];
        for (int i = 0; i < 8; ++i) {
            frame_809_data |= static_cast<uint64_t>(frame_809.data[i]) << (i * 8);
        }

        //int32_t int32_temp   = 0;        
        int32_temp = (frame_809_data  & 0x00000000000fffff) >> 0;
        int32_temp = int20_to_int32(int32_temp);
        IMU_Accelerated_Velocity.point.x = 0.0001 * int32_temp;
        
        int32_temp = (frame_809_data  & 0x000000fffff00000) >> 20;
        int32_temp = int20_to_int32(int32_temp);
        IMU_Accelerated_Velocity.point.y = 0.0001 * int32_temp;
        
        int32_temp = (frame_809_data  & 0x0fffff0000000000) >> 40;
        int32_temp = int20_to_int32(int32_temp);
        IMU_Accelerated_Velocity.point.z = 0.0001 * int32_temp;
        IMU_Accelerated_Velocity.header.stamp = LLA.header.stamp;    
        pub_IMU_Accel.publish(IMU_Accelerated_Velocity);

        // RTK_IMU  
        //frame_801
        uint64_t frame_801_data = 0;
        auto& frame_801 = can_frames_[DEVICE_IMU_GYRO_RAW];
        for (int i = 0; i < 8; ++i) {
            frame_801_data |= static_cast<uint64_t>(frame_801.data[i]) << (i * 8);
        }

        int32_temp = (frame_801_data & 0x00000000000fffff) >> 0;
        int32_temp = int20_to_int32(int32_temp);
        RTK_IMU.angular_velocity.x = (0.01 * int32_temp);//180.0*M_PI;
        
        int32_temp = (frame_801_data & 0x000000fffff00000) >> 20;
        int32_temp = int20_to_int32(int32_temp);
        RTK_IMU.angular_velocity.y = (0.01 * int32_temp);//180.0*M_PI;
        
        int32_temp = (frame_801_data & 0x0fffff0000000000) >> 40;
        int32_temp = int20_to_int32(int32_temp);
        RTK_IMU.angular_velocity.z = (0.01 * int32_temp);//180.0*M_PI;
        //frame_802
        uint64_t frame_802_data = 0;
        auto& frame_802 = can_frames_[DEVICE_IMU_ACCEL_RAW];
        for (int i = 0; i < 8; ++i) {
            frame_802_data |= static_cast<uint64_t>(frame_802.data[i]) << (i * 8);
        }

        int32_temp = (frame_802_data & 0x00000000000fffff) >> 0;
        int32_temp = int20_to_int32(int32_temp);
        RTK_IMU.linear_acceleration.x = 0.0001 * int32_temp;
        
        int32_temp = (frame_802_data & 0x000000fffff00000) >> 20;   
        int32_temp = int20_to_int32(int32_temp);
        RTK_IMU.linear_acceleration.y = 0.0001 * int32_temp;
        
        int32_temp = (frame_802_data & 0x0fffff0000000000) >> 40;
        int32_temp = int20_to_int32(int32_temp);
        RTK_IMU.linear_acceleration.z = 0.0001 * int32_temp;
        //
        rollAngle=(Eigen::AngleAxisd(roll/180.0*M_PI,Eigen::Vector3d::UnitX()));
        pitchAngle=(Eigen::AngleAxisd(pitch/180.0*M_PI,Eigen::Vector3d::UnitY()));
        yawAngle=(Eigen::AngleAxisd(yaw/180.0*M_PI,Eigen::Vector3d::UnitZ()));

        quaternion=yawAngle*pitchAngle*rollAngle;
        quaternion.normalized();
        RTK_IMU.orientation.x = quaternion.x();
        RTK_IMU.orientation.y = quaternion.y();
        RTK_IMU.orientation.z = quaternion.z();
        RTK_IMU.orientation.w = quaternion.w();
	    RTK_IMU.orientation_covariance[0] = Angle_Heading_uint16_t * 0.01;  //yaw
        RTK_IMU.orientation_covariance[1] = Angle_Pitch_int16_t * 0.01;     //pitch
        RTK_IMU.orientation_covariance[2] = Angle_Roll_int16_t * 0.01;      //roll
        
        RTK_IMU.header.stamp = LLA.header.stamp;
        pub_RTK_IMU.publish(RTK_IMU);

        // Angle_Heading_Velocity 812
        uint64_t frame_812_data = 0;
        auto& frame_812 = can_frames_[DEVICE_HEADING_VELOCITY];
        for (int i = 0; i < 8; ++i) {
            frame_812_data |= static_cast<uint64_t>(frame_812.data[i]) << (i * 8);
        }
        int32_temp = (frame_812_data & 0x0fffff0000000000) >> 40;
        int32_temp = int20_to_int32(int32_temp);
        Angle_Heading_Velocity.data = 0.01 * int32_temp;
        pub_Angle_Heading_Velocity.publish(Angle_Heading_Velocity);

        // 输出电流 DEVICE_OUTCURR
        auto& frame_OutCurr = can_frames_[DEVICE_OUTCURR];
        OUTPUT_CURRENT.data.resize(4);
        OUTPUT_CURRENT.data[0] = (frame_OutCurr.data[1] << 8) | frame_OutCurr.data[0];   
        OUTPUT_CURRENT.data[1] = (frame_OutCurr.data[3] << 8) | frame_OutCurr.data[2];
        OUTPUT_CURRENT.data[2] = (frame_OutCurr.data[5] << 8) | frame_OutCurr.data[4]; 
        OUTPUT_CURRENT.data[3] = (frame_OutCurr.data[7] << 8) | frame_OutCurr.data[6];
        pub_OutCurr.publish(OUTPUT_CURRENT);

            // 燃料使用总量解析
        auto& frame_FuelTotal = can_frames_[DEVICE_FUEL_TOTAL];
        uint32_t fuel_raw = 0;
        // 解析 D4, D5, D6, D7 字节
        fuel_raw = (frame_FuelTotal.data[7] << 24) | (frame_FuelTotal.data[6] << 16) | 
                   (frame_FuelTotal.data[5] << 8) | frame_FuelTotal.data[4];
        Fuel_Total.data = fuel_raw * 0.5;  // 0.5L/Bit
        pub_Fuel_Total.publish(Fuel_Total);
        
        // 发动机总工作时间解析
        auto& frame = can_frames_[DEVICE_ENGINE_TOTAL_HOURS];

        // 小端（Little Endian）: D0=LSB, D3=MSB
        uint32_t hours_raw = 
        static_cast<uint32_t>(frame.data[0])        |
        (static_cast<uint32_t>(frame.data[1]) << 8) |
        (static_cast<uint32_t>(frame.data[2]) << 16)|
        (static_cast<uint32_t>(frame.data[3]) << 24);

        Engine_Total_Hours.data = hours_raw * 0.05f;  // 单位：小时
        pub_Engine_Total_Hours.publish(Engine_Total_Hours);
        
        // 车速显示解析
        auto& frame_SpeedDisplay = can_frames_[DEVICE_VEHICLE_SPEED_DISPLAY];
        uint16_t speed_display_raw = 0;
        // 解析 D0, D1 字节
        speed_display_raw = (frame_SpeedDisplay.data[1] << 8) | frame_SpeedDisplay.data[0];
        Vehicle_Speed_Display.data = static_cast<float>(speed_display_raw) / 100.0;  // 除以100
        // 保留两位小数
        Vehicle_Speed_Display.data = round(Vehicle_Speed_Display.data * 100) / 100.0;
        pub_Vehicle_Speed_Display.publish(Vehicle_Speed_Display);

        // 发动机冷却液温度解析
        auto& frame_CoolantTemp = can_frames_[DEVICE_COOLANT_TEMP];
        uint8_t coolant_temp_raw = 0;
        // 解析 D0 字节
        coolant_temp_raw = frame_CoolantTemp.data[0];
        // 转换为实际温度：原始值 * 1 - 40
        Coolant_Temperature.data = static_cast<float>(coolant_temp_raw) * 1.0 - 40.0;
        pub_Coolant_Temperature.publish(Coolant_Temperature);

        // 警告灯状态和故障信息解析 
        auto& frame_WarningLights = can_frames_[DEVICE_WARNING_LIGHTS];
        uint8_t d0_data = frame_WarningLights.data[0];
        
        // 解析D0字节中的各个灯状态
        // bit0-1: 保护灯状态 (不使用，设为0x12)
        Protection_Light.data = 0x12;  // 固定值
        
        // bit2-3: 环境警告灯状态
        uint8_t env_warning_bits = (d0_data >> 2) & 0x03;
        Environment_Warning.data = (env_warning_bits == 0x01) ? 1 : 0;  // 0x01亮，其他灭
        
        // bit4-5: 红色停止灯状态
        uint8_t red_stop_bits = (d0_data >> 4) & 0x03;
        Red_Stop_Light.data = (red_stop_bits == 0x01) ? 1 : 0;  // 0x01亮，其他灭
        
        // bit6-7: MIL灯状态
        uint8_t mil_bits = (d0_data >> 6) & 0x03;
        MIL_Light.data = (mil_bits == 0x01) ? 1 : 0;  // 0x01亮，其他灭
        
        // 解析D2-D3: SPN码 (低16位)
        uint16_t spn_low = (frame_WarningLights.data[3] << 8) | frame_WarningLights.data[2];
        
        // 解析D4字节
        uint8_t d4_data = frame_WarningLights.data[4];
        // bit0-4: FMI码
        uint8_t fmi_code = d4_data & 0x1F;
        // bit6-7: SPN最高有效位
        uint8_t spn_msb = (d4_data >> 6) & 0x03;
        
        // 组合SPN码 (18位: 2位MSB + 16位LSB)
        uint32_t spn_full = (spn_msb << 16) | spn_low;
        
        // 解析D5字节
        uint8_t d5_data = frame_WarningLights.data[5];
        // bit0-6: 当前故障计数
        uint8_t fault_count = d5_data & 0x7F;
        // bit7: SPN转换模式 (设为0)
        uint8_t spn_conversion_mode = (d5_data >> 7) & 0x01;
        
        // 发布警告灯状态
        pub_Protection_Light.publish(Protection_Light);
        pub_Environment_Warning.publish(Environment_Warning);
        pub_Red_Stop_Light.publish(Red_Stop_Light);
        pub_MIL_Light.publish(MIL_Light);
        
        // 发布故障信息
        SPN.data = static_cast<uint16_t>(spn_full & 0xFFFF);  // 取低16位
        FMI.data = fmi_code;
        Fault_Count.data = fault_count;
        pub_SPN.publish(SPN);
        pub_FMI.publish(FMI);
        pub_Fault_Count.publish(Fault_Count);

        // =============== 警告灯状态2解析 ===============
        auto& frame_WarningLights2 = can_frames_[DEVICE_WARNING_LIGHTS2];
        uint8_t d0_data2 = frame_WarningLights2.data[0];  // 改为 d0_data2
        
        // 解析D0字节中的各个灯状态
        // bit0-1: 保护灯状态
        uint8_t protection_bits2 = d0_data2 & 0x03;  // 改为 d0_data2
        Protection_Light2.data = (protection_bits2 == 0x01) ? 1 : 0;  // 0x01亮，其他灭
        
        // bit2-3: 环境警告灯状态
        uint8_t env_warning_bits2 = (d0_data2 >> 2) & 0x03;  // 改为 d0_data2
        Environment_Warning2.data = (env_warning_bits2 == 0x01) ? 1 : 0;  // 0x01亮，其他灭
        
        // bit4-5: 红色停止灯状态
        uint8_t red_stop_bits2 = (d0_data2 >> 4) & 0x03;  // 改为 d0_data2
        Red_Stop_Light2.data = (red_stop_bits2 == 0x01) ? 1 : 0;  // 0x01亮，其他灭
        
        // bit6-7: MIL灯状态
        uint8_t mil_bits2 = (d0_data2 >> 6) & 0x03;  // 改为 d0_data2
        MIL_Light2.data = (mil_bits2 == 0x01) ? 1 : 0;  // 0x01亮，其他灭
        
        // 解析D2-D3: SPN码 (16位)
        uint16_t spn2 = (frame_WarningLights2.data[3] << 8) | frame_WarningLights2.data[2];
        
        // 解析D4字节
        uint8_t d4_data2 = frame_WarningLights2.data[4];
        // bit0-4: FMI码 (0-31)
        uint8_t fmi_code2 = d4_data2 & 0x1F;
        // bit5-7: SPN MSB (高3位)
        uint8_t spn_msb2 = (d4_data2 >> 5) & 0x07;
        
        // 组合SPN码 (19位: 3位MSB + 16位LSB)
        uint32_t spn_full2 = (spn_msb2 << 16) | spn2;
        
        // 解析D5字节
        uint8_t d5_data2 = frame_WarningLights2.data[5];
        // bit0-6: 当前故障计数 (0-127)
        uint8_t fault_count2 = d5_data2 & 0x7F;
        // bit7: SPN转换模式 (设为0)
        uint8_t spn_conversion_mode2 = (d5_data2 >> 7) & 0x01;
        
        // 发布警告灯状态2
        pub_Protection_Light2.publish(Protection_Light2);
        pub_Environment_Warning2.publish(Environment_Warning2);
        pub_Red_Stop_Light2.publish(Red_Stop_Light2);
        pub_MIL_Light2.publish(MIL_Light2);
        
        // 发布故障信息2
        SPN2.data = static_cast<uint16_t>(spn_full2 & 0xFFFF);  // 取低16位
        FMI2.data = fmi_code2;
        Fault_Count2.data = fault_count2;
        pub_SPN2.publish(SPN2);
        pub_FMI2.publish(FMI2);
        pub_Fault_Count2.publish(Fault_Count2);
        
        // =============== 速度限制状态解析 ===============
        auto& frame_SpeedLimit = can_frames_[DEVICE_SPEED_LIMIT];
        uint8_t d0_data_speed = frame_SpeedLimit.data[0];  // 改为 d0_data_speed
        
        // 解析D0字节的bit4-5: 速度限制状态
        uint8_t speed_limit_bits = (d0_data_speed >> 4) & 0x03;  // 改为 d0_data_speed
        // 0x00: 激活, 0x01: 未激活
        Speed_Limit_Status.data = (speed_limit_bits == 0x00) ? 1 : 0;  // 激活=1, 未激活=0
        pub_Speed_Limit_Status.publish(Speed_Limit_Status);

        // 档位模式解析 
        auto& frame_GearMode = can_frames_[DEVICE_GEAR_MODE];
        uint8_t gear_mode_raw = frame_GearMode.data[0];
        
        // 根据原始值解析档位模式
        uint8_t gear_mode = 0;
        switch (gear_mode_raw) {
            case 1:
                gear_mode = 1;  // 模式一
                break;
            case 2:
                gear_mode = 2;  // 模式二
                break;
            case 3:
                gear_mode = 3;  // 模式三
                break;
            case 4:
                gear_mode = 4;  // 模式四
                break;
            case 5:
                gear_mode = 5;  // 手动档
                break;
            default:
                gear_mode = 0;  // 未知模式
                break;
        }
        Gear_Mode.data = gear_mode;
        pub_Gear_Mode.publish(Gear_Mode);
        
        // 锁车状态解析
        auto& frame_LockStatus = can_frames_[DEVICE_LOCK_STATUS];
        uint8_t d0_data_lock = frame_LockStatus.data[0];  // 改为 d0_data_lock
        uint8_t d7_data_lock = frame_LockStatus.data[7];
        
        // 解析D0字节
        // bit0: 锁车状态
        uint8_t lock_bit = d0_data_lock & 0x01;  // 改为 d0_data_lock
        Vehicle_Lock.data = lock_bit;  // 1:锁车; 0:正常
        
        // bit1: 防拆状态
        uint8_t anti_theft_bit = (d0_data_lock >> 1) & 0x01;  // 改为 d0_data_lock
        Anti_Theft.data = anti_theft_bit;  // 1:防拆关闭; 0:防拆开启
        
        // bit2: 即将锁车状态 (注意：u9应该为u8，按u8解析)
        uint8_t about_to_lock_bit = (d0_data_lock >> 2) & 0x01;  // 改为 d0_data_lock
        About_To_Lock.data = about_to_lock_bit;  // 1:即将锁车
        
        // 解析D7字节
        // bit0: 未在空挡打火信号
        uint8_t neutral_start_bit = d7_data_lock & 0x01;
        Neutral_Start.data = neutral_start_bit;  // 1:手柄回空档后才允许打火
        
        // 发布锁车状态相关话题
        pub_Vehicle_Lock.publish(Vehicle_Lock);
        pub_Anti_Theft.publish(Anti_Theft);
        pub_About_To_Lock.publish(About_To_Lock);
        pub_Neutral_Start.publish(Neutral_Start);
        
        // 如果收到未在空挡打火信号为1，记录时间用于仪表弹窗（保留3秒）
        static ros::Time neutral_start_time;
        if (Neutral_Start.data == 1) {
            neutral_start_time = ros::Time::now();
            ROS_INFO("未在空挡打火信号：仪表需要弹窗提示");
        }
        
        // 检查是否需要清除仪表弹窗（3秒后）
        if (Neutral_Start.data == 0 && !neutral_start_time.isZero()) {
            ros::Duration time_since = ros::Time::now() - neutral_start_time;
            if (time_since.toSec() > 3.0) {
                ROS_INFO("仪表弹窗时间结束");
                neutral_start_time = ros::Time();  // 重置时间
            }
        }

        // =============== 手柄信息解析 ===============
        auto& frame_HandleInfo = can_frames_[DEVICE_HANDLE_INFO];
        
        // 解析D0字节
        uint8_t d0_data_handle = frame_HandleInfo.data[0];  // 改为 d0_data_handle
        
        // bit0-bit3: 档位指示 (0-15)
        uint8_t gear_position_bits = d0_data_handle & 0x0F;  // 改为 d0_data_handle
        uint8_t gear_position = 0;
        switch (gear_position_bits) {
            case 0x01:
                gear_position = 1;  // 1档
                break;
            case 0x02:
                gear_position = 2;  // 2档
                break;
            case 0x03:
                gear_position = 3;  // 3档
                break;
            default:
                gear_position = gear_position_bits;  // 其他档位保持原值
                break;
        }
        Gear_Position.data = gear_position;
        
        // bit4-bit5: 档位方向 (0-4)
        uint8_t gear_dir_bits = (d0_data_handle >> 4) & 0x03;  // 改为 d0_data_handle
        uint8_t gear_direction = 0;
        switch (gear_dir_bits) {
            case 0x00:
                gear_direction = 0;  // 方向控制关闭
                break;
            case 0x01:
                gear_direction = 1;  // 向前
                break;
            case 0x02:
                gear_direction = 2;  // 向后
                break;
            default:
                gear_direction = gear_dir_bits;  // 其他方向
                break;
        }
        Gear_Direction.data = gear_direction;
        
        // bit6: 手动自动切换 (0或1)
        uint8_t manual_auto_bit = (d0_data_handle >> 6) & 0x01;  // 改为 d0_data_handle
        Manual_Auto_Switch.data = manual_auto_bit;  // 0:手动挡, 1:自动挡
        
        // bit7: 超速手柄转向报警 (0或1)
        uint8_t overspeed_alarm_bit = (d0_data_handle >> 7) & 0x01;  // 改为 d0_data_handle
        OverSpeed_Alarm.data = overspeed_alarm_bit;  // 1:报警, 0:不报警
        
        // 发布D0字节相关信息
        pub_Gear_Position.publish(Gear_Position);
        pub_Gear_Direction.publish(Gear_Direction);
        pub_Manual_Auto_Switch.publish(Manual_Auto_Switch);
        pub_OverSpeed_Alarm.publish(OverSpeed_Alarm);
        
        // 如果超速报警，输出提示信息
        if (OverSpeed_Alarm.data == 1) {
            ROS_WARN("车速过高，请勿使用手柄进行换向");
        }
        
        // 解析D1字节
        uint8_t d1_data = frame_HandleInfo.data[1];
        
        // bit0: 手柄左转输入 (0或1)
        uint8_t handle_left_bit = d1_data & 0x01;
        Handle_Left_Turn.data = handle_left_bit;  // 1:手柄左转, 0:无动作
        
        // bit1: 手柄右转输入 (0或1)
        uint8_t handle_right_bit = (d1_data >> 1) & 0x01;
        Handle_Right_Turn.data = handle_right_bit;  // 1:手柄右转, 0:无动作
        
        // bit2: 拨轮档位加输入 (0或1)
        uint8_t wheel_gear_up_bit = (d1_data >> 2) & 0x01;
        Wheel_Gear_Up.data = wheel_gear_up_bit;  // 1:拨轮加档, 0:无动作
        
        // bit3: 拨轮档位减输入 (0或1)
        uint8_t wheel_gear_down_bit = (d1_data >> 3) & 0x01;
        Wheel_Gear_Down.data = wheel_gear_down_bit;  // 1:拨轮减档, 0:无动作
        
        // bit4-bit5: 档位值 (0-4)
        uint8_t wheel_gear_bits = (d1_data >> 4) & 0x03;
        uint8_t wheel_gear_value = 0;
        switch (wheel_gear_bits) {
            case 0x00:
                wheel_gear_value = 0;  // N档
                break;
            case 0x01:
                wheel_gear_value = 1;  // 向前
                break;
            case 0x02:
                wheel_gear_value = 2;  // 向后
                break;
            default:
                wheel_gear_value = wheel_gear_bits;  // 其他值
                break;
        }
        Wheel_Gear_Value.data = wheel_gear_value;
        
        // 发布D1字节相关信息
        pub_Handle_Left_Turn.publish(Handle_Left_Turn);
        pub_Handle_Right_Turn.publish(Handle_Right_Turn);
        pub_Wheel_Gear_Up.publish(Wheel_Gear_Up);
        pub_Wheel_Gear_Down.publish(Wheel_Gear_Down);
        pub_Wheel_Gear_Value.publish(Wheel_Gear_Value);
        
        // 解析D2字节: 手柄转向值 (0-255)
        uint8_t handle_turn_value = frame_HandleInfo.data[2];
        Handle_Turn_Value.data = handle_turn_value;
        pub_Handle_Turn_Value.publish(Handle_Turn_Value);
        
        // 解析D3字节: 拨轮档位方向值 (0-255)
        uint8_t wheel_gear_turn_value = frame_HandleInfo.data[3];
        Wheel_Gear_Turn_Value.data = wheel_gear_turn_value;
        pub_Wheel_Gear_Turn_Value.publish(Wheel_Gear_Turn_Value);
        
        // 燃油油位解析
        auto& frame_FuelLevel = can_frames_[DEVICE_FUEL_LEVEL];
        
        // 解析D0字节: 燃油油位 (0-100, 1%/位)
        uint8_t fuel_level_raw = frame_FuelLevel.data[0];
        
        // 限制范围在0-100
        uint8_t fuel_level = fuel_level_raw;
        if (fuel_level > 100) {
            fuel_level = 100;
        }
        Fuel_Level.data = fuel_level;
        pub_Fuel_Level.publish(Fuel_Level);
        
        // 检查低油位报警 (低于20%)
        if (fuel_level < 20) {
            Fuel_Low_Alarm.data = 1;  // 报警
        } else {
            Fuel_Low_Alarm.data = 0;  // 正常
        }
        pub_Fuel_Low_Alarm.publish(Fuel_Low_Alarm);

        // =============== 控制输入解析 ===============
        auto& frame_ControlInputs = can_frames_[DEVICE_CONTROL_INPUTS];
        
        // 解析D0字节
        uint8_t d0_data_control = frame_ControlInputs.data[0];
        
        // bit0: 切断动力 (bool, 0或1)
        uint8_t power_cutoff_bit = d0_data_control & 0x01;
        Power_Cutoff.data = power_cutoff_bit;  // 1:高电平，按下时0:发动机动力切断
        
        // bit1: 液压锁开关 (bool, 0或1)
        uint8_t hydraulic_lock_bit = (d0_data_control >> 1) & 0x01;
        Hydraulic_Lock.data = hydraulic_lock_bit;  // 1:有相应输入图标绿色，0:无相应输入图标红色
        
        // bit2: 低油效液压吸油油滤报警 (bool, 0或1)
        uint8_t low_oil_pressure_inlet_bit = (d0_data_control >> 2) & 0x01;
        Low_Oil_Pressure_Inlet.data = low_oil_pressure_inlet_bit;  // 1:有相应输入，0:无相应输入
        
        // bit3: 低油效液压回油油滤报警 (bool, 0或1)
        uint8_t low_oil_pressure_return_bit = (d0_data_control >> 3) & 0x01;
        Low_Oil_Pressure_Return.data = low_oil_pressure_return_bit;  // 1:有相应输入，0:无相应输入
        
        // bit4: 传动油滤报警（预留） (bool, 0或1)
        uint8_t transmission_oil_filter_bit = (d0_data_control >> 4) & 0x01;
        Transmission_Oil_Filter.data = transmission_oil_filter_bit;  // 1:有相应输入，0:无相应输入
        
        // bit5: 柴油油滤报警（预留） (bool, 0或1)
        uint8_t diesel_filter_bit = (d0_data_control >> 5) & 0x01;
        Diesel_Filter.data = diesel_filter_bit;  // 1:有相应输入，0:无相应输入
        
        // bit6: 手刹 (bool, 0或1)
        uint8_t hand_brake_bit = (d0_data_control >> 6) & 0x01;
        Hand_Brake.data = hand_brake_bit;  // 1:有相应输入，0:无相应输入
        
        // bit7: 变速箱档位选择输入1 (bool, 0或1)
        uint8_t gear_select_input1_bit = (d0_data_control >> 7) & 0x01;
        Gear_Select_Input1.data = gear_select_input1_bit;  // 1:有相应输入，0:无相应输入
        
        // 发布D0字节相关信息
        pub_Power_Cutoff.publish(Power_Cutoff);
        pub_Hydraulic_Lock.publish(Hydraulic_Lock);
        pub_Low_Oil_Pressure_Inlet.publish(Low_Oil_Pressure_Inlet);
        pub_Low_Oil_Pressure_Return.publish(Low_Oil_Pressure_Return);
        pub_Transmission_Oil_Filter.publish(Transmission_Oil_Filter);
        pub_Diesel_Filter.publish(Diesel_Filter);
        pub_Hand_Brake.publish(Hand_Brake);
        pub_Gear_Select_Input1.publish(Gear_Select_Input1);
        
        // 解析D1字节
        uint8_t d1_data_control = frame_ControlInputs.data[1];
        
        // bit0: 变速箱档位选择输入2 (bool, 0或1)
        uint8_t gear_select_input2_bit = d1_data_control & 0x01;
        Gear_Select_Input2.data = gear_select_input2_bit;  // 1:有相应输入，0:无相应输入
        
        // bit1: 变速箱档位选择输入3 (bool, 0或1)
        uint8_t gear_select_input3_bit = (d1_data_control >> 1) & 0x01;
        Gear_Select_Input3.data = gear_select_input3_bit;  // 1:有相应输入，0:无相应输入
        
        // bit2: 变速箱档位选择输入4 (bool, 0或1)
        uint8_t gear_select_input4_bit = (d1_data_control >> 2) & 0x01;
        Gear_Select_Input4.data = gear_select_input4_bit;  // 1:有相应输入，0:无相应输入
        
        // 发布D1字节相关信息
        pub_Gear_Select_Input2.publish(Gear_Select_Input2);
        pub_Gear_Select_Input3.publish(Gear_Select_Input3);
        pub_Gear_Select_Input4.publish(Gear_Select_Input4);      

         // =============== 变速箱档位阀解析 ===============
        auto& frame_TransmissionValve = can_frames_[DEVICE_TRANSMISSION_VALVE];
        
        // 解析D0字节
        uint8_t d0_data_valve = frame_TransmissionValve.data[0];
        
        // bit7: 变速箱档位阀1 (bool, 0或1)
        uint8_t transmission_valve1_bit = (d0_data_valve >> 7) & 0x01;
        Transmission_Valve1.data = transmission_valve1_bit;  // 1:有相应输入，0:无相应输入
        pub_Transmission_Valve1.publish(Transmission_Valve1);
        
        // 解析D1字节
        uint8_t d1_data_valve = frame_TransmissionValve.data[1];
        
        // bit0: 变速箱档位阀2 (bool, 0或1)
        uint8_t transmission_valve2_bit = d1_data_valve & 0x01;
        Transmission_Valve2.data = transmission_valve2_bit;  // 1:有相应输入，0:无相应输入
        pub_Transmission_Valve2.publish(Transmission_Valve2);
        
        // bit1: 变速箱档位阀3 (bool, 0或1)
        uint8_t transmission_valve3_bit = (d1_data_valve >> 1) & 0x01;
        Transmission_Valve3.data = transmission_valve3_bit;  // 1:有相应输入，0:无相应输入
        pub_Transmission_Valve3.publish(Transmission_Valve3);
        
        // bit2: 变速箱档位阀4 (bool, 0或1)
        uint8_t transmission_valve4_bit = (d1_data_valve >> 2) & 0x01;
        Transmission_Valve4.data = transmission_valve4_bit;  // 1:有相应输入，0:无相应输入
        pub_Transmission_Valve4.publish(Transmission_Valve4);

        // =============== 报警状态解析 ===============
        auto& frame_AlarmStatus = can_frames_[DEVICE_ALARM_STATUS];
        
        // 解析D0字节
        uint8_t d0_data_alarm = frame_AlarmStatus.data[0];
        
        // bit0: 刹车比例阀输出线路短路 (bool, 0或1)
        uint8_t brake_valve_short_bit = d0_data_alarm & 0x01;
        Brake_Valve_Short.data = brake_valve_short_bit;  // 1:短路状态，0:开路状态
        pub_Brake_Valve_Short.publish(Brake_Valve_Short);
        
        // bit1: 刹车比例阀输出线路开路 (bool, 0或1)
        uint8_t brake_valve_open_bit = (d0_data_alarm >> 1) & 0x01;
        Brake_Valve_Open.data = brake_valve_open_bit;    // 1:开路状态，0:短路到地
        pub_Brake_Valve_Open.publish(Brake_Valve_Open);
        
        // bit2: 刹车比例阀输出线路过流 (bool, 0或1)
        uint8_t brake_valve_overcurrent_bit = (d0_data_alarm >> 2) & 0x01;
        Brake_Valve_Overcurrent.data = brake_valve_overcurrent_bit;  // 1:过流状态，0:短路到电源
        pub_Brake_Valve_Overcurrent.publish(Brake_Valve_Overcurrent);
        
        // bit3: 左转比例阀输出线路短路 (bool, 0或1)
        uint8_t left_turn_valve_short_bit = (d0_data_alarm >> 3) & 0x01;
        Left_Turn_Valve_Short.data = left_turn_valve_short_bit;  // 1:短路状态，0:开路状态
        pub_Left_Turn_Valve_Short.publish(Left_Turn_Valve_Short);
        
        // bit4: 左转比例阀输出线路开路 (bool, 0或1)
        uint8_t left_turn_valve_open_bit = (d0_data_alarm >> 4) & 0x01;
        Left_Turn_Valve_Open.data = left_turn_valve_open_bit;    // 1:开路状态，0:短路到地
        pub_Left_Turn_Valve_Open.publish(Left_Turn_Valve_Open);
        
        // bit5: 左转比例阀输出线路过流 (bool, 0或1)
        uint8_t left_turn_valve_overcurrent_bit = (d0_data_alarm >> 5) & 0x01;
        Left_Turn_Valve_Overcurrent.data = left_turn_valve_overcurrent_bit;  // 1:过流状态，0:短路到电源
        pub_Left_Turn_Valve_Overcurrent.publish(Left_Turn_Valve_Overcurrent);
        
        // bit6: 右转比例阀输出线路短路 (bool, 0或1)
        uint8_t right_turn_valve_short_bit = (d0_data_alarm >> 6) & 0x01;
        Right_Turn_Valve_Short.data = right_turn_valve_short_bit;  // 1:短路状态，0:开路状态
        pub_Right_Turn_Valve_Short.publish(Right_Turn_Valve_Short);
        
        // bit7: 右转比例阀输出线路开路 (bool, 0或1)
        uint8_t right_turn_valve_open_bit = (d0_data_alarm >> 7) & 0x01;
        Right_Turn_Valve_Open.data = right_turn_valve_open_bit;    // 1:开路状态，0:短路到地
        pub_Right_Turn_Valve_Open.publish(Right_Turn_Valve_Open);
        
        // 解析D1字节
        uint8_t d1_data_alarm = frame_AlarmStatus.data[1];
        
        // bit0: 右转比例阀输出线路过流 (bool, 0或1)
        uint8_t right_turn_valve_overcurrent_bit = d1_data_alarm & 0x01;
        Right_Turn_Valve_Overcurrent.data = right_turn_valve_overcurrent_bit;  // 1:过流状态，0:短路到电源
        pub_Right_Turn_Valve_Overcurrent.publish(Right_Turn_Valve_Overcurrent);
        
        // bit1: 发动机水温高报警 (bool, 0或1)
        uint8_t engine_water_temp_high_bit = (d1_data_alarm >> 1) & 0x01;
        Engine_Water_Temp_High.data = engine_water_temp_high_bit;  // 1:水温高于100℃报警
        pub_Engine_Water_Temp_High.publish(Engine_Water_Temp_High);
        
        // bit2: 变速箱温度高报警 (bool, 0或1)
        uint8_t transmission_temp_high_bit = (d1_data_alarm >> 2) & 0x01;
        Transmission_Temp_High.data = transmission_temp_high_bit;  // 1:变速箱温度高于120℃报警
        pub_Transmission_Temp_High.publish(Transmission_Temp_High);
        
        // bit3: 变速箱压力高报警 (bool, 0或1)
        uint8_t transmission_pressure_high_bit = (d1_data_alarm >> 3) & 0x01;
        Transmission_Pressure_High.data = transmission_pressure_high_bit;  // 1:变速箱压力高于29bar报警
        pub_Transmission_Pressure_High.publish(Transmission_Pressure_High);
        
        // bit4: 变速箱压力低报警 (bool, 0或1)
        uint8_t transmission_pressure_low_bit = (d1_data_alarm >> 4) & 0x01;
        Transmission_Pressure_Low.data = transmission_pressure_low_bit;    // 1:变速箱压力低于20bar报警
        pub_Transmission_Pressure_Low.publish(Transmission_Pressure_Low);
        
        // bit5: 机油压力高报警 (bool, 0或1)
        uint8_t oil_pressure_high_bit = (d1_data_alarm >> 5) & 0x01;
        Oil_Pressure_High.data = oil_pressure_high_bit;  // 1:机油压力高于7bar报警
        pub_Oil_Pressure_High.publish(Oil_Pressure_High);
        
        // bit6: 机油压力低报警 (bool, 0或1)
        uint8_t oil_pressure_low_bit = (d1_data_alarm >> 6) & 0x01;
        Oil_Pressure_Low.data = oil_pressure_low_bit;    // 1:机油压力低于0.7bar报警
        pub_Oil_Pressure_Low.publish(Oil_Pressure_Low);
        
        // bit7: 加档报警（脚刹深踩） (bool, 0或1)
        uint8_t shift_alarm_brake_bit = (d1_data_alarm >> 7) & 0x01;
        Shift_Alarm_Brake.data = shift_alarm_brake_bit;  // 1:脚刹大于60%时还加档报警
        pub_Shift_Alarm_Brake.publish(Shift_Alarm_Brake);
        
        // 解析D2字节
        uint8_t d2_data_alarm = frame_AlarmStatus.data[2];
        
        // bit0: 燃油液位低报警 (bool, 0或1)
        uint8_t fuel_level_low_bit = d2_data_alarm & 0x01;
        Fuel_Level_Low.data = fuel_level_low_bit;        // 1:燃油液位低于20%报警
        pub_Fuel_Level_Low.publish(Fuel_Level_Low);
        
        // bit1: 尿素液位低报警 (bool, 0或1)
        uint8_t urea_level_low_bit = (d2_data_alarm >> 1) & 0x01;
        Urea_Level_Low.data = urea_level_low_bit;        // 1:尿素液位低于20%报警
        pub_Urea_Level_Low.publish(Urea_Level_Low);
        
        // bit2: 变速箱吸油报警 (bool, 0或1)
        uint8_t transmission_suction_alarm_bit = (d2_data_alarm >> 2) & 0x01;
        Transmission_Suction_Alarm.data = transmission_suction_alarm_bit;  // 1:液压吸油报警/变速箱吸油滤芯报警
        pub_Transmission_Suction_Alarm.publish(Transmission_Suction_Alarm);
        
        // bit3: 超速换挡报警 (bool, 0或1)
        uint8_t overspeed_shift_alarm_bit = (d2_data_alarm >> 3) & 0x01;
        OverSpeed_Shift_Alarm.data = overspeed_shift_alarm_bit;  // 1:超速换档报警
        pub_OverSpeed_Shift_Alarm.publish(OverSpeed_Shift_Alarm);
        
        // bit4: GPS已锁车 (bool, 0或1)
        uint8_t gps_locked_bit = (d2_data_alarm >> 4) & 0x01;
        GPS_Locked.data = gps_locked_bit;                // 1:GPS已锁车
        pub_GPS_Locked.publish(GPS_Locked);
        
        // 解析D3字节
        uint8_t d3_data_alarm = frame_AlarmStatus.data[3];
        
        // bit0: 手柄故障 (bool, 0或1)
        uint8_t handle_fault_bit = d3_data_alarm & 0x01;
        Handle_Fault.data = handle_fault_bit;            // 1:手柄错误
        pub_Handle_Fault.publish(Handle_Fault);
        
        // bit1: GPS信号丢失 (bool, 0或1)
        uint8_t gps_signal_lost_bit = (d3_data_alarm >> 1) & 0x01;
        GPS_Signal_Lost.data = gps_signal_lost_bit;      // 1:GPS失联超过10分钟
        pub_GPS_Signal_Lost.publish(GPS_Signal_Lost);
        
        // bit2: 液压回油报警 (bool, 0或1)
        uint8_t hydraulic_return_alarm_bit = (d3_data_alarm >> 2) & 0x01;
        Hydraulic_Return_Alarm.data = hydraulic_return_alarm_bit;  // 1:液压回油报警
        pub_Hydraulic_Return_Alarm.publish(Hydraulic_Return_Alarm);
        
        // bit3: 转向吸油滤报警 (bool, 0或1)
        uint8_t steering_suction_filter_bit = (d3_data_alarm >> 3) & 0x01;
        Steering_Suction_Filter.data = steering_suction_filter_bit;  // 1:液压转向吸油滤报警
        pub_Steering_Suction_Filter.publish(Steering_Suction_Filter);
        
        // bit4: 手刹未放下报警 (bool, 0或1)
        uint8_t handbrake_not_released_bit = (d3_data_alarm >> 4) & 0x01;
        Handbrake_Not_Released.data = handbrake_not_released_bit;  // 1:手刹未放下报警
        pub_Handbrake_Not_Released.publish(Handbrake_Not_Released);
        
        // bit5: 刹车踏板线路故障 (bool, 0或1)
        uint8_t brake_pedal_fault_bit = (d3_data_alarm >> 5) & 0x01;
        Brake_Pedal_Fault.data = brake_pedal_fault_bit;  // 1:刹车踏板线路故障
        pub_Brake_Pedal_Fault.publish(Brake_Pedal_Fault);
        
        // bit6: 燃油液位线路故障 (bool, 0或1)
        uint8_t fuel_level_fault_bit = (d3_data_alarm >> 6) & 0x01;
        Fuel_Level_Fault.data = fuel_level_fault_bit;    // 1:燃油液位线路故障
        pub_Fuel_Level_Fault.publish(Fuel_Level_Fault);
        
        // bit7: 变速箱压力线路故障 (bool, 0或1)
        uint8_t transmission_pressure_fault_bit = (d3_data_alarm >> 7) & 0x01;
        Transmission_Pressure_Fault.data = transmission_pressure_fault_bit;  // 1:变速箱压力线路故障
        pub_Transmission_Pressure_Fault.publish(Transmission_Pressure_Fault);
        
        // 解析D4字节
        uint8_t d4_data_alarm = frame_AlarmStatus.data[4];
        
        // bit2: 发动机水位低报警 (bool, 0或1)
        uint8_t engine_water_level_low_bit = (d4_data_alarm >> 2) & 0x01;
        Engine_Water_Level_Low.data = engine_water_level_low_bit;  // 1:发动机水位低报警
        pub_Engine_Water_Level_Low.publish(Engine_Water_Level_Low);
        
        // bit3: 空气滤芯报警 (bool, 0或1)
        uint8_t air_filter_alarm_bit = (d4_data_alarm >> 3) & 0x01;
        Air_Filter_Alarm.data = air_filter_alarm_bit;    // 1:空气滤芯报警
        pub_Air_Filter_Alarm.publish(Air_Filter_Alarm);
        
        // bit4: 液压油位低报警 (bool, 0或1)
        uint8_t hydraulic_oil_level_low_bit = (d4_data_alarm >> 4) & 0x01;
        Hydraulic_Oil_Level_Low.data = hydraulic_oil_level_low_bit;  // 1:液压油位低报警
        pub_Hydraulic_Oil_Level_Low.publish(Hydraulic_Oil_Level_Low);
        
        // bit5: 操作有误（刹车长时间踩下行进） (bool, 0或1)
        uint8_t operation_error_brake_bit = (d4_data_alarm >> 5) & 0x01;
        Operation_Error_Brake.data = operation_error_brake_bit;  // 1:操作有误，刹车长时间踩下行进
        pub_Operation_Error_Brake.publish(Operation_Error_Brake);
        
        // 解析D6字节
        uint8_t d6_data_alarm = frame_AlarmStatus.data[6];
        
        // bit0: 涡轮转速信号丢失报警 (bool, 0或1)
        uint8_t turbo_speed_signal_lost_bit = d6_data_alarm & 0x01;
        Turbo_Speed_Signal_Lost.data = turbo_speed_signal_lost_bit;  // 1:涡轮转速信号丢失报警
        pub_Turbo_Speed_Signal_Lost.publish(Turbo_Speed_Signal_Lost);
        
        // bit1: 变速箱温度线路故障预留 (bool, 0或1)
        uint8_t transmission_temp_fault_bit = (d6_data_alarm >> 1) & 0x01;
        Transmission_Temp_Fault.data = transmission_temp_fault_bit;  // 1:变速箱温度线路故障预留
        pub_Transmission_Temp_Fault.publish(Transmission_Temp_Fault);
        
        // bit2: 变速箱压力线路故障 (bool, 0或1)
        uint8_t transmission_pressure_fault2_bit = (d6_data_alarm >> 2) & 0x01;
        Transmission_Pressure_Fault2.data = transmission_pressure_fault2_bit;  // 1:变速箱压力线路故障
        pub_Transmission_Pressure_Fault2.publish(Transmission_Pressure_Fault2);
        
        // bit3: 一键启停按钮针脚故障 (bool, 0或1)
        uint8_t startstop_button_fault_bit = (d6_data_alarm >> 3) & 0x01;
        StartStop_Button_Fault.data = startstop_button_fault_bit;  // 1:一键启停按钮针脚故障
        pub_StartStop_Button_Fault.publish(StartStop_Button_Fault);

        // ============ 尿素浓度解析（修正版） ============
        auto& frame_Urea = can_frames_[DEVICE_UREA_CONCENTRATION];

        //uint16_t raw = (static_cast<uint16_t>(frame_Urea.data[1]) << 8) | (static_cast<uint16_t>(frame_Urea.data[0]));
        uint16_t raw = static_cast<uint16_t>(frame_Urea.data[1]);
        Urea_Concentration.data = raw * 0.25f; // 转换为百分比
        pub_Urea_Concentration.publish(Urea_Concentration);



        // 构建 systemErrorCode（32位，每位一个故障）
        /*uint32_t code = 0;

        #define SET_BIT_IF(cond, bit) if (cond) code |= (1U << (bit))

        SET_BIT_IF(Engine_Water_Temp_High.data,        0);
        SET_BIT_IF(Transmission_Temp_High.data,        1);
        SET_BIT_IF(Oil_Pressure_Low.data,              2);
        SET_BIT_IF(Fuel_Level_Low.data,                3);
        SET_BIT_IF(Urea_Level_Low.data,                4);
        SET_BIT_IF(Handbrake_Not_Released.data,        5);
        SET_BIT_IF(GPS_Signal_Lost.data,               6);
        SET_BIT_IF(OverSpeed_Shift_Alarm.data,         7);
        SET_BIT_IF(Brake_Valve_Short.data,             8);
        SET_BIT_IF(Brake_Valve_Open.data,              9);
        SET_BIT_IF(Brake_Valve_Overcurrent.data,      10);
        SET_BIT_IF(Transmission_Pressure_High.data,   11);
        SET_BIT_IF(Transmission_Pressure_Low.data,    12);
        SET_BIT_IF(Shift_Alarm_Brake.data,            13);
        SET_BIT_IF(Transmission_Suction_Alarm.data,   14);
        SET_BIT_IF(Hydraulic_Return_Alarm.data,       15);
        SET_BIT_IF(Steering_Suction_Filter.data,      16);
        SET_BIT_IF(Brake_Pedal_Fault.data,            17);
        SET_BIT_IF(Fuel_Level_Fault.data,             18);
        SET_BIT_IF(Transmission_Pressure_Fault.data,  19);
        SET_BIT_IF(Engine_Water_Level_Low.data,       20);
        SET_BIT_IF(Air_Filter_Alarm.data,             21);
        SET_BIT_IF(Hydraulic_Oil_Level_Low.data,      22);
        SET_BIT_IF(Operation_Error_Brake.data,        23);
        SET_BIT_IF(Turbo_Speed_Signal_Lost.data,      24);
        SET_BIT_IF(StartStop_Button_Fault.data,       25);
        SET_BIT_IF(Oil_Pressure_High.data,            26);
        SET_BIT_IF(Transmission_Temp_Fault.data,      27);
        SET_BIT_IF(Transmission_Pressure_Fault2.data, 28);
        SET_BIT_IF(Handle_Fault.data,                 29);
        SET_BIT_IF(GPS_Locked.data,                   30);
        SET_BIT_IF(Speed_Limit_Status.data,           31);

        #undef SET_BIT_IF

        systemErrorCode.data = code;
        pub_systemErrorCode.publish(systemErrorCode);*/


    // ================= 新增：构建并发布 VCU_Error_Code (原始字节) =================


    VCU_Error_Code.data[0] = frame_AlarmStatus.data[0]; // B0: 刹车/转向比例阀报警
    VCU_Error_Code.data[1] = frame_AlarmStatus.data[1]; // B1: 水温/变速箱/机油/加油报警
    VCU_Error_Code.data[2] = frame_AlarmStatus.data[2]; // B2: 燃油/尿素/GPS/超速等报警
    VCU_Error_Code.data[3] = frame_AlarmStatus.data[3]; // B3: 手柄/液压/转向/线路故障
    VCU_Error_Code.data[4] = frame_AlarmStatus.data[4]; // B4: 滤芯/油位/操作错误/涡轮报警 
    VCU_Error_Code.data[5] = 0;                         // B5: 协议未定义，填充 0
    VCU_Error_Code.data[6] = frame_AlarmStatus.data[6]; // B6: 涡轮/变速箱线路/一键启停故障 ← 新增！
    VCU_Error_Code.data[7] = 0;                         // B7: 协议未定义，填充 0

    pub_VCU_Error_Code.publish(VCU_Error_Code);



        // 构建 Light_Status (复用地灯状态)
        uint32_t light_code = 0;

        #define SET_LIGHT_BIT(cond, bit) if (cond) light_code |= (1U << (bit))

        SET_LIGHT_BIT(Red_Stop_Light.data, 0);       // 数位 0
        SET_LIGHT_BIT(Environment_Warning.data, 1);  // 数位 1
        SET_LIGHT_BIT(Protection_Light.data, 2);     // 数位 2

        #undef SET_LIGHT_BIT

        lightStatus.data = light_code;
        pub_Light_Status.publish(lightStatus);



        // 时间监控
        auto end = std::chrono::high_resolution_clock::now();
        // std::cout << "Duration: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() * 1e-3 << " ms" << std::endl;
        loop_rate1.sleep();
    }
    
    // 关闭所有CAN套接字
    for (auto& pair : can_fds_) {
        close(pair.second);
    }
}


template<typename... Args>
std::string can_to_ros::string_format(const std::string& format, Args... args) {
    size_t size = snprintf(nullptr, 0, format.c_str(), args...) + 1; 
    std::unique_ptr<char[]> buf(new char[size]); 
    snprintf(buf.get(), size, format.c_str(), args...); 
    return std::string(buf.get(), buf.get() + size - 1); 
}