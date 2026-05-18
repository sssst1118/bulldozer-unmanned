/**
 * @file can_to_ros.h
 * @brief Can To Ros
 * @author dozer-dev
 * @date 2026-03-15
 */
#ifndef SRC_NOISE_FILTER_H
#define SRC_NOISE_FILTER_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>

#include <cstdint>
#include <iomanip>
#include <math.h>
#include <thread>

#include <sensor_msgs/NavSatFix.h>  // 用于 sensor_msgs::NavSatFix
#include <std_msgs/Int8.h>          // 用于 std_msgs::Int8
#include <std_msgs/UInt8.h>         
#include <std_msgs/Float64.h>       // 若用到 Float64

// CAN
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>

#include <unordered_map>
#include <vector>

using namespace std;
const double PI = 3.14159265358979323846;

// CAN设备配置结构体
struct CanDeviceConfig {
    std::string if_name;          // CAN接口名称
    uint32_t can_id;              // CAN ID
    bool is_extended;             // 是否为扩展帧
};

// 注：以下 CAN ID 均为示例占位值，实际使用时应按所用 IMU 设备的 CAN 协议自行配置
typedef enum imu_acc_can_id {
    ACC_ANGULAR_X = 0x301,  // X轴角速度（data[4-6]）
    ACC_ANGULAR_Y = 0x302,  // Y轴角速度（data[4-6]）
    ACC_ANGULAR_Z = 0x303,  // Z轴角速度（data[4-6]）
    PITCH_ROLL    = 0x304   // 俯仰角（data[0-2]）、横滚角（data[4-6]）
} imu_acc_can_id;

// IMU 姿态融合帧 CAN ID 枚举（占位示例值）
typedef enum imu_ahrs_can_id {
    ROLL_PITCH = 0x311,
    YAW_GX = 0x312,
    GY_GZ = 0x313,
    AX_AY = 0x314,
    AZ_TEMP = 0x315
} imu_ahrs_can_id;

// CAN设备枚举
enum CanDevice {
    DEVICE_IMU_GYRO_RAW,  // IMU 角速度原始值
    DEVICE_IMU_ACCEL_RAW,  // IMU 加速度原始值
    DEVICE_GPS_LONGITUDE,  // 经度
    DEVICE_GPS_LATITUDE,  // 纬度
    DEVICE_GPS_ALTITUDE,  // 高度
    DEVICE_RTK_STATUS,  // RTK状态
    DEVICE_GPS_HEADING,  // 航向角
    DEVICE_POS_SIGMA,  // 车速
    DEVICE_VEHICLE_SPEED_ENU,  // 车速
    DEVICE_VEHICLE_ACCEL,  // 车辆坐标系加速度
    DEVICE_HEADING_VELOCITY,  // 航向角速度
    DEVICE_ENGINE_TORQUE_SPEED,  // 发动机实际扭矩百分比、转速
    DEVICE_TRANSMISSION_SPEED,  // 变速箱转速
    DEVICE_OUTCURR, // 输出电流
    DEVICE_IMU_ACC,  // ACC IMU设备
    DEVICE_IMU_AHRS, // AHRS IMU设备

    DEVICE_FUEL_TOTAL,            // 燃料使用总量
    DEVICE_ENGINE_TOTAL_HOURS,    // 发动机工作时间
    DEVICE_VEHICLE_SPEED_DISPLAY, // 车速显示
    DEVICE_COOLANT_TEMP,          // 发动机冷却液温度
    DEVICE_WARNING_LIGHTS,        // 警告灯状态和故障信息
    DEVICE_WARNING_LIGHTS2,       // 警告灯状态2
    DEVICE_SPEED_LIMIT,           // 速度限制状态
    DEVICE_GEAR_MODE,             // 档位模式
    DEVICE_LOCK_STATUS,           // 锁车状态
    DEVICE_HANDLE_INFO,           // 手柄信息
    DEVICE_FUEL_LEVEL,            // 燃油油位
    DEVICE_CONTROL_INPUTS,        // 控制输入
    DEVICE_TRANSMISSION_VALVE,    // 变速箱档位阀
    DEVICE_ALARM_STATUS,          // 报警状态
    DEVICE_UREA_CONCENTRATION,    // 尿素浓度

    DEVICE_COUNT // 设备数量

};

class can_to_ros {

public:
    explicit can_to_ros(ros::NodeHandle& nodeHandle);
    virtual ~can_to_ros() = default;
    void read_can();

private:
    // ROS节点句柄
    ros::NodeHandle nh;

    // CAN设备配置表
    std::unordered_map<CanDevice, CanDeviceConfig> can_configs_;
    
    // CAN设备帧存储
    std::unordered_map<CanDevice, can_frame> can_frames_;
    
    // CAN设备文件描述符
    std::unordered_map<CanDevice, int> can_fds_;

/*--------------     ros::Publisher    ---------------------*/
    // RTK
    ros::Publisher pub_LLA;
    ros::Publisher pub_Angle_Heading;
    ros::Publisher pub_Vehicle_Speed;   // 东北天（ENU）坐标系下的线速度
    ros::Publisher pub_Vehicle_Speed_Vel;
    ros::Publisher pub_Angle_Heading_Velocity;
    ros::Publisher pub_RTK;
    ros::Publisher pub_IMU_Accel;
    ros::Publisher pub_RTK_IMU;
    ros::Publisher pub_InsPva;          // 经纬高 + 横滚俯仰航向 + 状态质量字
    //ros::Publisher pub_TwistStamped;    // 东北天（ENU）坐标系下的线速度
    ros::Publisher pub_PosSigma;        // 位置标准差
    ros::Publisher pub_imu_link;        // 未经滤波的角速度 & 线加速度

    // 发动机
    ros::Publisher pub_Engine_Torque_Actual;
    ros::Publisher pub_Engine_Speed_Actual;

    //变速箱
    ros::Publisher pub_Transmission_Speed_Actual;

    // IMU
    ros::Publisher pub_IMU_Angular_Velocity;  // X/Y/Z轴角速度（PointStamped）
    ros::Publisher pub_IMU_Pitch;             // 俯仰角（Float64）
    ros::Publisher pub_IMU_Roll;              // 横滚角（Float64）

    // 输出电流
    ros::Publisher pub_OutCurr; 

    // 新增发布者
    ros::Publisher pub_Fuel_Total;
    ros::Publisher pub_Engine_Total_Hours;
    ros::Publisher pub_Vehicle_Speed_Display;
    ros::Publisher pub_Coolant_Temperature;
    ros::Publisher pub_Protection_Light;
    ros::Publisher pub_Environment_Warning;
    ros::Publisher pub_Red_Stop_Light;
    ros::Publisher pub_MIL_Light;
    ros::Publisher pub_SPN;
    ros::Publisher pub_FMI;
    ros::Publisher pub_Fault_Count;
    ros::Publisher pub_Protection_Light2;
    ros::Publisher pub_Environment_Warning2;
    ros::Publisher pub_Red_Stop_Light2;
    ros::Publisher pub_MIL_Light2;
    ros::Publisher pub_SPN2;
    ros::Publisher pub_FMI2;
    ros::Publisher pub_Fault_Count2;
    ros::Publisher pub_Speed_Limit_Status;
    ros::Publisher pub_Gear_Mode;
    ros::Publisher pub_Vehicle_Lock;
    ros::Publisher pub_Anti_Theft;
    ros::Publisher pub_About_To_Lock;
    ros::Publisher pub_Neutral_Start;
    ros::Publisher pub_Gear_Position;
    ros::Publisher pub_Gear_Direction;
    ros::Publisher pub_Manual_Auto_Switch;
    ros::Publisher pub_OverSpeed_Alarm;
    ros::Publisher pub_Handle_Left_Turn;
    ros::Publisher pub_Handle_Right_Turn;
    ros::Publisher pub_Wheel_Gear_Up;
    ros::Publisher pub_Wheel_Gear_Down;
    ros::Publisher pub_Wheel_Gear_Value;
    ros::Publisher pub_Handle_Turn_Value;
    ros::Publisher pub_Wheel_Gear_Turn_Value;
    ros::Publisher pub_Fuel_Level;
    ros::Publisher pub_Fuel_Low_Alarm;
    ros::Publisher pub_Power_Cutoff;
    ros::Publisher pub_Hydraulic_Lock;
    ros::Publisher pub_Low_Oil_Pressure_Inlet;
    ros::Publisher pub_Low_Oil_Pressure_Return;
    ros::Publisher pub_Transmission_Oil_Filter;
    ros::Publisher pub_Diesel_Filter;
    ros::Publisher pub_Hand_Brake;
    ros::Publisher pub_Gear_Select_Input1;
    ros::Publisher pub_Gear_Select_Input2;
    ros::Publisher pub_Gear_Select_Input3;
    ros::Publisher pub_Gear_Select_Input4;
    ros::Publisher pub_Transmission_Valve1;
    ros::Publisher pub_Transmission_Valve2;
    ros::Publisher pub_Transmission_Valve3;
    ros::Publisher pub_Transmission_Valve4;
    ros::Publisher pub_Brake_Valve_Short;
    ros::Publisher pub_Brake_Valve_Open;
    ros::Publisher pub_Brake_Valve_Overcurrent;
    ros::Publisher pub_Left_Turn_Valve_Short;
    ros::Publisher pub_Left_Turn_Valve_Open;
    ros::Publisher pub_Left_Turn_Valve_Overcurrent;
    ros::Publisher pub_Right_Turn_Valve_Short;
    ros::Publisher pub_Right_Turn_Valve_Open;
    ros::Publisher pub_Right_Turn_Valve_Overcurrent;
    ros::Publisher pub_Engine_Water_Temp_High;
    ros::Publisher pub_Transmission_Temp_High;
    ros::Publisher pub_Transmission_Pressure_High;
    ros::Publisher pub_Transmission_Pressure_Low;
    ros::Publisher pub_Oil_Pressure_High;
    ros::Publisher pub_Oil_Pressure_Low;
    ros::Publisher pub_Shift_Alarm_Brake;
    ros::Publisher pub_Fuel_Level_Low;
    ros::Publisher pub_Urea_Level_Low;
    ros::Publisher pub_Transmission_Suction_Alarm;
    ros::Publisher pub_OverSpeed_Shift_Alarm;
    ros::Publisher pub_GPS_Locked;
    ros::Publisher pub_Handle_Fault;
    ros::Publisher pub_GPS_Signal_Lost;
    ros::Publisher pub_Hydraulic_Return_Alarm;
    ros::Publisher pub_Steering_Suction_Filter;
    ros::Publisher pub_Handbrake_Not_Released;
    ros::Publisher pub_Brake_Pedal_Fault;
    ros::Publisher pub_Fuel_Level_Fault;
    ros::Publisher pub_Transmission_Pressure_Fault;
    ros::Publisher pub_Engine_Water_Level_Low;
    ros::Publisher pub_Air_Filter_Alarm;
    ros::Publisher pub_Hydraulic_Oil_Level_Low;
    ros::Publisher pub_Operation_Error_Brake;
    ros::Publisher pub_Turbo_Speed_Signal_Lost;
    ros::Publisher pub_Transmission_Temp_Fault;
    ros::Publisher pub_Transmission_Pressure_Fault2;
    ros::Publisher pub_StartStop_Button_Fault;
    ros::Publisher pub_Urea_Concentration;


    //ros::Publisher pub_systemErrorCode;

    ros::Publisher pub_Light_Status;

    // 新增：VCU 原始错误码发布器 (报警状态帧的原始字节)
    ros::Publisher pub_VCU_Error_Code;
    
/*--------------     数据格式    ---------------------*/
    // RTK
    geometry_msgs::PointStamped LLA;
    geometry_msgs::PointStamped Angle_Heading;
    geometry_msgs::TwistStamped Vehicle_Speed; 
    std_msgs::Float64 Vehicle_Speed_Vel; 
    std_msgs::Float64 Angle_Heading_Velocity;
    sensor_msgs::NavSatFix RTK;
    geometry_msgs::PointStamped IMU_Accelerated_Velocity;
    sensor_msgs::Imu RTK_IMU;  
    Eigen::Quaterniond quaternion;
    Eigen::AngleAxisd rollAngle,pitchAngle,yawAngle;
    //chc_msgs::InsPva ins_pva;                   // 经纬高 + 横滚俯仰航向 + 状态质量字
    //geometry_msgs::TwistStamped twist_stamped;  // 东北天（ENU）坐标系下的线速度
    geometry_msgs::PointStamped pos_sigma;      // 位置标准差
    sensor_msgs::Imu imu_link;                  // 未经滤波的角速度 & 线加速度


    // 发动机
    std_msgs::Int8 Engine_Torque_Actual; //发动机实际扭矩百分比
    std_msgs::Float64 Engine_Speed_Actual; //发动机实际转速

    // 变速箱
    std_msgs::Int16 Transmission_Speed_Actual; //变速箱实际转速

    // IMU
    geometry_msgs::PointStamped IMU_Angular_Velocity;  // 角速度（x:X轴, y:Y轴, z:Z轴）
    std_msgs::Float64 IMU_Pitch;                       // 俯仰角
    std_msgs::Float64 IMU_Roll;                        // 横滚角
    std_msgs::UInt16MultiArray OUTPUT_CURRENT;

    // 新增数据格式
    std_msgs::Float64 Fuel_Total;
    std_msgs::Float64 Engine_Total_Hours;
    std_msgs::Float64 Vehicle_Speed_Display;
    std_msgs::Float64 Coolant_Temperature;
    std_msgs::UInt8 Protection_Light;
    std_msgs::UInt8 Environment_Warning;
    std_msgs::UInt8 Red_Stop_Light;
    std_msgs::UInt8 MIL_Light;
    std_msgs::UInt16 SPN;
    std_msgs::UInt8 FMI;
    std_msgs::UInt8 Fault_Count;
    std_msgs::UInt8 Protection_Light2;
    std_msgs::UInt8 Environment_Warning2;
    std_msgs::UInt8 Red_Stop_Light2;
    std_msgs::UInt8 MIL_Light2;
    std_msgs::UInt16 SPN2;
    std_msgs::UInt8 FMI2;
    std_msgs::UInt8 Fault_Count2;
    std_msgs::UInt8 Speed_Limit_Status;
    std_msgs::UInt8 Gear_Mode;
    std_msgs::UInt8 Vehicle_Lock;
    std_msgs::UInt8 Anti_Theft;
    std_msgs::UInt8 About_To_Lock;
    std_msgs::UInt8 Neutral_Start;
    std_msgs::UInt8 Gear_Position;
    std_msgs::UInt8 Gear_Direction;
    std_msgs::UInt8 Manual_Auto_Switch;
    std_msgs::UInt8 OverSpeed_Alarm;
    std_msgs::UInt8 Handle_Left_Turn;
    std_msgs::UInt8 Handle_Right_Turn;
    std_msgs::UInt8 Wheel_Gear_Up;
    std_msgs::UInt8 Wheel_Gear_Down;
    std_msgs::UInt8 Wheel_Gear_Value;
    std_msgs::UInt8 Handle_Turn_Value;
    std_msgs::UInt8 Wheel_Gear_Turn_Value;
    std_msgs::UInt8 Fuel_Level;
    std_msgs::UInt8 Fuel_Low_Alarm;
    std_msgs::UInt8 Power_Cutoff;
    std_msgs::UInt8 Hydraulic_Lock;
    std_msgs::UInt8 Low_Oil_Pressure_Inlet;
    std_msgs::UInt8 Low_Oil_Pressure_Return;
    std_msgs::UInt8 Transmission_Oil_Filter;
    std_msgs::UInt8 Diesel_Filter;
    std_msgs::UInt8 Hand_Brake;
    std_msgs::UInt8 Gear_Select_Input1;
    std_msgs::UInt8 Gear_Select_Input2;
    std_msgs::UInt8 Gear_Select_Input3;
    std_msgs::UInt8 Gear_Select_Input4;
    std_msgs::UInt8 Transmission_Valve1;
    std_msgs::UInt8 Transmission_Valve2;
    std_msgs::UInt8 Transmission_Valve3;
    std_msgs::UInt8 Transmission_Valve4;
    std_msgs::UInt8 Brake_Valve_Short;
    std_msgs::UInt8 Brake_Valve_Open;
    std_msgs::UInt8 Brake_Valve_Overcurrent;
    std_msgs::UInt8 Left_Turn_Valve_Short;
    std_msgs::UInt8 Left_Turn_Valve_Open;
    std_msgs::UInt8 Left_Turn_Valve_Overcurrent;
    std_msgs::UInt8 Right_Turn_Valve_Short;
    std_msgs::UInt8 Right_Turn_Valve_Open;
    std_msgs::UInt8 Right_Turn_Valve_Overcurrent;
    std_msgs::UInt8 Engine_Water_Temp_High;
    std_msgs::UInt8 Transmission_Temp_High;
    std_msgs::UInt8 Transmission_Pressure_High;
    std_msgs::UInt8 Transmission_Pressure_Low;
    std_msgs::UInt8 Oil_Pressure_High;
    std_msgs::UInt8 Oil_Pressure_Low;
    std_msgs::UInt8 Shift_Alarm_Brake;
    std_msgs::UInt8 Fuel_Level_Low;
    std_msgs::UInt8 Urea_Level_Low;
    std_msgs::UInt8 Transmission_Suction_Alarm;
    std_msgs::UInt8 OverSpeed_Shift_Alarm;
    std_msgs::UInt8 GPS_Locked;
    std_msgs::UInt8 Handle_Fault;
    std_msgs::UInt8 GPS_Signal_Lost;
    std_msgs::UInt8 Hydraulic_Return_Alarm;
    std_msgs::UInt8 Steering_Suction_Filter;
    std_msgs::UInt8 Handbrake_Not_Released;
    std_msgs::UInt8 Brake_Pedal_Fault;
    std_msgs::UInt8 Fuel_Level_Fault;
    std_msgs::UInt8 Transmission_Pressure_Fault;
    std_msgs::UInt8 Engine_Water_Level_Low;
    std_msgs::UInt8 Air_Filter_Alarm;
    std_msgs::UInt8 Hydraulic_Oil_Level_Low;
    std_msgs::UInt8 Operation_Error_Brake;
    std_msgs::UInt8 Turbo_Speed_Signal_Lost;
    std_msgs::UInt8 Transmission_Temp_Fault;
    std_msgs::UInt8 Transmission_Pressure_Fault2;
    std_msgs::UInt8 StartStop_Button_Fault;
    std_msgs::Float64 Urea_Concentration;

    //std_msgs::UInt32 systemErrorCode;

    std_msgs::UInt32 lightStatus;

    // 新增：VCU 原始错误码消息容器 (UInt8MultiArray)
    std_msgs::UInt8MultiArray VCU_Error_Code;

    // 辅助函数：初始化CAN设备
    bool init_can_device(CanDevice device);
    
    // 辅助函数：读取CAN设备数据
    void read_can_device(CanDevice device);
    
    // 辅助函数（24位有符号整数转32位）
    int32_t int24_to_int32(int32_t num);
    int32_t int20_to_int32(int32_t num);
    
    // 解析IMU的单个CAN帧并发布对应数据（非阻塞）
    void parse_imu_acc_frame(const struct can_frame& frame);
    bool  parse_imu_ahrs_frame(const struct can_frame& frame);
    
    // 初始化所有CAN配置
    void init_can_configs();
    
    // 初始化所有发布者
    void init_publishers();


    sensor_msgs::Imu ahrs400_data;  
    ros::Publisher pub_ahrs_imu;     
    uint8_t can_output_temp_new = 0;    
    bool start_flag_new;            
    uint32_t seq_new;               

    template<typename... Args>
    std::string string_format(const std::string& format, Args... args);
};

#endif // SRC_NOISE_FILTER_H
