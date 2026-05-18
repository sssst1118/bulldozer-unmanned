/**
 * @file ros_to_can.h
 * @brief Ros To Can
 * @author dozer-dev
 * @date 2026-03-15
 */

#ifndef SRC_NOISE_FILTER_H
#define SRC_NOISE_FILTER_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <chrono>

//CAN
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>


class ros_to_can{

public:
    /*!
     * 构造函数
     */
    explicit ros_to_can(ros::NodeHandle& nodeHandle);

    /*!
     * 析构函数
     */
    virtual ~ros_to_can() = default;

    /*!
     * 回调函数
     */
    void U_Lever1_Callback(const std_msgs::Int16MultiArray& lever1);
    void U_Lever2_Callback(const std_msgs::Int16MultiArray& lever2);
    void U_Lever3_Callback(const std_msgs::Int32& level3);
    void U_Lever4_Callback(const std_msgs::Int32& level4);
    void U_Lever5_Callback(const std_msgs::Int16MultiArray& lever5);
    void U_Lever6_Callback(const std_msgs::Int16& lever6);
    void U_Lever7_Callback(const std_msgs::Int16& lever7);
    void U_Lever8_Callback(const std_msgs::Int16& lever8);

    void U_Lever_Moldboard_Callback(const std_msgs::Int16MultiArray& lever_Moldboard);

    // [合并 sys_control] 模式切换回调
    void Mode_Switch_Callback(const std_msgs::Int32& mode);

private:

    // ROS nodehandle.
    ros::NodeHandle nodeHandle_;

    // ROS subscribers.
    ros::Subscriber U_Lever1_Subscriber;
    ros::Subscriber U_Lever2_Subscriber;
    ros::Subscriber U_Brake_Subscriber;
    ros::Subscriber U_Engine_Speed_Subscriber;
    ros::Subscriber U_Lever5_Subscriber;
    ros::Subscriber U_Lever6_Subscriber;
    ros::Subscriber U_Lever7_Subscriber;
    ros::Subscriber U_Moldboard_Control_Flag_Subscriber;

    ros::Subscriber U_Lever_Moldboard_Subscriber;

    // [合并 sys_control] 模式切换
    ros::Subscriber Mode_Switch_Subscriber;
    ros::Timer mode_switch_timer_;         ///< 20Hz 持续发送模式帧
    int mode_switch_flag_ = 0;             ///< 0=人工, 1=无人
    void modeSwitchTimerCallback(const ros::TimerEvent& event);

    // CAN 
    int can_fd;
    struct sockaddr_can addr;
    struct ifreq ifr;

    struct can_frame Unmanned_Bulldozer_Operation;

    struct can_frame Bulldozer_Moldboard_Target;
    struct can_frame Bulldozer_Moldboard_Actual;
    struct can_frame Bulldozer_Moldboard_Right_Dvalue;
    struct can_frame Bulldozer_Moldboard_Left_Dvalue;
    struct can_frame Moldboard_Control_Flag;

    struct can_frame U_Brake;
    struct can_frame U_Engine_Speed;
    struct can_frame U_Engine_Speed_2;

    struct can_frame U_Lever_Moldboard;

    // [合并 sys_control] 模式切换 CAN 帧（ID 为占位示例值，需按车辆协议替换）
    struct can_frame Mode_Switch_Vehicle;    ///< 整车无人/有人模式
    struct can_frame Mode_Switch_Moldboard;  ///< 铲刀无人/有人模式
    
};


#endif //SRC_NOISE_FILTER_H
