/**
 * @file ros_to_can.cpp
 * @brief Ros To Can
 * @author dozer-dev
 * @date 2026-03-15
 */
#include "ros_to_can.h"
#include <cerrno>
#include <cstring>

// [优化] CAN 发送辅助函数: 检查 fd 有效性 + write 返回值
static inline bool safe_can_write(int fd, const struct can_frame& frame) {
    if (fd < 0) return false;
    ssize_t nbytes = write(fd, &frame, sizeof(frame));
    if (nbytes < 0) {
        ROS_WARN_THROTTLE(5, "[ros_to_can] write failed (CAN ID 0x%X): %s",
                          frame.can_id, strerror(errno));
        return false;
    }
    return true;
}

/*!
 * 构造函数.
 *
 * @param nodeHandle the ROS node handle.
 */
ros_to_can::ros_to_can(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
{

    //ROS
    U_Lever1_Subscriber = nodeHandle_.subscribe("/U_Lever1",1,&ros_to_can::U_Lever1_Callback, this);

    U_Lever2_Subscriber = nodeHandle_.subscribe("/Bulldozer_Moldboard_Target",1,&ros_to_can::U_Lever2_Callback, this);
    U_Lever5_Subscriber = nodeHandle_.subscribe("/Bulldozer_Moldboard_Actual",1,&ros_to_can::U_Lever5_Callback, this);
    U_Lever6_Subscriber = nodeHandle_.subscribe("/Bulldozer_Moldboard_Right_Dvalue",1,&ros_to_can::U_Lever6_Callback, this);
    U_Lever7_Subscriber = nodeHandle_.subscribe("/Bulldozer_Moldboard_Left_Dvalue",1,&ros_to_can::U_Lever7_Callback, this);

    U_Brake_Subscriber = nodeHandle_.subscribe("/U_Brake",1,&ros_to_can::U_Lever3_Callback, this);
    U_Engine_Speed_Subscriber = nodeHandle_.subscribe("/U_Engine_Speed",1,&ros_to_can::U_Lever4_Callback, this);

    U_Moldboard_Control_Flag_Subscriber = nodeHandle_.subscribe("/Moldboard_Control_Flag_Plan",1,&ros_to_can::U_Lever8_Callback, this);

    //U_Honk_Flag_Subscriber = nodeHandle_.subscribe("/Honk_Flag",1,&ros_to_can::U_Lever9_Callback, this);  

    U_Lever_Moldboard_Subscriber = nodeHandle_.subscribe("/U_Lever_Moldboard",1,&ros_to_can::U_Lever_Moldboard_Callback, this);

    // CAN 初始化 (can1 = VCU/发动机/铲刀总线)
    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd < 0) {
        ROS_FATAL("[ros_to_can] socket() failed: %s", strerror(errno));
        return;
    }
    strcpy(ifr.ifr_name, "can1");
    if (ioctl(can_fd, SIOCGIFINDEX, &ifr) < 0) {
        ROS_FATAL("[ros_to_can] ioctl failed for can1: %s", strerror(errno));
        close(can_fd); can_fd = -1;
        return;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_FATAL("[ros_to_can] bind() failed for can1: %s", strerror(errno));
        close(can_fd); can_fd = -1;
        return;
    }
    ROS_INFO("[ros_to_can] can1 opened, fd=%d", can_fd);

    // 注意：以下 CAN ID 均为示例占位值，实际使用时应按目标车辆的
    // CAN 总线协议文档逐一替换
    Unmanned_Bulldozer_Operation.can_dlc = 8;
    Unmanned_Bulldozer_Operation.can_id = CAN_EFF_FLAG | 0x401;

    Bulldozer_Moldboard_Target.can_dlc = 8;
    Bulldozer_Moldboard_Target.can_id = 0x402;

    Bulldozer_Moldboard_Actual.can_dlc = 8;
    Bulldozer_Moldboard_Actual.can_id = 0x403;

    Bulldozer_Moldboard_Right_Dvalue.can_dlc = 8;
    Bulldozer_Moldboard_Right_Dvalue.can_id = 0x404;

    Bulldozer_Moldboard_Left_Dvalue.can_dlc = 8;
    Bulldozer_Moldboard_Left_Dvalue.can_id = 0x405;

    Moldboard_Control_Flag.can_dlc = 8;
    Moldboard_Control_Flag.can_id = 0x406;

    U_Brake.can_dlc = 8;
    U_Brake.can_id = CAN_EFF_FLAG | 0x407;

    U_Engine_Speed.can_dlc = 8;
    U_Engine_Speed.can_id = CAN_EFF_FLAG | 0x408;

    U_Engine_Speed_2.can_dlc = 8;
    U_Engine_Speed_2.can_id = CAN_EFF_FLAG | 0x409;

    //Honk_Flag.can_dlc = 8;
    //Honk_Flag.can_id = 0x500;

    U_Lever_Moldboard.can_dlc = 8;
    U_Lever_Moldboard.can_id = 0x40A;

    // [合并 sys_control] 模式切换 CAN 帧 + 订阅 + 定时器
    Mode_Switch_Vehicle.can_dlc = 8;
    Mode_Switch_Vehicle.can_id = CAN_EFF_FLAG | 0x40B;
    memset(Mode_Switch_Vehicle.data, 0, 8);

    Mode_Switch_Moldboard.can_dlc = 8;
    Mode_Switch_Moldboard.can_id = CAN_EFF_FLAG | 0x40C;
    memset(Mode_Switch_Moldboard.data, 0, 8);

    Mode_Switch_Subscriber = nodeHandle_.subscribe("/Mode_Switch", 1,
        &ros_to_can::Mode_Switch_Callback, this);
    mode_switch_timer_ = nodeHandle_.createTimer(
        ros::Duration(0.05),  // 20Hz, 和原 sys_control_node 一致
        &ros_to_can::modeSwitchTimerCallback, this);
    ROS_INFO("[ros_to_can] Mode_Switch merged from sys_control (20Hz)");

}

/*!
 * 回调函数
 */

void ros_to_can::U_Lever1_Callback(const std_msgs::Int16MultiArray & lever1)
{
      int16_t bit_from_0 = (lever1.data[0] & 0x01) << 2; // 取 lever1.data[0] 的第0位并移到目标位置（第2位） 1时左转
      int16_t bit_from_1 = (lever1.data[1] & 0x01) << 3; // 取 lever1.data[1] 的第0位并移到目标位置（第3位） 1时右转
      int16_t bit_from_3 = (lever1.data[3] & 0x01) << 4; // 取 lever1.data[3] 的第0位并移到目标位置（第4位） 挡位加
      int16_t bit_from_4 = (lever1.data[4] & 0x01) << 5; // 取 lever1.data[4] 的第0位并移到目标位置（第5位） 挡位减
      // 将四个位组合起来，并保留低8位
      Unmanned_Bulldozer_Operation.data[1] = static_cast<uint8_t>((bit_from_0 | bit_from_1 | bit_from_3 | bit_from_4) & 0xFF);//整车转向

      Unmanned_Bulldozer_Operation.data[3] = lever1.data[2] & 0xFF;//转向控制值，0-255

      int16_t bit_from_5 = (lever1.data[5] & 0x03);      //0时空档信号；1时前进档位信号；2时后退档信号
      int16_t bit_from_6 = (lever1.data[6] & 0x01) << 2; //1时手柄解锁信号
      Unmanned_Bulldozer_Operation.data[5]  = static_cast<uint8_t>((bit_from_5 | bit_from_6) & 0xFF);

      safe_can_write(can_fd, Unmanned_Bulldozer_Operation);

}
/*--------------      铲刀控制    ---------------------*/
// 铲刀基准高程
void ros_to_can::U_Lever2_Callback(const std_msgs::Int16MultiArray & lever2)
{
      int16_t height_right_moldboard = abs(lever2.data[0]);
      int16_t height_left_moldboard = abs(lever2.data[1]);
      
      Bulldozer_Moldboard_Target.data[0] = height_right_moldboard & 0xff;
      Bulldozer_Moldboard_Target.data[1] = (height_right_moldboard >> 8) & 0xff;
      Bulldozer_Moldboard_Target.data[2] = 0;

      Bulldozer_Moldboard_Target.data[4] = height_left_moldboard & 0xff;
      Bulldozer_Moldboard_Target.data[5] = (height_left_moldboard >> 8) & 0xff;
      Bulldozer_Moldboard_Target.data[6] = 0;

      if(lever2.data[0] >= 0)
      {
           Bulldozer_Moldboard_Target.data[3] = 0; 
      }else{
            Bulldozer_Moldboard_Target.data[3] = 1; 
      }

      if(lever2.data[1] >= 0)
      {
           Bulldozer_Moldboard_Target.data[7] = 0; 
      }else{
            Bulldozer_Moldboard_Target.data[7] = 1; 
      }
      

      safe_can_write(can_fd, Bulldozer_Moldboard_Target);

}

// 铲刀实际高程
void ros_to_can::U_Lever5_Callback(const std_msgs::Int16MultiArray & lever5)
{
      int16_t height_right_actual = abs(lever5.data[0]);
      int16_t height_left_actual = abs(lever5.data[1]);
      
      Bulldozer_Moldboard_Actual.data[0] = height_right_actual & 0xff;
      Bulldozer_Moldboard_Actual.data[1] = (height_right_actual >> 8) & 0xff;
      Bulldozer_Moldboard_Actual.data[2] = 0;

      Bulldozer_Moldboard_Actual.data[4] = height_left_actual & 0xff;
      Bulldozer_Moldboard_Actual.data[5] = (height_left_actual >> 8) & 0xff;
      Bulldozer_Moldboard_Actual.data[6] = 0;

      if(lever5.data[0] >= 0)
      {
           Bulldozer_Moldboard_Actual.data[3] = 0; 
      }else{
            Bulldozer_Moldboard_Actual.data[3] = 1; 
      }

      if(lever5.data[1] >= 0)
      {
           Bulldozer_Moldboard_Actual.data[7] = 0; 
      }else{
            Bulldozer_Moldboard_Actual.data[7] = 1; 
      }
      
      safe_can_write(can_fd, Bulldozer_Moldboard_Actual);

}

// 右铲刀高程差
void ros_to_can::U_Lever6_Callback(const std_msgs::Int16& lever6)
{
      int16_t Right_Dvalue;
      if (lever6.data < 0){
            Right_Dvalue = 65536 + lever6.data; 
      }else{
            Right_Dvalue = lever6.data;
      }
      memset(Bulldozer_Moldboard_Right_Dvalue.data, 0, 8);
      Bulldozer_Moldboard_Right_Dvalue.data[0] = Right_Dvalue & 0xff;
      Bulldozer_Moldboard_Right_Dvalue.data[1] = (Right_Dvalue >> 8) & 0xff;

      safe_can_write(can_fd, Bulldozer_Moldboard_Right_Dvalue);

}

// 左铲刀高程差
void ros_to_can::U_Lever7_Callback(const std_msgs::Int16& lever7)
{
      int16_t Left_Dvalue;
      if (lever7.data < 0){
            Left_Dvalue = 65536 + lever7.data; 
      }else{
            Left_Dvalue = lever7.data; 
      }
      memset(Bulldozer_Moldboard_Left_Dvalue.data, 0, 8);
      Bulldozer_Moldboard_Left_Dvalue.data[0] = Left_Dvalue & 0xff;
      Bulldozer_Moldboard_Left_Dvalue.data[1] = (Left_Dvalue >> 8) & 0xff;

      safe_can_write(can_fd, Bulldozer_Moldboard_Left_Dvalue);

}

// 3D找平人自动切换
void ros_to_can::U_Lever8_Callback(const std_msgs::Int16& lever8)
{
      memset(Moldboard_Control_Flag.data, 0, 8);
      
      Moldboard_Control_Flag.data[0] = (lever8.data & 0x01) & 0xFF;

      safe_can_write(can_fd, Moldboard_Control_Flag);

}


// 刹车
void ros_to_can::U_Lever3_Callback(const std_msgs::Int32& level3)
{

      U_Brake.data[0] = static_cast<uint8_t>(level3.data);//刹车
  
      safe_can_write(can_fd, U_Brake);

}

// 发动机转速
void ros_to_can::U_Lever4_Callback(const std_msgs::Int32& level4)
{       
      U_Engine_Speed.data[0] = 0x11;
      U_Engine_Speed.data[1] = (level4.data * 8) & 0xff;
      U_Engine_Speed.data[2] = ((level4.data * 8) >> 8) & 0xff;
      U_Engine_Speed.data[3] = 0xff;
      U_Engine_Speed.data[4] = 0xff;
      U_Engine_Speed.data[5] = 0xff;
      U_Engine_Speed.data[6] = 0xff;
      U_Engine_Speed.data[7] = 0xff;
      safe_can_write(can_fd, U_Engine_Speed);

      U_Engine_Speed_2.data[0] = 0x11;
      U_Engine_Speed_2.data[1] = (level4.data * 8) & 0xff;
      U_Engine_Speed_2.data[2] = ((level4.data * 8) >> 8) & 0xff;
      U_Engine_Speed_2.data[3] = 0xff;
      U_Engine_Speed_2.data[4] = 0xff;
      U_Engine_Speed_2.data[5] = 0xff;
      U_Engine_Speed_2.data[6] = 0xff;
      U_Engine_Speed_2.data[7] = 0xff;
      safe_can_write(can_fd, U_Engine_Speed_2);
      
}


// void ros_to_can::U_Lever9_Callback(const std_msgs::Int8& lever9)
// {
      
//       Honk_Flag.data[0] = (lever9.data & 0x01) & 0xFF;

//       write(can_fd, &Honk_Flag, sizeof(Honk_Flag)); 

// }


void ros_to_can::U_Lever_Moldboard_Callback(const std_msgs::Int16MultiArray& lever_Moldboard)
{
      // 安全保护: 所有通道除以2, 确保不超过500 (超500会烧电磁阀)
      int16_t ch0 = lever_Moldboard.data[0] / 2;
      int16_t ch1 = lever_Moldboard.data[1] / 2;
      int16_t ch2 = lever_Moldboard.data[2] / 2;
      int16_t ch3 = lever_Moldboard.data[3] / 2;

      U_Lever_Moldboard.data[0] = ch0 & 0xff;
      U_Lever_Moldboard.data[1] = (ch0 >> 8) & 0xff;

      U_Lever_Moldboard.data[2] = ch1 & 0xff;
      U_Lever_Moldboard.data[3] = (ch1 >> 8) & 0xff;
      
      U_Lever_Moldboard.data[4] = ch2 & 0xff;
      U_Lever_Moldboard.data[5] = (ch2 >> 8) & 0xff;

      U_Lever_Moldboard.data[6] = ch3 & 0xff;
      U_Lever_Moldboard.data[7] = (ch3 >> 8) & 0xff;

      safe_can_write(can_fd, U_Lever_Moldboard);

}

//==============================================================================
// [合并 sys_control] 模式切换
//==============================================================================

void ros_to_can::Mode_Switch_Callback(const std_msgs::Int32& mode)
{
    mode_switch_flag_ = (mode.data == 1) ? 1 : 0;
}

void ros_to_can::modeSwitchTimerCallback(const ros::TimerEvent& /*event*/)
{
    uint8_t val = (mode_switch_flag_ == 1) ? 0x01 : 0x00;
    Mode_Switch_Vehicle.data[0] = val;
    Mode_Switch_Moldboard.data[0] = val;
    safe_can_write(can_fd, Mode_Switch_Vehicle);
    safe_can_write(can_fd, Mode_Switch_Moldboard);
}