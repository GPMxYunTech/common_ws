#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float32.h"
#include <stm32.h>
#include <string>
#include <cmath> //using isinf()
#define Sign(A) ((A) >= 0 ? 1 : -1)
using namespace std;
class SubscribeAndPublish
{
private:
    // publisher & subscriber declare
    string TopicOdom, TopicImu, TopicForkPose, TopicCmdVel;
    ros::Publisher PubOdom;
    ros::Publisher PubImu;
    ros::Publisher PubForkPose;
    ros::Subscriber SubCmdVel;
    tf::TransformBroadcaster OdomtfBroadcaster;
    void CmdVelCB(const geometry_msgs::Twist &msg);
    // variable declare
    float wheel_base, wheel_angle, wheel_speed, motor_fork;

    void GetAndInitParam()
    {
        float wheel_base(0.0), wheel_angle(0.0), wheel_speed(0.0), motor_fork(0.0);
        ros::param::get("/TopicOdom", TopicOdom);
        ros::param::get("/TopicImu", TopicImu);
        ros::param::get("/TopicForkPose", TopicForkPose);
        ros::param::get("/TopicCmdVel", TopicCmdVel);
        ros::param::get("/wheel_base", wheel_base);
    };
public:
    SubscribeAndPublish(ros::NodeHandle &nh, stm32 &stm32); // 建構子constructor
    ~SubscribeAndPublish();                                // 解構子destructor 當程式結束時停止機器人
};

SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle &nh, stm32 &stm32)
{
    GetAndInitParam();
    PubOdom = nh.advertise<nav_msgs::Odometry>(TopicOdom, 10);
    PubImu = nh.advertise<sensor_msgs::Imu>(TopicImu, 10);
    PubForkPose = nh.advertise<std_msgs::Float32>(TopicForkPose, 10);
    SubCmdVel = nh.subscribe(TopicCmdVel, 1, &SubscribeAndPublish::CmdVelCB, this);

    while (ros::ok())
    {
        ros::spinOnce();
        stm32.read_data();
        stm32.send_data(1, wheel_speed, 0, wheel_angle, motor_fork, 0, 0, 0, 0, 0, 0, 0); // 电机(启动/停止)(1/0)，前轮速度 m/s，0，前轮转角 °/s . 起重电机PWM，范围-3600 ~ +3600 （PWM值）
        ros::Duration(0.04).sleep();
    }
};
SubscribeAndPublish::~SubscribeAndPublish()
{
    ROS_WARN("forklift close");
};
void SubscribeAndPublish::CmdVelCB(const geometry_msgs::Twist &msg) // 參考cmd_vel_to_ackermann_drive.py
{
    static float  r;// r = 旋轉半徑
    wheel_speed = msg.linear.x;
    //判斷旋轉半徑是否為無限大(直走)或小於wheel_base，如果是則r = wheel_base，否則r = wheel_speed / msg.angular.z
    (isnan(r = msg.linear.x / msg.angular.z) || r < wheel_base) ? r = wheel_base : r = wheel_speed / msg.angular.z;
    wheel_angle = atan(wheel_base / r);
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "forklift");        // 先初始化ROS node，在ROS Master上註冊node
    ros::NodeHandle nh;                       // 接下來建立ROS node的handle，用來與ROS Master溝通
    stm32 stm32;                              // 再建立與stm32溝通的物件，建構式會初始化串口，解構式會關閉串口，
    SubscribeAndPublish SAPObject(nh, stm32); // 最後再初始ROS subccriber&publisher這些與其他node的接口，並使用call by reference將stm32與nh物件傳入
    return 0;
}
/*
stm32 物件的 public variable
Data1   // 电机启动停止控制位（1/0 启动/停止）
Data2   // 前轮线速度
Data3   // 前轮转角
Data4   // 绕X轴角速度 gyro_Roll 原始数值
Data5   // 绕Y轴角速度 gyro_Pitch 原始数值
Data6   // 绕Z轴角速度 gyro_Yaw 原始数值
Data7   // X轴加速度 accel_x 原始数值
Data8   // Y轴加速度 accel_y 原始数值
Data9   // Z轴加速度 accel_z 原始数值
Data10  // Yaw Z轴角度
Data11  // 电池电压              24-25   <24.3  low
Data12  // 红色紧急开关位0/1 运行/停止
Data13  // 起重电机编码器原始数据（未转换） 如果有需要可以添加发送指令去清0，上面的发送命令还有剩余   gearrate 30  dt 5 ms
Data14  // 起重电机下行限位开关（用于校准） 1代表开关被压住
Data15  // 起重电机上行限位开关（用于校准） 1代表开关被压住

angular_velocity_x = Data4 * 0.001064;  //转换成 rad/s
angular_velocity_y = Data5 * 0.001064;  //转换成 rad/s
angular_velocity_z = Data6 * 0.001064;  //转换成 rad/s
accelerated_wheel_speed = Data7 / 2048; //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
accelerated_speed_y = Data8 / 2048;     //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
accelerated_speed_z = Data9 / 2048;     //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
*/
