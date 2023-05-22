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
#include <cmath> //using isinf(), isnan()
#include <forklift_msg/meteorcar.h>
#define Sign(A) ((A) >= 0 ? 1 : -1)
using namespace std;

class SubscribeAndPublish
{
private:
    // publisher & subscriber declare
    string topic_odom, topic_imu, topic_forklift_pose, topic_cmd_vel, topic_cmd_fork;
    ros::Publisher *pub_odom, *pub_imu, *pub_forklift;
    ros::Subscriber *sub_cmd_vel, *sub_cmd_fork;
    tf::TransformBroadcaster odom_tfbroadcaster;

    // forklift variable declare
    float wheel_base, wheel_angle, wheel_speed, fork_velocity, theta_bias;
    double timeout;
    bool use_imu_flag, odom_tf_flag, init_fork_flag;
    ros::Time last_time, current_time, last_cmdvelcb_time, last_cmdforkcb_time;
    ros::Rate *r;
    int rate;
    STM32 *stm32;

    // function declare
    void CmdVelCB(const geometry_msgs::Twist &msg);
    void CmdForkCB(const forklift_msg::meteorcar &msg);
    void PublishOdom();
    void PublishImu();
    void PublishForklift();

public:
    SubscribeAndPublish(ros::NodeHandle *, ros::NodeHandle *, STM32 &); // SubscribeAndPublish建構子，當成main function
    ~SubscribeAndPublish();                                             // SubscribeAndPublish destructor 釋放new出來的記憶體
};

SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle *nh, ros::NodeHandle *priv_nh, STM32 &stm32_) : stm32(&stm32_)
{
    // get variable
    priv_nh->param<int>("rate", rate, 20);
    priv_nh->param<double>("timeout", timeout, 1.0);
    priv_nh->param<float>("wheel_base", wheel_base, 0.3);
    priv_nh->param<float>("theta_bias", theta_bias, 0.0);
    priv_nh->param<bool>("use_imu_flag", use_imu_flag, false);
    priv_nh->param<bool>("odom_tf_flag", odom_tf_flag, false);
    priv_nh->param<bool>("init_fork_flag", init_fork_flag, false);

    priv_nh->param<string>("topic_cmd_vel", topic_cmd_vel, "/cmd_vel");
    priv_nh->param<string>("topic_cmd_fork", topic_cmd_fork, "/cmd_fork");

    priv_nh->param<string>("topic_odom", topic_odom, "/wheel_odom");
    priv_nh->param<string>("topic_forklift_pose", topic_forklift_pose, "/forklift_pose");
    priv_nh->param<string>("topic_imu", topic_imu, "/imu");

    // definition publisher & subscriber
    pub_odom = new ros::Publisher(nh->advertise<nav_msgs::Odometry>(topic_odom, 10));
    pub_imu = new ros::Publisher(nh->advertise<sensor_msgs::Imu>(topic_imu, 10));
    pub_forklift = new ros::Publisher(nh->advertise<forklift_msg::meteorcar>(topic_forklift_pose, 10));
    sub_cmd_vel = new ros::Subscriber(nh->subscribe(topic_cmd_vel, 1, &SubscribeAndPublish::CmdVelCB, this));
    sub_cmd_fork = new ros::Subscriber(nh->subscribe(topic_cmd_fork, 1, &SubscribeAndPublish::CmdForkCB, this));
    
    // Initialize variable
    wheel_angle = wheel_speed = fork_velocity = 0.0f; // :TODO float wheel_speed(0.0)會出現錯誤
    r = new ros::Rate(rate);
    last_time = current_time = last_cmdvelcb_time = last_cmdforkcb_time = ros::Time::now();

    //initialize fork
    
    while (ros::ok() && init_fork_flag && stm32->Data14 < 1.0f)
    {
        stm32->read_data();
        stm32->send_data(1, 0, 0, 0, 2500, 0, 0, 0, 0, 0, 0, 0); // 电机(启动/停止)(1/0)，前轮速度 m/s，0，前轮转角 °/s . 起重电机PWM，范围-3600 ~ +3600 （PWM值）
        r->sleep();
    }
    (init_fork_flag) 
    ? printf("\x1B[0;32m""Forklift fork initialization complete. Forklift is now ready to be activated.\n""\x1B[0m")
    : printf("\x1B[0;32m""Forklift is already active.\n""\x1B[0m");

    // main loop
    while (ros::ok())
    {
        current_time = ros::Time::now();
        ros::spinOnce();
        if ((current_time - last_cmdvelcb_time).toSec() > timeout)
            wheel_speed = 0.0f;
        if ((current_time - last_cmdforkcb_time).toSec() > timeout)
            fork_velocity = 0.0f;

        stm32->read_data();
        PublishOdom();
        PublishImu();
        PublishForklift();
        stm32->send_data(1, wheel_speed, 0, wheel_angle + theta_bias, fork_velocity, 0, 0, 0, 0, 0, 0, 0); // 电机(启动/停止)(1/0)，前轮速度 m/s，0，前轮转角 °/s . 起重电机PWM，范围-3600 ~ +3600 （PWM值）
        last_time = current_time;
        r->sleep();
    }
};

SubscribeAndPublish::~SubscribeAndPublish()
{
    delete pub_odom, pub_imu, pub_forklift, sub_cmd_vel, sub_cmd_fork, r, stm32;
    ROS_WARN("forklift close");
};

void SubscribeAndPublish::CmdVelCB(const geometry_msgs::Twist &msg) // 參考cmd_vel_to_ackermann_drive.py
{
    static float r; // r = 旋轉半徑
    last_cmdvelcb_time = ros::Time::now();
    wheel_speed = msg.linear.x;                  // 速度v = 圓周運動速度
    if (isnan(r = msg.linear.x / msg.angular.z)) // 判斷旋轉半徑是否為無限大(直走)
        r = INFINITY;
    else if (abs(r) < wheel_base) // 判斷旋轉半徑是否小於wheel_base(自轉)
    {
        wheel_speed = msg.angular.z * wheel_base;
        r = 0.0;
    }
    else
        r = wheel_speed / msg.angular.z; // 旋轉半徑r = 速度v / 角速度w
    wheel_angle = atan(wheel_base / r);  // theta = arctan(前輪到兩後輪中心軸距wheel_base / 旋轉半徑r)
    wheel_angle *= 180 / M_PI;           // 轉換為角度
};

void SubscribeAndPublish::CmdForkCB(const forklift_msg::meteorcar &msg)// pwm range -3600 ~ +3600
{
    last_cmdforkcb_time = current_time;
    fork_velocity = -msg.fork_velocity; //fork_velocity上升為負，下降為正，因為stm32的起重電機PWM是負值上升，正值下降
    // 上限3600，下限1000
    if(abs(fork_velocity) > 3600)
        fork_velocity = 3600 * Sign(fork_velocity);
    else if (abs(fork_velocity) < 1000 && abs(fork_velocity) > 1)
        fork_velocity = 1000 * Sign(fork_velocity);
    else if (abs(fork_velocity) < 1)
        fork_velocity = 0;
};

void SubscribeAndPublish::PublishOdom()
{
    static nav_msgs::Odometry odom;
    static geometry_msgs::Quaternion th_quat;
    static geometry_msgs::TransformStamped odom_trans;
    static tf::TransformBroadcaster odom_broadcaster;
    static float x, y, th, linear_x, angular_z, delta_th, delta_x, delta_y, dt;

    linear_x = stm32->Data2 * cos(stm32->Data3 * M_PI / 180);
    (use_imu_flag) ? angular_z = stm32->angular_velocity_z :                     // 使用imu計匴角速度
        angular_z = stm32->Data2 * sin(stm32->Data3 * M_PI / 180) / wheel_base; // 使用里程計計算角速度

    dt = (current_time - last_time).toSec();
    delta_th = angular_z * dt;
    delta_x = linear_x * cos(th + delta_th / 2);
    delta_y = linear_x * sin(th + delta_th / 2);

    x += delta_x;
    y += delta_y;
    th += delta_th;

    th_quat = tf::createQuaternionMsgFromYaw(th); // 歐拉腳轉換為四元數

    odom.header.stamp = current_time;
    odom.header.frame_id = topic_odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0f;
    odom.pose.pose.orientation = th_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = 0.0f;
    odom.twist.twist.angular.z = angular_z;
    odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3};
    odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
                             0, 1e-3, 0, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e3};
    pub_odom->publish(odom);

    if (odom_tf_flag)
    {
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = topic_odom;
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = th_quat;
        odom_broadcaster.sendTransform(odom_trans);
    }
};

void SubscribeAndPublish::PublishImu()
{
    static sensor_msgs::Imu imu_data;
    static float dt, delta_th, th;
    dt = (current_time - last_time).toSec();
    delta_th = stm32->angular_velocity_z * dt;
    th += delta_th;

    imu_data.header.stamp = current_time;
    imu_data.header.frame_id = "imu";
    imu_data.orientation = tf::createQuaternionMsgFromYaw(th);
    imu_data.linear_acceleration.x = stm32->accelerated_wheel_speed;
    imu_data.linear_acceleration.y = stm32->accelerated_speed_y;
    imu_data.linear_acceleration.z = stm32->accelerated_speed_z;

    imu_data.angular_velocity.x = stm32->angular_velocity_x;
    imu_data.angular_velocity.y = stm32->angular_velocity_y;
    imu_data.angular_velocity.z = stm32->angular_velocity_z;
    imu_data.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
    imu_data.linear_acceleration_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};

    pub_imu->publish(imu_data);
};

void SubscribeAndPublish::PublishForklift()
{
    static float dt;
    static forklift_msg::meteorcar forklift_msg;

    forklift_msg.wheel_velocity = stm32->Data2;
    forklift_msg.wheel_angle = stm32->Data3;

    dt = (current_time - last_time).toSec();
    forklift_msg.fork_velocity = stm32->Data13 / 30 / 60 * 2;
    forklift_msg.fork_position -= forklift_msg.fork_velocity * dt;

    (stm32->Data14 == true /*限位開關被壓住*/) ? forklift_msg.fork_position = 0.0 : forklift_msg.fork_position = forklift_msg.fork_position;

    pub_forklift->publish(forklift_msg);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forklift"); // 先初始化ROS node，在ROS Master上註冊node
    ros::NodeHandle nh, priv_nh("~");  // 接下來建立ROS node的handle，用來與ROS Master溝通
    STM32 stm32;                       // 再建立與stm32溝通的物件，建構式會初始化串口，解構式會關閉串口，
    SubscribeAndPublish SAP_object(&nh, &priv_nh, stm32); // 最後再初始ROS subccriber&publisher這些與其他node的接口，並使用call by address, reference將stm32與nh物件傳入
    return 0;
}

/*
STM32 物件的 public variable
Data1   // 电机启动停止控制位（1/0 启动/停止）
Data2   // 前轮线速度(m/s)
Data3   // 前轮转角(角度°)
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
