#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include <cmath>
#include <cfloat>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float32.h"
#include <map>
#include <string>
#include <stm32_controller.h>

using namespace std;

#define Sign(A) ((A) >= 0 ? 1 : -1)

serial::Serial sp; // Serial Port
typedef unsigned char byte;
double Data1 = 0, Data2, Data3, Data4, Data5, Data6, Data7, Data8, Data9, Data10, Data11, Data12, Data13, Data14, Data15; // recv data
//只簡單輸出前輪的行程和前輪的轉角,未涉及坐標轉換
float wheel_speed; //前驅三輪車前輪速度
float wheel_angle; //前驅三輪車前輪轉角
uint8_t Flag_start = 0;
float motor_Fork;
double wheel_base;

double vth_odom = 0;
double vth_imu;

double bias_x = 0.0, bias_y = 0.0, bias_z = 0.0;
double max_steer_ang;

float angular_velocity_x;
float angular_velocity_y;
float angular_velocity_z;
float accelerated_wheel_speed;
float accelerated_speed_y;
float accelerated_speed_z;

bool tf_publish;
bool fork_init;
bool odom_use_imu;

string state;
string odom_topic_name;
string odom_header_tf_name, odom_child_tf_name;
string fork_header_tf_name, fork_child_tf_name;
string odom_header_topic_name, odom_child_topic_name;

map<string, int> Trigger =
    {{"LowVoltageError", 0},
     {"OpenPowerSupplyError", 0},
     {"InitialFork", 0},
     {"InitialIMU", 0}};

/*ros topic*/
geometry_msgs::Twist cmd_vel;
sensor_msgs::Imu imu_data;
geometry_msgs::TransformStamped odom_trans, fork_trans;
geometry_msgs::Quaternion odom_quat, imu_quat;
std_msgs::Float32 fork_pos;

/**Subscriber**/
void cmd_vel_CB(const geometry_msgs::Twist &msg) //獲取鍵盤控制的回調函數
{
    cmd_vel = msg;

    wheel_angle = 0;
    wheel_speed = 0;
    if (cmd_vel.angular.y > 0)
    {
        motor_Fork = -2000;
    } //Control fork up
    else if (cmd_vel.angular.y < 0)
    {
        motor_Fork = +1700;
    } //Control fork down

    // normal move
    if (cmd_vel.linear.x == 0 && cmd_vel.angular.z != 0)
    {
        wheel_angle = 90;
        wheel_speed = (fabs(cmd_vel.angular.z) > 1.0 ? Sign(cmd_vel.angular.z) * 1.0 : cmd_vel.angular.z) * wheel_base;
    }
    else if (cmd_vel.linear.x != 0)
    {
        wheel_angle = tanh(wheel_base * cmd_vel.angular.z / cmd_vel.linear.x) * 180 / 3.1415;
        wheel_speed = cmd_vel.linear.x / cos(wheel_angle / 180 * 3.1415);
    }
    else
    {
        wheel_angle = wheel_speed = 0;
    }
    wheel_angle = fabs(wheel_angle) > 90 ? Sign(wheel_angle) * 90 : wheel_angle;
    wheel_speed = fabs(wheel_speed) > 0.8 ? Sign(wheel_speed) * 0.8 : wheel_speed;

    // stop
    if (cmd_vel.linear.x || cmd_vel.angular.z || cmd_vel.angular.y)
    {
        Flag_start = 1;
    }
    else
    {
        Flag_start = 0;
        wheel_speed = wheel_angle = motor_Fork = 0.0;
    }
}
/**Subscriber end**/

int main(int argc, char **argv)
{

    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh_priv("~");

    // Subscriber
    ros::Subscriber sub = n.subscribe("cmd_vel", 200, cmd_vel_CB); //Subcribe teleop_twist_keyboard

    // Publisher
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("wheel_odom", 50);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 20);
    ros::Publisher fork_position = n.advertise<std_msgs::Float32>("fork_position", 20);

    // Transform TF
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster fork_tf;

    // Parameter
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    double th_imu = 0.0;
    double delta_angle = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;
    double V_fork = 0.0;
    double last_ang = 0.0;

    double Ka_odom;
    fork_pos.data = 0.0;

    // ros parameter
    nh_priv.param<bool>("odom_use_imu", odom_use_imu, "true");
    nh_priv.param<bool>("fork_init", fork_init, "true");
    nh_priv.param<bool>("tf_publish", tf_publish, "true");
    nh_priv.param<double>("max_steer_ang", max_steer_ang, 30);
    nh_priv.param<double>("Ka_odom", Ka_odom, 1);
    nh_priv.param<double>("wheel_base", wheel_base, 0.3);

    // nh_priv.getParam("odom_use_imu", odom_use_imu);
    // nh_priv.getParam("fork_init", fork_init);
    // nh_priv.getParam("tf_publish", tf_publish);
    // nh_priv.getParam("max_steer_ang", max_steer_ang);
    // nh_priv.getParam("Ka_odom", Ka_odom);
    // nh_priv.getParam("wheel_base", wheel_base);

    // nh_priv.param<string>("odom_topic_name", odom_topic_name);
    // nh_priv.param<string>("odom_header_tf_name", odom_header_tf_name);
    // nh_priv.param<string>("odom_child_tf_name", odom_child_tf_name);

    // nh_priv.param<string>("fork_header_tf_name", fork_header_tf_name);
    // nh_priv.param<string>("fork_child_tf_name", fork_child_tf_name);

    // nh_priv.param<string>("odom_header_topic_name", odom_header_topic_name);
    // nh_priv.param<string>("odom_child_topic_name", odom_child_topic_name);

    nh_priv.getParam("odom_topic_name", odom_topic_name);
    nh_priv.getParam("odom_header_tf_name", odom_header_tf_name);
    nh_priv.getParam("odom_child_tf_name", odom_child_tf_name);

    nh_priv.getParam("fork_header_tf_name", fork_header_tf_name);
    nh_priv.getParam("fork_child_tf_name", fork_child_tf_name);

    nh_priv.getParam("odom_header_topic_name", odom_header_topic_name);
    nh_priv.getParam("odom_child_topic_name", odom_child_topic_name);

    max_steer_ang = fabs(max_steer_ang);
    printf("ros parameter \n");

    printf("Ka_odom         =%f\n", Ka_odom);
    printf("wheel_base      =%f\n", wheel_base);
    printf("odom_use_imu    =%d\n", odom_use_imu);
    printf("fork_init       =%d\n", fork_init);
    printf("tf_publish      =%d\n", tf_publish);
    printf("*****************\n\n");

    printf("odom_topic_name         = %s\n", odom_topic_name.c_str());
    printf("odom_header_tf_name     = %s\n", odom_header_tf_name.c_str());
    printf("odom_child_tf_name      = %s\n", odom_child_tf_name.c_str());

    printf("odom_header_topic_name  = %s\n", odom_header_topic_name.c_str());
    printf("odom_child_topic_name   = %s\n\n", odom_child_topic_name.c_str());

    printf("fork_header_tf_name     = %s\n", fork_header_tf_name.c_str());
    printf("fork_child_tf_name      = %s\n\n", fork_child_tf_name.c_str());

    printf("*****************\n\n");

    ros::Time current_time, last_time;
    current_time = last_time = ros::Time::now();
    ros::Rate loop_rate(30); // Set the loop rate, code execution frequency is 100 Hz

    /***********Serial Port*******************/

    serial::Timeout to = serial::Timeout::simpleTimeout(100); //Create timeout

    sp.setPort("/dev/ttyUSB0"); //Set the name of the serial port to open
    sp.setBaudrate(115200);     //Set the baud rate of serial communication
    sp.setTimeout(to);          //Serial set timeout

    try // open serial port
    {
        sp.open(); //打開串口
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if (sp.isOpen()) //判斷串口是否打開成功
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    /***********serial port end****************/

    for (uint8_t j = 0; j < 3; j++) // clear buff
        send_data(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    while (ros::ok()) // pub odom && cmd
    {

        current_time = ros::Time::now(); // 記錄當前時間
        double dt = (current_time - last_time).toSec();
        last_ang = Data3;

        if (state == "Motor Calibration Complete")
        {
            accelerated_wheel_speed *= bias_x;
            accelerated_speed_y *= bias_y;
            accelerated_speed_z *= bias_z;
            V_fork = Data13 / 30 / 60 * 2;
            fork_pos.data -= V_fork * dt;
            if (Data14 == 1)
            {
                fork_pos.data = 0;
            }
            fork_position.publish(fork_pos);
        }
        fork_trans.header.stamp = current_time;

        fork_trans.header.frame_id = fork_header_tf_name;
        fork_trans.child_frame_id = fork_child_tf_name;
        fork_trans.transform.translation.x = -0.1;
        fork_trans.transform.translation.y = 0;
        fork_trans.transform.translation.z = fork_pos.data;
        fork_trans.transform.rotation = tf::createQuaternionMsgFromYaw(3.14);
        fork_tf.sendTransform(fork_trans); // Send Transformation

        // if(Data3 == 0)
        //     vx =Data2;
        // else
        vx = Data2 * cos(Data3 * 3.1415 / 180);
        vy = 0;                         //三輪前驅無Y軸速度
        vth_imu = angular_velocity_z;   //設備當前Z軸角速度 rad/s
        if (fabs(Data2) > 1e-6)
            vth_odom = Data2 * sin(Data3 * 3.1415 / 180) / wheel_base;
        else
            vth_odom = 0.0;

        // ROS_WARN("vth_odom %f",vth_odom);
        // ROS_WARN("vth_imu %f",vth);
        if (!odom_use_imu)
        {
            vth = vth_odom * Ka_odom;
        }
        else
        {
            vth = vth_imu;
        }

        //以給定機器人速度的典型方式計算里程計
        double delta_th = vth * dt;
        double delta_x = (vx * cos(th + delta_th / 2) - vy * sin(th + delta_th / 2)) * dt;
        double delta_y = (vx * sin(th + delta_th / 2) + vy * cos(th + delta_th / 2)) * dt;

        x += delta_x;   // X軸速度累積位移 m
        y += delta_y;   // Y軸速度累積位移 m
        th += delta_th; // Z軸角速度累積求車體朝向角度  rad //存在漂移

        th_imu += vth_imu * dt;
        /***************************odom tf**************************/
        //因為所有的里程表都是6自由度的，所以我們需要一個由偏航創建的四元數
        odom_quat = tf::createQuaternionMsgFromYaw(th);
        imu_quat = tf::createQuaternionMsgFromYaw(th_imu);
        odom_trans.header.stamp = current_time;

        odom_trans.header.frame_id = odom_header_tf_name;
        odom_trans.child_frame_id = odom_child_tf_name;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        // Send Transformation
        if (tf_publish)
            odom_broadcaster.sendTransform(odom_trans);

        // Published Odom Msgs by ROS
        /***************************odom topic**************************/

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_header_topic_name;
        // Set Position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat;
        // Set Speed
        odom.child_frame_id = odom_child_topic_name;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
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
        // Publish Msgs
        odom_pub.publish(odom);

        /***************************imu topic**************************/
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "imu";

        imu_data.orientation = imu_quat;
        imu_data.linear_acceleration.x = accelerated_wheel_speed; // Data7;
        imu_data.linear_acceleration.y = accelerated_speed_y;     // Data8;
        imu_data.linear_acceleration.z = accelerated_speed_z;     // Data9;

        imu_data.angular_velocity.x = angular_velocity_x; // Data4;
        imu_data.angular_velocity.y = angular_velocity_y; // Data5;
        imu_data.angular_velocity.z = angular_velocity_z; // Data6;
        imu_data.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
        imu_data.linear_acceleration_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};

        imu_pub.publish(imu_data);

        last_time = current_time; //保存为上次时间
        /**publish end**/

        ros::spinOnce(); //执行回调处理函数，完后继续往下执行
        // std::cout<<"send data motor_Fork = "<<motor_Fork<<std::endl;
        if (state == "Motor Calibration Complete")
        {

            if (fabs(Data3 - wheel_angle) > max_steer_ang && wheel_speed != 0.0)
            {
                send_data(Flag_start, 0, 0, wheel_angle - delta_angle, motor_Fork, 0, 0, 0, 0, 0, 0, 0); //电机(启动/停止)(1/0)，前轮速度 m/s，0，前轮转角 °/s . 起重电机PWM，范围-3600 ~ +3600 （PWM值）
            }
            else
            {
                send_data(Flag_start, wheel_speed, 0, wheel_angle - delta_angle, motor_Fork, 0, 0, 0, 0, 0, 0, 0); //电机(启动/停止)(1/0)，前轮速度 m/s，0，前轮转角 °/s . 起重电机PWM，范围-3600 ~ +3600 （PWM值）
            }
        }

        size_t n = sp.available(); //获取缓冲区内的字节数
        if (n > 0)
        {

            uint8_t buffer[65];
            sp.read(buffer, n); //读出数据

            if (buffer[0] == 0XAA && buffer[1] == 0XAA && buffer[2] == 0XF1)
            {
                uint8_t sum;
                for (uint8_t j = 0; j < 64; j++)
                    sum += buffer[j]; //计算校验和
                if (sum == buffer[64])
                {
                    Data1 = b2f(buffer[4], buffer[5], buffer[6], buffer[7]);      //电机启动停止控制位（1/0 启动/停止）
                    Data2 = b2f(buffer[8], buffer[9], buffer[10], buffer[11]);    //前轮线速度
                    Data3 = b2f(buffer[12], buffer[13], buffer[14], buffer[15]);  //前轮转角
                    Data4 = b2f(buffer[16], buffer[17], buffer[18], buffer[19]);  //绕X轴角速度 gyro_Roll 原始数值
                    Data5 = b2f(buffer[20], buffer[21], buffer[22], buffer[23]);  //绕Y轴角速度 gyro_Pitch 原始数值
                    Data6 = b2f(buffer[24], buffer[25], buffer[26], buffer[27]);  //绕Z轴角速度 gyro_Yaw 原始数值
                    Data7 = b2f(buffer[28], buffer[29], buffer[30], buffer[31]);  // X轴加速度 accel_x 原始数值
                    Data8 = b2f(buffer[32], buffer[33], buffer[34], buffer[35]);  // Y轴加速度 accel_y 原始数值
                    Data9 = b2f(buffer[36], buffer[37], buffer[38], buffer[39]);  // Z轴加速度 accel_z 原始数值
                    Data10 = b2f(buffer[40], buffer[41], buffer[42], buffer[43]); // Yaw Z轴角度
                    Data11 = b2f(buffer[44], buffer[45], buffer[46], buffer[47]); //电池电压              24-25   <24.3  low
                    Data12 = b2f(buffer[48], buffer[49], buffer[50], buffer[51]); //红色紧急开关位0/1 运行/停止
                    Data13 = b2f(buffer[52], buffer[53], buffer[54], buffer[55]); //起重电机编码器原始数据（未转换） 如果有需要可以添加发送指令去清0，上面的发送命令还有剩余   gearrate 30  dt 5 ms
                    Data14 = b2f(buffer[56], buffer[57], buffer[58], buffer[59]); //起重电机下行限位开关（用于校准） 1代表开关被压住
                    Data15 = b2f(buffer[60], buffer[61], buffer[62], buffer[63]); //起重电机上行限位开关（用于校准） 1代表开关被压住
                    // printf("Data 1 = %g\n", Data1);
                    // printf("Data 2 = %g\n", Data2);
                    // printf("Data 3 = %g\n", Data3);
                    // printf("Data 4 = %g\n", Data4);
                    // printf("Data 5 = %g\n", Data5);
                    // printf("Data 6 = %g\n", Data6);
                    // printf("Data 7 = %g\n", Data7);
                    // printf("Data 8 = %g\n", Data8);
                    // printf("Data 9 = %g\n", Data9);
                    // printf("Data 10 = %g\n", Data10);
                    // printf("Data 11 = %g\n", Data11);
                    // printf("Data 12 = %g\n", Data12);
                    // printf("Data 13 = %g\n", Data13);
                    // printf("Data 14 = %g\n", Data14);
                    // printf("Data 15 = %g\n", Data15);
                }
                sum = 0;
                memset(buffer, 0, sizeof(uint8_t) * 65);
            }
            angular_velocity_x = Data4 * 0.001064;  //转换成 rad/s
            angular_velocity_y = Data5 * 0.001064;  //转换成 rad/s
            angular_velocity_z = Data6 * 0.001064;  //转换成 rad/s
            accelerated_wheel_speed = Data7 / 2048; //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
            accelerated_speed_y = Data8 / 2048;     //转换            // if(Data1)
            accelerated_speed_z = Data9 / 2048;     //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒

            if (Trigger["LowVoltageError"] == 0 && Data11 / 100 < 23.0 && Data11 != 0.0)
            {
                ROS_WARN("Low Voltage : %f V", Data11 / 100);
                Trigger["LowVoltageError"] = 1; // Trigger
            }

            if (Trigger["OpenPowerSupplyError"] == 0 && (Data11 / 100 < 5) && Data11 != 0)
            {
                ROS_ERROR("PLEASE OPEN \"POWER SUPPLY\" before CONNECT USB!!!");
                Trigger["OpenPowerSupplyError"] = 1; // Trigger
            }

            if (Trigger["InitialFork"] == 0 && fork_init == 1)
            {
                if (Data14 != 1)
                {
                    send_data(1, 0, 0, 0.00, 1500, 0, 0, 0, 0, 0, 0, 0);
                }
                else if (Data14 == 1)
                {
                    Trigger["InitialFork"] = 1; // Trigger
                }
            }

            if (Trigger["InitialFork"] == 1 && Trigger["InitialIMU"] == 0 && odom_use_imu == 1)
            {
                state = "Wait for Receiving IMU Data ";
                ROS_INFO("*****%s*****", state.c_str());
                for (int imu_data_receive = 0; imu_data_receive < 100; imu_data_receive++)
                {
                    bias_x += accelerated_wheel_speed;
                    bias_y += accelerated_speed_y;
                    bias_z += accelerated_speed_z;
                }

                state = "IMU Data Receiving Finish";
                ROS_INFO("*****%s*****", state.c_str());

                ROS_INFO("Bias x %f y %f z %f", bias_x, bias_y, bias_z);

                bias_x /= 100.0;
                bias_y /= 100.0;
                bias_z /= 100.0;

                bias_x = 1e-3 / bias_x;
                bias_y = 1e-3 / bias_y;
                bias_z = 9.8 / bias_z;

                ROS_INFO("Bias x %f y %f z %f (Calibrated)", bias_x, bias_y, bias_z);

                if (bias_x != 0 || bias_y != 0 || bias_z != 0)
                {
                    Trigger["InitialIMU"] = 1;
                }

                state = "Motor Calibration Complete";
                ROS_INFO("*****%s*****", state.c_str());
                ros::Duration(1).sleep();
            }
        }
        loop_rate.sleep(); //循环延时时间
    }

    for (uint8_t j = 0; j < 3; j++) // clear buff
        send_data(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    //关闭串口
    sp.close();

    return 0;
}
