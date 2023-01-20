#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float32.h"
#include <string>
#include <stm32_controller.h>

using namespace std;
#define Sign(A) ((A) >= 0 ? 1 : -1)
void init_stat();
serial::Serial sp; // serial port
typedef unsigned char byte;
double Data1 = 0, Data2, Data3, Data4, Data5, Data6, Data7, Data8, Data9, Data10, Data11, Data12, Data13, Data14, Data15; // recv data
//只简单输出前轮的行程和前轮的转角,未涉及坐标转换
float wheel_speed; //前驱三轮车前轮速度
float wheel_angle; //前驱三轮车前轮转角
uint8_t Flag_start = 0;
float motor_Fork;
double wheel_base;
// int calibration_flag = 0;
double vth_odom = 0;
double vth_imu;
bool odom_use_imu;
bool tf_publish;
double bias_x = 0.0, bias_y = 0.0, bias_z = 0.0;
double max_steer_ang;
float angular_velocity_x;
float angular_velocity_y;
float angular_velocity_z;
float accelerated_wheel_speed;
float accelerated_speed_y;
float accelerated_speed_z;
bool fork_init;

string state;
string odom_topic_name;
string odom_header_tf_name, odom_child_tf_name;
string fork_header_tf_name, fork_child_tf_name;
string odom_header_topic_name, odom_child_topic_name;
/*ros topic*/
geometry_msgs::Twist cmd_vel;
sensor_msgs::Imu imu_data;
geometry_msgs::TransformStamped odom_trans, fork_trans;
geometry_msgs::Quaternion odom_quat, imu_quat;
std_msgs::Float32 fork_pos;

/**Subscriber**/
void cmd_vel_CB(const geometry_msgs::Twist &msg) //获取键盘控制的回调函数
{
    cmd_vel = msg;

    wheel_angle = 0;
    wheel_speed = 0;
    if (cmd_vel.angular.y > 0)
    {
        motor_Fork = -2000;
    } //控制起落架上升
    else if (cmd_vel.angular.y < 0)
    {
        motor_Fork = +1700;
    } //控制起落架下降
    /*
    else if(cmd_vel.angular.y==0)
    {
        std::cout<<"send data motor_Fork = "<<motor_Fork<<std::endl;motor_Fork=0;
    }
    */

    // normal move
    if (cmd_vel.linear.x == 0 && cmd_vel.angular.z != 0)
    {
        // wheel_angle  =  Sign(msg.angular.z) *90;
        // wheel_speed  =  fabs(msg.angular.z);
        wheel_angle = 90;
        wheel_speed = (fabs(cmd_vel.angular.z) > 1.0 ? Sign(cmd_vel.angular.z) * 1.0 : cmd_vel.angular.z) * wheel_base;
        // ROS_INFO("vel %F",wheel_speed);
    }
    else if (cmd_vel.linear.x != 0)
    {
        /*
        if(fabs(cmd_vel.linear.x)>1.0) //等比減速
        {
            cmd_vel.angular.z /= fabs(cmd_vel.linear.x);
            cmd_vel.linear.x =Sign(cmd_vel.linear.x)*1.0;
        }*/
        wheel_angle = tanh(wheel_base * cmd_vel.angular.z / cmd_vel.linear.x) * 180 / 3.1415;
        wheel_speed = cmd_vel.linear.x / cos(wheel_angle / 180 * 3.1415);
        // printf("%f  %f  %f\n",wheel_speed,cmd_vel.linear.x,cos(wheel_angle));
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
    // printf("Lx %f Ly %f Lz %f Ax %f Ay %f Az %f \n",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z,cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);
    // printf("wheel_speed  %f wheel_angle %f  \n",wheel_speed,wheel_angle);
}
/**Subscriber end**/
void init_stat()
{

    // if (Data14 > 1.0)
    // {
    //     send_data(1, 0, 0, 0.00, 0, 0, 0, 0, 0, 0, 0, 0);
    // }
    // else
    // {
    //     send_data(1, 0, 0, 0.00, 1500, 0, 0, 0, 0, 0, 0, 0); // Motor Fork == 1500
    // }

    // send_data(1, 0, 0, 0.00, 1500, 0, 0, 0, 0, 0, 0, 0);
    // send_data(1, 0, 0, 0.00, 1500, 0, 0, 0, 0, 0, 0, 0);
    // send_data(1, 0, 0, 0.00, 1500, 0, 0, 0, 0, 0, 0, 0);
    // send_data(1, 0, 0, 0.00, 1500, 0, 0, 0, 0, 0, 0, 0);
    // send_data(1, 0, 0, 0.00, 1500, 0, 0, 0, 0, 0, 0, 0);
    // ros::Duration(5).sleep();
    if (fork_init)
    {
        while (fork_pos.data == 1.0)
        {
            send_data(1, 0, 0, 0.00, 1500, 0, 0, 0, 0, 0, 0, 0); // Motor Fork == 1500
            printf("Data14 = %f\n", Data14);
            printf("fork_pos.data = %f\n", fork_pos.data);
            if (fork_pos.data == 0.0)
            {
                break;
            }
        }
    }
    // for (int i = 0; i < 100; i++)
    // {
    //     printf("Data 1 = %g\n", Data1);
    //     printf("Data 2 = %g\n", Data2);
    //     printf("Data 3 = %g\n", Data3);
    //     printf("Data 4 = %g\n", Data4);
    //     printf("Data 5 = %g\n", Data5);
    //     printf("Data 6 = %g\n", Data6);
    //     printf("Data 7 = %g\n", Data7);
    //     printf("Data 8 = %g\n", Data8);
    //     printf("Data 9 = %g\n", Data9);
    //     printf("Data 10 = %g\n", Data10);
    //     printf("Data 11 = %g\n", Data11);
    //     printf("Data 12 = %g\n", Data12);
    //     printf("Data 13 = %g\n", Data13);
    //     printf("Data 14 = %g\n", Data14);
    //     printf("Data 15 = %g\n", Data15);
    //     ros::Duration(1).sleep();
    // }

    ros::Duration(1).sleep();

    // send_data(Flag_start, wheel_speed, 0, wheel_angle, motor_Fork, 0, 0, 0, 0, 0, 0, 0);
    // else if (calibration_flag == 100)
    // {
    //     if (Data14 >= 1.0 || fork_init == 0)
    //         calibration_flag = 200;
    //     else
    //         calibration_flag = 0;
    // }

    // if (calibration_flag >= 200 && calibration_flag < 300) // 100  imu data
    // {
    //     bias_x += accelerated_wheel_speed;
    //     bias_y += accelerated_speed_y;
    //     bias_z += accelerated_speed_z;
    //     calibration_flag++;
    // }

    state = "Wait for Receiving IMU Data ";
    ROS_INFO("*****%s*****", state.c_str());
    for (int imu_data_receive = 0; imu_data_receive < 100; imu_data_receive++)
    {
        bias_x += accelerated_wheel_speed;
        bias_y += accelerated_speed_y;
        bias_z += accelerated_speed_z;
    }
    ros::Duration(1).sleep();
    state = "IMU Data Receiving Finish";
    ROS_INFO("*****%s*****", state.c_str());

    ROS_INFO("Bias x %f y %f z %f  (Not Calibrated)", bias_x, bias_y, bias_z);

    bias_x /= 100.0;
    bias_y /= 100.0;
    bias_z /= 100.0;

    bias_x = 1e-3 / bias_x;
    bias_y = 1e-3 / bias_y;
    bias_z = 9.8 / bias_z;

    ROS_INFO("Bias x %f y %f z %f  (Calibrated)", bias_x, bias_y, bias_z);

    // calibration_flag = 999;
    // if (Data3 > 1.0)
    //     ROS_ERROR("Motor angle need to adjustment");
    // else
    // {
    //     ROS_INFO("wheel_ang %f ", Data3);
    // }
    state = "Motor Calibration Complete";
    ROS_INFO("*****%s*****", state.c_str());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh_priv("~");

    // subscriber
    ros::Subscriber sub = n.subscribe("cmd_vel", 200, cmd_vel_CB); //订阅键盘控制

    // publisher
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("wheel_odom", 50);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 20);
    ros::Publisher fork_position = n.advertise<std_msgs::Float32>("fork_position", 20);

    // transform TF
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster fork_tf;

    // parameter
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
    fork_pos.data = 1.0;

    // ros parameter
    nh_priv.param<bool>("odom_use_imu", odom_use_imu, 0);
    nh_priv.param<bool>("fork_init", fork_init, "false");
    nh_priv.param<bool>("tf_publish", tf_publish, "false");
    nh_priv.param<double>("max_steer_ang", max_steer_ang, 30);
    nh_priv.param<double>("Ka_odom", Ka_odom, 1);
    nh_priv.param<double>("wheel_base", wheel_base, 0.3);

    nh_priv.getParam("odom_topic_name", odom_topic_name);
    nh_priv.getParam("odom_header_tf_name", odom_header_tf_name);
    nh_priv.getParam("odom_child_tf_name", odom_child_tf_name);

    nh_priv.getParam("fork_header_tf_name", fork_header_tf_name);
    nh_priv.getParam("fork_child_tf_name", fork_child_tf_name);

    nh_priv.getParam("odom_header_topic_name", odom_header_topic_name);
    nh_priv.getParam("odom_child_topic_name", odom_child_topic_name);

    max_steer_ang = fabs(max_steer_ang);

    printf("ros parameter \n");

    printf("Ka_odom         = %f\n", Ka_odom);
    printf("wheel_base      = %f\n", wheel_base);
    printf("odom_use_imu    = %d\n", odom_use_imu);
    printf("tf_publish      = %d\n\n", tf_publish);

    // ROS_INFO("odom_topic_name       = %s\n",odom_topic_name);

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
    ros::Rate loop_rate(30); //设置循环间隔，即代码执行频率 100HZ

    /***********serial port*******************/

    serial::Timeout to = serial::Timeout::simpleTimeout(100); //创建timeout

    sp.setPort("/dev/ttyUSB0"); //设置要打开的串口名称
    sp.setBaudrate(115200);     //设置串口通信的波特率
    sp.setTimeout(to);          //串口设置timeout

    try // open serial port
    {
        sp.open(); //打开串口
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if (sp.isOpen()) //判断串口是否打开成功
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

    /**/
    // init_stat();
    /**/

    while (ros::ok()) // pub odom && cmd
    {

        current_time = ros::Time::now(); //记录当前时间
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
                fork_pos.data = 0;
            fork_position.publish(fork_pos);
        }

        fork_trans.header.stamp = current_time;

        fork_trans.header.frame_id = fork_header_tf_name;
        fork_trans.child_frame_id = fork_child_tf_name;
        fork_trans.transform.translation.x = -0.1;
        fork_trans.transform.translation.y = 0;
        fork_trans.transform.translation.z = fork_pos.data;
        fork_trans.transform.rotation = tf::createQuaternionMsgFromYaw(3.14);
        fork_tf.sendTransform(fork_trans);
        //发送转换

        // if(Data3 == 0)
        //     vx =Data2;
        // else
        vx = Data2 * cos(Data3 * 3.1415 / 180);
        // vx  = Data2;//前轮当前线速度 m/s  未转换为X轴速度
        vy = 0;                       //三轮前驱无Y轴速度
        vth_imu = angular_velocity_z; //设备当前Z轴角速度 rad/s
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

        //以给定机器人速度的典型方式计算里程计
        double delta_th = vth * dt;
        double delta_x = (vx * cos(th + delta_th / 2) - vy * sin(th + delta_th / 2)) * dt;
        double delta_y = (vx * sin(th + delta_th / 2) + vy * cos(th + delta_th / 2)) * dt;

        x += delta_x;   // X轴速度累积位移 m
        y += delta_y;   // Y轴速度累积位移 m
        th += delta_th; // Z轴角速度累积求车体朝向角度  rad //存在漂移

        th_imu += vth_imu * dt;
        /***************************odom tf**************************/
        //因为所有的里程表都是6自由度的，所以我们需要一个由偏航创建的四元数
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
        // printf("th = %f",th);
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
        while (n > 0)
        {

            uint8_t buffer[64];
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
                }
                sum = 0;
                memset(buffer, 0, sizeof(uint8_t) * 65);
            }
            angular_velocity_x = Data4 * 0.001064;  //转换成 rad/s
            angular_velocity_y = Data5 * 0.001064;  //转换成 rad/s
            angular_velocity_z = Data6 * 0.001064;  //转换成 rad/s
            accelerated_wheel_speed = Data7 / 2048; //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
            accelerated_speed_y = Data8 / 2048;     //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
            accelerated_speed_z = Data9 / 2048;     //转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
            // for (int i = 0; i < 100; i++)
            // {
            //     printf("Data 1 = %f\n", Data1);
            //     printf("Data 2 = %f\n", Data2);
            //     printf("Data 3 = %f\n", Data3);
            //     printf("Data 4 = %f\n", Data4);
            //     printf("Data 5 = %f\n", Data5);
            //     printf("Data 6 = %f\n", Data6);
            //     printf("Data 7 = %f\n", Data7);
            //     printf("Data 8 = %f\n", Data8);
            //     printf("Data 9 = %f\n", Data9);
            //     printf("Data 10 = %f\n", Data10);
            //     printf("Data 11 = %f\n", Data11);
            //     printf("Data 12 = %f\n", Data12);
            //     printf("Data 13 = %f\n", Data13);
            //     printf("Data 14 = %f\n", Data14);
            //     printf("Data 15 = %f\n", Data15);
            //     ros::Duration(1).sleep();
            // }
            // if(Data1)
            //     ROS_INFO("Flag_start: ON");//下位机电机启动/停止标志，1启动，0停止
            // else
            //     ROS_INFO("Flag_start: OFF");//下位机电机启动/停止标志，1启动，0停止

            // ROS_INFO("Current_linear_x: [%f m/s]", Data2);//前轮当前线速度
            // ROS_INFO("Current_angle_z: [%f deg/s]", Data3); //前轮当前转角

            // ROS_INFO("gyro_Roll: [%g ]", fmod(Data4,1.0) ); //车体绕X轴当前角速度 原始数值
            // ROS_INFO("gyro_Pitch: [%g ]", fmod(Data5,1.0)); //车体绕Y轴当前角速度 原始数值
            // ROS_INFO("gyro_Yaw: [%g ", fmod(Data6,1.0) ); //车体绕Z轴当前角速度 原始数值

            // ROS_INFO("accel_x: [%g ]", Data7); //车体X轴当前加速度 原始数值
            // ROS_INFO("accel_y: [%g ]", Data8); //车体Y轴当前加速度 原始数值
            // ROS_INFO("accel_z: [%g ", Data9); //车体Z轴当前加速度 原始数值

            // ROS_INFO("Yaw: [%f deg/s]", Data10); //车体Z轴当前角度 deg
            // ROS_INFO("Voltage: [%f V]", Data11/100); // 电池电压
            // ROS_INFO("red_key: [%f]", Data12); // 红色紧急开关位0/1 运行/停止
            // ROS_INFO("Encoder_C: [%f]", Data13); // 起重电机编码器原始数据（未转换）
            // ROS_INFO("LIMIT_KEY_LOW: [%f]", Data14); // 起重电机下行限位开关（用于校准）
            // ROS_INFO("LIMIT_KEY_UP: [%f]", Data15); // 起重电机上行限位开关（用于校准）
            // ROS_INFO("-----------------------");
            if (Data11 / 100 < 23.5 && Data11 != 0.0)
            {
                ROS_WARN("Low Voltage  %f vol", Data11 / 100);
                if (Data11 / 100 < 5)
                {
                    ROS_ERROR("PLEASE OPEN \"POWER SUPPLY\" before CONNECT USB!!!");
                }
            }
        }

        if (state != "Motor Calibration Complete")
        {
            init_stat();
        }

        // ROS_INFO("count %d",calibration_flag);
        loop_rate.sleep(); //循环延时时间
    }

    for (uint8_t j = 0; j < 3; j++) // clear buff
        send_data(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    //关闭串口
    sp.close();

    return 0;
}
