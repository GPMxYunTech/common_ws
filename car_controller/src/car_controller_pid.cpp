/*
*******************************************
*/
#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define Sign(A) ((A) >= 0 ? 1 : -1)  //210727 char sdd
//创建一个serial类
serial::Serial sp;


//只简单输出前轮的行程和前轮的转角,未涉及坐标转换
float speed_x;//前驱三轮车前轮速度
float turn_z;//前驱三轮车前轮转角
uint8_t i_u;
uint8_t FLAG_USART; 
uint8_t FLAG_US; 
uint16_t count;
uint8_t Flag_start=0;
float x_mid_speed;
float z_mid_speed;
float z_mid_angle;
float z_angle_real;
int size;
float Data1,Data2,Data3,Data4,Data5,Data6,Data7,Data8,Data9,Data10,Data11;
double wheel_base =0.33;  
/* pid parameter */
//210727 char add 
double target_Vx,target_Vth,real_Vx,real_Vth,target_x,target_th,real_x,real_th;  
double Ierr_x,Ierr_th;
double Kp_x=0.1,Ki_x=0.001;
double Kp_th=1,Ki_th=0.001;
//210727 char add end
/* pid parameter end*/

void send_data(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8,float data9,float data10,float data11,float data12);
/*210727 char add*/
void set_vel(double x_mid_speed,double z_mid_speed,double z_mid_angle)
{
    if(x_mid_speed>0 && z_mid_speed==0 && z_mid_angle==0){speed_x= x_mid_speed;turn_z=0;}//前进
    else if(x_mid_speed<0 && z_mid_speed==0 && z_mid_angle==0){speed_x= x_mid_speed;turn_z=0;}//后退

    else if(x_mid_speed==0 && z_mid_speed>0 && z_mid_angle==0){speed_x= +z_mid_speed;turn_z=+90;}//左自转
    else if(x_mid_speed==0 && z_mid_speed<0 && z_mid_angle==0){speed_x= +z_mid_speed;turn_z=+90;}//右自转

    else if(x_mid_speed>0 && z_mid_speed==0 && z_mid_angle>0){speed_x= x_mid_speed;turn_z=z_mid_angle;}//左前转弯
    else if(x_mid_speed>0 && z_mid_speed==0 && z_mid_angle<0){speed_x= x_mid_speed;turn_z=z_mid_angle;}//右前转弯

    else if(x_mid_speed<0 && z_mid_speed==0 && z_mid_angle<0){speed_x= x_mid_speed;turn_z=-z_mid_angle;}//左后转弯
    else if(x_mid_speed<0 && z_mid_speed==0 && z_mid_angle>0){speed_x= x_mid_speed;turn_z=-z_mid_angle;}//右后转弯

    else if(x_mid_speed==0 && z_mid_speed==0 && z_mid_angle>0){speed_x= 0;turn_z=z_mid_angle;}//纯轮子转动
    else if(x_mid_speed==0 && z_mid_speed==0 && z_mid_angle<0){speed_x= 0;turn_z=z_mid_angle;}//纯轮子转动

    //if(x_mid_speed==0 && z_mid_speed==0 && z_mid_angle==0)Flag_start=0;   //210803 char market
    //else Flag_start=1;    //210803 char market
    if(x_mid_speed==0 && z_mid_speed==0 && z_mid_angle==0){speed_x= 0;turn_z=0;};   //210803 char market
    Flag_start=1; //210803 char add

    FLAG_USART=1;

}
/*210727 char add end*/
double Vth2Steering(double vx ,double vth)
{
    double radius,steering;
    if(vth ==0)    
    {
        radius=0;
        steering =0;
    
    }
    else
    {
        radius= vx/vth;
        steering = atan(wheel_base/radius)*180/3.14;
        if(vx ==0)
        {
            steering =90*Sign(vth);
            x_mid_speed = 0.1; //210803 char market
            //x_mid_speed = 0;
        
        }
        
    }
    return steering; 
}

void chatterCallback(const geometry_msgs::Twist &msg)//获取键盘控制的回调函数
{


/*rot_vel to steering_angle*/
//210727 cahr add start

/*
double radius,steering;  
if(msg.angular.z ==0)    
{
    radius=0;
    steering =0;
    
}
else
{
    radius=msg.linear.x/msg.angular.z;
    steering = atan(wheel_base/radius)*180/3.14;
    if(msg.linear.x ==0)
    {
        steering =90*Sign(msg.angular.z);
        x_mid_speed = 0.1;
    }
        
}
//210727 cahr add start end
*/

/*rot_vel to steering_angl end*/ 
    

    x_mid_speed =msg.linear.x;
    z_mid_speed =msg.linear.z;
    //z_mid_angle =msg.angular.z;
    //z_mid_angle =steering;  //210727 cahr add
    z_mid_angle =Vth2Steering(msg.linear.x,msg.angular.z);  //210727 cahr add
    //ROS_INFO("X_speed: [%g]", msg.linear.x);//
    //ROS_INFO("Z_turn: [%g]", msg.linear.z);
    //ROS_INFO("WHEEL_turn: [%g]", msg.angular.z);
    //ROS_INFO("steering: [%g]",z_mid_angle);
    
    //ROS_INFO("-------------");
    if(x_mid_speed > +1.1)x_mid_speed = +1.1;
    if(x_mid_speed < -1.1)x_mid_speed = -1.1;

    if(z_mid_speed > +1.1)z_mid_speed = +1.1;
    if(z_mid_speed < -1.1)z_mid_speed = -1.1;

    if(z_mid_angle > +90)z_mid_angle = +90;
    if(z_mid_angle < -90)z_mid_angle = -90;

    /* pid get trget vel*/
    //210727 char add
    //target_Vx =(fabs(msg.linear.x)>1.1)?1.1*Sign(msg.linear.x):msg.linear.x;
    //target_Vth =msg.angular.z;
    target_Vx =x_mid_speed;                 //210729 char change
    target_Vth =msg.angular.z;                //210729 char change
    
    target_x =target_th =real_x=real_th =0; //210727 char add init pid
    ROS_ERROR("reset pid parameter");  //210729 char add
    //PID error reset
    Ierr_x=0;
    Ierr_th =0;
    //210727 char add end
    /* pid get trget vel end*/
    set_vel(x_mid_speed,z_mid_speed,z_mid_angle);//210727 char add

}
/*pid control function start */
//210727 char add

void pid_control()
{
    double err_x  =   target_x -real_x ;    //2100729 char change
    double err_th =   target_th - real_th;   //2100729 char change
    Ierr_x += err_x;
    Ierr_th += err_th;
    if(fabs(Ierr_th)>10)
    {
       Ierr_th =Sign(Ierr_th)*10.0; 
    }
    double revise_x  = Kp_x*err_x + Ki_x*Ierr_x;
    double revise_th = Kp_th*err_th + Ki_th*Ierr_th;
    real_Vx += revise_x;                              //210729 char marker
    real_Vth += revise_th;
    float z_error = z_mid_angle -z_angle_real;          //210729 char add

    ROS_INFO("KP_th %f KI_th %f",Kp_th,Ki_th);
    set_vel(target_Vx,0,2*Vth2Steering(target_x,real_th)); //210803 char change
    //set_vel(target_Vx,0,z_angle_real+10*z_error); //210803 char change
    //ROS_INFO("target_x %f target_th %f",target_x,target_th);
    //ROS_INFO("real_x %f real_th %f",real_x,real_th);
    ROS_INFO("real_th %f target_th %f",real_th,target_th);  //210803 char change
    //ROS_INFO("err_x %f err_th %f",err_x,err_th);
    //ROS_INFO("Ierr_x %f Ierr_th %f",Ierr_x,Ierr_th);
    ROS_INFO("err_th %f Ierr_th %f",err_th,Ierr_th);         //210803 char change
    //ROS_INFO("target_Vx %f target_Vth %f",target_Vx,target_Vth);
    //ROS_INFO("real_Vx %f real_Vth %f",real_Vx,real_Vth);   
    ROS_INFO("target_Vth %f real_Vth %f",target_Vth,real_Vth);       //210803 char change
    //ROS_INFO("th_err %f cmd_th %f CB_th %f",z_error,(z_angle_real+10*z_error),z_angle_real);       //210803 char change
    ROS_INFO("steering %f",2*Vth2Steering(target_x,real_th));       //210803 char change
    ROS_INFO("***********");
    
    

}
//210727 char add end
/*pid control function end*/

typedef unsigned char byte;

float b2f(byte m0, byte m1, byte m2, byte m3)
{
    //求符号位
    float sig = 1.;
    if (m0 >=128.)
        sig = -1.;
    float jie = 0.;
    if (m0 >=128.)
    {
        jie = m0-128.  ;
    }
    else
    {
        jie = m0;
    }
    jie = jie * 2.;
    if (m1 >=128.)
        jie += 1.;

    jie -= 127.;
    //求尾码
    float tail = 0.;
    if (m1 >=128.)
        m1 -= 128.;
    tail =  m3 + (m2 + m1 * 256.) * 256.;
    tail  = (tail)/8388608;   //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1+tail);

    return f;
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle np;//为这个进程的节点创建一个句柄
    ros::NodeHandle nh_priv("~");//210727
    /*get parameter*/ 
    nh_priv.param<double>("Kp_x",Kp_x,1); //210729 char change
    nh_priv.param<double>("Ki_x",Ki_x,0.001); //210729 char change
    nh_priv.param<double>("Kp_th",Kp_th,1); //210729 char change
    nh_priv.param<double>("Ki_th",Ki_th,0.001); //210729 char change
    printf("***********************\n");
    printf("Kp_x  %f\n",Kp_x);
    printf("Ki_x  %f\n",Ki_x);
    printf("Kp_th %f\n",Kp_th);
    printf("Ki_th %f\n",Ki_th);
    printf("***********************\n");

    ros::Subscriber sub = np.subscribe("cmd_vel", 200, chatterCallback);//订阅键盘控制
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;

    double y = 0.0;

    double th = 0.0;



    double vx = 0.0;

    double vy = 0.0;

    double vth = 0.0;

    double init_flag =0; //210727 char add init parameter
    int init_count=0;   //210727 char add  init parameter
    int error_count=0;   //210729 char add  init parameter
    int pid_count =0; //210727 char add pid count
    ros::Time current_time, last_time;

    current_time = ros::Time::now();

    last_time = ros::Time::now();


    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);

    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");

    }
    else
    {
        return -1;
    }   
    for(uint8_t j=0;j<3;j++)send_data(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);  
    ros::Rate loop_rate(100);//设置循环间隔，即代码执行频率 100HZ
    
    //set_vel(0,0,0.05);  //2100727 char mark
    send_data(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); //210803 char add
    ROS_INFO_STREAM("wating for agv initialize."); //210727 char add

    while(ros::ok())
    {

        current_time = ros::Time::now();//记录当前时间


        /*odom calculate*/
        //210726 char add
        double th_f = Data3 *3.1415/180; //前輪角度
        z_angle_real =th_f;
        double v_f  = Data2; //前輪轉速
        //ROS_ERROR("th %f, v %f",th_f,v_f);

        //vx  = Data2;//前轮当前线速度 m/s  未转换为X轴速度
        //vy  = 0;//三轮前驱无Y轴速度
        //vx=(v_f*cos(th_f)*cos(th));
        //vy=(v_f*sin(th_f)*sin(th));
        //vth = (Data6/32768.f)*1000*(3.1415926f/180.f);//设备当前Z轴角速度 rad/s
        vth = (Data6/16384.f)*1000*(3.1415926f/180.f);//设备当前Z轴角速度 rad/s
        //vth = v_f*sin(th_f)/0.33;//设备当前Z轴角速度 rad/s
        //ROS_ERROR("x %f y %f th %f",vx,vy,vth);
        vx =v_f *cos(th_f) ;
        vy =0;
        //ROS_ERROR("vx %f,th %f,cos(th) %f ",vx,th_f,cos(th_f) ) ;
        //vth =v_f*sin(th_f)/0.33;
        
        //以给定机器人速度的典型方式计算里程计
        double dt = (current_time - last_time).toSec();

        double delta_x = (vx * cos(th) ) * dt;

        double delta_y = (vx * sin(th) ) * dt;
        double l =0.33;
        //double delta_x =  -l/tanh(th_f)*sin(th) +l/tanh(th_f)*sin(th+v_f*dt*sin(th_f)/l);

        //double delta_y = l/tanh(th_f)*sin(th) -l/tanh(th_f)*sin(th+v_f*dt*sin(th_f)/l);

        //double delta_th = v_f*sin(th_f) * dt/l;
        double delta_th = vth* dt;
        
        //210726 char add end
        /*odom calculate end*/
        
        //ROS_ERROR("dx %f dy %f dth %f",delta_x,delta_y,delta_th);
        
        
        /*pid getting target value start*/
        //210727 char add
        target_x +=(target_Vx*dt*cos(th));
        target_th +=(target_Vth*dt) ;
        //210727 char add end
        /*pid getting target value end*/


        x += delta_x;//X轴速度累积位移 m

        y += delta_y;//Y轴速度累积位移 m

        th += delta_th;//Z轴角速度累积求车体朝向角度  rad //存在漂移


        //ROS_ERROR("x %f y %f th %f",x,y,th);
        
        //因为所有的里程表都是6自由度的，所以我们需要一个由偏航创建的四元数
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);



        //首先，我们将通过tf发布转换

        geometry_msgs::TransformStamped odom_trans;

        odom_trans.header.stamp = current_time;

        odom_trans.header.frame_id = "odom";

        odom_trans.child_frame_id = "base_link";



        odom_trans.transform.translation.x = x;

        odom_trans.transform.translation.y = y;

        odom_trans.transform.translation.z = 0.0;

        odom_trans.transform.rotation = odom_quat;



        //发送转换

        odom_broadcaster.sendTransform(odom_trans);


        //接下来，我们将通过ROS发布里程计信息

        nav_msgs::Odometry odom;

        odom.header.stamp = current_time;

        odom.header.frame_id = "odom";



        //设置位置
        odom.pose.pose.position.x = x;

        odom.pose.pose.position.y = y;

        //odom.pose.pose.position.z = th; //210803 char mark
        odom.pose.pose.position.z = 0; //210803 char change


        odom.pose.pose.orientation = odom_quat;



        //设定速度
        odom.child_frame_id = "base_link";

        odom.twist.twist.linear.x = vx;

        odom.twist.twist.linear.y = vy;

        odom.twist.twist.angular.z = vth;
        /*pid getting value start*/
            //210727 char add
        real_Vx = vx;
        real_Vth = vth;
        real_x +=  (real_Vx*dt);
        real_th += (real_Vth*dt);
        //if(init_flag &&  target_Vx )           //210730 char change
        //    pid_control();                                        //PID test code 210727 char 
            //210727 char add end
        /*pid getting value end*/
        //发布消息
        odom_pub.publish(odom);

        last_time = current_time;//保存为上次时间


        ros::spinOnce();//执行回调处理函数，完后继续往下执行

        //发送指令控制电机运行 其中Flag_start是控制电机启动和停止的（1/0 启动/停止） speed_x为前轮线速度 turn_z为前轮转角    
        if(init_flag)  //210803 char add
            send_data(Flag_start, speed_x, 0, turn_z, 0, 0, 0, 0, 0, 0, 0, 0);//电机(启动/停止)(1/0)，前轮速度 m/s，0，前轮转角 °/s .

        size_t n = sp.available();//获取缓冲区内的字节数
        if(n>0 )
        {

            uint8_t buffer[54];
            sp.read(buffer, n);//读出数据  

            if(buffer[0]==0XAA && buffer[1]==0XAA && buffer[2]==0XF1)
            {               
                uint8_t sum; 
                for(uint8_t j=0;j<52;j++)sum+=buffer[j];    //计算校验和	
                if(sum==buffer[52])
                {					
                    Data1=  b2f(buffer[4],  buffer[5],  buffer[6],  buffer[7]); //电机启动停止控制位（1/0 启动/停止）
                    Data2=  b2f(buffer[8],  buffer[9],  buffer[10], buffer[11]);//前轮线速度 
                    Data3=  b2f(buffer[12], buffer[13], buffer[14], buffer[15]);//前轮转角
                    Data4=  b2f(buffer[16], buffer[17], buffer[18], buffer[19]);//绕X轴角速度 gyro_Roll 原始数值	
                    Data5=  b2f(buffer[20], buffer[21], buffer[22], buffer[23]);//绕Y轴角速度 gyro_Pitch 原始数值	
                    Data6=  b2f(buffer[24], buffer[25], buffer[26], buffer[27]);//绕Z轴角速度 gyro_Yaw 原始数值	
                    Data7=  b2f(buffer[28], buffer[29], buffer[30], buffer[31]);//X轴加速度 accel_x 原始数值	
                    Data8=  b2f(buffer[32], buffer[33], buffer[34], buffer[35]);//Y轴加速度 accel_y 原始数值	
                    Data9=  b2f(buffer[36], buffer[37], buffer[38], buffer[39]);//Z轴加速度 accel_z 原始数值	
                    Data10= b2f(buffer[40], buffer[41], buffer[42], buffer[43]); //Yaw Z轴角度
                    Data11= b2f(buffer[44], buffer[45], buffer[46], buffer[47]); //电池电压
                        //该位预留
                                    
                }
                sum =0;
                memset(buffer, 0, sizeof(uint8_t)*54);								
            }
            // if(Data1)
            //   ROS_INFO("Flag_start: ON");//下位机电机启动/停止标志，1启动，0停止
            // else
            //   ROS_INFO("Flag_start: OFF");//下位机电机启动/停止标志，1启动，0停止

            /*
            ROS_INFO("Current_linear_x: [%f m/s]", Data2);//前轮当前线速度 
            ROS_INFO("Current_angle_z: [%f deg/s]", Data3); //前轮当前转角  //+-0.5degree after reset

            ROS_INFO("gyro_Roll: [%g ]", Data4); //车体绕X轴当前角速度 原始数值	
            ROS_INFO("gyro_Pitch: [%g ]", Data5); //车体绕Y轴当前角速度 原始数值	
            ROS_INFO("gyro_Yaw: [%g ", Data6); //车体绕Z轴当前角速度 原始数值

            ROS_INFO("accel_x: [%g ]", Data7); //车体X轴当前加速度 原始数值	
            ROS_INFO("accel_y: [%g ]", Data8); //车体Y轴当前加速度 原始数值	
            ROS_INFO("accel_z: [%g ", Data9); //车体Z轴当前加速度 原始数值	

            ROS_INFO("Yaw: [%f deg/s]", Data10); //车体Z轴当前角度 deg
            ROS_INFO("Voltage: [%f V]", Data11/100); // 电池电压
            /
            //    ROS_INFO("-----------------------");  


                    }
            /*printf init result*/
            //210727 char end     
            if(!init_flag)
            {
                if(fabs(Data3)< 0.3)
                    init_count++;
                else
                    init_count =0;

                if(fabs(Data3)> 0.3 && fabs(Data3)< 0.75) //210729 char add
                    error_count++;                      //210729 char add
                else 
                    error_count =0;
                if (init_count > 50  || error_count>75)
                {
                    init_flag =1;
                    set_vel(0,0,0);  //210727 char add
                    target_x =target_th =real_x=real_th =0; //210727 char add init pid
                    ROS_INFO("Current_angle_z: [%f deg]", Data3); //前轮当前转角  //+-0.5degree after reset
                    if(init_count>50)
                        ROS_INFO_STREAM("AGV initialize sucessed");
                    else
                        ROS_WARN("AGV initialize error > 0.3");     //210729 char add
                } 
            }
            /*printf init result end*/  
            loop_rate.sleep();//循环延时时间
        }
    }


    //关闭串口
    for(uint8_t j=0;j<3;j++)send_data(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    sp.close();  

    return 0;
}




//************************发送12个数据**************************// 
void send_data(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8,float data9,float data10,float data11,float data12)
{
    uint8_t tbuf[53];
    unsigned char *p;
    p=(unsigned char *)&data1;
    tbuf[4]=(unsigned char)(*(p+3));
    tbuf[5]=(unsigned char)(*(p+2));
    tbuf[6]=(unsigned char)(*(p+1));
    tbuf[7]=(unsigned char)(*(p+0));

    p=(unsigned char *)&data2;
    tbuf[8]=(unsigned char)(*(p+3));
    tbuf[9]=(unsigned char)(*(p+2));
    tbuf[10]=(unsigned char)(*(p+1));
    tbuf[11]=(unsigned char)(*(p+0));
    
    p=(unsigned char *)&data3;
    tbuf[12]=(unsigned char)(*(p+3));
    tbuf[13]=(unsigned char)(*(p+2));
    tbuf[14]=(unsigned char)(*(p+1));
    tbuf[15]=(unsigned char)(*(p+0));
    
    p=(unsigned char *)&data4;
    tbuf[16]=(unsigned char)(*(p+3));
    tbuf[17]=(unsigned char)(*(p+2));
    tbuf[18]=(unsigned char)(*(p+1));
    tbuf[19]=(unsigned char)(*(p+0));
        
    p=(unsigned char *)&data5;
    tbuf[20]=(unsigned char)(*(p+3));
    tbuf[21]=(unsigned char)(*(p+2));
    tbuf[22]=(unsigned char)(*(p+1));
    tbuf[23]=(unsigned char)(*(p+0));
    
    p=(unsigned char *)&data6;
    tbuf[24]=(unsigned char)(*(p+3));
    tbuf[25]=(unsigned char)(*(p+2));
    tbuf[26]=(unsigned char)(*(p+1));
    tbuf[27]=(unsigned char)(*(p+0));
        
    p=(unsigned char *)&data7;
    tbuf[28]=(unsigned char)(*(p+3));
    tbuf[29]=(unsigned char)(*(p+2));
    tbuf[30]=(unsigned char)(*(p+1));
    tbuf[31]=(unsigned char)(*(p+0));
    
    p=(unsigned char *)&data8;
    tbuf[32]=(unsigned char)(*(p+3));
    tbuf[33]=(unsigned char)(*(p+2));
    tbuf[34]=(unsigned char)(*(p+1));
    tbuf[35]=(unsigned char)(*(p+0));
    
    p=(unsigned char *)&data9;
    tbuf[36]=(unsigned char)(*(p+3));
    tbuf[37]=(unsigned char)(*(p+2));
    tbuf[38]=(unsigned char)(*(p+1));
    tbuf[39]=(unsigned char)(*(p+0));
    
    p=(unsigned char *)&data10;
    tbuf[40]=(unsigned char)(*(p+3));
    tbuf[41]=(unsigned char)(*(p+2));
    tbuf[42]=(unsigned char)(*(p+1));
    tbuf[43]=(unsigned char)(*(p+0));
        
    p=(unsigned char *)&data11;
    tbuf[44]=(unsigned char)(*(p+3));
    tbuf[45]=(unsigned char)(*(p+2));
    tbuf[46]=(unsigned char)(*(p+1));
    tbuf[47]=(unsigned char)(*(p+0));
    
    p=(unsigned char *)&data12;
    tbuf[48]=(unsigned char)(*(p+3));
    tbuf[49]=(unsigned char)(*(p+2));
    tbuf[50]=(unsigned char)(*(p+1));
    tbuf[51]=(unsigned char)(*(p+0));
        
    //fun:功能字 0XA0~0XAF
    //data:数据缓存区，48字节
    //len:data区有效数据个数


    uint8_t len=48;

    tbuf[len+4]=0;  //校验位置零
    tbuf[0]=0XAA;   //帧头
    tbuf[1]=0XAA;   //帧头
    tbuf[2]=0XF1;    //功能字
    tbuf[3]=len;    //数据长度
    for(uint8_t i=0;i<len+4;i++)tbuf[len+4]+=tbuf[i]; //计算和校验

    try
    {
        sp.write(tbuf, len+5);//发送数据下位机

    }
    catch (serial::IOException& e)   
    {
        ROS_ERROR_STREAM("Unable to send data through serial port"); //如果发送数据失败，打印错误信息
    }
}