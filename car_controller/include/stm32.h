#ifndef _STM32_H 
#define _STM32_H 
#include "ros/ros.h"
#include <serial/serial.h>
typedef unsigned char byte;
class STM32
{
    private:
        serial::Serial sp; // serial port
        uint8_t tbuf[53];  //send data buffer
        uint8_t buffer[65];//read data buffer
        size_t n;          //緩衝區內字節數
        float b2f(byte m0, byte m1, byte m2, byte m3);// decode received data function
    public:
        //從read_data讀出的數據
        double Data1, Data2, Data3, Data4, Data5, Data6, Data7, Data8, Data9, Data10, Data11, Data12, Data13, Data14, Data15; // received data
        float angular_velocity_x, angular_velocity_y, angular_velocity_z;
        float accelerated_wheel_speed, accelerated_speed_y, accelerated_speed_z;

        STM32(); // 建構子，初始化串口
        ~STM32();// 解構子，關閉串口
        void send_data(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10, float data11, float data12);
        void read_data();
};
#endif