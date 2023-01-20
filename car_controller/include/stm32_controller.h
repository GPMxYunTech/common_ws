#ifndef _STM32_CONTROLLER_H
#define _STM32_CONTROLLER_H
extern serial::Serial sp; // serial port
typedef unsigned char byte;


void send_data(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10, float data11, float data12);
float b2f(byte m0, byte m1, byte m2, byte m3);







#endif