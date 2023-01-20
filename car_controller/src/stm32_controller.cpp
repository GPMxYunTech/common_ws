#include <serial/serial.h>
#include <iostream>
#include <math.h>
#include <stm32_controller.h>

using namespace std;

// serial::Serial sp; // serial port
typedef unsigned char byte;
// double Data1 = 0, Data2, Data3, Data4, Data5, Data6, Data7, Data8, Data9, Data10, Data11, Data12, Data13, Data14, Data15; // recv data

void send_data(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10, float data11, float data12)
{
    isnan(data4) ? data4 = 0.0 : data4 = data4;
    uint8_t tbuf[53];
    unsigned char *p;
    p = (unsigned char *)&data1;
    tbuf[4] = (unsigned char)(*(p + 3));
    tbuf[5] = (unsigned char)(*(p + 2));
    tbuf[6] = (unsigned char)(*(p + 1));
    tbuf[7] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data2;
    tbuf[8] = (unsigned char)(*(p + 3));
    tbuf[9] = (unsigned char)(*(p + 2));
    tbuf[10] = (unsigned char)(*(p + 1));
    tbuf[11] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data3;
    tbuf[12] = (unsigned char)(*(p + 3));
    tbuf[13] = (unsigned char)(*(p + 2));
    tbuf[14] = (unsigned char)(*(p + 1));
    tbuf[15] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data4;
    tbuf[16] = (unsigned char)(*(p + 3));
    tbuf[17] = (unsigned char)(*(p + 2));
    tbuf[18] = (unsigned char)(*(p + 1));
    tbuf[19] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data5;
    tbuf[20] = (unsigned char)(*(p + 3));
    tbuf[21] = (unsigned char)(*(p + 2));
    tbuf[22] = (unsigned char)(*(p + 1));
    tbuf[23] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data6;
    tbuf[24] = (unsigned char)(*(p + 3));
    tbuf[25] = (unsigned char)(*(p + 2));
    tbuf[26] = (unsigned char)(*(p + 1));
    tbuf[27] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data7;
    tbuf[28] = (unsigned char)(*(p + 3));
    tbuf[29] = (unsigned char)(*(p + 2));
    tbuf[30] = (unsigned char)(*(p + 1));
    tbuf[31] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data8;
    tbuf[32] = (unsigned char)(*(p + 3));
    tbuf[33] = (unsigned char)(*(p + 2));
    tbuf[34] = (unsigned char)(*(p + 1));
    tbuf[35] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data9;
    tbuf[36] = (unsigned char)(*(p + 3));
    tbuf[37] = (unsigned char)(*(p + 2));
    tbuf[38] = (unsigned char)(*(p + 1));
    tbuf[39] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data10;
    tbuf[40] = (unsigned char)(*(p + 3));
    tbuf[41] = (unsigned char)(*(p + 2));
    tbuf[42] = (unsigned char)(*(p + 1));
    tbuf[43] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data11;
    tbuf[44] = (unsigned char)(*(p + 3));
    tbuf[45] = (unsigned char)(*(p + 2));
    tbuf[46] = (unsigned char)(*(p + 1));
    tbuf[47] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data12;
    tbuf[48] = (unsigned char)(*(p + 3));
    tbuf[49] = (unsigned char)(*(p + 2));
    tbuf[50] = (unsigned char)(*(p + 1));
    tbuf[51] = (unsigned char)(*(p + 0));

    // fun:功能字 0XA0~0XAF
    // data:数据缓存区，48字节
    // len:data区有效数据个数

    uint8_t len = 48;

    tbuf[len + 4] = 0; //校验位置零
    tbuf[0] = 0XAA;    //帧头
    tbuf[1] = 0XAA;    //帧头
    tbuf[2] = 0XF1;    //功能字
    tbuf[3] = len;     //数据长度
    for (uint8_t i = 0; i < len + 4; i++)
        tbuf[len + 4] += tbuf[i]; //计算和校验

    try
    {
        sp.write(tbuf, len + 5); //发送数据下位机
    }
    catch (serial::IOException &e)
    {
        // ROS_ERROR_STREAM("Unable to send data through serial port");
    }
}

float b2f(byte m0, byte m1, byte m2, byte m3)
{
    //求符號位
    float sig = 1.;
    if (m0 >= 128.)
        sig = -1.;

    //求價碼
    float jie = 0.;
    if (m0 >= 128.)
    {
        jie = m0 - 128.;
    }
    else
    {
        jie = m0;
    }
    jie = jie * 2.;
    if (m1 >= 128.)
        jie += 1.;

    jie -= 127.;
    //求尾码
    float tail = 0.;
    if (m1 >= 128.)
        m1 -= 128.;
    tail = m3 + (m2 + m1 * 256.) * 256.;
    tail = (tail) / 8388608; //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1 + tail);

    return f;
}
