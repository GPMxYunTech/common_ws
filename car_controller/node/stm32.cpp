#include <stm32.h>
STM32::STM32()
{
    sp.setPort("/dev/ttyUSB0");                                // 設定要打開的串口名稱
    sp.setBaudrate(115200);                                    // 設定串口波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 創建timeout
    sp.setTimeout(to);                                         // 串口設置timeout
    try                                                        // try to open serial port
    {
        sp.open(); // 打開串口
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return ;
    }

    if (sp.isOpen()) // 判斷串口是否打開
    {
        ROS_INFO_STREAM("\x1B[0;32m""/dev/ttyUSB0 is opened.""\x1B[0m");
    }
    else
    {
        return ;
    }
    for (uint8_t j = 0; j < 3; j++) // clear buff
        send_data(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}
STM32::~STM32()
{
    for (uint8_t j = 0; j < 3; j++) // clear buff
        send_data(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    // 關閉串口
    sp.close();
    ROS_WARN("STM32 closed");
}

void STM32::send_data(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10, float data11, float data12)
{
    static uint8_t tbuf[53]{0};  //send data buffer
    // printf("%f, %f, %f, %f, %f\n", data1, data2, data3, data4, data5);
    isnan(data4) ? data4 = 0.0 : data4 = data4;
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

    tbuf[len + 4] = 0; // 校验位置零
    tbuf[0] = 0XAA;    // 帧头
    tbuf[1] = 0XAA;    // 帧头
    tbuf[2] = 0XF1;    // 功能字
    tbuf[3] = len;     // 数据长度
    for (uint8_t i = 0; i < len + 4; i++)
        tbuf[len + 4] += tbuf[i]; // 计算和校验

    try
    {
        sp.write(tbuf, len + 5); // 发送数据下位机
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to send data through serial port"); // 如果发送数据失败，打印错误信息
    }
};
void STM32::read_data()
{
    static size_t n;          //緩衝區內字節數
    static uint8_t buffer[65]{0};
    n = sp.available(); // 獲取緩衝區內字節數
    if (n > 0)
    {
        sp.read(buffer, n);
        if (buffer[0] == 0XAA && buffer[1] == 0XAA && buffer[2] == 0XF1)
        {
            uint8_t sum{0};
            for (uint8_t j = 0; j < 64; j++)
                sum += buffer[j]; // 計算校驗和
            if (sum == buffer[64])
            {
                Data1 = b2f(buffer[4], buffer[5], buffer[6], buffer[7]);      // 电机启动停止控制位（1/0 启动/停止）
                Data2 = b2f(buffer[8], buffer[9], buffer[10], buffer[11]);    // 前轮线速度
                Data3 = b2f(buffer[12], buffer[13], buffer[14], buffer[15]);  // 前轮转角
                Data4 = b2f(buffer[16], buffer[17], buffer[18], buffer[19]);  // 绕X轴角速度 gyro_Roll 原始数值
                Data5 = b2f(buffer[20], buffer[21], buffer[22], buffer[23]);  // 绕Y轴角速度 gyro_Pitch 原始数值
                Data6 = b2f(buffer[24], buffer[25], buffer[26], buffer[27]);  // 绕Z轴角速度 gyro_Yaw 原始数值
                Data7 = b2f(buffer[28], buffer[29], buffer[30], buffer[31]);  // X轴加速度 accel_x 原始数值
                Data8 = b2f(buffer[32], buffer[33], buffer[34], buffer[35]);  // Y轴加速度 accel_y 原始数值
                Data9 = b2f(buffer[36], buffer[37], buffer[38], buffer[39]);  // Z轴加速度 accel_z 原始数值
                Data10 = b2f(buffer[40], buffer[41], buffer[42], buffer[43]); // Yaw Z轴角度
                Data11 = b2f(buffer[44], buffer[45], buffer[46], buffer[47]); // 电池电压              24-25   <24.3  low
                Data12 = b2f(buffer[48], buffer[49], buffer[50], buffer[51]); // 红色紧急开关位0/1 运行/停止
                Data13 = b2f(buffer[52], buffer[53], buffer[54], buffer[55]); // 起重电机编码器原始数据（未转换） 如果有需要可以添加发送指令去清0，上面的发送命令还有剩余   gearrate 30  dt 5 ms
                Data14 = b2f(buffer[56], buffer[57], buffer[58], buffer[59]); // 起重电机下行限位开关（用于校准） 1代表开关被压住
                Data15 = b2f(buffer[60], buffer[61], buffer[62], buffer[63]); // 起重电机上行限位开关（用于校准） 1代表开关被压住
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

        static bool once_flag = true;        
        if (Data11 / 100 < 23.5 && Data11 != 0.0 and once_flag == true)
        {
            once_flag = false;
            ROS_WARN("low voltage  %f vol", Data11 / 100.0f);
            if (Data11 / 100 < 5)
            {
                ROS_ERROR("PLEASE OPEN \"POWER SUPPLY\" before CONNECT USB!!!");
            }
        }
    }
}

float STM32::b2f(byte m0, byte m1, byte m2, byte m3)
{
    static float sig, jie, tail, f;
    //求符号位
    sig = 1.;
    if (m0 >= 128.)
        sig = -1.;

    //求阶码
    jie = 0.;
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
    tail = 0.;
    if (m1 >= 128.)
        m1 -= 128.;
    tail = m3 + (m2 + m1 * 256.) * 256.;
    tail = (tail) / 8388608; //   8388608 = 2^23

    f = sig * pow(2., jie) * (1 + tail);
    return f;
}