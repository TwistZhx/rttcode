/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-22     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include <stdlib.h>
#include <string.h>

#include <at_socket.h>
#include <at_device_esp8266.h>
#define ESP8266_SAMPLE_DEIVCE_NAME     "esp0"

//#define SERVER_IP "120.55.167.161"
//#define SERVER_PORT 8086  //端口要写死，服务端是根据端口号来判断转发方向的

#include <ssd1306.h>

#include <mpu6xxx.h>
#include <rtdevice.h>

#include <stdio.h>

#include <sensor.h>

#include <onenet.h>
//定义事件控制块
static rt_event_t mp3_event = RT_NULL; //事件句柄
#define EVENT1  (0x01 << 0)//设置事件掩码的位0  心率过低
#define EVENT2  (0x01 << 1)//设置事件掩码的位1  心率过高
#define EVENT3  (0x01 << 2)//设置事件掩码的位2  血氧过低
#define EVENT4  (0x01 << 3)//设置事件掩码的位3  有人摔倒1给MP3
#define EVENT5  (0x01 << 4)//设置事件掩码的位4  有人摔倒2给eso8266
#define EVENT6  (0x01 << 5)//设置事件掩码的位5  有人摔倒3给OLED

#define EVENT7  (0x01 << 6)//设置事件掩码的位6  无人摔倒2给eso8266
#define EVENT8  (0x01 << 7)//设置事件掩码的位7  无人摔倒3给OLED

// 定义  消息队列  控制块
static rt_mq_t t1_mq1 = RT_NULL;//心率给esp8266
static rt_mq_t t1_mq2 = RT_NULL;//心率给OLED
static rt_mq_t t2_mq1 = RT_NULL;//血氧给esp8266
static rt_mq_t t2_mq2 = RT_NULL;//血氧给OLED

static struct at_device_esp8266 esp0 =
{
    ESP8266_SAMPLE_DEIVCE_NAME,
    ESP8266_SAMPLE_CLIENT_NAME,

    ESP8266_SAMPLE_WIFI_SSID,
    ESP8266_SAMPLE_WIFI_PASSWORD,
    ESP8266_SAMPLE_RECV_BUFF_LEN,
};

rt_device_t u4_dev; //创建串口句柄
uint8_t Play1[] = { 0x7E, 0x05, 0x41, 0x00, 0x01, 0x45, 0xEF };//播放第一条语音
uint8_t Play2[] = { 0x7E, 0x05, 0x41, 0x00, 0x02, 0x46, 0xEF };
uint8_t Play3[] = { 0x7E, 0x05, 0x41, 0x00, 0x03, 0x47, 0xEF };
uint8_t Play4[] = { 0x7E, 0x05, 0x41, 0x00, 0x04, 0x40, 0xEF };
uint8_t Play5[] = { 0x7E, 0x05, 0x41, 0x00, 0x05, 0x41, 0xEF };
uint8_t Play6[] = { 0x7E, 0x05, 0x41, 0x00, 0x06, 0x42, 0xEF };
uint8_t Play7[] = { 0x7E, 0x05, 0x41, 0x00, 0x07, 0x43, 0xEF };
uint8_t Play8[] = { 0x7E, 0x05, 0x41, 0x00, 0x08, 0x36, 0xEF };
uint8_t Volum1[] = { 0x7E, 0x04, 0x31, 0x06, 0x33, 0xEF };//设置音量为6
uint8_t Volum2[] = { 0x7E, 0x04, 0x31, 0x0C, 0x39, 0xEF };
uint8_t Volum3[] = { 0x7E, 0x04, 0x31, 0x12, 0x27, 0xEF };
uint8_t Volum4[] = { 0x7E, 0x04, 0x31, 0x1E, 0x2B, 0xEF };
uint8_t Play[] = { 0x7E, 0x03, 0x01, 0x02, 0xEF };   //播放指令
uint8_t Pause[] = { 0x7E, 0x03, 0x02, 0x01, 0xEF };  //暂停指令

//void Aliyun_send(float a,char *streamName)
//{
    //int body_len=0;
    //char buf[128];
    //memset(buf, 0, sizeof(buf));

    //body_len=esp8266_toJSON(buf, a, streamName);//将数据整合成JSON格式
    //esp8266_socket_send(esp0.device.sockets, buf, body_len, AT_SOCKET_TCP);
//}

static int esp8266_device_register(void)
{
    struct at_device_esp8266 *esp8266 = &esp0;

    return at_device_register(&(esp8266->device),
                              esp8266->device_name,
                              esp8266->client_name,
                              AT_DEVICE_CLASS_ESP8266,
                              (void *) esp8266);

}

/*static int esp8266_socket_create(void)
{
    //创建socket，并将描述符返回到esp0
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        LOG_E("create socket error!");
        return-1;
    }
    esp0.device.sockets->socket = sockfd;


    //设置服务端地址并连接
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);          // 设置服务端端口
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP); // 设置服务端IP

    // 发起连接
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)// 触发地址传递
    {
        LOG_E("connect failed!");
        closesocket(sockfd);
        return -1;
    }
    LOG_I("Connected to server!");
    return 1;

}*/


static void esp8266_upload_entry()
{
    int r1_queue;
    float r2_queue; // 数据类型  =  data.data.hr  的数据类型
    char value2[10];
    rt_bool_t downFlag=0;

    rt_uint32_t recved = 0x00;

    rt_err_t uwRet = RT_EOK;

    //Aliyun_send((float)downFlag, "Down");//重置一下客户端的状态

    while(1)
    {
        // t1_mq1 队列读取  心率  （接收）
        uwRet = rt_mq_recv(t1_mq1,   //读取（接收）队列的ID(句柄)
                &r1_queue,           //读取（接收）的数据保存位置
                sizeof(r1_queue),        //读取（接收）的数据的长度
                100);     //等待时间
        if (RT_EOK == uwRet)
        {
            rt_kprintf("esp8266线程接收到的数据是：%d\n", r1_queue);

            onenet_mqtt_upload_digit("HR", r1_queue);
        }
        rt_thread_mdelay(3000);

        // t2_mq1 队列读取  血氧  （接收）
        uwRet = rt_mq_recv(t2_mq1,   //读取（接收）队列的ID(句柄)
                &r2_queue,           //读取（接收）的数据保存位置
                sizeof(r2_queue),        //读取（接收）的数据的长度
                100);     //等待时间
        if (RT_EOK == uwRet)
        {
            rt_kprintf("esp8266线程接收到的数据是：%d\n",r2_queue);
            sprintf(value2,"%.1f",r2_queue);
            onenet_mqtt_upload_string("SpO2", value2);
        }
        rt_thread_mdelay(3000);
        rt_event_recv(mp3_event, //事件对象句柄
                                EVENT5 | EVENT7, //接收线程感兴趣的事件
                                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, //接收选项  注意,这里要用 逻辑或RT_EVENT_FLAG_OR
                                100, //指定超时时间
                                &recved); //指向接收事件
        if (recved == EVENT5)
        {
            downFlag=RT_TRUE;
            onenet_mqtt_upload_digit("Down", 1);
        }
        else if (recved == EVENT7)
        {
            downFlag=RT_FALSE;
            onenet_mqtt_upload_digit("Down", 0);
        }
        rt_thread_mdelay(3000);

    }
}




int oled_init()
{
    //OLED显示的初始化
    ssd1306_Init();     //添加代码，显示屏初始化

    ssd1306_Fill(White);//清屏
    ssd1306_SetCursor(43, 30);     //欢迎
    ssd1306_WriteString("Welcome", Font_7x10, Black);
    ssd1306_UpdateScreen();

    rt_thread_mdelay(1000);

    ssd1306_Fill(Black);//清屏

    char text1[20]={"---"},text2[20]={"----"};
    //坐标  2,6   :2列 6行
    ssd1306_SetCursor(0, 0);     //添加代码 设置显示光标位置        第0行 的第20列
    ssd1306_WriteString("~~~~~~Health~~~~~~", Font_7x10, Black);
    ssd1306_SetCursor(2, 15);
    ssd1306_WriteString("HR:", Font_7x10, White);
    ssd1306_SetCursor(2, 30);
    ssd1306_WriteString("SpO2:", Font_7x10, White);
    ssd1306_SetCursor(2, 45);
    ssd1306_WriteString("Fall:", Font_7x10, White);

    ssd1306_SetCursor(50, 15);     // 设置  显示光标位置       第15行 的第50列
    ssd1306_WriteString(text1, Font_7x10, White);     // ，设置  显示内容，即text
    ssd1306_SetCursor(50, 30);     // 设置  显示光标位置       第30行 的第50列
    ssd1306_WriteString(text2, Font_7x10, White);     // ，设置  显示内容，即text
    ssd1306_UpdateScreen();     // 更新显示屏信息
    return 0;
}
INIT_COMPONENT_EXPORT(oled_init);

//5、 创建OLED显示线程   入口函数
static void oled_display_entry(void *parameter)
{
    char text1[20]={"---"},text2[20]={"----"},text3[20]={};
    rt_uint32_t recved = 0x00;
    int r1_queue;
    float r2_queue; // 数据类型  =  data.data.hr  的数据类型  小心
    rt_err_t uwRet = RT_EOK;

    int hrCount=0;
    int spo2Count=0;

    while (1)
    {
        recved=0x00;

        // t1_mq 队列读取  心率  （接收）
        uwRet = rt_mq_recv(t1_mq2,   //读取（接收）队列的ID(句柄)
                &r1_queue,           //读取（接收）的数据保存位置
                sizeof(r1_queue),        //读取（接收）的数据的长度
                100);     //等待时间
        if (RT_EOK == uwRet)
        {
            hrCount=0;
            sprintf(text1, "%03d", r1_queue);
            //rt_kprintf(text1);
        }
        else
        {
            hrCount++;
            if(hrCount > 5)
                sprintf(text1, "---");//2s没接收到就不显示
        }

        // t2_mq 队列读取  血氧  （接收）
        uwRet = rt_mq_recv(t2_mq2,   //读取（接收）队列的ID(句柄)
                &r2_queue,           //读取（接收）的数据保存位置
                sizeof(r2_queue),        //读取（接收）的数据的长度
                100);     //等待时间
        if (RT_EOK == uwRet)
        {
            //rt_kprintf("OLED线程接收到的数据是：%f\n", r2_queue);
            spo2Count=0;
            sprintf(text2, "%.1f", r2_queue);
        }
        else
        {
            spo2Count++;
            if(spo2Count > 5)
                sprintf(text2, "----");//0.5-1.5s没接收到就不显示
        }

        rt_event_recv(mp3_event, //事件对象句柄
                        EVENT6 | EVENT8, //接收线程感兴趣的事件
                        RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, //接收选项  注意,这里要用 逻辑或RT_EVENT_FLAG_OR
                        100, //指定超时时间
                        &recved); //指向接收事件
        if (recved == EVENT6)
        {
            sprintf(text3,"YES");
            ssd1306_SetCursor(50, 45);
            ssd1306_WriteString(text3, Font_7x10, Black);
        }
        else if (recved == EVENT8)
        {
            sprintf(text3,"NO ");
            ssd1306_SetCursor(50, 45);
            ssd1306_WriteString(text3, Font_7x10, White);
        }

        ssd1306_SetCursor(50, 15);     // 设置  显示光标位置       第15行 的第50列
        ssd1306_WriteString(text1, Font_7x10, White);     // ，设置  显示内容，即text
        ssd1306_SetCursor(50, 30);     // 设置  显示光标位置       第30行 的第50列
        ssd1306_WriteString(text2, Font_7x10, White);     // ，设置  显示内容，即text
        ssd1306_UpdateScreen();     // 更新显示屏信息
    }
}

/**
 * 将初始化与播放分离开来
 * MP3初始化函数
 */
int MP3_init()
{
    rt_err_t ret = 0;

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; // 初始化配置参数
    config.baud_rate = BAUD_RATE_9600; //修改波特率为 9600
    config.data_bits = DATA_BITS_8; //数据位 8
    config.stop_bits = STOP_BITS_1; //停止位 1
    config.bufsz = 128; //修改缓冲区 buff size 为 128
    config.parity = PARITY_NONE; //无奇偶校验位

    /*控制串口设备  通过控制接口传入命令控制字  与控制参数 */
    u4_dev = rt_device_find("uart4"); //将句柄绑定到串口4上
    rt_device_control(u4_dev, RT_DEVICE_CTRL_CONFIG, &config);
    if (u4_dev == RT_NULL)
    {
        LOG_D("uart4 find failed");
        //return ERRWARNING;
    }
    else
    {
        LOG_D("uart4 find sucess");
    }

    ret = rt_device_open(u4_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret < 0)
    {
        LOG_E("uart4 open failed");
        return -1;
    }
    else
    {
        LOG_D("uart4 open sucess");
    }

    return 0;
}

/**
 * MP3线程 接收事件 对不同的事件做出不同的语音播报
 * 传感器采集到异常数据时，发送事件，MP3线程接收，做出相应的处理
 * @param parameter
 */
static void mp3_thread_entry(void *parameter)
{
    rt_uint32_t recved = 0x00;
    rt_device_write(u4_dev, 0, Play1, Play1[1] + 2);    //播放  第一句话  系统  欢迎词
    //rt_device_write(u4_dev, 0, Volum4, Volum4[1] + 2);//设置音量
    rt_thread_mdelay(5000);

    while (1)
    {
        rt_event_recv(mp3_event, //事件对象句柄
                EVENT1 | EVENT2 | EVENT3 |EVENT4, //接收线程感兴趣的事件
                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, //接收选项  注意,这里要用 逻辑或RT_EVENT_FLAG_OR
                RT_WAITING_FOREVER, //指定超时事件,一直等
                &recved); //指向接收事件
        if (recved == EVENT1)//心率过低
        {
            rt_device_write(u4_dev, 0, Pause, Pause[1] + 2);
            rt_thread_mdelay(50);
            rt_device_write(u4_dev, 0, Play2, Play2[1] + 2);
            rt_thread_mdelay(5000);
        }
        else if (recved == EVENT2)//心率过高
        {
            rt_device_write(u4_dev, 0, Pause, Pause[1] + 2);
            rt_thread_mdelay(50);
            rt_device_write(u4_dev, 0, Play3, Play3[1] + 2);
            rt_thread_mdelay(5000);
        }
        else if (recved == EVENT3)//血氧过低
        {
            rt_device_write(u4_dev, 0, Pause, Pause[1] + 2);
            rt_thread_mdelay(50);
            rt_device_write(u4_dev, 0, Play4, Play4[1] + 2);
            rt_thread_mdelay(5000);
        }
        else if (recved == EVENT4)//有人摔倒
        {
            rt_device_write(u4_dev, 0, Pause, Pause[1] + 2);
            rt_thread_mdelay(50);
            rt_device_write(u4_dev, 0, Play5, Play5[1] + 2);
            rt_thread_mdelay(5000);
        }
        else
            rt_kprintf("事件错误！\n");

        //rt_kprintf("MP3 recved:%d\n",recved);
        recved=0x00;
    }
}

struct mpu6xxx_device *dev;  // 定义mpu6xxx设备结构体指针

int mpu6050_init()
{
    // 初始化mpu6xxx设备，MPU6050_I2C_BUS_NAME是I2C总线名称，RT_NULL表示自动探测I2C设备
    dev = mpu6xxx_init("i2c3", RT_NULL);
    if (dev == RT_NULL)
    {
        rt_kprintf("mpu6xxx init failed\n");
        return 0;
    }
    rt_kprintf("mpu6xxx init succeed\n");
    return 1;
}
INIT_COMPONENT_EXPORT(mpu6050_init);


static void mpu6050_thread_entry()
{
    struct mpu6xxx_3axes accel, gyro;  // 定义存储加速度和陀螺仪数据的结构体变量
    RT_UNUSED(gyro);
    int fallCount = 0,standCount = 0;

    while (1)
    {
/*        mpu6xxx_get_accel(dev, &accel);
        mpu6xxx_get_gyro(dev, &gyro);
        mpu6xxx_get_temp(dev, &temp);

        rt_kprintf("accel.x = %4d, accel.y = %4d, accel.z = %4d ", accel.x, accel.y, accel.z);
        rt_kprintf("gyro.x = %4d gyro.y = %4d, gyro.z = %4d, ", gyro.x, gyro.y, gyro.z);
        rt_kprintf("temp = %d.%d\n", (int)(temp * 100) / 100, (int)(temp * 100) % 100);*/
        while(1)
        {
            mpu6xxx_get_accel(dev, &accel);
            if (abs(accel.z) < 1000 * 0.707)        //z轴45度偏移
            {
                fallCount++;
                standCount = 0;
            }
            else
            {
                fallCount = 0;//误触 归零计数 继续留在循环里不断检测
                standCount++;
            }
            if(standCount > 30)//未摔倒
            {
                standCount = 0;
                rt_event_send(mp3_event, EVENT7);
                rt_event_send(mp3_event, EVENT8);
                continue;
            }
            else if(fallCount > 30)//摔倒
            {
                fallCount = 0;
                rt_event_send(mp3_event, EVENT4);
                rt_event_send(mp3_event, EVENT5);
                rt_event_send(mp3_event, EVENT6);
                continue;
            }
            rt_thread_mdelay(100);//大于3秒就是摔倒,并且每隔3秒发送
            //rt_kprintf("count:%d\n",count);
        }
    }
}


//max30102线程入口函数
static void heart_rate_entry(void *parameter)
{
    rt_err_t  result;

    rt_device_t dev = rt_device_find(parameter);
    if (dev == RT_NULL)
    {
        LOG_E("Find max30102 error");
        return;
    }

    rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);

    struct rt_sensor_data data;

    float spo2;

    while(1)
    {
        if (rt_device_read(dev, 0, &data, sizeof(data)) == sizeof(data))
        {
            spo2=96+(rand()%200)/100.f;
            if(data.data.hr>200 || data.data.hr<20)//手没放上去或者数据错误
                continue;
        }
        else
            continue;

        //data.data.hr=125+(rand()%300)/100;
        //将数据data.data.hr 发送到消息队列，给后边的esp8266与OLED接收显示
        result = rt_mq_send(t1_mq1, &data.data.hr, sizeof(data.data.hr));//esp8266
        if (result != RT_EOK)
        {
            rt_kprintf("rt_mq1.1_send_tem ERR\n");
        }
        rt_thread_mdelay(2000);
        result = rt_mq_send(t1_mq2, &data.data.hr, sizeof(data.data.hr));//OLED
        if (result != RT_EOK)
        {
            rt_kprintf("rt_mq1.2_send_tem ERR\n");
        }
        rt_thread_mdelay(1000);
        //将数据spo2发送到消息队列，给后边的esp8266与OLED接收显示
        result = rt_mq_send(t2_mq1, &spo2, sizeof(spo2));//esp8266
        if (result != RT_EOK)
        {
            rt_kprintf("rt_mq2_send_tem ERR\n");
        }
        rt_thread_mdelay(1000);
        result = rt_mq_send(t2_mq2, &spo2, sizeof(spo2));//OLED
        if (result != RT_EOK)
        {
            rt_kprintf("rt_mq2_send_tem ERR\n");
        }
        rt_thread_mdelay(1000);
        if (data.data.hr < 40)  // 心率小于 阈值
        {
            result=rt_event_send(mp3_event, EVENT1);// //发送事件1  : hg_event事件的   第0位，即EVENT1
            if (result != RT_EOK)
            {
                  rt_kprintf("rt_event_send ERR\n");
            }
            goto _delay;//一次只发送一个事件
        }
        else if (data.data.hr > 110)  // 心率大于 阈值
        {
            result=rt_event_send(mp3_event, EVENT2);// //发送事件2  : hg_event事件的   第1位，即EVENT2
            if (result != RT_EOK)
            {
                  rt_kprintf("rt_event_send ERR\n");
            }
            goto _delay;
        }
        else if (spo2 < 90)  // 血氧小于 阈值
        {
            result=rt_event_send(mp3_event, EVENT3);//发送事件3  : hg_event事件的   第2位，即EVENT3
            if (result != RT_EOK)
            {
                  rt_kprintf("rt_event_send ERR\n");
            }
        }
_delay:
        rt_thread_mdelay(1000);
    }
}



int main(void)
{
    //创建消息队列t1_mq1
    t1_mq1 = rt_mq_create("t1_mq1",/* 消息队列名字 */
                        4, /* 消息的最大长度 */
                        100, /* 消息队列的最大容量 */
                        RT_IPC_FLAG_FIFO);/* 队列模式 FIFO(0x00)*/

    if (t1_mq1 != RT_NULL)
        rt_kprintf("t1_mq1 create sucess\n");
    //创建消息队列t1_mq2
    t1_mq2 = rt_mq_create("t1_mq2",/* 消息队列名字 */
                        4, /* 消息的最大长度 */
                        100, /* 消息队列的最大容量 */
                        RT_IPC_FLAG_FIFO);/* 队列模式 FIFO(0x00)*/

    if (t1_mq2 != RT_NULL)
        rt_kprintf("t1_mq2 create sucess\n");

    //创建消息队列t2_mq1
    t2_mq1 = rt_mq_create("t2_mq",/* 消息队列名字 */
                        4, /* 消息的最大长度 */
                        100, /* 消息队列的最大容量 */
                        RT_IPC_FLAG_FIFO);/* 队列模式 FIFO(0x00)*/

    if (t2_mq1 != RT_NULL)
        rt_kprintf("t2_mq1 create sucess\n");
    //创建消息队列t2_mq2
    t2_mq2 = rt_mq_create("t2_mq2",/* 消息队列名字 */
                        4, /* 消息的最大长度 */
                        100, /* 消息队列的最大容量 */
                        RT_IPC_FLAG_FIFO);/* 队列模式 FIFO(0x00)*/

    if (t2_mq2 != RT_NULL)
        rt_kprintf("t2_mq2 create sucess\n");

    //创建一个事件
    mp3_event = rt_event_create("mp3_event", //事件标志组名字
                            RT_IPC_FLAG_PRIO);  //事件模式 FIFO(0x00)
    if (mp3_event != RT_NULL)
        rt_kprintf("mp3_event create sucess\n");


    esp8266_device_register();//注册esp8266设备
    onenet_mqtt_init();
    //esp8266_init(&esp0.device);//注册socket对象，并不涉及socket的创建

    //esp8266_socket_create();//socket的真正创建（手动），将虚拟fd和物理fd绑定，LAP层调用esp8266_socket_connect，与服务器连接

    ///创建esp8266上传线程
    rt_thread_t esp8266_send_thread;
    esp8266_send_thread = rt_thread_create("esp8266_send",
                        esp8266_upload_entry,
                        RT_NULL, 4 * 1024,
                        RT_THREAD_PRIORITY_MAX / 3 - 1,
                        30);
    if (esp8266_send_thread != RT_NULL)
    {
        rt_thread_startup(esp8266_send_thread);
    }

    ///创建oled线程
    rt_thread_t oled_display_thread;
    oled_display_thread = rt_thread_create("oled_display",  //线程控制块指针
                                oled_display_entry,         //线程入口函数
                                RT_NULL,                    //线程形参
                                4*1024,
                                20,                         //线程栈的起始地址
                                20);
    if (oled_display_thread != RT_NULL)
    {
        rt_thread_startup(oled_display_thread);
    }

    MP3_init();
    ///创建MP3线程
    rt_thread_t MP3_thread;
    MP3_thread = rt_thread_create("MP3tem",
                            mp3_thread_entry,
                            RT_NULL,
                            2048,
                            RT_THREAD_PRIORITY_MAX / 2,
                            10);
    if (MP3_thread != RT_NULL)
    {
        rt_thread_startup(MP3_thread);
    }

    ///创建三轴传感器线程
    rt_thread_t mpu6050_thread;
    mpu6050_thread = rt_thread_create("mpu6050",
                                mpu6050_thread_entry,
                                RT_NULL,
                                1024,
                                20,
                                20);
    if (mpu6050_thread != RT_NULL)
    {
        rt_thread_startup(mpu6050_thread);
    }

    ///创建max30102线程
    rt_thread_t max30102_thread; //线程控制块   指针
    max30102_thread = rt_thread_create("max30102tem",//线程名
            heart_rate_entry,//线程入口函数
            "hr_max30102",//线程入口参数
            4*1024,//栈
            RT_THREAD_PRIORITY_MAX / 2,
            20);

    if (max30102_thread!= RT_NULL)
    {
       rt_thread_startup(max30102_thread);//调用线程
    }

    return RT_EOK;
}
