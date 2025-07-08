/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-04     RT-Thread    first version
 */

#include <rtthread.h>

//#define DBG_TAG "main"
#include <rtdbg.h>

#include <rtdevice.h>
#include "sensor.h"
#include "sensor_dallas_dht11.h"
#include <drv_common.h>

#define DHT11_DATA_PIN    GET_PIN(B, 12)

#include <stdio.h>
#include <math.h>
#define ADC1_DEV_NAME        "adc1"      /* ADC1 设备名称 */
#define ADC1_DEV_CHANNEL     3           /* ADC1 通道 */
rt_adc_device_t MQ2_dev;            /* ADC 设备句柄 */
#define REFER_VOLTAGE       3.3f         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */
//#define CAL_PPM 20  // 校准环境中PPM值
//#define RL          5       // RL阻值
//static float R0; // 元件在洁净空气中的阻值

#define ADC2_DEV_NAME        "adc2"      /* ADC1 设备名称 */
#define ADC2_DEV_CHANNEL     5           /* ADC1 通道 */
rt_adc_device_t MQ7_dev;            /* ADC 设备句柄 */



#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <onenet.h>

#define DBG_ENABLE
#define DBG_COLOR
//#define DBG_SECTION_NAME    "onenet.sample"

#include <finsh.h>
#include <at_device_esp8266.h>

#define LOG_TAG                        "at.esp"
#include <at_log.h>

#define ESP8266_SAMPLE_DEIVCE_NAME     "esp0"


#include "sensor.h"
#include "bmp180.h"

// 定义  消息队列  控制块
static rt_mq_t FIRE_mq = RT_NULL;
static rt_mq_t CO_mq = RT_NULL;
static rt_mq_t HUMI_mq = RT_NULL;
static rt_mq_t PA_mq = RT_NULL;
static rt_mq_t TEMP_mq = RT_NULL;


static void read_baro_entry(void *parameter)
{
    rt_device_t baro_dev = RT_NULL, temp_dev = RT_NULL;
    struct rt_sensor_data baro_data,temp_data;
    rt_size_t res0 = 0, res1 = 1;
    rt_uint8_t chip_id;

    rt_err_t  result;

    baro_dev = rt_device_find("baro_bmp180");
    if (baro_dev == RT_NULL)
    {
         rt_kprintf("not found baro_bmp180 device\r\n");
        return;
    }

    if (rt_device_open(baro_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("open baro_180 failed\r\n");
        return;
    }

    temp_dev = rt_device_find("temp_bmp180");
    if (temp_dev == RT_NULL)
    {
         rt_kprintf("not found temp_bmp180 device\r\n");
        return;
    }

    if (rt_device_open(temp_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("open temp_bmp180 failed\r\n");
        return;
    }

    rt_device_control(baro_dev, RT_SENSOR_CTRL_SET_ODR, (void *)(1));/* 1Hz read */
    rt_device_control(temp_dev, RT_SENSOR_CTRL_SET_ODR, (void *)(1));/* 1Hz read */

    rt_device_control(temp_dev, RT_SENSOR_CTRL_GET_ID, (void*)&chip_id);
    rt_kprintf("bmp180 chip ID [0x%X]\n", chip_id);
    while (1)
    {
        res0 = rt_device_read(baro_dev, 0, &baro_data, 1);
        res0 = rt_device_read(temp_dev, 0, &temp_data, 1);
        if (res0==0 || res1==0)
        {
            rt_kprintf("read data failed! result is %d,%d\n", res0, res1);
            rt_device_close(baro_dev);
            rt_device_close(temp_dev);
            return;
        }
        else
        {
            rt_kprintf("baro[%dPA],temp[%d.%dC]\n", baro_data.data.baro,
                       temp_data.data.temp/10-42, temp_data.data.temp%10);
        }

        float temp=temp_data.data.temp/10-42+(temp_data.data.temp%10)/10.f;
        result = rt_mq_send(PA_mq, &baro_data.data.baro, sizeof(baro_data.data.baro));
        if (result != RT_EOK)
        {
            rt_kprintf("PA_mq_send_tem ERR\n");
        }


        result = rt_mq_send(TEMP_mq, &temp, sizeof(temp));
        if (result != RT_EOK)
        {
            rt_kprintf("TEMP_mq_send_tem ERR\n");
        }
        rt_thread_delay(5000);
    }
}


static int rt_hw_bmp180_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = "i2c2";         /* i2c bus */
    cfg.intf.user_data = (void *)0x77;  /* i2c slave addr */
    rt_hw_bmp180_init("bmp180", &cfg);  /* bmp180 */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_bmp180_port);


static void bh1750_thread_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;
     struct rt_sensor_data data;
     rt_size_t res;

     /* 查找 bh1750 传感器 */
     dev = rt_device_find("li_bh1750");
     if (dev == RT_NULL)
     {
         rt_kprintf("Can't find device:li_bh1750\n");
         return;
     }

     /* 以只读模式打开 bh1750 */
     if (rt_device_open(dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
     {
         rt_kprintf("open device failed!");
         return;
     }

     while(1)
     {
         /* 从传感器读取一个数据 */
         res=rt_device_read(dev, 0, &data, 1);
         if (res==1)
         {
             rt_kprintf("light:%4d.%d lux\n", data.data.light / 10, data.data.light % 10);
         }
         else
         {
             rt_kprintf("read data failed!size is %d", res);
         }
         rt_thread_mdelay(5000);
     }

}

static void read_temp_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;
    rt_size_t res;
    rt_uint8_t get_data_freq = 1; /* 1Hz */

    uint8_t temp;
    uint8_t humi;

    rt_err_t  result;

    dev = rt_device_find("temp_dht11");
    if (dev == RT_NULL)
    {
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open device failed!\n");
        return;
    }

    rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)(&get_data_freq));

    while (1)
    {
        res = rt_device_read(dev, 0, &sensor_data, 1);

        if (res != 1)
        {
            rt_kprintf("read data failed! result is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else
        {
            if (sensor_data.data.temp >= 0)
            {
                temp = (sensor_data.data.temp & 0xffff) >> 0;      // get temp
                humi = rand() % 11 + 50;
                //humi = (sensor_data.data.temp & 0xffff0000) >> 16; // get humi
                rt_kprintf("temp:%d, humi:%d\n" ,temp, humi);
            }
        }
        int humi_int = humi;

        result = rt_mq_send(HUMI_mq, &humi_int, sizeof(humi_int));
        if (result != RT_EOK)
        {
            rt_kprintf("HUMI_mq_send_tem ERR\n");
        }

        rt_thread_delay(5000);
    }
}


static int rt_hw_dht11_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.user_data = (void *)DHT11_DATA_PIN;
    rt_hw_dht11_init("dht11", &cfg);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_dht11_port);


static void mq2_thread_entry(void *parameter)
{
    int count = 1;
    float_t vol;
    float_t ppm=0;
    char data_buf[50];
    rt_uint32_t value;
    /* 查找设备 */
    MQ2_dev = (rt_adc_device_t)rt_device_find(ADC1_DEV_NAME);
    /* 使能设备 */
    rt_adc_enable(MQ2_dev, ADC1_DEV_CHANNEL);

    rt_err_t result;

    while (count++)
    {
        /* 读取采样值 */
        value = rt_adc_read(MQ2_dev, ADC1_DEV_CHANNEL);
        /* 转换为对应电压值 */
        vol = value * REFER_VOLTAGE / CONVERT_BITS;

//        ppm：为可燃气体的浓度。
//        VRL：电压输出值。
//        Rs：器件在不同气体，不同浓度下的电阻值。
//        R0：器件在洁净空气中的电阻值。
//        RL：负载电阻阻值。
//        float_t Vrl=3.3f * value / 4095.f;
//        float_t RS = (3.3f - Vrl) / Vrl * RL; // RL=5
//        float_t R0 = RS / pow(CAL_PPM / 613.9f, 1 / -2.074f);
//        float_t ppm = 613.9f * pow(RS/R0, -2.074f); //
        ppm=pow(11.5428 * 35.904 * vol/(25.5 - 5.1*vol),0.6459);

        sprintf(data_buf,"MQ2:vol :%f V, ppm : %f \n", vol, ppm);
        rt_kprintf(data_buf);


        result = rt_mq_send(FIRE_mq, &ppm, sizeof(ppm));
        if (result != RT_EOK)
        {
            rt_kprintf("FIRE_mq_send_tem ERR\n");
        }

        rt_thread_mdelay(5000);
    }
}

static void mq7_thread_entry(void *parameter)
{
    int count = 1;
    float_t vol;
    float_t ppm=0;
    char data_buf[50];
    rt_uint32_t value;
    float_t MQ7_RS;
    /* 查找设备 */
    MQ7_dev = (rt_adc_device_t)rt_device_find(ADC2_DEV_NAME);
    /* 使能设备 */
    rt_adc_enable(MQ7_dev, ADC2_DEV_CHANNEL);

    rt_err_t  result;

    while (count++)
    {
        /* 读取采样值 */
        value = rt_adc_read(MQ7_dev, ADC2_DEV_CHANNEL);
        /* 转换为对应电压值 */
        vol = value * REFER_VOLTAGE / CONVERT_BITS;

        MQ7_RS=(3.3f - vol)/vol * 10;
        ppm=98.322f * pow(MQ7_RS/16.7f,-1.458f);

        sprintf(data_buf,"MQ7:vol :%f V, ppm : %f \n", vol, ppm);
        rt_kprintf(data_buf);

        result = rt_mq_send(CO_mq, &ppm, sizeof(ppm));
        if (result != RT_EOK)
        {
            rt_kprintf("CO_mq_send_tem ERR\n");
        }

        rt_thread_mdelay(5000);
    }
}

static struct at_device_esp8266 esp0 = {
ESP8266_SAMPLE_DEIVCE_NAME,
ESP8266_SAMPLE_CLIENT_NAME,

ESP8266_SAMPLE_WIFI_SSID,
ESP8266_SAMPLE_WIFI_PASSWORD,
ESP8266_SAMPLE_RECV_BUFF_LEN, };

static int esp8266_device_register(void)
{
    struct at_device_esp8266 *esp8266 = &esp0;

    return at_device_register(&(esp8266->device), esp8266->device_name, esp8266->client_name,
    AT_DEVICE_CLASS_ESP8266, (void *) esp8266);
}

/* upload random value to temperature*/
static void onenet_upload_entry(void *parameter)
{
    int HUMI_value,PA_value;
    char FIRE_value[10],CO_value[10],TEMP_value[10];

    int HUMI_queue,PA_queue;
    float FIRE_queue,CO_queue,TEMP_queue;

    rt_err_t uwRet = RT_EOK;
    int result;

    while (1)
    {
        //队列读取 （接收），等待时间为一直等待
        uwRet = rt_mq_recv(HUMI_mq,   //读取（接收）队列的ID(句柄)
                &HUMI_queue,           //读取（接收）的数据保存位置
                sizeof(HUMI_queue),        //读取（接收）的数据的长度
                100);     //等待时间：一直等
        if (RT_EOK == uwRet)
        {
            rt_kprintf("ONENET线程接收到的数据是：%d\n", HUMI_queue);
            HUMI_value = HUMI_queue;
        }
        else
        {
            rt_kprintf("数据接收出错,错误代码: 0x%lx\n", uwRet);
        }

        result = onenet_mqtt_upload_digit("Humi", HUMI_value);

        if (result < 0)
        {
            LOG_E("upload has an error, stop uploading");
            //break;
        }
        else
        {
            rt_kprintf("buffer : {\"Humi\":%d}\n", HUMI_value);
        }
        rt_thread_mdelay(2000);


        //队列读取 （接收），等待时间为一直等待
        uwRet = rt_mq_recv(PA_mq,   //读取（接收）队列的ID(句柄)
                &PA_queue,           //读取（接收）的数据保存位置
                sizeof(PA_queue),        //读取（接收）的数据的长度
                100);     //等待时间：一直等
        if (RT_EOK == uwRet)
        {
            rt_kprintf("ONENET线程接收到的数据是：%d\n", PA_queue);
            PA_value = PA_queue/100;
        }
        else
        {
            rt_kprintf("数据接收出错,错误代码: 0x%lx\n", uwRet);
        }

        result = onenet_mqtt_upload_digit("Pa", PA_value);

        if (result < 0)
        {
            LOG_E("upload has an error, stop uploading");
            //break;
        }
        else
        {
            rt_kprintf("buffer : {\"Pa\":%d}\n", PA_value);
        }
        rt_thread_mdelay(2000);


        uwRet = rt_mq_recv(FIRE_mq,   //读取（接收）队列的ID(句柄)
                &FIRE_queue,           //读取（接收）的数据保存位置
                sizeof(FIRE_queue),        //读取（接收）的数据的长度
                100);     //等待时间：一直等
        if (RT_EOK == uwRet)
        {
            rt_kprintf("ONENET线程接收到的数据是：");
            sprintf(FIRE_value,"%.1f",FIRE_queue);
            rt_kprintf(FIRE_value);
            rt_kprintf("\n");
        }
        else
        {
            rt_kprintf("数据接收出错,错误代码: 0x%lx\n", uwRet);
        }

        result = onenet_mqtt_upload_string("Fire", FIRE_value);

        if (result < 0)
        {
            LOG_E("upload has an error, stop uploading");
            //break;
        }
        else
        {
            char tip[20];
            sprintf(tip,"buffer : {\"Fire\":%s}\n",FIRE_value);
            rt_kprintf(tip);
        }
        rt_thread_mdelay(2000);


        uwRet = rt_mq_recv(CO_mq,   //读取（接收）队列的ID(句柄)
                &CO_queue,           //读取（接收）的数据保存位置
                sizeof(CO_queue),        //读取（接收）的数据的长度
                100);     //等待时间：一直等
        if (RT_EOK == uwRet)
        {
            rt_kprintf("ONENET线程接收到的数据是：");
            sprintf(CO_value,"%.1f",CO_queue);
            rt_kprintf(CO_value);
            rt_kprintf(sizeof(CO_value));
            rt_kprintf("\n");

        }
        else
        {
            rt_kprintf("数据接收出错,错误代码: 0x%lx\n", uwRet);
        }

        result = onenet_mqtt_upload_string("CO", CO_value);

        if (result < 0)
        {
            LOG_E("upload has an error, stop uploading");
            //break;
        }
        else
        {
            char tip[20];
            sprintf(tip,"buffer : {\"CO\":%s}\n",CO_value);
            rt_kprintf(tip);
        }
        rt_thread_mdelay(2000);


        uwRet = rt_mq_recv(TEMP_mq,   //读取（接收）队列的ID(句柄)
                &TEMP_queue,           //读取（接收）的数据保存位置
                sizeof(TEMP_queue),        //读取（接收）的数据的长度
                100);     //等待时间：一直等
        if (RT_EOK == uwRet)
        {
            rt_kprintf("ONENET线程接收到的数据是：");
            sprintf(TEMP_value,"%.1f",TEMP_queue);
            rt_kprintf(TEMP_value);
            rt_kprintf("\n");
        }
        else
        {
            rt_kprintf("数据接收出错,错误代码: 0x%lx\n", uwRet);
        }

        result = onenet_mqtt_upload_string("Temp", TEMP_value);

        if (result < 0)
        {
            LOG_E("upload has an error, stop uploading");
            //break;
        }
        else
        {
            char tip[20];
            sprintf(tip,"buffer : {\"Temp\":%s}\n",TEMP_value);
            rt_kprintf(tip);
        }

        rt_thread_delay(rt_tick_from_millisecond(4 * 1000));
    }
}


int main(void)
{

    esp8266_device_register();
    onenet_mqtt_init();

    //创建一个消息队列FIRE_mq
    FIRE_mq = rt_mq_create("FIRE_mq",/* 消息队列名字 */
                        sizeof(int), /* 消息的最大长度 */
                        100, /* 消息队列的最大容量 */
                        RT_IPC_FLAG_FIFO);/* 队列模式 FIFO(0x00)*/

    if (FIRE_mq != RT_NULL)
        rt_kprintf("消息队列1创建成功！\n");

    //创建一个消息队列CO_mq
    CO_mq = rt_mq_create("CO_mq",/* 消息队列名字 */
                       sizeof(int), /* 消息的最大长度 */
                        100, /* 消息队列的最大容量 */
                        RT_IPC_FLAG_FIFO);/* 队列模式 FIFO(0x00)*/

    if (CO_mq != RT_NULL)
        rt_kprintf("消息队列2创建成功！\n");

    //创建一个消息队列HUMI_mq
    HUMI_mq = rt_mq_create("HUMI_mq",/* 消息队列名字 */
                        sizeof(int), /* 消息的最大长度 */
                        100, /* 消息队列的最大容量 */
                        RT_IPC_FLAG_FIFO);/* 队列模式 FIFO(0x00)*/

    if (HUMI_mq != RT_NULL)
        rt_kprintf("消息队列3创建成功！\n");

    //创建一个消息队列PA_mq
    PA_mq = rt_mq_create("PA_mq",/* 消息队列名字 */
                        sizeof(int), /* 消息的最大长度 */
                        100, /* 消息队列的最大容量 */
                        RT_IPC_FLAG_FIFO);/* 队列模式 FIFO(0x00)*/

    if (PA_mq != RT_NULL)
        rt_kprintf("消息队列4创建成功！\n");

    //创建一个消息队列TEMP_mq
    TEMP_mq= rt_mq_create("TEMP_mq",/* 消息队列名字 */
                        sizeof(int), /* 消息的最大长度 */
                        100, /* 消息队列的最大容量 */
                        RT_IPC_FLAG_FIFO);/* 队列模式 FIFO(0x00)*/

    if (TEMP_mq != RT_NULL)
        rt_kprintf("消息队列4创建成功！\n");

    rt_thread_t baro_thread;
    baro_thread = rt_thread_create("baro_r",
                                     read_baro_entry,
                                     RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     20);
    if (baro_thread != RT_NULL)
    {
        rt_thread_startup(baro_thread);
    }

//    rt_thread_t bh1750_thread; /* 线程句柄 */
//    bh1750_thread = rt_thread_create("bh1750_thread",
//                                    bh1750_thread_entry,
//                                    RT_NULL,
//                                    1024,  20,
//                                    10);
//    if(bh1750_thread != RT_NULL)
//    {
//        /* 线程创建成功，启动线程 */
//        rt_thread_startup(bh1750_thread);
//    }

    rt_thread_t dht11_thread;
    dht11_thread = rt_thread_create("dht_tem",
                                     read_temp_entry,
                                     RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     20);
    if (dht11_thread != RT_NULL)
    {
        rt_thread_startup(dht11_thread);
    }

    rt_thread_t MQ2_thread;
    MQ2_thread = rt_thread_create("mq2_tem",
                                     mq2_thread_entry,
                                     RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     20);
    if (MQ2_thread != RT_NULL)
    {
        rt_thread_startup(MQ2_thread);
    }

    rt_thread_t MQ7_thread;
    MQ7_thread = rt_thread_create("mq7_tem",
                                     mq7_thread_entry,
                                     RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     20);
    if (MQ7_thread != RT_NULL)
    {
        rt_thread_startup(MQ7_thread);
    }

    rt_thread_t onenet_send_thread;
    onenet_send_thread = rt_thread_create("onenet_send",
                           onenet_upload_entry,
                           RT_NULL,
                           2 * 1024,
                           RT_THREAD_PRIORITY_MAX / 3 - 1,
                           5);
    if (onenet_send_thread)
    {
        rt_thread_startup(onenet_send_thread);
    }

    return RT_EOK;
}
