#include <rtthread.h>
#include <define.h>

static void fan_thread_entry(void *parameter)
{
    while (1)
    {
        Auto_FAN();
        rt_thread_mdelay(100); // 100ms周期
    }
}

void create_fan_thread(void)
{
    rt_thread_t tid = rt_thread_create("fan", fan_thread_entry, RT_NULL, 512, 15, 10);
    if (tid)
        rt_thread_startup(tid);
} 

void Auto_FAN(void)
{
    float TEMP = GET_NTC_Temperature(); // 获取NTC温度值
    if (TEMP < 35)
    {
        FAN_PWM_set(0); // 设置风扇转速为0
    }
    else if (TEMP >= 35)
    {
        FAN_PWM_set(35); // 设置风扇转速为35%
    }
    else if (TEMP >= 40)
    {
        FAN_PWM_set(45);
    }
    else if (TEMP >= 45)
    {
        FAN_PWM_set(60);
    }
    else if (TEMP >= 50)
    {
        FAN_PWM_set(70);
    }
    else if (TEMP >= 55)
    {
        FAN_PWM_set(80);
    }
    else if (TEMP >= 60)
    {
        FAN_PWM_set(90);
    }
    else if (TEMP >= 65)
    {
        FAN_PWM_set(100);
    }
}
