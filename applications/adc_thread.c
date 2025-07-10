/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-02     dream       the first version
 */
// adc_thread.c
// adc_thread.c
#include "adc_thread.h"
#include <rtthread.h>
#include <adc.h>
#include <drv_common.h>
#include <main.h>
#include <define.h>

static void adc_thread_entry(void *parameter)
{
    struct _ADC_Data *adc_data;
    
    while (1)
    {
        // 从内存池分配一个ADC数据块
        adc_data = (struct _ADC_Data *)rt_mp_alloc(adc_data_mp, RT_WAITING_NO);
        
        if (adc_data != RT_NULL)
        {
            // 采集ADC数据
            ADC_calculate();
            
            // 填充ADC数据结构
            adc_data->vin = VIN;
            adc_data->vout = VOUT;
            adc_data->iin = IIN;
            adc_data->iout = IOUT;
            adc_data->temp_mb = MainBoard_TEMP;
            adc_data->temp_cpu = CPU_TEMP;
            
            // 发送数据到消息队列
            if (rt_mq_send(adc_data_mq, &adc_data, sizeof(struct _ADC_Data*)) != RT_EOK)
            {
                // 发送失败，释放内存
                rt_mp_free(adc_data);
            }
            
            // 发送事件通知状态机线程
            rt_event_send(system_event, EVENT_ADC_DATA_READY);
        }
        
        rt_thread_mdelay(10); // 10ms周期
    }
}

void create_adc_thread(void)
{
    rt_thread_t tid = rt_thread_create("adc", adc_thread_entry, RT_NULL, 
                                      ADC_THREAD_STACK_SIZE, 
                                      ADC_THREAD_PRIO, 10);
    if (tid)
        rt_thread_startup(tid);
}

