/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-03     dream       the first version
 */
// state_machine_thread.c
#include "rtthread.h"
#include "state_machine_thread.h"
#include "tim.h"
#include "hrtim.h"
#include "main.h"
#include "define.h"

// 状态机线程入口
static void state_machine_thread_entry(void* parameter)
{
    rt_uint32_t recv_event;
    struct _ADC_Data *adc_data;
    
    while (1)
    {
        // 等待事件，接收ADC数据就绪事件或其他事件
        if (rt_event_recv(system_event, 
                         EVENT_ADC_DATA_READY | EVENT_MODE_CHANGE | EVENT_ERROR_OCCUR,
                         RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                         RT_WAITING_FOREVER, &recv_event) == RT_EOK)
        {
            // 如果是ADC数据就绪事件
            if (recv_event & EVENT_ADC_DATA_READY)
            {
                // 从消息队列接收ADC数据
                if (rt_mq_recv(adc_data_mq, &adc_data, sizeof(struct _ADC_Data*), RT_WAITING_NO) == RT_EOK)
                {
                    // 处理ADC数据
                    // 状态机主逻辑
                    StateM();
                    OVP();
                    OCP();
                    OTP();
                    BBMode(); // 模式切换检测
                    
                    // 释放内存池中的数据块
                    rt_mp_free(adc_data);
                }
            }
            
            // 如果是错误事件
            if (recv_event & EVENT_ERROR_OCCUR)
            {
                // 处理错误
                // ...
            }
            
            // 如果是模式变化事件
            if (recv_event & EVENT_MODE_CHANGE)
            {
                // 处理模式变化
                // ...
            }
            
            // 检查是否需要写入Flash
            if (SET_Value.SET_modified_flag)
            {
                rt_mutex_take(set_value_mutex, RT_WAITING_FOREVER);
                // 发送Flash写入事件
                rt_event_send(system_event, EVENT_FLASH_WRITE_READY);
                SET_Value.SET_modified_flag = 0;
                rt_mutex_release(set_value_mutex);
            }
        }
        
        rt_thread_mdelay(5);
    }
}

void create_state_machine_thread(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);                     // 开启HRTIM波形计数器
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_F);                     // 开启HRTIM波形计数器
    __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIM_IT_REP);

    rt_thread_t thread = rt_thread_create("state_machine",
                                          state_machine_thread_entry,
                                          RT_NULL,
                                          STATE_MACHINE_STACK_SIZE,
                                          STATE_MACHINE_PRIO,
                                          10);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
}

