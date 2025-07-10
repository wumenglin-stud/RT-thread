/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-02     dream       the first version
 */
#ifndef APPLICATIONS_ADC_THREAD_H_
#define APPLICATIONS_ADC_THREAD_H_

#include "rtthread.h"
#include "adc.h"

// ADC线程栈大小
#define ADC_THREAD_STACK_SIZE 1024
// ADC线程优先级，设置为仅次于状态机的优先级
#define ADC_THREAD_PRIO 6

void create_adc_thread(void);
void ADC_calculate(void);

#define CAL_VOUT_K 4099 // 输出电压矫正K值
#define CAL_VOUT_B 1    // 输出电压矫正B值
#define CAL_IOUT_K 4095 // 输出电流矫正K值
#define CAL_IOUT_B 1    // 输出电流矫正B值
#define ADC_MAX_VALUE 8190.0F
#define REF_3V3 3.3006F //参考电压

#endif /* APPLICATIONS_ADC_THREAD_H_ */
