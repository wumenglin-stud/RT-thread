/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-04     dream       the first version
 */
#ifndef APPLICATIONS_PID_H_
#define APPLICATIONS_PID_H_

// 环路的参数buck输出-恒压-PID型补偿器
#define BUCKPIDb0 5271
#define BUCKPIDb1 -10363
#define BUCKPIDb2 5093
// 环路的参数BOOST输出-恒压-PID型补偿器
#define BOOSTPIDb0 8044
#define BOOSTPIDb1 -15813
#define BOOSTPIDb2 7772

#define ILOOP_KP 6 // 电流环PID补偿器P值
#define ILOOP_KI 3 // 电流环PID补偿器I值
#define ILOOP_KD 1 // 电流环PID补偿器D值
#include "state_machine_thread.h"
#include <adc_thread.h>
#include <stdint.h>
#include <hrtim.h>
#define PERIOD 30000
#define CCMRAM __attribute__((section("ccmram")))
void PID_Init(void);
void BuckBoostVILoopCtlPID(void);
// 控制参数结构体



#endif /* APPLICATIONS_PID_H_ */
