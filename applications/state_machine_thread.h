/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-03     dream       the first version
 */
#ifndef APPLICATIONS_STATE_MACHINE_THREAD_H_
#define APPLICATIONS_STATE_MACHINE_THREAD_H_

#include "rtthread.h"

// 状态机线程栈大小
#define STATE_MACHINE_STACK_SIZE 2048
// 状态机线程优先级，调整为更高优先级（RT-Thread优先级0-31，数字越小优先级越高）
#define STATE_MACHINE_PRIO 5

void create_state_machine_thread(void);

#endif /* APPLICATIONS_STATE_MACHINE_THREAD_H_ */
