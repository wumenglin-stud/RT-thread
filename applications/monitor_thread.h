#ifndef APPLICATIONS_MONITOR_THREAD_H_
#define APPLICATIONS_MONITOR_THREAD_H_

#include "rtthread.h"

// 监控线程栈大小
#define MONITOR_THREAD_STACK_SIZE 1024
// 监控线程优先级
#define MONITOR_THREAD_PRIO 15

// 系统运行状态结构体
typedef struct {
    uint32_t run_time;           // 系统运行时间(秒)
    uint32_t error_count;        // 错误计数
    uint32_t mode_switch_count;  // 模式切换计数
    uint32_t adc_sample_count;   // ADC采样计数
    float max_temp;              // 最高温度记录
    float min_vin;               // 最低输入电压
    float max_vout;              // 最高输出电压
    float max_iout;              // 最大输出电流
    float avg_efficiency;        // 平均效率
} sys_monitor_t;

// 系统监控数据
extern sys_monitor_t sys_monitor;

// 创建系统监控线程
void create_monitor_thread(void);

// 重置系统监控数据
void reset_monitor_data(void);

// 记录错误事件
void record_error_event(uint16_t error_code);

// 记录模式切换事件
void record_mode_switch_event(uint8_t old_mode, uint8_t new_mode);

#endif /* APPLICATIONS_MONITOR_THREAD_H_ */ 