/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-08     dream       the first version
 */
#ifndef APPLICATIONS_DEFINE_H_
#define APPLICATIONS_DEFINE_H_

#include "rtthread.h"
#include "main.h"
#include "stdio.h"

// 添加内存池相关定义
#define ADC_DATA_POOL_SIZE 10
#define ADC_DATA_BLOCK_SIZE sizeof(struct _ADC_Data)

// 内存池和消息队列声明
extern rt_mp_t adc_data_mp;
extern rt_mq_t adc_data_mq;

// 添加事件集定义
extern rt_event_t system_event;

// 事件标志位定义
#define EVENT_ADC_DATA_READY    (1 << 0)
#define EVENT_FLASH_WRITE_READY (1 << 1)
#define EVENT_TEMP_ALARM        (1 << 2)
#define EVENT_MODE_CHANGE       (1 << 3)
#define EVENT_ERROR_OCCUR       (1 << 4)

/*
 * 定义一个宏 CCMRAM，用于将函数或变量指定到CCM RAM段。
 * 使用此宏的声明将会被编译器放置在CCM（Cacheable Memory）RAM区域中。
 * 这对于需要快速访问且不被系统缓存机制影响的变量或函数非常有用。
 */
#define CCMRAM __attribute__((section("ccmram")))

/* 常用宏定义 */
#define ADC_MAX_VALUE 8190.0F                  // ADC最大值
#define REF_3V3 3.3006F                        // VREF参考电压
#define TS_CAL1 *((__IO uint16_t *)0x1FFF75A8) // 内部温度传感器在30度和VREF为3V时的校准数据
#define TS_CAL2 *((__IO uint16_t *)0x1FFF75CA) // 内部温度传感器在130度和VREF为3V时的校准数据
#define TS_CAL1_TEMP 30.0F
#define TS_CAL2_TEMP 130.0F

#define MIN_BUKC_DUTY 100     // BUCK最小占空比
#define MAX_BUCK_DUTY 28200   // BUCK最大占空比94%
#define MAX_BUCK_DUTY1 24000  // MIX模式下 BUCK固定占空比80%
#define MIN_BOOST_DUTY 100    // BOOST最小占空比
#define MIN_BOOST_DUTY1 1800  // BOOST最小占空6%
#define MAX_BOOST_DUTY 19500  // BOOST工作模式下最大占空比65%
#define MAX_BOOST_DUTY1 28200 // BOOST最大占空比94%

#define CAL_VOUT_K 4099 // 输出电压矫正K值
#define CAL_VOUT_B 1    // 输出电压矫正B值
#define CAL_IOUT_K 4095 // 输出电流矫正K值
#define CAL_IOUT_B 1    // 输出电流矫正B值

/***************故障类型*****************/
#define F_NOERR 0x0000       // 无故障
#define F_SW_VIN_UVP 0x0001  // 输入欠压
#define F_SW_VIN_OVP 0x0002  // 输入过压
#define F_SW_VOUT_UVP 0x0004 // 输出欠压
#define F_SW_VOUT_OVP 0x0008 // 输出过压
#define F_SW_IOUT_OCP 0x0010 // 输出过流
#define F_SW_SHORT 0x0020    // 输出短路
#define F_OTP 0x0040         // 温度过高

#define MAX_SHORT_I 10.1F    // 短路电流判据
#define MIN_SHORT_V 0.5F     // 短路电压判据
#define STATE_MACHINE_STACK_SIZE 2048

/* 寄存器操作宏 */
#define setRegBits(reg, mask) (reg |= (unsigned int)(mask))
#define clrRegBits(reg, mask) (reg &= (unsigned int)(~(unsigned int)(mask)))
#define getRegBits(reg, mask) (reg & (unsigned int)(mask))
#define getReg(reg) (reg)

/* 全局变量声明 */
extern volatile int32_t VErr0, VErr1, VErr2; // 电压误差
extern volatile int32_t IErr0, IErr1;        // 电流误差
extern volatile int32_t u0, u1;              // 电压环输出量
extern volatile int32_t i0, i1;              // 电流环输出量

/* 结构体定义 */
struct _ADI {
    volatile uint32_t Iout;    // 输出电流
    volatile uint32_t IoutAvg; // 输出电流平均值
    volatile uint32_t Vout;    // 输出电压
    volatile uint32_t VoutAvg; // 输出电压平均值
    volatile uint32_t Iin;     // 输入电流
    volatile uint32_t IinAvg;  // 输入电流平均值
    volatile uint32_t Vin;     // 输入电压
    volatile uint32_t VinAvg;  // 输入电压平均值
};

struct _SET_Value {
    volatile float SET_modified_flag; // 设置被修改标志位
    volatile float Vout;              // 输出电压设置值
    volatile float Iout;              // 输出电流设置值
    volatile uint8_t currentSetting;  // 当前设置项标志位
    volatile uint8_t SET_bit;         // 当前设置位标志位
};

struct _Ctr_value {
    volatile int32_t Vout_ref;     // 输出参考电压
    volatile int32_t Vout_SSref;   // 软启动时的输出参考电压
    volatile int32_t Vout_SETref;  // 设置的参考电压
    volatile int32_t Iout_ref;     // 输出参考电流
    volatile int32_t I_Limit;      // 限流参考电流
    volatile int16_t BUCKMaxDuty;  // Buck最大占空比
    volatile int16_t BoostMaxDuty; // Boost最大占空比
    volatile int16_t BuckDuty;     // Buck控制占空比
    volatile int16_t BoostDuty;    // Boost控制占空比
    volatile int32_t Ilimitout;    // 电流环输出
};

struct _FLAG {
    volatile uint16_t SMFlag;      // 状态机标志位
    volatile uint16_t CtrFlag;     // 控制标志位
    volatile uint16_t ErrFlag;     // 故障标志位
    volatile uint8_t BBFlag;       // 运行模式标志位
    volatile uint8_t PWMENFlag;    // 启动标志位
    volatile uint8_t BBModeChange; // 工作模式切换标志位
    volatile uint8_t OUTPUT_Flag;  // 输出开关标志位
};

/* 枚举类型定义 */
typedef enum {
    Init, Wait, Rise, Run, Err
} State;

typedef enum {
    SSInit, // 软启初始化
    SSWait, // 软启等待
    SSRun   // 开始软启
} SState_M;

typedef enum {
    NA,    // 未定义
    Buck,  // BUCK模式
    Boost, // BOOST模式
    Mix    // MIX混合模式
} BB_M;

typedef enum {
    VIset_page = 1, // 电压电流设置页面
    DATA1_page,     // 数据显示页面1
    DATA2_page,     // 数据显示页面2
    SET_page        // 设置页面
} _Screen_page;

typedef enum {
    CV, // 恒压模式
    CC  // 恒流模式
} _CVCC_Mode;

/* 全局变量声明 */
extern volatile uint16_t adc_dma_buffer[4];
extern volatile uint16_t ADC1_RESULT[4];
extern volatile uint8_t BUZZER_Short_Flag;
extern volatile uint8_t BUZZER_Flag;
extern volatile uint8_t BUZZER_Middle_Flag;
extern struct _ADI SADC;
extern struct _FLAG DF;
extern struct _Ctr_value CtrValue;
extern struct _SET_Value SET_Value;
extern volatile float MAX_OTP_VAL;
extern volatile float MAX_VOUT_OVP_VAL;
extern  volatile float MAX_VOUT_OCP_VAL;
extern SState_M STState;
extern  volatile float VIN, VOUT, IIN, IOUT;
extern volatile float MainBoard_TEMP, CPU_TEMP;
extern volatile float powerEfficiency;
extern volatile _CVCC_Mode CVCC_Mode;

/* RT-Thread 同步对象 */
extern rt_sem_t adc_data_ready;
extern rt_sem_t flash_write_ready;
extern rt_mutex_t df_mutex;
extern rt_mutex_t set_value_mutex;
extern rt_mutex_t flash_mutex;

// 添加ADC数据结构体定义
struct _ADC_Data {
    float vin;
    float vout;
    float iin;
    float iout;
    float temp_mb;
    float temp_cpu;
};

float GET_NTC_Temperature(void);
void ADC_calculate(void);
void StateM(void);
void StateMInit(void);
void StateMWait(void);
void StateMRise(void);
void StateMRun(void);
void StateMErr(void);
void ValInit(void);
void OTP(void);
void OVP(void);
void OCP(void);
void ShortOff(void);
void BBMode(void);
void BUZZER_Short(void);
void BUZZER_Middle(void);
float GET_CPU_Temperature(void);
void FAN_PWM_set(uint16_t pwm);
void float_to_bytes(float value, uint8_t *bytes);
float bytes_to_float(uint8_t *bytes);
void Auto_FAN(void);
float GET_NTC_Temperature(void);
#endif /* APPLICATIONS_DEFINE_H_ */

