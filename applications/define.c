/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-08     dream       the first version
 */
#ifndef APPLICATIONS_DEFIND_C_
#define APPLICATIONS_DEFIND_C_

#include "define.h"
volatile uint8_t usart_dma_tx_over = 1;

 volatile uint16_t adc_dma_buffer[4] = {0, 0, 0, 0};
 volatile uint16_t ADC1_RESULT[4]= {0, 0, 0, 0};
 volatile int32_t VErr0 = 0, VErr1 = 0, VErr2 = 0; // 电压误差
 volatile  int32_t IErr0 = 0, IErr1 = 0;            // 电流误差
 volatile  int32_t u0 = 0, u1 = 0;                  // 电压环输出量
 volatile int32_t i0 = 0, i1 = 0;                  // 电流环输出量
         // 恒流恒压模式标志位

struct _ADI SADC = {0, 0, 0, 0, 0, 0, 0, 0};                                // 输出过流保护阈值
SState_M STState = SSInit;
struct _SET_Value SET_Value = {0, 0, 0, 0, 0};
struct _FLAG DF = {0, 0, 0, 0, 0, 0, 0};
struct _Ctr_value CtrValue = {0, 0, 0, 0, MIN_BUKC_DUTY, 0, 0, 0};
volatile float VIN, VOUT, IIN, IOUT; //实际值
volatile float MainBoard_TEMP, CPU_TEMP;                           // 主板和CPU温度实际值
volatile float powerEfficiency = 0;                                // 电源转换效率


// 信号量与互斥量定义
rt_sem_t adc_data_ready;    // ADC 数据就绪信号量
rt_sem_t flash_write_ready;//flash 写入
rt_mutex_t df_mutex;        // 保护 DF 结构体
rt_mutex_t set_value_mutex; // 保护 SET_Value
rt_mutex_t flash_mutex;     // 保护 Flash 操作


volatile uint8_t BUZZER_Short_Flag = 0;                            // 蜂鸣器短叫触发标志位
volatile uint8_t BUZZER_Middle_Flag = 0;                           // 蜂鸣器中等时间长度鸣叫触发标志位
volatile uint8_t BUZZER_Flag = 0;                                  // 蜂鸣器当前状态标志位
volatile float MAX_OTP_VAL;                                        // 过温保护阈值
volatile float MAX_VOUT_OVP_VAL;                                   // 输出过压保护阈值
volatile float MAX_VOUT_OCP_VAL;                                   // 输出过流保护阈值
#define MAX_SHORT_I 10.1F                                          // 短路电流判据
#define MIN_SHORT_V 0.5F                                       // 短路电压判据


volatile float VIN, VOUT, IIN, IOUT;                               // 电压电流实际值
volatile float MainBoard_TEMP, CPU_TEMP;                           // 主板和CPU温度实际值







CCMRAM void ADCSample(void)
{
    // 输入输出采样参数求和，用以计算平均值
    static uint32_t VinAvgSum = 0, IinAvgSum = 0, VoutAvgSum = 0, IoutAvgSum = 0;

    // 从DMA缓冲器中获取数据
    SADC.Vin = (uint32_t)ADC1_RESULT[0];
    SADC.Iin = (uint32_t)ADC1_RESULT[1];
    SADC.Vout = (uint32_t)((ADC1_RESULT[2] * CAL_VOUT_K >> 12) + CAL_VOUT_B);
    SADC.Iout = (uint32_t)((ADC1_RESULT[3] * CAL_IOUT_K >> 12) + CAL_IOUT_B);

    if (SADC.Vin < 15) // 采样有零偏离，采样值很小时，直接为0
        SADC.Vin = 0;
    if (SADC.Vout < 15)
        SADC.Vout = 0;
    if (SADC.Iout < 16)
        SADC.Iout = 0;

    // 计算各个采样值的平均值-滑动平均方式
    VinAvgSum = VinAvgSum + SADC.Vin - (VinAvgSum >> 3); // 求和，新增入一个新的采样值，同时减去之前的平均值。
    SADC.VinAvg = VinAvgSum >> 3;                        // 求平均
    IinAvgSum = IinAvgSum + SADC.Iin - (IinAvgSum >> 3);
    SADC.IinAvg = IinAvgSum >> 3;
    VoutAvgSum = VoutAvgSum + SADC.Vout - (VoutAvgSum >> 3);
    SADC.VoutAvg = VoutAvgSum >> 3;
    IoutAvgSum = IoutAvgSum + SADC.Iout - (IoutAvgSum >> 3);
    SADC.IoutAvg = IoutAvgSum >> 3;
}

/**
 * @brief ADC数据计算转换成实际数值的浮点数
 *
 */
void ADC_calculate(void)
{
    VIN = SADC.VinAvg * REF_3V3 / ADC_MAX_VALUE / (4.7F / 75.0F);   // 计算ADC1通道0输入电压采样结果
    IIN = SADC.IinAvg * REF_3V3 / ADC_MAX_VALUE / 62.0F / 0.005F;   // 计算ADC1通道1输入电流采样结果
    VOUT = SADC.VoutAvg * REF_3V3 / ADC_MAX_VALUE / (4.7F / 75.0F); // 计算ADC1通道2输出电压采样结果
    IOUT = SADC.IoutAvg * REF_3V3 / ADC_MAX_VALUE / 62.0F / 0.005F; // 计算ADC1通道3输出电流采样结果
    MainBoard_TEMP = GET_NTC_Temperature();                         // 获取NTC温度(主板温度)
    CPU_TEMP = GET_CPU_Temperature();                               // 获取单片机CPU温度
}

/*
 * @brief 状态机函数，在5ms中断中运行，5ms运行一次
 */
CCMRAM void StateM(void)
{
    // 判断状态类型
    switch (DF.SMFlag)
    {
    // 初始化状态
    case Init:
        StateMInit();
        break;
    // 等待状态
    case Wait:
        StateMWait();
        break;
    // 软启动状态
    case Rise:
        StateMRise();
        break;
    // 运行状态
    case Run:
        StateMRun();
        break;
    // 故障状态
    case Err:
        StateMErr();
        break;
    }
}

/*
 * @brief 初始化状态函数，参数初始化
 */
void StateMInit(void)
{
    // 相关参数初始化
    ValInit();
    // 状态机跳转至等待软启状态
    DF.SMFlag = Wait;
}

/*
 * @brief 相关参数初始化函数
 */
void ValInit(void)
{
    // 关闭PWM
    DF.PWMENFlag = 0;
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); // 关闭BUCK电路的PWM输出
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2); // 关闭BOOST电路的PWM输出
    DF.BBFlag = NA;
    // 清除故障标志位
    DF.ErrFlag = 0;
    // 初始化电压参考量
    CtrValue.Vout_ref = 0;
    // 限制占空比
    CtrValue.BuckDuty = MIN_BUKC_DUTY;
    CtrValue.BUCKMaxDuty = MIN_BUKC_DUTY;
    CtrValue.BoostDuty = MIN_BOOST_DUTY;
    CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
    // 环路计算变量初始化
    VErr0 = 0;
    VErr1 = 0;
    VErr2 = 0;
    u0 = 0;
    u1 = 0;
    // 设置值初始化
    SET_Value.Vout = 5.0;
    SET_Value.Iout = 10.0;
    MAX_OTP_VAL = 80.0F;      // 过温保护阈值
    MAX_VOUT_OVP_VAL = 50.0F; // 输出过压保护阈值
    MAX_VOUT_OCP_VAL = 10.5F; // 输出过流保护阈值
}

/*
 * @brief 正常运行，主处理函数在中断中运行
 */
void StateMRun(void)
{
}

/*
 * @brief 故障状态
 */
void StateMErr(void)
{
    // 关闭PWM
    DF.PWMENFlag = 0;
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); // 关闭BUCK电路的PWM输出
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2); // 关闭BOOST电路的PWM输出

    DF.BBFlag = NA;                                                              // 切换运行模式
    // 若故障消除跳转至等待重新软启
    if (DF.ErrFlag == F_NOERR)
    {
        DF.SMFlag = Wait;

    }
}

/*
 * @brief 等待状态机
 */
void StateMWait(void)
{
    // 计数器定义
    static uint16_t CntS = 0;
    static uint32_t IinSum = 0, IoutSum = 0;

    // 关PWM
    DF.PWMENFlag = 0;
    // 计数器累加
    CntS++;
    // 等待1S，进入启动状态
    if (CntS > 200)
    {
        CntS = 200;
        if ((DF.ErrFlag == F_NOERR) && (DF.OUTPUT_Flag == 1))
        {
            // 计数器清0
            CntS = 0;
            IinSum = 0;
            IoutSum = 0;
            // 状态标志位跳转至等待状态
            DF.SMFlag = Rise;
            // 软启动子状态跳转至初始化状态
            STState = SSInit;
        }
    }
}

/*
 * @brief 软启动阶段
 */
void StateMRise(void)
{
    // 计时器
    static uint16_t Cnt = 0;
    // 最大占空比限制计数器
    static uint16_t BUCKMaxDutyCnt = 0, BoostMaxDutyCnt = 0;

    // 判断软启状态
    switch (STState)
    {
    // 初始化状态
    case SSInit:
    {
        // 关闭PWM
        DF.PWMENFlag = 0;
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); // 关闭BUCK电路的PWM输出
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2); // 关闭BOOST电路的PWM输出
        // 软启中将运行限制占空比启动，从最小占空比开始启动
        CtrValue.BUCKMaxDuty = MIN_BUKC_DUTY;
        CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
        // 环路计算变量初始化
        VErr0 = 0;
        VErr1 = 0;
        VErr2 = 0;
        u0 = 0;
        u1 = 0;
        // 将设置值传到参考值
        CtrValue.Vout_SETref = SET_Value.Vout * (4.7F / 75.0F) / REF_3V3 * ADC_MAX_VALUE;
        CtrValue.Iout_ref = SET_Value.Iout * 0.005F * (6200.0F / 100.0F) / REF_3V3 * ADC_MAX_VALUE;
        // 跳转至软启等待状态
        STState = SSWait;

        break;
    }
    // 等待软启动状态
    case SSWait:
    {
        // 计数器累加
        Cnt++;
        // 等待25ms
        if (Cnt > 5)
        {
            // 计数器清0
            Cnt = 0;
            // 限制启动占空比
            CtrValue.BuckDuty = MIN_BUKC_DUTY;
            CtrValue.BUCKMaxDuty = MIN_BUKC_DUTY;
            CtrValue.BoostDuty = MIN_BOOST_DUTY;
            CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
            // 环路计算变量初始化
            VErr0 = 0;
            VErr1 = 0;
            VErr2 = 0;
            u0 = 0;
            u1 = 0;
            CtrValue.Vout_SSref = CtrValue.Vout_SETref >> 1; // 输出参考电压从一半开始启动，避免过冲，然后缓慢上升
            STState = SSRun;                                 // 跳转至软启状态
        }
        break;
    }
    // 软启动状态
    case SSRun:
    {
        if (DF.PWMENFlag == 0) // 正式发波前环路变量清0
        {
            // 环路计算变量初始化
            VErr0 = 0;
            VErr1 = 0;
            VErr2 = 0;
            u0 = 0;
            u1 = 0;
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, 30000); // BUCK电路下管占空比拉满
            __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, 30000); // BOOST电路下管占空比拉满
            HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);           // 开启HRTIM的PWM输出
            HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);           // 开启HRTIM的PWM输出
        }
        // 发波标志位置位
        DF.PWMENFlag = 1;
        // 最大占空比限制逐渐增加
        BUCKMaxDutyCnt++;
        BoostMaxDutyCnt++;
        // 最大占空比限制累加
        CtrValue.BUCKMaxDuty = CtrValue.BUCKMaxDuty + BUCKMaxDutyCnt * 15;
        CtrValue.BoostMaxDuty = CtrValue.BoostMaxDuty + BoostMaxDutyCnt * 15;
        // 累加到最大值
        if (CtrValue.BUCKMaxDuty > MAX_BUCK_DUTY)
            CtrValue.BUCKMaxDuty = MAX_BUCK_DUTY;
        if (CtrValue.BoostMaxDuty > MAX_BOOST_DUTY)
            CtrValue.BoostMaxDuty = MAX_BOOST_DUTY;

        if ((CtrValue.BUCKMaxDuty == MAX_BUCK_DUTY) && (CtrValue.BoostMaxDuty == MAX_BOOST_DUTY))
        {
            // 状态机跳转至运行状态
            DF.SMFlag = Run;
            // 软启动子状态跳转至初始化状态
            STState = SSInit;
        }
        break;
    }
    default:
        break;
    }
}

void ShortOff(void)
{
    static int32_t RSCnt = 0;
    static uint8_t RSNum = 0;
    float Vout = SADC.Vout * REF_3V3 / ADC_MAX_VALUE / (4.7F / 75.0F);
    float Iout = SADC.Iout * REF_3V3 / ADC_MAX_VALUE / 62.0F / 0.005F;
    // 当输出电流大于 *A，且电压小于*V时，可判定为发生短路保护
    if ((Iout > MAX_SHORT_I) && (Vout < MIN_SHORT_V))
    {
        // 关闭PWM
        DF.PWMENFlag = 0;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); // 开启HRTIM的PWM输出
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2); // 开启HRTIM的PWM输出
        // 故障标志位
        setRegBits(DF.ErrFlag, F_SW_SHORT);
        // 跳转至故障状态
        DF.SMFlag = Err;
    }
    // 输出短路保护恢复
    // 当发生输出短路保护，关机后等待4S后清楚故障信息，进入等待状态等待重启
    if (getRegBits(DF.ErrFlag, F_SW_SHORT))
    {
        // 等待故障清楚计数器累加
        RSCnt++;
        // 等待2S
        if (RSCnt > 400)
        {
            // 计数器清零
            RSCnt = 0;
            // 短路重启只重启10次，10次后不重启
            if (RSNum > 10)
            {
                // 确保不清除故障，不重启
                RSNum = 11;
                // 关闭PWM
                DF.PWMENFlag = 0;
                HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); // 开启HRTIM的PWM输出
                HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2); // 开启HRTIM的PWM输出
            }
            else
            {
                // 短路重启计数器累加
                RSNum++;
                // 清除过流保护故障标志位
                clrRegBits(DF.ErrFlag, F_SW_SHORT);
            }
        }
    }
}

/**
 * @brief OVP 输出过压保护函数
 * OVP 函数用于处理输出电压过高的情况。
 * 函数需放5ms中断里执行。
 */
void OVP(void)
{
    // 过压保护判据保持计数器定义
    static uint16_t OVPCnt = 0;
    float Vout = SADC.Vout * REF_3V3 / ADC_MAX_VALUE / (4.7F / 75.0F);
    // 当输出电压大于50V，且保持10ms
    if (Vout >= MAX_VOUT_OVP_VAL)
    {
        // 条件保持计时
        OVPCnt++;
        // 条件保持10ms
        if (OVPCnt > 2)
        {
            // 计时器清零
            OVPCnt = 0;
            // 关闭PWM
            DF.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); // 关闭BUCK电路的PWM输出
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2); // 关闭BOOST电路的PWM输出
            // 故障标志位
            setRegBits(DF.ErrFlag, F_SW_VOUT_OVP);
            // 跳转至故障状态
            DF.SMFlag = Err;
        }
    }
    else
        OVPCnt = 0;
}

/**
 * @brief OCP 输出过流保护函数
 * OCP 函数用于处理输出电流过高的情况。
 * 函数需放5ms中断里执行。
 */
void OCP(void)
{
    // 过流保护判据保持计数器定义
    static uint16_t OCPCnt = 0;
    // 故障清楚保持计数器定义
    static uint16_t RSCnt = 0;
    // 保留保护重启计数器
    static uint16_t RSNum = 0;

    float Iout = SADC.Iout * REF_3V3 / ADC_MAX_VALUE / 62.0F / 0.005F;

    // 当输出电流大于*A，且保持50ms
    if ((Iout >= MAX_VOUT_OCP_VAL) && (DF.SMFlag == Run))
    {
        // 条件保持计时
        OCPCnt++;
        // 条件保持50ms，则认为过流发生
        if (OCPCnt > 10)
        {
            // 计数器清0
            OCPCnt = 0;
            // 关闭PWM
            DF.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); // 关闭BUCK电路的PWM输出
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2); // 关闭BOOST电路的PWM输出
            // 故障标志位
            setRegBits(DF.ErrFlag, F_SW_IOUT_OCP);
            // 跳转至故障状态
            DF.SMFlag = Err;
        }
    }
    else
        // 计数器清0
        OCPCnt = 0;

    // 输出过流后恢复
    // 当发生输出软件过流保护，关机后等待4S后清楚故障信息，进入等待状态等待重启
    if (getRegBits(DF.ErrFlag, F_SW_IOUT_OCP))
    {
        // 等待故障清楚计数器累加
        RSCnt++;
        // 等待2S
        if (RSCnt > 400)
        {
            // 计数器清零
            RSCnt = 0;
            // 过流重启计数器累加
            RSNum++;
            // 过流重启只重启10次，10次后不重启（严重故障）
            if (RSNum > 10)
            {
                // 确保不清除故障，不重启
                RSNum = 11;
                // 关闭PWM
                DF.PWMENFlag = 0;
                HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); // 关闭BUCK电路的PWM输出
                HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2); // 关闭BOOST电路的PWM输出
            }
            else
            {
                // 清除过流保护故障标志位
                clrRegBits(DF.ErrFlag, F_SW_IOUT_OCP);
            }
        }
    }
}

/**
 * @brief OTP 过温保护函数
 * OTP 函数用于处理温度过高的情况。
 * 函数需放5ms中断里执行。
 */
void OTP(void)
{
    float TEMP = GET_NTC_Temperature(); // 获取NTC温度值
    if (TEMP >= MAX_OTP_VAL)
    {
        DF.SMFlag = Wait;
        // 关闭PWM
        DF.PWMENFlag = 0;
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); // 关闭BUCK电路的PWM输出
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2); // 关闭BOOST电路的PWM输出
        setRegBits(DF.ErrFlag, F_OTP);                                               // 故障标志位
        DF.SMFlag = Err;                                                             // 跳转至故障状态
    }
}

/**
 * @brief 运行模式判断。
 * BUCK模式：输出参考电压<0.8倍输入电压
 * BOOST模式：输出参考电压>1.2倍输入电压
 * MIX模式：1.15倍输入电压>输出参考电压>0.85倍输入电压
 * 当进入MIX（buck-boost）模式后，退出到BUCK或者BOOST时需要滞缓，防止在临界点来回振荡
 */
CCMRAM void BBMode(void)
{
    // 上一次模式状态量
    uint8_t PreBBFlag = 0;
    // 暂存当前的模式状态量
    PreBBFlag = DF.BBFlag;

    uint32_t VIN_ADC = ADC1_RESULT[0]; // 输入电压ADC采样值

    // 对输入电压ADC采样值累计取平均值
    static uint32_t VIN_ADC_SUM = 0;
    static uint8_t VIN_ADC_Count = 0;

    if (VIN_ADC_Count < 5)
    {
        VIN_ADC_SUM += ADC1_RESULT[0];
        VIN_ADC_Count++;
    }
    if (VIN_ADC_Count == 5)
    {
        VIN_ADC = VIN_ADC_SUM / 5;
        VIN_ADC_SUM = 0;
        VIN_ADC_Count = 0;
    }

    // 判断当前模块的工作模式
    switch (DF.BBFlag)
    {
    // NA-初始化模式
    case NA:
    {
        if (CtrValue.Vout_ref < (VIN_ADC * 0.8F))      // 输出参考电压小于0.8倍输入电压时
            DF.BBFlag = Buck;                          // 切换到buck模式
        else if (CtrValue.Vout_ref > (VIN_ADC * 1.2F)) // 输出参考电压大于1.2倍输入电压时
            DF.BBFlag = Boost;                         // 切换到boost模式
        else
            DF.BBFlag = Mix; // buck-boost（MIX） mode
        break;
    }
    // BUCK模式
    case Buck:
    {
        if (CtrValue.Vout_ref > (VIN_ADC * 1.2F))       // vout>1.2*vin
            DF.BBFlag = Boost;                          // boost mode
        else if (CtrValue.Vout_ref > (VIN_ADC * 0.85F)) // 1.2*vin>vout>0.85*vin
            DF.BBFlag = Mix;                            // buck-boost（MIX） mode
        break;
    }
    // Boost模式
    case Boost:
    {
        if (CtrValue.Vout_ref < ((VIN_ADC * 0.8F)))     // vout<0.8*vin
            DF.BBFlag = Buck;                           // buck mode
        else if (CtrValue.Vout_ref < (VIN_ADC * 1.15F)) // 0.8*vin<vout<1.15*vin
            DF.BBFlag = Mix;                            // buck-boost（MIX） mode
        break;
    }
    // Mix模式
    case Mix:
    {
        if (CtrValue.Vout_ref < (VIN_ADC * 0.8F))      // vout<0.8*vin
            DF.BBFlag = Buck;                          // buck mode
        else if (CtrValue.Vout_ref > (VIN_ADC * 1.2F)) // vout>1.2*vin
            DF.BBFlag = Boost;                         // boost mode
        break;
    }
    }

    // 当模式发生变换时（上一次和这一次不一样）,则标志位置位，标志位用以环路计算复位，保证模式切换过程不会有大的过冲
    if (PreBBFlag == DF.BBFlag)
        DF.BBModeChange = 0;
    else
        DF.BBModeChange = 1;
}

/**
 * @brief 一阶低通滤波器。
 * 使用一阶低通滤波算法对输入信号进行滤波处理。
 * @param input 输入信号
 * @param alpha 滤波系数
 * @return 滤波后的输出信号
 */
float one_order_lowpass_filter(float input, float alpha)
{
    static float prev_output = 0.0F;                             // 静态变量，用于保存上一次的输出值
    float output = alpha * input + (1.0F - alpha) * prev_output; // 一阶低通滤波算法
    prev_output = output;                                        // 保存本次输出值，以备下一次使用
    return output;                                               // 返回滤波后的输出信号
}


float calculateTemperature(float voltage)
{
    // 数据进入前，可先做滤波处理
    float Rt = 0;                                                  // NTC电阻
    float R = 10000;                                               // 10K固定阻值电阻
    float T0 = 273.15F + 25;                                       // 转换为开尔文温度
    float B = 3950;                                                // B值
    float Ka = 273.15F;                                            // K值
    Rt = (REF_3V3 - voltage) * 10000.0F / voltage;                 // 计算Rt
    float temperature = 1.0F / (1.0F / T0 + log(Rt / R) / B) - Ka; // 计算温度
    return temperature;
}


float GET_NTC_Temperature(void)
{
    HAL_ADC_Start(&hadc2); // 启动ADC2采样，采样NTC温度
    // HAL_ADC_PollForConversion(&hadc2, 100); // 等待ADC采样结束
    uint32_t TEMP_adcValue = HAL_ADC_GetValue(&hadc2);                            // 读取ADC2采样结果
    float temperature = calculateTemperature(TEMP_adcValue * REF_3V3 / 65520.0F); // 计算温度
    return temperature;                                                           // 返回温度值
}


float GET_CPU_Temperature(void)
{
    HAL_ADC_Start(&hadc5); // 启动ADC5采样，采样单片机CPU温度
    // HAL_ADC_PollForConversion(&hadc5, 100); // 等待ADC采样结束
    float Temp_Scale = (float)(TS_CAL2_TEMP - TS_CAL1_TEMP) / (float)(TS_CAL2 - TS_CAL1); // 计算温度比例因子
    // 读取ADC5采样结果, 除以8是因为开启了硬件超采样到15bit，但下面计算用的是12bit，开启硬件超采样是为了得到一个比较平滑的采样结果
    float TEMP_adcValue = HAL_ADC_GetValue(&hadc5) / 8.0F;
    float temperature = Temp_Scale * (TEMP_adcValue * (REF_3V3 / 3.0F) - TS_CAL1) + TS_CAL1_TEMP; // 计算温度
    return one_order_lowpass_filter(temperature, 0.1F);                                           // 返回温度值
}

int USART2_Printf(const char *format, ...)
{
  va_list arg;
  static char SendBuff[200] = {0};
  int rv;
  //while (!usart_dma_tx_over); // 等待前一次DMA发送完成
  for (volatile uint16_t i = 0; i < 1000 && (!usart_dma_tx_over); i++);  // 等待前一次DMA发送完成

  // 使用可变参数列表进行格式化输出
  va_start(arg, format);
  rv = vsnprintf((char *)SendBuff, sizeof(SendBuff), (char *)format, arg);
  va_end(arg);

  usart_dma_tx_over = 0;                                   // 清0全局标志，发送完成后重新置1
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)SendBuff, rv); // 使用DMA发送数据

  return rv;
}
void FAN_PWM_set(uint16_t dutyCycle)
{
    if (dutyCycle > 100)
    {
        dutyCycle = 100;
    }
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, dutyCycle * 10);
}



/**
 * 将浮点数转换为字节序列的辅助函数。
 * 该函数将一个浮点数转换为其对应的字节序列，通过内存拷贝的方式将浮点数的二进制表示复制到指定的字节数组中。
 * @param value 需要转换为字节序列的浮点数。
 * @param bytes 指向接收浮点数字节序列的字节数组的指针。
 * @note 该函数依赖于特定平台的浮点数和整数类型大小以及内存对齐规则。
 */
void float_to_bytes(float value, uint8_t *bytes)
{
    memcpy(bytes, &value, sizeof(float)); // 将浮点数转换为字节序列
}

/**
 * 将字节序列转换为浮点数的辅助函数
 * @param bytes 指向包含浮点数的字节序列的指针
 * @return 转换后的浮点数值
 */
float bytes_to_float(uint8_t *bytes)
{
    float value;
    memcpy(&value, bytes, sizeof(float));
    return value;
}


#endif /* APPLICATIONS_DEFIND_C_ */
