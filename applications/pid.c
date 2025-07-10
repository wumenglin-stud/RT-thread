/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-04     dream       the first version
 */
#include "pid.h"
#include "define.h"

#define CCMRAM __attribute__((section("ccmram")))
volatile _CVCC_Mode CVCC_Mode = CV;               // 恒流恒压模式标志位

// 预计算的PID参数，减少中断中的计算量
CCMRAM static int32_t pid_params_initialized = 0;
CCMRAM static int32_t buck_pid_b0, buck_pid_b1, buck_pid_b2;
CCMRAM static int32_t boost_pid_b0, boost_pid_b1, boost_pid_b2;

void PID_Init(void)
{
    VErr0 = 0;
    VErr1 = 0;
    VErr2 = 0;
    u0 = 0;
    u1 = 0;
    i0 = 0;
    IErr0 = 0;
    
    // 预计算PID参数，避免在中断中计算
    buck_pid_b0 = BUCKPIDb0;
    buck_pid_b1 = BUCKPIDb1;
    buck_pid_b2 = BUCKPIDb2;
    boost_pid_b0 = BOOSTPIDb0;
    boost_pid_b1 = BOOSTPIDb1;
    boost_pid_b2 = BOOSTPIDb2;
    
    pid_params_initialized = 1;
}

// 优化的PID控制中断处理函数
CCMRAM void BuckBoostVILoopCtlPID(void)
{
    static int32_t I_Integral = 0; // 电流环路积分量

    // 快速获取ADC值
    int32_t VoutTemp = (adc_dma_buffer[2] * CAL_VOUT_K >> 12) + CAL_VOUT_B;
    int32_t IoutTemp = (adc_dma_buffer[3] * CAL_IOUT_K >> 12) + CAL_IOUT_B;

    // 计算电流误差量
    IErr0 = CtrValue.Iout_ref - IoutTemp;
    
    // 使用预计算参数的简化计算
    if (!pid_params_initialized) {
        PID_Init(); // 确保参数已初始化
    }
    
    // 电流环路计算
    i0 = I_Integral + IErr0 * ILOOP_KP + (IErr0 - IErr1) * ILOOP_KD;
    I_Integral += IErr0 * ILOOP_KI;

    // 积分限幅
    if (I_Integral > ADC_MAX_VALUE)
        I_Integral = ADC_MAX_VALUE;
    else if (I_Integral < 0)
        I_Integral = 0;

    // 参考电压计算
    if (DF.SMFlag == Rise && (VoutTemp < (CtrValue.Vout_ref / 2)))
    {
        CtrValue.Vout_ref = CtrValue.Vout_ref + i0;
        CVCC_Mode = CC;
        if (CtrValue.Vout_ref > CtrValue.Vout_SSref)
        {
            CtrValue.Vout_ref = CtrValue.Vout_SSref;
            CVCC_Mode = CV;
        }
        if (CtrValue.Vout_ref < 0)
        {
            CtrValue.Vout_ref = 0;
        }
    }
    else
    {
        CtrValue.Vout_ref = CtrValue.Vout_ref + i0;
        CVCC_Mode = CC;
        if (CtrValue.Vout_ref > CtrValue.Vout_SETref)
        {
            CtrValue.Vout_ref = CtrValue.Vout_SETref;
            CVCC_Mode = CV;
        }
        if (CtrValue.Vout_ref < 0)
        {
            CtrValue.Vout_ref = 0;
        }
    }

    VErr0 = CtrValue.Vout_ref - VoutTemp;

    // 模式切换处理
    if (DF.BBModeChange)
    {
        u1 = 0;
        I_Integral = 0;
        i0 = 0;
        DF.BBModeChange = 0;
    }

    // 根据工作模式计算控制量
    switch (DF.BBFlag)
    {
    case NA:
        VErr0 = 0;
        VErr1 = 0;
        VErr2 = 0;
        u0 = 0;
        u1 = 0;
        i0 = 0;
        I_Integral = 0;
        IErr0 = 0;
        IErr1 = 0;
        break;
        
    case Buck:
        // 使用预计算的PID参数
        u0 = u1 + VErr0 * buck_pid_b0 + VErr1 * buck_pid_b1 + VErr2 * buck_pid_b2;
        VErr2 = VErr1;
        VErr1 = VErr0;
        u1 = u0;

        CtrValue.BoostDuty = MIN_BOOST_DUTY1;
        CtrValue.BuckDuty = (u0 >> 8) * 3;

        if (CtrValue.BuckDuty > CtrValue.BUCKMaxDuty)
            CtrValue.BuckDuty = CtrValue.BUCKMaxDuty;
        if (CtrValue.BuckDuty < MIN_BUKC_DUTY)
            CtrValue.BuckDuty = MIN_BUKC_DUTY;
        break;
        
    case Boost:
        // 使用预计算的PID参数
        u0 = u1 + VErr0 * boost_pid_b0 + VErr1 * boost_pid_b1 + VErr2 * boost_pid_b2;
        VErr2 = VErr1;
        VErr1 = VErr0;
        u1 = u0;

        CtrValue.BuckDuty = MAX_BUCK_DUTY;
        CtrValue.BoostDuty = (u0 >> 8) * 3;

        if (CtrValue.BoostDuty > CtrValue.BoostMaxDuty)
            CtrValue.BoostDuty = CtrValue.BoostMaxDuty;
        if (CtrValue.BoostDuty < MIN_BOOST_DUTY)
            CtrValue.BoostDuty = MIN_BOOST_DUTY;
        break;
        
    case Mix:
        // 使用预计算的PID参数
        u0 = u1 + VErr0 * boost_pid_b0 + VErr1 * boost_pid_b1 + VErr2 * boost_pid_b2;
        VErr2 = VErr1;
        VErr1 = VErr0;
        u1 = u0;
        IErr1 = IErr0;

        CtrValue.BuckDuty = MAX_BUCK_DUTY1;
        CtrValue.BoostDuty = (u0 >> 8) * 3;

        if (CtrValue.BoostDuty > CtrValue.BoostMaxDuty)
            CtrValue.BoostDuty = CtrValue.BoostMaxDuty;
        if (CtrValue.BoostDuty < MIN_BOOST_DUTY)
            CtrValue.BoostDuty = MIN_BOOST_DUTY;
        break;
    }

    // PWM使能控制
    if (DF.PWMENFlag == 0)
        CtrValue.BuckDuty = MIN_BUKC_DUTY;

    // 直接更新PWM寄存器
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, PERIOD - CtrValue.BuckDuty);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_3, __HAL_HRTIM_GETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1) >> 1);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, CtrValue.BoostDuty);
}
