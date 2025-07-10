/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-02     RT-Thread    first version
 */

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <main.h>
#include <tim.h>
#include <hrtim.h>
#include <rtdbg.h>
#include <define.h>
#include <iwdg.h>
#include <define.h>
#include <fan_thread.h>
#include <uart_print_thread.h>
#include <adc_thread.h>
#include <state_machine_thread.h>
#include <storage_thread.h>
#include <rtthread.h>
#include <finsh.h>

// 定义内存池和消息队列
rt_mp_t adc_data_mp = RT_NULL;
rt_mq_t adc_data_mq = RT_NULL;
// 定义事件集
rt_event_t system_event = RT_NULL;

int main(void)
{
    // 初始化信号量
    adc_data_ready = rt_sem_create("adc_data", 0, RT_IPC_FLAG_FIFO);
    flash_write_ready=rt_sem_create("flash", 0, RT_IPC_FLAG_FIFO);
    // 初始化互斥量
    df_mutex = rt_mutex_create("df_lock", RT_IPC_FLAG_FIFO);
    set_value_mutex = rt_mutex_create("set_value_lock", RT_IPC_FLAG_FIFO);
    flash_mutex = rt_mutex_create("flash_lock", RT_IPC_FLAG_FIFO);
    
    // 初始化内存池和消息队列
    adc_data_mp = rt_mp_create("adc_mp", ADC_DATA_POOL_SIZE, ADC_DATA_BLOCK_SIZE);
    adc_data_mq = rt_mq_create("adc_mq", sizeof(struct _ADC_Data*), ADC_DATA_POOL_SIZE, RT_IPC_FLAG_FIFO);
    
    // 初始化事件集
    system_event = rt_event_create("sys_event", RT_IPC_FLAG_FIFO);

    MX_GPIO_Init();
    MX_HRTIM1_Init();
    MX_TIM8_Init();
    MX_DMA_Init();
    MX_ADC2_Init();
    MX_ADC1_Init();
    MX_ADC5_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, 4);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc5);
    MX_IWDG_Init();

    // 各线程初始化
    create_adc_thread();
    create_fan_thread();
    create_uart_print_thread();
    create_state_machine_thread();
    create_flash_thread();
    main_control_thread_init();

    return RT_EOK;
}

void HRTIM1_TIMD_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_TIMD_IRQn 0 */

  /* USER CODE END HRTIM1_TIMD_IRQn 0 */
  HAL_HRTIM_IRQHandler(&hhrtim1,HRTIM_TIMERINDEX_TIMER_D);
  /* USER CODE BEGIN HRTIM1_TIMD_IRQn 1 */
  BuckBoostVILoopCtlPID();
  /* USER CODE END HRTIM1_TIMD_IRQn 1 */
}
// 10ms定时器回调
static void timer_10ms_entry(void *parameter)
{
    ADC_calculate();
}

// 50ms定时器回调
static void timer_50ms_entry(void *parameter)
{
   // BUZZER_Middle();

    if ((DF.SMFlag == Rise) || (DF.SMFlag == Run))
    {
      //  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET); 灯
    }
    else
    {
       // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    }

    if (IOUT >= 0.1)
    {
        powerEfficiency = (VOUT * IOUT) / (VIN * IIN) * 100.0;
    }
    else
    {
        powerEfficiency = 0;
    }

    USART1_Printf("%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d\n",
                  VIN, IIN, VOUT, IOUT, MainBoard_TEMP, CPU_TEMP, powerEfficiency, CVCC_Mode);//vofe
}

// 100ms定时器回调
static void timer_100ms_entry(void *parameter)
{
    Auto_FAN();
}

// 500ms定时器回调
static void timer_500ms_entry(void *parameter)
{
    //HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    Update_Flash();
}

//喂狗
static void main_control_thread(void *parameter)
{
    while (1)
    {
        HAL_IWDG_Refresh(&hiwdg);
        rt_thread_mdelay(50);
    }
}

// 线程和定时器自动初始化
int main_control_thread_init(void)
{
    // 创建并启动主控线程
    rt_thread_t tid = rt_thread_create("main_ctrl", main_control_thread, RT_NULL, 2048, 10, 10);
    if (tid)
        rt_thread_startup(tid);

    return 0;
}

// 命令行调试函数 - 显示系统状态
static void cmd_show_status(int argc, char **argv)
{
    rt_kprintf("系统状态信息：\n");
    rt_kprintf("输入电压: %.2f V\n", VIN);
    rt_kprintf("输出电压: %.2f V\n", VOUT);
    rt_kprintf("输入电流: %.2f A\n", IIN);
    rt_kprintf("输出电流: %.2f A\n", IOUT);
    rt_kprintf("主板温度: %.2f ℃\n", MainBoard_TEMP);
    rt_kprintf("CPU温度: %.2f ℃\n", CPU_TEMP);
    rt_kprintf("效率: %.2f %%\n", powerEfficiency);
    rt_kprintf("工作模式: %s\n", CVCC_Mode == CV ? "恒压" : "恒流");
    
    // 显示状态机状态
    rt_kprintf("状态机状态: ");
    switch(DF.SMFlag)
    {
        case Init: rt_kprintf("初始化\n"); break;
        case Wait: rt_kprintf("等待\n"); break;
        case Rise: rt_kprintf("软启动\n"); break;
        case Run: rt_kprintf("运行\n"); break;
        case Err: rt_kprintf("错误\n"); break;
        default: rt_kprintf("未知\n");
    }
    
    // 显示工作模式
    rt_kprintf("工作模式: ");
    switch(DF.BBFlag)
    {
        case NA: rt_kprintf("未定义\n"); break;
        case Buck: rt_kprintf("Buck降压\n"); break;
        case Boost: rt_kprintf("Boost升压\n"); break;
        case Mix: rt_kprintf("Buck-Boost混合\n"); break;
        default: rt_kprintf("未知\n");
    }
    
    // 显示错误状态
    if(DF.ErrFlag != F_NOERR)
    {
        rt_kprintf("错误状态: 0x%04X\n", DF.ErrFlag);
        if(DF.ErrFlag & F_SW_VIN_UVP) rt_kprintf("- 输入欠压\n");
        if(DF.ErrFlag & F_SW_VIN_OVP) rt_kprintf("- 输入过压\n");
        if(DF.ErrFlag & F_SW_VOUT_UVP) rt_kprintf("- 输出欠压\n");
        if(DF.ErrFlag & F_SW_VOUT_OVP) rt_kprintf("- 输出过压\n");
        if(DF.ErrFlag & F_SW_IOUT_OCP) rt_kprintf("- 输出过流\n");
        if(DF.ErrFlag & F_SW_SHORT) rt_kprintf("- 输出短路\n");
        if(DF.ErrFlag & F_OTP) rt_kprintf("- 温度过高\n");
    }
    else
    {
        rt_kprintf("错误状态: 无错误\n");
    }
}
MSH_CMD_EXPORT(cmd_show_status, 显示系统状态信息);

// 命令行调试函数 - 设置输出电压
static void cmd_set_vout(int argc, char **argv)
{
    float vout;
    
    if (argc != 2)
    {
        rt_kprintf("用法: set_vout <电压值>\n");
        return;
    }
    
    vout = atof(argv[1]);
    
    // 检查电压范围
    if (vout < 0 || vout > 30.0)
    {
        rt_kprintf("电压值超出范围 (0-30V)\n");
        return;
    }
    
    rt_mutex_take(set_value_mutex, RT_WAITING_FOREVER);
    SET_Value.Vout = vout;
    SET_Value.SET_modified_flag = 1;
    rt_mutex_release(set_value_mutex);
    
    rt_kprintf("输出电压设置为 %.2f V\n", vout);
}
MSH_CMD_EXPORT(cmd_set_vout, 设置输出电压);

// 命令行调试函数 - 设置输出电流
static void cmd_set_iout(int argc, char **argv)
{
    float iout;
    
    if (argc != 2)
    {
        rt_kprintf("用法: set_iout <电流值>\n");
        return;
    }
    
    iout = atof(argv[1]);
    
    // 检查电流范围
    if (iout < 0 || iout > 5.0)
    {
        rt_kprintf("电流值超出范围 (0-5A)\n");
        return;
    }
    
    rt_mutex_take(set_value_mutex, RT_WAITING_FOREVER);
    SET_Value.Iout = iout;
    SET_Value.SET_modified_flag = 1;
    rt_mutex_release(set_value_mutex);
    
    rt_kprintf("输出电流设置为 %.2f A\n", iout);
}
MSH_CMD_EXPORT(cmd_set_iout, 设置输出电流);

// 命令行调试函数 - 输出开关控制
static void cmd_output(int argc, char **argv)
{
    if (argc != 2)
    {
        rt_kprintf("用法: output <on/off>\n");
        return;
    }
    
    if (strcmp(argv[1], "on") == 0)
    {
        rt_mutex_take(df_mutex, RT_WAITING_FOREVER);
        DF.PWMENFlag = 1;
        DF.OUTPUT_Flag = 1;
        rt_mutex_release(df_mutex);
        rt_kprintf("输出已开启\n");
    }
    else if (strcmp(argv[1], "off") == 0)
    {
        rt_mutex_take(df_mutex, RT_WAITING_FOREVER);
        DF.PWMENFlag = 0;
        DF.OUTPUT_Flag = 0;
        rt_mutex_release(df_mutex);
        rt_kprintf("输出已关闭\n");
    }
    else
    {
        rt_kprintf("参数错误，请使用 on 或 off\n");
    }
}
MSH_CMD_EXPORT(cmd_output, 控制输出开关);

