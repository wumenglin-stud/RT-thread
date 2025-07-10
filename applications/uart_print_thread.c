#include <rtthread.h>
#include <define.h>

static void uart_print_thread_entry(void *parameter)
{
    while (1)
    {
        if (IOUT >= 0.1)
        {
            powerEfficiency = (VOUT * IOUT) / (VIN * IIN) * 100.0;
        }
        else
        {
            powerEfficiency = 0;
        }
        USART2_Printf("%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d\n",
                      VIN, IIN, VOUT, IOUT, MainBoard_TEMP, CPU_TEMP, powerEfficiency, CVCC_Mode);
        rt_thread_mdelay(50); // 50ms周期
    }
}

void create_uart_print_thread(void)
{
    rt_thread_t tid = rt_thread_create("uart_print", uart_print_thread_entry, RT_NULL, 1024, 20, 10);
    if (tid)
        rt_thread_startup(tid);
} 
