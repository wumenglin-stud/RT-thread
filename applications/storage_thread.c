/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-04     dream       the first version
 */
#include "define.h"
#include "W25Q64.h"
#include <rtthread.h>
#include <string.h>
#include <storage_thread.h>



void Init_Flash(void) {
    uint8_t Flash_flag[1];
    W25Q64_ReadData(FLASH_FLAG_ADDR, Flash_flag, 1);

    if (Flash_flag[0] != 0x00) { // 标志位不为 0x00 表示未初始化
        W25Q64_SectorErase(FLASH_FLAG_ADDR); // 擦除扇区

        uint8_t Flash_data[21] = {0};
        Flash_data[0] = 0x00; // 设置标志位
        uint8_t VSETtemp[4], ISETtemp[4], OTPtemp[4], OCPtemp[4], OVPtemp[4];
        float_to_bytes(12.0F, VSETtemp);  // 默认电压 12V
        float_to_bytes(1.0F, ISETtemp);   // 默认电流 1A
        float_to_bytes(80.0F, OTPtemp);   // 默认过温保护 80°C
        float_to_bytes(15.0F, OCPtemp);   // 默认过流保护 15A
        float_to_bytes(20.0F, OVPtemp);   // 默认过压保护 20V

        for (uint8_t i = 0; i < 4; i++) {
            Flash_data[i + 1] = VSETtemp[i];
            Flash_data[i + 5] = ISETtemp[i];
            Flash_data[i + 9] = OTPtemp[i];
            Flash_data[i + 13] = OCPtemp[i];
            Flash_data[i + 17] = OVPtemp[i];
        }

        W25Q64_PageProgram(FLASH_FLAG_ADDR, Flash_data, 21); // 写入 Flash
    }
}


void Update_Flash(void) {
    if (SET_Value.SET_modified_flag == 1) {

        W25Q64_SectorErase(FLASH_FLAG_ADDR); // 擦除扇区

        uint8_t Flash_data[21] = {0};
        Flash_data[0] = 0x00; // 标志位

        uint8_t VSETtemp[4], ISETtemp[4], OTPtemp[4], OCPtemp[4], OVPtemp[4];
        float_to_bytes(SET_Value.Vout, VSETtemp);
        float_to_bytes(SET_Value.Iout, ISETtemp);
        float_to_bytes(MAX_OTP_VAL, OTPtemp);
        float_to_bytes(MAX_VOUT_OCP_VAL, OCPtemp);
        float_to_bytes(MAX_VOUT_OVP_VAL, OVPtemp);

        for (uint8_t i = 0; i < 4; i++) {
            Flash_data[i + 1] = VSETtemp[i];
            Flash_data[i + 5] = ISETtemp[i];
            Flash_data[i + 9] = OTPtemp[i];
            Flash_data[i + 13] = OCPtemp[i];
            Flash_data[i + 17] = OVPtemp[i];
        }

        W25Q64_PageProgram(FLASH_FLAG_ADDR, Flash_data, 21); // 写入新数据
        SET_Value.SET_modified_flag = 0;
    }
}

// 从 Flash 读取数据
void Read_Flash(void) {
    uint8_t Flash_data[20];
    W25Q64_ReadData(FLASH_DATA_ADDR, Flash_data, 20);

    uint8_t VSETtemp[4], ISETtemp[4], OTPtemp[4], OCPtemp[4], OVPtemp[4];
    for (uint8_t i = 0; i < 4; i++) {
        VSETtemp[i] = Flash_data[i];
        ISETtemp[i] = Flash_data[i + 4];
        OTPtemp[i] = Flash_data[i + 8];
        OCPtemp[i] = Flash_data[i + 12];
        OVPtemp[i] = Flash_data[i + 16];
    }

    SET_Value.Vout = bytes_to_float(VSETtemp);
    SET_Value.Iout = bytes_to_float(ISETtemp);
    MAX_OTP_VAL = bytes_to_float(OTPtemp);
    MAX_VOUT_OCP_VAL = bytes_to_float(OCPtemp);
    MAX_VOUT_OVP_VAL = bytes_to_float(OVPtemp);
}


static void flash_thread_entry(void* parameter)
{
    while (1)
    {
        {
            rt_sem_take(flash_write_ready, RT_WAITING_FOREVER);
            rt_mutex_take(flash_mutex, RT_WAITING_FOREVER);
            Update_Flash();
            rt_mutex_release(flash_mutex);
        }
    }
}

void create_flash_thread(void) {

    rt_thread_t thread = rt_thread_create("flash",
                                        flash_thread_entry,
                                        RT_NULL,
                                        1024,
                                        STORAGE_THREAD_PRIO,
                                        10);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
}
