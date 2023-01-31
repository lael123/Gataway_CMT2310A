/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-24     Rick       the first version
 */
#include <rtthread.h>
#include <stdio.h>
#include <stdbool.h>
#include <finsh.h>
#include <fal.h>

#define DBG_TAG "wifi_ota"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define DEFAULT_DOWNLOAD_PART "download"

static char* recv_partition = DEFAULT_DOWNLOAD_PART;
static size_t update_file_total_size, update_file_cur_size;
static const struct fal_partition * dl_part = RT_NULL;

void wifi_ota_begin(rt_uint32_t file_size)
{
    update_file_total_size = file_size;
    LOG_I("wifi_ota_begin file_size:%d\n", update_file_total_size);

    /* Get download partition information and erase download partition data */
    if ((dl_part = fal_partition_find(recv_partition)) == RT_NULL)
    {
        LOG_E("Firmware download failed! Partition (%s) find error!", recv_partition);
        return ;
    }

    if (update_file_total_size > dl_part->len)
    {
        LOG_E("Firmware is too large! File size (%d), '%s' partition size (%d)", update_file_total_size, recv_partition, dl_part->len);
        return ;
    }

    LOG_I("Start erase. Size (%d)", update_file_total_size);

    /* erase DL section */
    if (fal_partition_erase(dl_part, 0, update_file_total_size) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) erase error!", dl_part->name);
        return ;
    }
}

void wifi_ota_receive(rt_uint8_t *buf, rt_size_t offset, rt_size_t len)
{
    /* write data of application to DL partition  */
    if(update_file_cur_size != offset)//old
    {
        return;
    }
    if (fal_partition_write(dl_part, update_file_cur_size, buf, len) < 0)
    {
        LOG_E("Firmware download failed! Partition (%s) write data error!", dl_part->name);
        return;
    }
    update_file_cur_size += len;//new
    LOG_D("Write success,offset (%d),recv size (%d),total size (%d)\r\n",offset,update_file_cur_size,update_file_total_size);
}
void wifi_ota_end(rt_uint8_t *buf, rt_size_t offset)
{
    LOG_I("update successfully,system will boot in 1 sec...\r\n");
    rt_thread_mdelay(1000);
    LOG_I("Reboot");
    rt_hw_cpu_reset();
}
