/**
 * fx_stm32_sd_driver.c
 *
 * Version:     V1.0.0
 * Created on:  2021-06-15
 * Author:      Qiu Chengwei
 * Description: FileX的I/O驱动程序, STM32平台SDIO接口的SD卡
 *
 *
 * Copyright (c) 2021 Nanjing Zitai Xinghe Electronics Co., Ltd.
 * All rights reserved.
 *
 * 1 tab == 2 spaces!
 */

/**************************************************************************/
/*                                                                        */
/*       Partial Copyright (c) Microsoft Corporation. All rights reserved.*/
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*      Partial Copyright (c) STMicroelctronics 2020. All rights reserved */
/**************************************************************************/


/** 头文件包含顺序(以空行分隔): 关联.h, C库, C++库(无扩展), 其它库, 本项目.h
 * Includes ------------------------------------------------------------------*/
#include "fx_stm32_sd_driver.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Types, Constants, Macros, Variables ---------------------------------------*/

/* Prototypes of private function --------------------------------------------*/

/* Definition of functions ---------------------------------------------------*/


UINT  _fx_partition_offset_calculate(void  *partition_sector, UINT partition, ULONG *partition_start, ULONG *partition_size);

static UINT sd_read_data(FX_MEDIA *media_ptr, ULONG sector, UINT num_sectors, UINT use_scratch_buffer);
static UINT sd_write_data(FX_MEDIA *media_ptr, ULONG sector, UINT num_sectors, UINT use_scratch_buffer);

static uint8_t is_initialized = 0;

#define BSP_ERROR_NONE                    0
#define BSP_ERROR_BUSY                   -3

extern ULONG tx_time_get(void);

static int32_t check_sd_status(uint32_t instance)
{
    // TODO(Qiu Chengwei): 时间获取函数需实现.
    uint32_t start = tx_time_get();

    while (tx_time_get() - start < DEFAULT_TIMEOUT)
    {
      if (SD_GetStatus() == 0)
      {
        return BSP_ERROR_NONE;
      }
    }

    return BSP_ERROR_BUSY;
}

/**
  * @brief This function is the entry point to the STM32 SDIO disk driver.     */
 /*        It relies on the STM32 peripheral library from ST.
  * @param media_ptr: FileX's Media Config Block
  * @retval None
  */
VOID  fx_stm32_sd_driver(FX_MEDIA *media_ptr)
{
    UINT status;
    UINT unaligned_buffer = 0;
    ULONG partition_start;
    ULONG partition_size;

#if (FX_DRIVER_CALLS_SD_INIT == 0)
    is_initialized = 1; /* the SD  was initialized by the application*/
#endif
   /* before performing any operation, check the status of the SDMMC */
    if (is_initialized == 1)
    {
        if (check_sd_status(SD_INSTANCE) != BSP_ERROR_NONE)
        {
            media_ptr->fx_media_driver_status =  FX_IO_ERROR;
            return;
        }
    }

    /* Process the driver request specified in the media control block.  */
    switch(media_ptr->fx_media_driver_request)
    {
        case FX_DRIVER_INIT:
        {
#if (FX_DRIVER_CALLS_SD_INIT == 1)
            /* Initialize the SD instance */
            if (is_initialized == 0)
            {
                status = SD_Init();

                if (status == SD_OK)
                {
                    is_initialized = 1;
                    media_ptr->fx_media_driver_status =  FX_SUCCESS;
                }
                else
                {
                    media_ptr->fx_media_driver_status =  FX_IO_ERROR;
                }
            }
#endif
            break;
        }

        case FX_DRIVER_UNINIT:
        {
#if (FX_DRIVER_CALLS_SD_INIT == 1)
            SD_DeInit();
            is_initialized = 0;
#endif
            /* Successful driver request.  */
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;
        }

        case FX_DRIVER_READ:
        {
            media_ptr->fx_media_driver_status = FX_IO_ERROR;
            unaligned_buffer = (UINT)(media_ptr->fx_media_driver_buffer) & 0x3;

            if (sd_read_data(media_ptr, media_ptr->fx_media_driver_logical_sector + media_ptr->fx_media_hidden_sectors,
                             media_ptr->fx_media_driver_sectors, unaligned_buffer) == FX_SUCCESS)
            {
                media_ptr->fx_media_driver_status = FX_SUCCESS;
            }

            break;
        }

        case FX_DRIVER_WRITE:
        {
            media_ptr->fx_media_driver_status = FX_IO_ERROR;
            unaligned_buffer = (UINT)(media_ptr->fx_media_driver_buffer) & 0x3;

            if (sd_write_data(media_ptr, media_ptr->fx_media_driver_logical_sector + media_ptr->fx_media_hidden_sectors,
                              media_ptr->fx_media_driver_sectors, unaligned_buffer) == FX_SUCCESS)
            {
                media_ptr->fx_media_driver_status = FX_SUCCESS;
            }

            break;
        }

        case FX_DRIVER_FLUSH:
        {
            /* Return driver success.  */
            media_ptr->fx_media_driver_status =  FX_SUCCESS;
            break;
        }

        case FX_DRIVER_ABORT:
        {
            /* Return driver success.  */
            media_ptr->fx_media_driver_status =  FX_SUCCESS;
            break;
        }

        case FX_DRIVER_BOOT_READ:
        {
            /* the boot sector is the sector zero */
            status = sd_read_data(media_ptr, 0, media_ptr->fx_media_driver_sectors, 1);

            if (status != FX_SUCCESS)
            {
                media_ptr->fx_media_driver_status = status;
                break;
            }

            /* Check if the sector 0 is the actual boot sector, otherwise calculate the offset into it.
            Please note that this should belong to higher level of MW to do this check and it is here
            as a temporary work solution */

            partition_start =  0;

            status =  _fx_partition_offset_calculate(media_ptr -> fx_media_driver_buffer, 0,
                                                                &partition_start, &partition_size);

            /* Check partition read error.  */
            if (status)
            {
                /* Unsuccessful driver request.  */
                media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                break;
            }

            /* Now determine if there is a partition...   */
            if (partition_start)
            {

                if (check_sd_status(SD_INSTANCE) != BSP_ERROR_NONE)
                {
                    media_ptr->fx_media_driver_status =  FX_IO_ERROR;
                    break;
                }

                /* Yes, now lets read the actual boot record.  */
                status = sd_read_data(media_ptr, partition_start, media_ptr->fx_media_driver_sectors, 1);

                if (status != FX_SUCCESS)
          	    {
            	    media_ptr->fx_media_driver_status = status;
                    break;
                }
            }

            /* Successful driver request.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;
        }

        case FX_DRIVER_BOOT_WRITE:
        {
            status = sd_write_data(media_ptr, 0, media_ptr->fx_media_driver_sectors, 1);

            media_ptr->fx_media_driver_status = status;

            break;
        }

        default:
        {
            media_ptr->fx_media_driver_status =  FX_IO_ERROR;
            break;
        }
    }
}

/**
  * @brief Read buffer using BSP SD API taking into account the scratch buffer
  * @param FX_MEDIA *media_ptr a pointer the main FileX structure
  * @param ULONG start_sector first sector to start reading from
  * @param UINT num_sectors number of sectors to be read
  * @param UINT use_scratch_buffer to enable scratch buffer usage or not.
  * @retval FX_SUCCESS on success FX_BUFFER_ERROR / FX_ACCESS_ERROR / FX_IO_ERROR otherwise
  */

static UINT sd_read_data(FX_MEDIA *media_ptr, ULONG start_sector, UINT num_sectors, UINT use_scratch_buffer)
{
    UINT status;

    // uartstdio_printf("read  %10d, %d \n", start_sector, num_sectors);
    status = SD_ReadBlocks((uint8_t*)media_ptr->fx_media_driver_buffer, start_sector << 9, DEFAULT_SECTOR_SIZE, num_sectors);

    if (status != SD_OK)
    {
        /* DMA transfer failed */
        status = FX_ACCESS_ERROR;
    }
    else
    {
      status = FX_SUCCESS;
    }

    return status;
}

/**
  * @brief write buffer using BSP SD API taking into account the scratch buffer
  * @param FX_MEDIA *media_ptr a pointer the main FileX structure
  * @param ULONG start_sector first sector to start writing from
  * @param UINT num_sectors number of sectors to be written
  * @param UINT use_scratch_buffer to enable scratch buffer usage or not.
  * @retval FX_SUCCESS on success FX_BUFFER_ERROR / FX_ACCESS_ERROR / FX_IO_ERROR otherwise
  */

static UINT sd_write_data(FX_MEDIA *media_ptr, ULONG start_sector, UINT num_sectors, UINT use_scratch_buffer)
{
    UINT status;

    // uartstdio_printf("write %10d, %d \n", start_sector, num_sectors);
    status = SD_WriteBlocks((uint8_t*)media_ptr->fx_media_driver_buffer, start_sector << 9, DEFAULT_SECTOR_SIZE, num_sectors);
   
    if (status != SD_OK)
    {
        /* DMA transfer failed */
        status = FX_ACCESS_ERROR;
    }
    else
    {
      status = FX_SUCCESS;
    }

    return status;
}
