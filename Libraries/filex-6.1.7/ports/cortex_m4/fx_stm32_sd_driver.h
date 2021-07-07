/**
 * fx_stm32_sd_driver.h
 *
 * Version:     V1.0.0
 * Created on:  2021-06-15
 * Author:      Qiu Chengwei
 * Description: FileX的I/O驱动程序头文件, STM32平台SDIO接口的SD卡
 *
 *
 * Copyright (c) 2021 Nanjing Zitai Xinghe Electronics Co., Ltd.
 * All rights reserved.
 *
 * 1 tab == 2 spaces!
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FX_STM32_SD_DRIVER_H_
#define FX_STM32_SD_DRIVER_H_

/** 头文件包含顺序(以空行分隔): 关联.h, C库, C++库(无扩展), 其它库, 本项目.h
 * Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fx_api.h"

#include "stm32_sdio_sd.h"

/**
 * If building with a C++ compiler, make all of the definitions in this header
 * have a C binding.
 */
#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

/* Types, Constants, Macros --------------------------------------------------*/
// #define DEFAULT_TIMEOUT                         (10 * TX_TIMER_TICKS_PER_SECOND)
#define DEFAULT_TIMEOUT                         (10 * 100)

#define SD_INSTANCE                             0
#define DEFAULT_SECTOR_SIZE                     512

#define FX_DRIVER_CALLS_SD_INIT                 1

/* Declarations of public variables ------------------------------------------*/

/* Prototypes of public function ---------------------------------------------*/
VOID  fx_stm32_sd_driver(FX_MEDIA *media_ptr);

/* Mark the end of the C bindings section for C++ compilers. */
#ifdef __cplusplus
}
#endif

#endif  // FX_STM32_SD_DRIVER_H_
