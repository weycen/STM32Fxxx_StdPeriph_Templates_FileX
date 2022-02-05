/**
 * main.c
 *
 * Version:     V1.0.0
 * Created on:  2021-04-05
 * Author:      Qiu Chengwei
 * Description:
 *
 *
 * Copyright (c) 2021 Nanjing Zitai Xinghe Electronics Co., Ltd.
 * All rights reserved.
 *
 * 1 tab == 4 spaces!
 */

/** 头文件包含顺序(以空行分隔): 关联.h, C库, C++库(无扩展), 其它库, 本项目.h
 * Includes ------------------------------------------------------------------*/
#include "main.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "led.h"
#include "asmdelay.h"
#include "uartstdio.h"

#include "fx_api.h"
#include "fx_user.h"
#ifdef FX_ENABLE_FAULT_TOLERANT
#include "fx_fault_tolerant.h"
#endif /* FX_ENABLE_FAULT_TOLERANT */

//
#if defined(SWLIB_STM32F0)
#include "periph_f0xx_uart.h"
#elif defined(SWLIB_STM32F1)
#include "periph_f1xx_uart.h"
#include "stm3210c_eval.h"
#include "stm32_spi_sd.h"
#include "fx_stm32_sd_driver.h"
#elif defined(SWLIB_STM32F4)
#include "periph_f4xx_uart.h"
#include "stm324xg_eval.h"
#include "stm32_sdio_sd.h"
#include "fx_stm32_sd_driver.h"
#endif  // SWLIB_STM32F4



/* Types, Constants, Macros, Variables ---------------------------------------*/
#define DEBUG_ENABLE         1
#define MEDIA_CHECK_ENABLE   0


#if DEBUG_ENABLE
  #define DUBUG(fmt, args...)  do{ uartstdio_printf(fmt, ##args); }while(0)
#else 
  #define DUBUG(fmt, args...)
#endif

struct led led_fxxx;

#define PERIOD 10

uint32_t system_tick = 0; //系统时间戳

#define  TEST_DATA_LENGTH  (4 * 256)
uint8_t  test_data[TEST_DATA_LENGTH] = {0};
uint32_t test_count = 0;
  
  
/* 给FileX开的动态内存 */
uint8_t media_memory[512];            // FileX工作内存, 必须为扇区大小（通常为 512 个字节）

#if defined(FX_ENABLE_FAULT_TOLERANT)
uint8_t fault_tolerant_memory[FX_FAULT_TOLERANT_MAXIMUM_LOG_FILE_SIZE];  // 容错模块所需内存, 为扇区大小整数倍, 且至少为3072
#endif /* FX_ENABLE_FAULT_TOLERANT */


/* FileX全局结构体 */
FX_MEDIA     sdio_disk;
FX_FILE      fx_file;

#define PATH   "count.dat"

#if MEDIA_CHECK_ENABLE
ULONG detected_errors;
#define SRATCH_MEMORY_SIZE   (1024 * 16)
UCHAR        sratch_memory[SRATCH_MEMORY_SIZE];
#endif

/* Prototypes of private function --------------------------------------------*/
void NVIC_Configuration(void);

/* Definition of functions ---------------------------------------------------*/





/**
 * Main program.
 */
int main(void) {
  UINT status;
  ULONG bw;
  
  
  /* At this stage the microcontroller clock setting is already configured,
     this is done through SystemInit() function which is called from startup
     file (startup_stm32fxxx.s) before to branch to application main.
     To reconfigure the default setting of SystemInit() function, refer to
     system_stm32f0xx.c file
  */
  SystemInit();
  
	
//  /* Configure the NVIC Preemption Priority Bits */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
	
#if defined(SWLIB_STM32F4)
  /* NVIC Configuration */
  NVIC_Configuration();
#endif  // SWLIB_STM32F4

	
  // 调试串口配置
  Com_LowLevel_Init();
  uartstdio_init(1, 256000);
	DUBUG("\n\n");
  DUBUG("1s delay test begin\n");
  asmdelay_msec(1000);
  DUBUG("1s delay test complete \n");    


  // 初始化板载led
#if defined(SWLIB_STM32F4)
    led_init(&led_fxxx, GPIOB, 9, LED_LEVEL_LOW, LED_PULL_NONE, 1000, 500);
#elif defined(SWLIB_STM32F1)
    led_init(&led_fxxx, GPIOA, 8, LED_LEVEL_LOW, LED_PULL_NONE, 1000, 500);
#elif defined(SWLIB_STM32F0)
    led_init(&led_fxxx, GPIOC, 13, LED_LEVEL_LOW, LED_PULL_NONE, 1000, 500);
#endif  // SWLIB_STM32F1
    led_control(&led_fxxx, LED_REPEAT_MAX);


    /* Setup SysTick Timer for 10 msec interrupts.
     ------------------------------------------
    1. The SysTick_Config() function is a CMSIS function which configure:
       - The SysTick Reload register with value passed as function parameter.
       - Configure the SysTick IRQ priority to the lowest value (0x0F).
       - Reset the SysTick Counter register.
       - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
       - Enable the SysTick Interrupt.
       - Start the SysTick Counter.
    
    2. You can change the SysTick Clock source to be HCLK_Div8 by calling the
       SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8) just after the
       SysTick_Config() function call. The SysTick_CLKSourceConfig() is defined
       inside the misc.c file.

    3. You can change the SysTick IRQ priority by calling the
       NVIC_SetPriority(SysTick_IRQn,...) just after the SysTick_Config() function 
       call. The NVIC_SetPriority() is defined inside the core_cm3.h file.

    4. To adjust the SysTick time base, use the following formula:
                            
         Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
    
       - Reload Value is the parameter to be passed for SysTick_Config() function
       - Reload Value should not exceed 0xFFFFFF
   */
  if (SysTick_Config(SystemCoreClock / 100))
  { 
    /* Capture error */ 
    while (1) { }
  }

	
  DUBUG("sizeof(sdio_disk) = %d bytes \n", sizeof(sdio_disk));
  DUBUG("sizeof(fx_file)   = %d bytes \n", sizeof(fx_file));
  
	
  /* 初始化FileX文件系统 */
	DUBUG("fx_system_initialize() begin \n");
  fx_system_initialize();
	DUBUG("fx_system_initialize() complete \n");
	asmdelay_msec(1000);

	
  /* 打开SD卡 */
	DUBUG("fx_media_open() begin \n");
  status =  fx_media_open(&sdio_disk, "STM32_SDIO_DISK", fx_stm32_sd_driver, 0, media_memory, sizeof(media_memory));
  if (status != FX_SUCCESS)
  {
      DUBUG("fx_media_open() failed -- 0x%X \n", status);
      while(1){ }
  }
	DUBUG("fx_media_open() complete \n");
	asmdelay_msec(1000);
  
	
#if defined(FX_ENABLE_FAULT_TOLERANT)
  /* 启用容错模块 */
  DUBUG("fx_fault_tolerant_enable() begin \n");
  status =  fx_fault_tolerant_enable(&sdio_disk, fault_tolerant_memory, sizeof(fault_tolerant_memory));
  if (status != FX_SUCCESS)
  {
      DUBUG("fx_fault_tolerant_enable() failed -- 0x%X \n", status);
      while(1){ }
  }
  DUBUG("fx_fault_tolerant_enable() complete \n");
  asmdelay_msec(1000);
#endif  // FX_ENABLE_FAULT_TOLERANT
  

#if MEDIA_CHECK_ENABLE
  /* Check the media and correct all errors. */
	DUBUG("fx_media_check() begin \n");
	status = fx_media_check(&sdio_disk, sratch_memory, SRATCH_MEMORY_SIZE,
												FX_FAT_CHAIN_ERROR |
												FX_DIRECTORY_ERROR |
												FX_LOST_CLUSTER_ERROR, &detected_errors);
  DUBUG("media check -- 0x%X \n", (uint32_t)detected_errors);
  if (status != FX_SUCCESS)
  {
      DUBUG("media check failed -- 0x%X \n", status);
      while(1){ }
  }
  DUBUG("fx_media_check() complete \n");
	asmdelay_msec(1000);
#endif	


  /* Create a file called boot_count.dat in the root directory.  */
	DUBUG("fx_file_create() begin \n");
  status = fx_file_create(&sdio_disk, PATH);
  if (status != FX_SUCCESS) /* Check the create status.  */
  {
      /* Check for an already created status. This is expected on the
         second pass of this loop!  */
      if (status != FX_ALREADY_CREATED)
      {
          DUBUG("fx_file_create() failed -- 0x%X \n", status);
          while(1){ }
      }
  }
  DUBUG("fx_file_create() complete \n");
	asmdelay_msec(1000);
  
	
  /* 初始化测试数组 */
  for(uint16_t i = 0; i < TEST_DATA_LENGTH; i++) {
    test_data[i] = i;
  }

    
  /* Infinite loop */
  while (1) {

    /* 写方式打开文件, 文件打开后偏移为文件末尾 */
		DUBUG("fx_file_open() begin \n");
    status =  fx_file_open(&sdio_disk, &fx_file, PATH, FX_OPEN_FOR_WRITE);
    if (status != FX_SUCCESS)
    {
        DUBUG("fx_file_open() failed -- 0x%X \n", status);
        //while(1){ }
    }
		DUBUG("fx_file.fx_file_current_file_offset = %u \n", (uint32_t)fx_file.fx_file_current_file_offset);
		DUBUG("fx_file_open() complete \n");

		
//    /* 设置偏移位置 */
//		DUBUG("fx_file_seek() begin \n");
//    status =  fx_file_seek(&fx_file, test_count * TEST_DATA_LENGTH);
//    if (status != FX_SUCCESS)
//    {
//        DUBUG("fx_file_seek() failed -- 0x%X \n", status);
//        //while(1){ }
//    }
//		DUBUG("fx_file_seek() complete \n");

		
//    /* 读文件 */
//    status =  fx_file_read(&fx_file, &boot_count, sizeof(boot_count), &bw);
//    if(bw != sizeof(boot_count)) {
//      DUBUG("-- boot_count %d, ReadLen = 0x%X \n", boot_count, (int)bw);
//    }
//    if (status != FX_SUCCESS)
//    {
//      if(status == FX_END_OF_FILE) {
//      } else {
//        DUBUG("file read failed -- 0x%X \n", status);
//        while(1){ }
//      }
//    }

//    // print the boot count
//    DUBUG("boot_count: %d \n", boot_count);
//    
//    // update boot count
//    boot_count += 1;

//    /* 设置到起始位置写  */
//    status =  fx_file_seek(&fx_file, 0);
//    if (status != FX_SUCCESS)
//    {
//        DUBUG("set file oft failed -- 0x%X \n", status);
//        while(1){ }
//    }


    /* 向文件写入数据 */
		DUBUG("write test_count: %d \n", test_count);
    ((uint32_t *)test_data)[0] = 0x5555AAAA;                           // 第一个word
		((uint32_t *)test_data)[(TEST_DATA_LENGTH >> 2) - 2] = test_count++; // 倒数第二个word
    ((uint32_t *)test_data)[(TEST_DATA_LENGTH >> 2) - 1] = 0x5555AAAA; // 最后一个word
    if (status == FX_SUCCESS) {
			DUBUG("fx_file_write() begin \n");
      status = fx_file_write(&fx_file, test_data, sizeof(test_data));
      if (status != FX_SUCCESS)
      {
          DUBUG("fx_file_write() failed -- 0x%X \n", status);
          //while(1){ }
      }
			DUBUG("fx_file_write() complete \n");
    }
    
		
    /* 关闭文件 */
		DUBUG("fx_file_close() begin \n");
    status =  fx_file_close(&fx_file);
    if (status != FX_SUCCESS)
    {
        DUBUG("fx_file_close() failed -- 0x%X \n", status);
        //while(1){ }
    }
		DUBUG("fx_file_close() complete \n");

		
#if !defined(FX_ENABLE_FAULT_TOLERANT)    
    /* 保证文件写入全部生效 */
		DUBUG("fx_media_flush() begin \n");
    status = fx_media_flush(&sdio_disk);
    if (status != FX_SUCCESS)
    {
        DUBUG("fx_media_flush() failed-- 0x%X \n", status);
    }
		DUBUG("fx_media_flush() complete \n");
#endif  // !FX_ENABLE_FAULT_TOLERANT    

		
    /* 延时 */ 
    asmdelay_msec(1000);
  }
  
   /* 卸载SD卡 */
  status =  fx_media_close(&sdio_disk);
  if (status != FX_SUCCESS)
  {
      DUBUG("media close failed -- 0x%X \n", status);
      while(1){ }
  }
}

void assert_failed(uint8_t* file, uint32_t line)
{
    while(1){ }
}

/**
 *  获取当前时刻值
 */
uint32_t tx_time_get(void) {
  return system_tick;
}


/**
  * @brief  Configures SDIO IRQ channel.
  * @param  None
  * @retval None
  */
#if defined(SWLIB_STM32F4)
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);  
}
#endif // SWLIB_STM32F4
