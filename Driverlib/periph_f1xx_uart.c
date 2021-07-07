//*****************************************************************************
//
// uartstdio.c - Utility driver to provide simple UART console functions.
//
// Copyright (c) 2007-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the Tiva Utility Library.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "periph_f1xx_uart.h"



/* 初始化串口USART1, PA9, PA10 */ 
void Com_LowLevel_Init(void)
{
    GPIO_InitTypeDef  sGPIO_Init;

    /* Enable GPIO clock and AFIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    
    /* Configure USART Tx as alternate function push-pull */
    sGPIO_Init.GPIO_Mode    = GPIO_Mode_AF_PP;
    sGPIO_Init.GPIO_Pin     = GPIO_Pin_9;
    sGPIO_Init.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &sGPIO_Init);

    /* Configure USART Rx as input floating */
    sGPIO_Init.GPIO_Mode    = GPIO_Mode_IN_FLOATING;
    sGPIO_Init.GPIO_Pin     = GPIO_Pin_10;
    GPIO_Init(GPIOA, &sGPIO_Init);
}
