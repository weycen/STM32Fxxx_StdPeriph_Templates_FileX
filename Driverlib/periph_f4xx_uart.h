/*
 * periph_f4xx_uart.c
 *
 * uart drivers includes for stm32f4xx.
 *
 *  Created on: 2019年9月24日
 *      Author: weycen
 */
#ifndef	PERIPH_F4XX_UART_H__
#define PERIPH_F4XX_UART_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "stm32f4xx_conf.h"
#include "periph_f4xx_gpio.h"


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif


/* UART 外设结构体 */ 
typedef struct
{
    uint32_t             periphClk;
    USART_TypeDef       *periph;

    struct_port         *portTx;     //  TX管脚
    struct_port         *portRx;     //  RX管脚
    
    USART_InitTypeDef   *uartInit;
}struct_uart;





// extern function declare.
extern void Com_LowLevel_Init(void);
    
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* PERIPH_F4XX_UART_H__  */

