/*
 * drv_uart.c
 *
 * uart drivers for stm32f407.
 *
 *  Created on: 2019年9月24日
 *      Author: weycen
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "stm32f4xx_conf.h"
#include "periph_f4xx_uart.h"



// 璋冭瘯涓插彛
static struct_port uart1_tx = {
    .periphClk = RCC_AHB1Periph_GPIOA,
    .gpio = GPIOA,
    .pin = GPIO_Pin_9,
    .pinSource = GPIO_PinSource9,
    .pinAF = GPIO_AF_USART1,
};
static struct_port uart1_rx = {
    .periphClk = RCC_AHB1Periph_GPIOA,
    .gpio = GPIOA,
    .pin = GPIO_Pin_10,
    .pinSource = GPIO_PinSource10,
    .pinAF = GPIO_AF_USART1,
};
static struct_uart debug_uart = {
    .portTx = &uart1_tx, //  TX绠¤剼
    .portRx = &uart1_rx, //  RX绠¤剼
};

//
// 初始化UART的GPIO
//
void Com_LowLevel_Init(void)
{
    struct_uart *uart = &debug_uart;
  
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(uart->portTx->periphClk | uart->portRx->periphClk, ENABLE);

    /* Connect USART pins to AFx */
    GPIO_PinAFConfig(uart->portTx->gpio, uart->portTx->pinSource, uart->portTx->pinAF);
    GPIO_PinAFConfig(uart->portRx->gpio, uart->portRx->pinSource, uart->portRx->pinAF);

    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin   = uart->portTx->pin;
    GPIO_Init(uart->portTx->gpio, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = uart->portRx->pin;
    GPIO_Init(uart->portRx->gpio, &GPIO_InitStructure);
}
