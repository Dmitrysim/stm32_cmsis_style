#ifndef __GLOBAL_H
#define __GLOBAL_H

#define STM32F407xx
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "uart.h"

#define digit_arr_size 10

#ifdef glob
 char msg[200];
 uint16_t cnt = 200;
 uint8_t arr[5] = {0, 1, 2, 3, 0};
 // парсинг
 char * ptr1, * ptr2; // указатели
 char digit_arr[digit_arr_size];
 uint8_t digit;
 uint8_t operation;
 // DMA
 char uart_rx_dma[20];
#else
 //
 extern void delay_ms(uint16_t ms);
 // extern
 extern char msg[200];
 extern uint16_t cnt;
 extern uint8_t arr[5];
 // парсинг
 extern char * ptr1, * ptr2; // указатели
 extern char digit_arr[digit_arr_size];
 extern uint8_t digit;
 extern uint8_t operation;
 // DMA
 extern char uart_rx_dma[20];
 
#endif

#endif
 