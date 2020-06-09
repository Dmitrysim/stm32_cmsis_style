#ifndef __GLOBAL_H
#define __GLOBAL_H

#define STM32F407xx
#include "stm32f4xx.h"
#include "uart.h"

#ifdef glob
 char msg[200];
 uint16_t cnt = 200;
 uint8_t arr[5] = {0, 1, 2, 3, 0};
#else
 // extern
 extern char msg[200];
 extern uint16_t cnt;
 extern uint8_t arr[5];
#endif

#endif