#ifndef __UART_H
#define __UART_H

void USART1_init(uint32_t baudrate, uint32_t USART1_clock);
void USART1_tx_byte(uint8_t byte);
void USART1_tx_string(char * string);
void USART1_tx_array(char * array, uint16_t array_size);


#endif