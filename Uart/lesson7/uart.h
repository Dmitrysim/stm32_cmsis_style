#ifndef __UART_H
#define __UART_H

void USART1_init(uint32_t baudrate, uint32_t USART1_clock);
void USART1_tx_byte(uint8_t byte);
void USART1_tx_string(char * string);
void USART1_tx_array(char * array, uint16_t array_size);

#define USART1_RX_buf_size 200

#ifdef glob
 // буфер под одно сообщение
 char USART1_RX_buf[USART1_RX_buf_size]; //
 uint16_t USART1_RX_buf_ptr = 0; // указатель на текущую позицию буфера
 uint8_t USART1_RX_buf_flag = 0; // флаг принятия сообщения
#else
// буфер под одно сообщение
 extern char USART1_RX_buf[USART1_RX_buf_size]; //
 extern uint16_t USART1_RX_buf_ptr; // указатель на текущую позицию буфера
 extern uint8_t USART1_RX_buf_flag; // флаг принятия сообщения#endif
#endif

#endif
