#include "global.h"

// PB6 - USART1_TX
// PB7 - USART1_RX
void USART1_init(uint32_t baudrate, uint32_t USART1_clock)
{
	// USART
	// 8N1 - 8 бит данные, безз паритета, 1 стоп-бит
	// скорость - 115200
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // вкл. тактирование
	// 
	// Bit 12 M: Word length - 0: 1 Start bit, 8 Data bits, n Stop bit
	// Bit 10 PCE: Parity control enable - 0: Parity control disabled
	// Bits 13:12 STOP: STOP bits 00: 1 Stop bit
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // разрешили передачу, прием, приерывания при приеме
	NVIC_EnableIRQ(USART1_IRQn); // разр. прер. в NVIC
	USART1->BRR = USART1_clock/baudrate;
	USART1->CR1 |= USART_CR1_UE; // Разрешили работу USART
	// порт
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; // альтернативная ф-я
	GPIOB->AFR[0] |= 0x77000000; // AFR[0] = AFRL, AFR[1] = AFRH, USART1 - AF7

}	

void USART1_tx_byte(uint8_t byte)
{
	while(!(USART1->SR & USART_SR_TC)) {} // ждем готовности к передаче
	USART1->DR = byte;	
}

void USART1_tx_string(char * string)
{
	while(*string != 0x00) // проверка текущего байта
	{
		USART1_tx_byte(*string++); // передача текущего байта и переход на следующий элемент
	}
}

void USART1_tx_array(char * array, uint16_t array_size)
{
	uint16_t i;
	for(i = 0; i < array_size; i++) // проверка текущего байта
	{
		USART1_tx_byte(*array++); // передача текущего байта и переход на следующий элемент
	}
}

// обработчик прерывания
void USART1_IRQHandler(void)
{
	if(USART1->SR & USART_SR_RXNE)
	{
		// проверка ошибок
		if(USART1->SR & USART_SR_ORE)
		{
			sprintf(msg, "ORE err\r\n"); USART1_tx_string(msg);
		}
		if(USART1->SR & USART_SR_NE)
		{
			sprintf(msg, "NF err\r\n"); USART1_tx_string(msg);
		}
		if(USART1->SR & USART_SR_PE)
		{
			sprintf(msg, "PE err\r\n"); USART1_tx_string(msg);
		}
		if(USART1->SR & USART_SR_FE)
		{
			sprintf(msg, "FE err\r\n"); USART1_tx_string(msg);
		}
		
		//USART1_tx_byte(USART1->DR); // эхо печать
		USART1_RX_buf[USART1_RX_buf_ptr++] = USART1->DR;
		//delay_ms(100);
		// проверка на принятие сообщения полностью
		// AT - команды, CR-LF (0x0D 0x0A) (\r\n)
		if((USART1_RX_buf_ptr >= 2) && (USART1_RX_buf[USART1_RX_buf_ptr-2] == 0x0D) && (USART1_RX_buf[USART1_RX_buf_ptr-1] == 0x0A))
		{
			USART1_RX_buf[USART1_RX_buf_ptr] = 0x00; // конец строки
			//sprintf(msg, "%s\r\n", USART1_RX_buf); USART1_tx_string(msg);
			USART1_RX_buf_flag = 1;
		}
		
	}
}
