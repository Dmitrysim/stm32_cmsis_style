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

void USART1_DMA_TX_init(uint32_t baudrate, uint32_t USART1_clock)
{
	// USART
	// 8N1 - 8 бит данные, безз паритета, 1 стоп-бит
	// скорость - 115200
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // вкл. тактирование
	// 
	// Bit 12 M: Word length - 0: 1 Start bit, 8 Data bits, n Stop bit
	// Bit 10 PCE: Parity control enable - 0: Parity control disabled
	// Bits 13:12 STOP: STOP bits 00: 1 Stop bit
	//USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // разрешили передачу, прием, приерывания при приеме
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE /*| USART_CR1_RXNEIE*/; // разрешили передачу, прием, приерывания при приеме
	NVIC_EnableIRQ(USART1_IRQn); // разр. прер. в NVIC
	USART1->BRR = USART1_clock/baudrate;
	//USART1->CR3 |= USART_CR3_DMAT; // Bit 7 DMAT: DMA enable transmitter
	USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; // Bit 7 DMAT: DMA enable transmitter
	USART1->CR1 |= USART_CR1_UE; // Разрешили работу USART
	// порт
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; // альтернативная ф-я
	GPIOB->AFR[0] |= 0x77000000; // AFR[0] = AFRL, AFR[1] = AFRH, USART1 - AF7

	// DMA2
	// USART1_TX: Stream 7 - Channel 4
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // вкл. тактирование
	//
	// Bits 27:25 CHSEL[2:0]: Channel selection - 100: channel 4 selected
	DMA2_Stream7->CR |= DMA_SxCR_CHSEL_2; // 4 канал
	// Bits 24:23 MBURST: Memory burst transfer configuration - 00: single transfer
	// Bits 22:21 PBURST[1:0]: Peripheral burst transfer configuration -  00: single transfer
	// Bits 19 CT: Current target (only in double buffer mode)
	// Bits 18 DBM: Double buffer mode - 0: No buffer switching at the end of transfer
	// Bits 17:16 PL[1:0]: Priority level - 00: Low
	// Bits 15 PINCOS: Peripheral increment offset size - 0: The offset size for the peripheral address calculation is linked to the PSIZE
	// Bits 14:13 MSIZE[1:0]: Memory data size - 00: byte (8-bit)
	// Bits 12:11 PSIZE[1:0]: Peripheral data size - 00: byte (8-bit)
	// Bits 10 MINC: Memory increment mode
	DMA2_Stream7->CR |= DMA_SxCR_MINC; // инкремент памяти
	// Bits 9 PINC: Peripheral increment mode - 0: Peripheral address pointer is fixed
	// Bits 8 CIRC: Circular mode - 0: Circular mode disabled
	// Bits 7:6 DIR[1:0]: Data transfer direction - 01: Memory-to-peripheral
	DMA2_Stream7->CR |= DMA_SxCR_DIR_0;
	// Bits 5 PFCTRL: Peripheral flow controller - 0: The DMA is the flow controller
	DMA2_Stream7->PAR = (uint32_t )(&USART1->DR);
	
	// USART1_RX: Stream 5 - Channel 4
	DMA2_Stream5->CR |= DMA_SxCR_CHSEL_2; // 4 канал
	DMA2_Stream5->CR |= DMA_SxCR_MINC; // инкремент памяти
	// Bits 7:6 DIR[1:0]: Data transfer direction - 00: Peripheral-to-memory
	DMA2_Stream5->CR |= DMA_SxCR_TCIE; // разр. прер.
	NVIC_EnableIRQ(DMA2_Stream5_IRQn); // разр. прер. в NVIC
	DMA2_Stream5->PAR = (uint32_t )(&USART1->DR);
	DMA2_Stream5->NDTR = 10;
	DMA2_Stream5->M0AR = (uint32_t )(uart_rx_dma);
	DMA2_Stream5->CR |= DMA_SxCR_EN; // Bits 0 EN: Stream enable / flag stream ready when read low
}

void USART1_tx_string_DMA(char * string)
{
	char msg2[200];
	
	DMA2_Stream7->M0AR = (uint32_t )string;
	DMA2_Stream7->NDTR = strlen(string);
	//DMA2_Stream7->NDTR = 10;
	sprintf(msg2,"DMA2_Stream7->PAR - %x\r\n", DMA2_Stream7->PAR); USART1_tx_string(msg2); 
	sprintf(msg2,"DMA2_Stream7->M0AR - %x\r\n", DMA2_Stream7->M0AR); USART1_tx_string(msg2); 
	sprintf(msg2,"DMA2_Stream7->NDTR - %d\r\n", DMA2_Stream7->NDTR); USART1_tx_string(msg2); 
	
	USART1_tx_string("Start\r\n");
	DMA2_Stream7->CR |= DMA_SxCR_EN; // Bits 0 EN: Stream enable / flag stream ready when read low
	sprintf(msg2,"DMA2->HISR - %x\r\n", DMA2->HISR); USART1_tx_string(msg2); 
	while(!(DMA2->HISR & DMA_HISR_TCIF7)) {} // ждем окончания передачи
	DMA2->HIFCR |= DMA_HIFCR_CTCIF7; // сбросим флаг
	//USART1->CR3 &= ~USART_CR3_DMAT; // Bit 7 DMAT: DMA enable transmitter
	USART1_tx_string("Stop\r\n");

}

// обработчик прерывания
void DMA2_Stream5_IRQHandler(void)
{
	DMA2->HIFCR |= DMA_HIFCR_CTCIF5; // сбросим флаг
	uart_rx_dma[10] = 0x00; //конец строки	
	sprintf(msg, "data = %s\r\n", uart_rx_dma); 	USART1_tx_string(msg);

}