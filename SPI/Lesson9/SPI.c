#include "global.h"

// SPI1
// PA5 - 
// PA6 - 
// PA7 - 
// PE3 - CS
void SPI1_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // вкл. тактирование
	// CR1
	// Bit 11 DFF: Data frame format - 0: 8-bit data frame format is selected for transmission/reception
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;// Bit 9 SSM: Software slave management, Bit 8 SSI: Internal slave select - отключаем сигнал NSS с ножки
	// Bit 7 LSBFIRST: Frame format - 0: MSB transmitted first
	// F=16000000
	// <= 10000000
	// BR[2:0]: Baud rate control - 000: fPCLK/2 - 5000000
	//SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
	SPI1->CR1 |= SPI_CR1_MSTR; //Bit 2 MSTR: Master selection
	SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA; // полярность - 1, работа по 2-му фронту
	// CR2
	// Bit 4 FRF: Frame format - 0: SPI Motorola mode
	SPI1->CR1 |= SPI_CR1_SPE; // разрешили работу интерфейса
	// порты
	GPIOA->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; // альтернативная ф-я
	GPIOA->AFR[0] |= 0x55500000; // AFR[0] = AFRL, AFR[1] = AFRH, SPI1 - AF5
	GPIOE->ODR |= GPIO_ODR_ODR_3; // 1 - не выбрано
	GPIOE->MODER |= GPIO_MODER_MODER3_0; // на выход
}

void SPI1_TX_RX(char * SPI1_TX_data, char * SPI1_RX_data, uint8_t SPI1_TX_RX_data_size)
{
	uint8_t i;
	GPIOE->ODR &= ~GPIO_ODR_ODR_3; // 0 - выбрано
	for(i=0; i<SPI1_TX_RX_data_size; i++)
	{
		//sprintf(msg, "*SPI1_TX_data %x\r\n", *(SPI1_TX_data)); USART1_tx_string(msg);
		SPI1->DR = *SPI1_TX_data++; // старт отправки
		//while(!(SPI1->SR & SPI_SR_RXNE)) {} // ожидаем окончание приема
		while(!(SPI1->SR & SPI_SR_TXE)) {} // ожидаем окончание приема
		//sprintf(msg, "SPI1->DR %x\r\n", SPI1->DR); USART1_tx_string(msg);
		*SPI1_RX_data++	= SPI1->DR;
		//sprintf(msg, "*SPI1_RX_data %x\r\n", *(SPI1_RX_data-1)); USART1_tx_string(msg);
	}
	GPIOE->ODR |= GPIO_ODR_ODR_3; // 1 - не выбрано
}