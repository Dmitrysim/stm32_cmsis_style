#include "stm32f4xx.h"

void delay_ms(uint16_t ms)
{
	// 16000000/PSC/ARR = частота срабатывания таймера
	// 1 ms - 1 kHz
	// 16000000/PSC = 1000
	// PSC = 16000000/1000 = 16000 < 65535
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // вкл. тактирование
	TIM6->CR1 |= TIM_CR1_OPM; // однократн. срабат.
	TIM6->PSC = 16000 - 1;
	TIM6->ARR = ms;
	TIM6->EGR |= TIM_EGR_UG; // переинициализация таймера
	TIM6->SR &= ~ TIM_SR_UIF; // сбросим флаг
	TIM6->CR1 |= TIM_CR1_CEN; // включить таймер
	while(!(TIM6->SR & TIM_SR_UIF)) {} // ждем
	
}

typedef struct
{
	uint16_t cnt; // счетчик, мс
	uint16_t time; // время счета, мс
	uint8_t flag; // 1 - таймер сработал
} soft_tim;

soft_tim st_LED_PD12 = {0, 100, 0};
soft_tim st_LED_PD13 = {0, 2000, 0};
//soft_tim st_LED_PD15 = {0, 12, 0};
soft_tim st_LED_PD15 = {0, 28, 0};

typedef struct
{
	uint16_t cnt; // счетчик, мс
	uint16_t time; // время счета, мс
	uint8_t flag; // 1 - таймер сработал
	uint8_t en; // 1 - работа разрешена
} soft_tim2;

soft_tim2 st_LED_PD14 = {0, 100, 0, 0};

#include <stdio.h>
#include <string.h>
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000


struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
   if (DEMCR & TRCENA) {

while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return(ch);
}

//
enum measure_states
{
	measure_first,  // 1 измерение
	measure_second, // 2 измерение
	process_result // обработка результата
};

volatile uint8_t measure_states = measure_first;
uint16_t measure_1, measure_2, period;

int main (void)
{
	unsigned long int i;
	
	printf("Text\n\r");
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // включили тактирование
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // включили тактирование
	//GPIOD->MODER |= GPIO_MODER_MODER12_0; // на выход
	//GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0; // на выход
	GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0; // на выход
	// 0: Output push-pull (reset state)
	// 00: 2 MHz Low speed
	
	// 16000000/PSC/ARR = частота срабатывания таймера
	// 1 ms - 1 kHz
	// 16000000/PSC/ARR = 1000
	// 16000000/16/1000 = 1000 < 65535
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // вкл. тактирование
	TIM7->PSC = 16 - 1;
	//TIM7->PSC = 16000 - 1;
	TIM7->ARR = 1000;
	TIM7->DIER |= TIM_DIER_UIE; // разр прер.
	NVIC_EnableIRQ(TIM7_IRQn); // разр. прер. в NVIC
	// глобальное разрешение прерываний 
	TIM7->CR1 |= TIM_CR1_CEN; // включить таймер
	
	//ШИМ
	GPIOD->MODER |= GPIO_MODER_MODER15_1; // альтернативная ф-я
	GPIOD->AFR[1] |= 0x20000000; // AFR[0] = AFRL, AFR[1] = AFRH, TIM4_CH4 - AF2
	//
	// частота ШИМ  - 100 Hz
	// ARR - период - 256 уровней яркости
	// 16000000/PSC/ARR = 100
	// 16000000/PSC/256 = 100
	// 16000000/625/256 = 1000 < 65535
	// серво машина
	// частота ШИМ  - 50 Hz (период 20 мс)
	// длительность импульса ~1мс(одно крайнее) - 1,5мс(среднее) - 2,0мс(второе крайнее)
	// от 1 мс до 2 мс - 180 град.
	// с точностью 1 град - 20мс * 180 = 3600
	// 16000000/PSC/3600 = 50
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // вкл. тактирование
	/*
	TIM4->PSC = 625 - 1;
	TIM4->ARR = 256;
	*/
	TIM4->PSC = 89 - 1;
	TIM4->ARR = 3600;
	
	// CC4S: Capture/Compare 1 selection - 00: CC4 channel is configured as output.
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; // 110: PWM mode 1
	TIM4->CCER |= TIM_CCER_CC4E; // подключить сигнал к ножке
	TIM4->CR1 |= TIM_CR1_CEN; // включить таймер
	//TIM4->CCR4 = 0;
	//TIM4->CCR4 = 180;
	TIM4->CCR4 = 360;

	// измеритель 
	// TIM3__CH3 PB0
	// соединить PD15 (выход генератора) с PB0 (вход измерителя)
	GPIOB->MODER |= GPIO_MODER_MODER0_1; // альтернативная ф-я
	GPIOB->AFR[0] |= 0x00000002; // AFR[0] = AFRL, AFR[1] = AFRH, TIM3_CH3 - AF2
	//
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // вкл. тактирование
	// 16000000/16 = 1000000 (1 мкс)
	TIM3->PSC = 16 - 1; // счет в мкс
	TIM3->ARR = 0xFFFF; // максимальное значение
	//
	TIM3->CCMR2 |= TIM_CCMR2_CC3S_0; // 01: CC3 channel is configured as input, IC3 is mapped on TI3
	TIM3->CCER |= TIM_CCER_CC3E; // подключить сигнал к ножке
	TIM3->DIER |= TIM_DIER_CC3IE; // разр. прер. СС3 
	NVIC_EnableIRQ(TIM3_IRQn); // разр. прер. в NVIC
	TIM3->CR1 |= TIM_CR1_CEN; // включить таймер
	
	while(1)
	{

		
		
		/*
		GPIOD->ODR |= GPIO_ODR_ODR_12; // 1
		//for(i=0; i<0xFFFFF; i++) {} // задержка
		delay_ms(100);
		GPIOD->ODR &= ~GPIO_ODR_ODR_12; // 1
		//for(i=0; i<0xFFFFF; i++) {} // задержка
		delay_ms(100);
		*/
		if(st_LED_PD12.flag)
		{
			st_LED_PD12.flag = 0;
			GPIOD->ODR ^= GPIO_ODR_ODR_12;
		}
		if(st_LED_PD13.flag)
		{
			st_LED_PD13.flag = 0;
			GPIOD->ODR ^= GPIO_ODR_ODR_13;
			st_LED_PD14.en ^= 1;
		}
		if(st_LED_PD14.flag)
		{
			st_LED_PD14.flag = 0;
			GPIOD->ODR ^= GPIO_ODR_ODR_14;
		}

		/*
		// яркость от 0 до макс за 3 сек. - 3/256 = 12 мс
		if(st_LED_PD15.flag)
		{
			st_LED_PD15.flag = 0;
			if(TIM4->CCR4 < TIM4->ARR)
			{
				TIM4->CCR4++;	
			}				
			else TIM4->CCR4 = 0;
		}
		*/
		
		// серво от 1 до 2 мс макс за 5 сек. - 5/180 = 28 мс
		/*
		if(st_LED_PD15.flag)
		{
			st_LED_PD15.flag = 0;
			if(TIM4->CCR4 < 360)
			{
				TIM4->CCR4++;	
			}				
			else TIM4->CCR4 = 180;
		}
		*/
		
		
		// измеритель
		if(measure_states == process_result)
		{
			printf("measure_1 - %d, measure_2 - %d, period = %d mks \r\n", measure_1, measure_2, period);
			delay_ms(1000);
			measure_states = measure_first;
		}
		
	}
	
}

// обработчик прерывания
void TIM7_IRQHandler(void)
{
	TIM7->SR &= ~TIM_SR_UIF; // сброс флага
	//GPIOD->ODR ^= GPIO_ODR_ODR_12;
	if(st_LED_PD12.cnt > 0) st_LED_PD12.cnt--; // уменьшение счетчика
	else
	{
		st_LED_PD12.cnt = st_LED_PD12.time; // перезагрузка счетчика
		st_LED_PD12.flag = 1;
	}

	if(st_LED_PD13.cnt > 0) st_LED_PD13.cnt--; // уменьшение счетчика
	else
	{
		st_LED_PD13.cnt = st_LED_PD13.time; // перезагрузка счетчика
		st_LED_PD13.flag = 1;
	}

	if(st_LED_PD14.en)
	{
		if(st_LED_PD14.cnt > 0) st_LED_PD14.cnt--; // уменьшение счетчика
		else
		{
			st_LED_PD14.cnt = st_LED_PD14.time; // перезагрузка счетчика
			st_LED_PD14.flag = 1;
		}
	}
	else
	{
		GPIOD->ODR &= ~GPIO_ODR_ODR_14;
	}

	if(st_LED_PD15.cnt > 0) st_LED_PD15.cnt--; // уменьшение счетчика
	else
	{
		st_LED_PD15.cnt = st_LED_PD15.time; // перезагрузка счетчика
		st_LED_PD15.flag = 1;
	}
	
}

// обработчик прерывания
void TIM3_IRQHandler(void)
{
	if(TIM3->SR & TIM_SR_CC3IF)
	{
		TIM3->SR &= ~TIM_SR_CC3IF; // сброс флага

		if(measure_states == measure_first)
		{
			measure_1 = TIM3->CCR3; // сохр.рез-т
			measure_states = measure_second;
			TIM3->CCER |= TIM_CCER_CC3P; // 01: inverted/falling edge
		}
		else if(measure_states == measure_second)
		{
			measure_2 = TIM3->CCR3; // сохр.рез-т
			if(measure_2 > measure_1)
			{
				period = measure_2 - measure_1;
			}
			else
			{
				period = (uint32_t)measure_2 + 0x10000 - (uint32_t)measure_1;
			}
			measure_states = process_result;
			TIM3->CCER &= ~TIM_CCER_CC3P; // 00: noninverted/rising edge
		}
		else if(measure_states == process_result)
		{
			
		}
		
volatile uint8_t measure_states = measure_first;
uint16_t measure_1, measure_2, period;
		
		
	}

	/*
	if(TIM3->SR & TIM_SR_CC4IF)
	{
		TIM3->SR &= ~TIM_SR_CC4IF; // сброс флага
	}
	*/
	
}