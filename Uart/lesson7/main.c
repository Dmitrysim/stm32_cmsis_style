#define glob
#include "global.h"
#include "uart.h"

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

// энкодер
uint16_t CNT3_last = 0;

// кнопка
enum key_states
{
	key_off,
	key_bounce,
	key_on
};

typedef struct
{
	volatile uint32_t * key_port; // порт кнопки
	uint16_t pin_mask; // маска пина
	//
	uint8_t key_state; // состояние кнопки
	//
	uint16_t bounce_cnt; // счетчик времени дребезга, мс
	uint16_t bounce_time; // время дребезга, мс
	//
	uint8_t key_pressed_flag; // флаг нажатия на кнопку
	//
	//
	uint16_t repeat_cnt; // счетчик времени автоповтора, мс
	uint16_t repeat_time; // время автоповтора, мс
	//
	uint16_t hold_cnt; // счетчик времени удержания, мс
	uint16_t hold_time; // время удержания, мс
	uint16_t hold_time2; // время удержания, мс
	uint16_t hold_time3; // время удержания, мс
} key;

//key key_USER = {&(GPIOA->IDR), GPIO_IDR_IDR_0, key_off, 0, 100, 0};
//key key_USER = {&(GPIOA->IDR), GPIO_IDR_IDR_0, key_off, 0, 100, 0, 0, 700};
//key key_USER = {&(GPIOA->IDR), GPIO_IDR_IDR_0, key_off, 0, 100, 0, 0, 700, 0, 2000};
key key_USER = {&(GPIOA->IDR), GPIO_IDR_IDR_0, key_off, 0, 100, 0, 0, 700, 0, 2000, 4000, 6000};

int main (void)
{
	unsigned long int i;
	
	printf("Text\n\r");
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // включили тактирование
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

/*
	// измеритель периода
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
	*/
	
	/*
	// частотомер
	// PD2 - TIM3_ETR
	GPIOD->MODER |= GPIO_MODER_MODER2_1; // альтернативная ф-я
	GPIOD->AFR[0] |= 0x00000200; // AFR[0] = AFRL, AFR[1] = AFRH, TIM3_ETR - AF2
	// слейв таймер
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // вкл. тактирование
	// 16000000/16 = 1000000 (1 мкс)
	TIM3->PSC = 0; // счет импульсов
	TIM3->ARR = 0xFFFF; // максимальное значение
	TIM3->EGR |= TIM_EGR_UG; // инициализация
	TIM3->SR &= ~TIM_SR_UIF;
	// Bit 15 ETP: External trigger polarity - 0: ETR is noninverted, active at high level or rising edge
	// ETPS: External trigger prescaler - 00: Prescaler OFF
	// Bits 11:8 ETF[3:0]: External trigger filter - 0000: No filter, sampling is done at fDTS
	// ITR1 (TS = 001) - мастер таймер TIM2
	TIM3->SMCR |= TIM_SMCR_TS_0; 
	TIM3->SMCR |= TIM_SMCR_ECE;
	TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_2; //101: Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high. 
	TIM3->CR1 |= TIM_CR1_CEN; // включить таймер
	// мастер таймер
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // вкл. тактирование
	// 16000000/16000/1000 = 1 с
	TIM2->PSC = 16000-1; // счет импульсов
	TIM2->ARR = 1000; // максимальное значение
	TIM2->CR1 |= TIM_CR1_OPM; // однократный зап.
	TIM2->CR2 |= TIM_CR2_MMS_0; //001: Enable - the Counter enable signal, CNT_EN, is used as trigger output (TRGO).
	// начальный запуск
	TIM2->EGR |= TIM_EGR_UG; // инициализация
	TIM2->SR &= ~TIM_SR_UIF;
	TIM2->CR1 |= TIM_CR1_CEN;
	// соединить PD15 (выход генератора) с PD2 (вход измерителя)
	*/
	
	// энкодер
	// C (средний) - GND
	// A B - PB4/PB5 - TIM3_CH1/CH2
	GPIOB->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1; // альтернативная ф-я
	GPIOB->AFR[0] |= 0x00220000; // AFR[0] = AFRL, AFR[1] = AFRH, TIM3_CH1/2 - AF2
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0; // 01: Pull-up
	//
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // вкл. тактирование
	TIM3->PSC = 0; // счет импульсов
	TIM3->ARR = 0xFFFF; // максимальное значение
	//
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0; // 01: CC1 channel is configured as input, IC1 is mapped on TI1
	TIM3->CCMR1 |= TIM_CCMR1_CC2S_0; // 01: CC2 channel is configured as input, IC2 is mapped on TI2
	TIM3->CR1 |= TIM_CR1_CKD_1; // 10: tDTS = 4 ? tCK_INT
	TIM3->CCMR1 |= TIM_CCMR1_IC1F; // 1111: fSAMPLING=fDTS/32, N=8
	TIM3->CCMR1 |= TIM_CCMR1_IC2F; // 1111: fSAMPLING=fDTS/32, N=8
	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // подключить сигнал к ножке
	//TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // 011: Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges
	TIM3->SMCR |= TIM_SMCR_SMS_0; // 001: Encoder mode 1 - Counter counts up/down on TI2FP2 edge depending on TI1FP1
	TIM3->CR1 |= TIM_CR1_CEN; // включить таймер
	
	// кнопка PA0
	// ножка н авход при сбросе
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1; // подтяжка на GND
	// по прерываниям
	// PA0 подкл к EXTI0
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// SYSCFG_EXTICR1 
	// EXTIx[3:0]: EXTI x configuration (x = 0 to 3) = 0000: PA[x] pin
	EXTI->IMR |= EXTI_IMR_MR0; // разрешили прерывание
	EXTI->RTSR |= EXTI_RTSR_TR0; // нарастающий фронт
	NVIC_EnableIRQ(EXTI0_IRQn); // разр. прер. в NVIC
	
	// USART
	USART1_init(115200L, 16000000L);
	//USART1_tx_byte('%');
	USART1_tx_string("Hello Programmer!\r\n");
	//sprintf(msg, "%d %d %d %d %d\r\n", arr[0], arr[1], arr[2], arr[3], arr[4]); 
	USART1_tx_array((char *)arr, 5);
	
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
		/*
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
		*/

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
		
		
		/*
		// измеритель периода
		if(measure_states == process_result)
		{
			printf("measure_1 - %d, measure_2 - %d, period = %d mks \r\n", measure_1, measure_2, period);
			delay_ms(1000);
			measure_states = measure_first;
		}
		*/
		
		/*
		// измеритель 
		if(!(TIM2->CR1 & TIM_CR1_CEN))
		{
			printf("F - %d Hz \r\n", TIM3->CNT);
			// следующий запуск
			TIM3->EGR |= TIM_EGR_UG; // инициализация
			TIM3->SR &= ~TIM_SR_UIF;
			TIM2->EGR |= TIM_EGR_UG; // инициализация
			TIM2->SR &= ~TIM_SR_UIF;
			TIM2->CR1 |= TIM_CR1_CEN;
			
		}
		*/
	
	/*
		// энкодер
		if(TIM3->CNT != CNT3_last)
		{
			CNT3_last = TIM3->CNT;
			printf("CNT - %d \r\n", CNT3_last);
		}
		*/
		
		// кнопка
		/*
		if(key_USER.key_pressed_flag)
		{
			key_USER.key_pressed_flag = 0;
			GPIOD->ODR ^= GPIO_ODR_ODR_12;
		}
		*/
		
		/*
		if(key_USER.key_pressed_flag)
		{
			GPIOD->ODR |= GPIO_ODR_ODR_12;
		}
		else GPIOD->ODR &= ~GPIO_ODR_ODR_12;
		if(key_USER.key_pressed_flag == 2)
		{
			key_USER.key_pressed_flag = 0;
			GPIOD->ODR |= GPIO_ODR_ODR_13;
		}
		*/
		
		/*
		sprintf(msg, "cnt = %d\r\n", cnt++); USART1_tx_string(msg);
		delay_ms(1000);
		*/
		
		if(USART1_RX_buf_flag)
		{
			sprintf(msg, "%s\r\n", USART1_RX_buf); USART1_tx_string(msg);
			// парсинг
			// LED,1,ON (OFF)
			if(strstr(USART1_RX_buf, "LED") != NULL)
			{
				sprintf(msg, "\"LED\" found\r\n"); USART1_tx_string(msg);
				ptr1 = strchr(USART1_RX_buf, ','); // ищем 1-ю ','
				ptr2 = strchr(ptr1+1, ','); // ищем 2-ю ','
				sprintf(msg, "ptr1 = %p, ptr2 = %p\r\n", ptr1, ptr2); USART1_tx_string(msg);
				//
				memset(digit_arr, 0x00, digit_arr_size); // обнуляем массив
				strncpy(digit_arr, ptr1+1, ptr2 - (ptr1 + 1));
				sprintf(msg, "digit_arr = %s\r\n", digit_arr); USART1_tx_string(msg);
				digit = atoi(digit_arr); // бинарное число
				sprintf(msg, "digit binary = %d\r\n", digit); USART1_tx_string(msg);
				//
				if(strstr(ptr2+1, "ON") != NULL) operation = 1;
				else if(strstr(ptr2+1, "OFF") != NULL) operation = 0;
				else
				{
						sprintf(msg, "operation not correct\r\n"); USART1_tx_string(msg);
				}
				sprintf(msg, "operation = %d\r\n", operation); USART1_tx_string(msg);
				// действие
				if((digit == 1) && (operation == 0)) GPIOD->ODR &= ~GPIO_ODR_ODR_12; // выкл
				else if((digit == 1) && (operation == 1)) GPIOD->ODR |= GPIO_ODR_ODR_12; // выкл
				if((digit == 2) && (operation == 0)) GPIOD->ODR &= ~GPIO_ODR_ODR_13; // выкл
				else if((digit == 2) && (operation == 1)) GPIOD->ODR |= GPIO_ODR_ODR_13; // выкл
			}
			USART1_RX_buf_flag = 0;
			USART1_RX_buf_ptr = 0; // очистка буфера
			USART1_RX_buf[0] = 0x00; // конец строки
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
	
	// кнопка
	/*
	if(key_USER.key_state == key_off)
	{
		if(*key_USER.key_port & key_USER.pin_mask)
		{
			key_USER.key_state = key_bounce;
			key_USER.bounce_cnt = key_USER.bounce_time;
		}
	}
	else*/ if(key_USER.key_state == key_bounce)
	{
		if(key_USER.bounce_cnt) key_USER.bounce_cnt--;
		else
		{
			if(*key_USER.key_port & key_USER.pin_mask)
			{
				key_USER.key_state = key_on;
				key_USER.key_pressed_flag = 1;
				//
				//key_USER.repeat_cnt = key_USER.repeat_time;
				//
				key_USER.hold_cnt = key_USER.hold_time;
			}
			else
			{
				key_USER.key_state = key_off;
			}
		}
	}
	else if(key_USER.key_state == key_on)
	{
		if(!(*key_USER.key_port & key_USER.pin_mask))
		{
			key_USER.key_state = key_off;
			key_USER.key_pressed_flag = 0;
			//
			EXTI->PR |= EXTI_PR_PR0; // сбросили флаг
			EXTI->IMR |= EXTI_IMR_MR0; // разрешили прерывание
		}
		//
		/*
		if(key_USER.repeat_cnt) key_USER.repeat_cnt--;
		else
		{
			key_USER.key_pressed_flag = 1;
			key_USER.repeat_cnt = key_USER.repeat_time; 
		}
		*/
		//
		if(key_USER.hold_cnt) key_USER.hold_cnt--;
		else
		{
			key_USER.key_pressed_flag = 2;
		}
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


void EXTI0_IRQHandler(void)
{
	EXTI->PR |= EXTI_PR_PR0; // сброс флага
	EXTI->IMR &= ~ EXTI_IMR_MR0; // запретили прерывания
	key_USER.key_state = key_bounce;
}