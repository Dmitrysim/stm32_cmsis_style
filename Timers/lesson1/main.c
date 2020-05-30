#include "stm32f4xx.h"

void timer_settings(void);
void delay_ms(uint16_t ms);

typedef struct
{
	uint16_t cnt; 	// counter, ms
	uint16_t time; 	// time of count, ms
	uint8_t  flag; 	// 1 - timer worked
} soft_tim;

soft_tim st_LED_PD12 = {0, 100,  0};
soft_tim st_LED_PD13 = {0, 2000, 0};
//soft_tim st_LED_PD15 = {0, 12, 0};
soft_tim st_LED_PD15 = {0, 28,   0};

typedef struct
{
	uint16_t cnt; 	// counter, ms
	uint16_t time; 	// time of count, ms
	uint8_t  flag; 	// 1 - timer worked
	uint8_t  en; 	// 1 - work enable
} soft_tim2;

soft_tim2 st_LED_PD14 = {0, 100, 0, 0};

////////////////////////////////////////////////////////////////////////////
																	////////		
int main (void)
{
	unsigned long int i;
	
	timer_settings();

	while(1)
	{

		
		
		/*
		GPIOD->ODR |= GPIO_ODR_ODR_12; // 1
		//for(i=0; i<0xFFFFF; i++) {} // delay
		delay_ms(100);
		GPIOD->ODR &= ~GPIO_ODR_ODR_12; // 1
		//for(i=0; i<0xFFFFF; i++) {} // delay
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
		// brightness from 0 untill max for 3 sec. - 3/256 = 12 ms
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
		
		// servo from 1 untill 2 ms max for 5 sec. - 5/180 = 28 ms
		if(st_LED_PD15.flag)
		{
			st_LED_PD15.flag = 0;
			if(TIM4->CCR4 < 360)
			{
				TIM4->CCR4++;	
			}				
			else TIM4->CCR4 = 180;
		}
		
	}
	
}

//////////////////////////////////////////////////////////////////////////////
																	//////////

// interrupt handler
void TIM7_IRQHandler(void)
{
	TIM7->SR &= ~TIM_SR_UIF; // reset flag
	//GPIOD->ODR ^= GPIO_ODR_ODR_12;
	if(st_LED_PD12.cnt > 0) st_LED_PD12.cnt--; 		// decrease counter
	else
	{
		st_LED_PD12.cnt = st_LED_PD12.time; 		// counter overload
		st_LED_PD12.flag = 1;
	}

	if(st_LED_PD13.cnt > 0) st_LED_PD13.cnt--; 		// decrease counter
	else
	{
		st_LED_PD13.cnt = st_LED_PD13.time; 		// counter overload
		st_LED_PD13.flag = 1;
	}

	if(st_LED_PD14.en)
	{
		if(st_LED_PD14.cnt > 0) st_LED_PD14.cnt--; 	// decrease counter
		else
		{
			st_LED_PD14.cnt = st_LED_PD14.time; 	// counter overload
			st_LED_PD14.flag = 1;
		}
	}
	else
	{
		GPIOD->ODR &= ~GPIO_ODR_ODR_14;
	}

	if(st_LED_PD15.cnt > 0) st_LED_PD15.cnt--; 		// decrease counter
	else
	{
		st_LED_PD15.cnt = st_LED_PD15.time; 		// counter overload
		st_LED_PD15.flag = 1;
	}
	
}

void timer_settings(void) {
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; 													// enabled timing
	//GPIOD->MODER |= GPIO_MODER_MODER12_0; 												// output
	//GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0; 							// output
	GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0; 	// output
																							// 0: Output push-pull (reset state)
																							// 00: 2 MHz Low speed
	
	// 16000000/PSC/ARR = frequency of timer
	// 1 ms - 1 kHz
	// 16000000/PSC/ARR = 1000
	// 16000000/16/1000 = 1000 < 65535
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; 													// enabled timing
	TIM7->PSC = 16 - 1;
	//TIM7->PSC = 16000 - 1;
	TIM7->ARR = 1000;
	TIM7->DIER |= TIM_DIER_UIE; 															// enabled of interrupt
	NVIC_EnableIRQ(TIM7_IRQn); 																// enabled of interrupt NVIC
	// global interrupt resolution 
	TIM7->CR1 |= TIM_CR1_CEN; 																// enabled timer
	
	//PWM
	GPIOD->MODER |= GPIO_MODER_MODER15_1; 													// alternative function
	GPIOD->AFR[1] |= 0x20000000; 															// AFR[0] = AFRL, AFR[1] = AFRH, TIM4_CH4 - AF2
	
	// Frequency PWM  - 100 Hz
	// ARR - perid - 256 levels of brightness
	// 16000000/PSC/ARR = 100
	// 16000000/PSC/256 = 100
	// 16000000/625/256 = 1000 < 65535
	// servo
	// Frequency PWM  - 50 Hz (period 20 ms)
	// pulse duration ~1ms(first end) - 1,5ms(average) - 2,0ms(second end)
	// from 1 ms untill 2 ms - 180 degr.
	// with precision 1 degr - 20ms * 180 = 3600
	// 16000000/PSC/3600 = 50
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; 													// enabled timing
	/*
	TIM4->PSC = 625 - 1;
	TIM4->ARR = 256;
	*/
	TIM4->PSC = 89 - 1;
	TIM4->ARR = 3600;
	
	// CC4S: Capture/Compare 1 selection - 00: CC4 channel is configured as output.
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; 									// 110: PWM mode 1
	TIM4->CCER |= TIM_CCER_CC4E; 															// signal for pin
	TIM4->CR1 |= TIM_CR1_CEN; 																// timer enable
	//TIM4->CCR4 = 0;
	TIM4->CCR4 = 180;
	
}

void delay_ms(uint16_t ms)
{
	// 16000000/PSC/ARR = timer frequency 
	// 1 ms - 1 kHz
	// 16000000/PSC = 1000
	// PSC = 16000000/1000 = 16000 < 65535
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; 													// enabled timing
	TIM6->CR1 |= TIM_CR1_OPM; 																// single actuation
	TIM6->PSC = 16000 - 1;
	TIM6->ARR = ms;
	TIM6->EGR |= TIM_EGR_UG; 																// re-initialization of the timer
	TIM6->SR &= ~ TIM_SR_UIF; 																// reset the flag
	TIM6->CR1 |= TIM_CR1_CEN; 																// timer enable
	while(!(TIM6->SR & TIM_SR_UIF)) {} 														// wait
	
}

