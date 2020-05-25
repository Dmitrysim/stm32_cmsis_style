#include "stm32f4xx.h"

void timer_settings(void);
void delay_ms(uint16_t ms);

typedef struct
{
	uint16_t cnt; // �������, ��
	uint16_t time; // ����� �����, ��
	uint8_t flag; // 1 - ������ ��������
} soft_tim;

soft_tim st_LED_PD12 = {0, 100, 0};
soft_tim st_LED_PD13 = {0, 2000, 0};
//soft_tim st_LED_PD15 = {0, 12, 0};
soft_tim st_LED_PD15 = {0, 28, 0};

typedef struct
{
	uint16_t cnt; // �������, ��
	uint16_t time; // ����� �����, ��
	uint8_t flag; // 1 - ������ ��������
	uint8_t en; // 1 - ������ ���������
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
		//for(i=0; i<0xFFFFF; i++) {} // ��������
		delay_ms(100);
		GPIOD->ODR &= ~GPIO_ODR_ODR_12; // 1
		//for(i=0; i<0xFFFFF; i++) {} // ��������
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
		// ������� �� 0 �� ���� �� 3 ���. - 3/256 = 12 ��
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
		
		// ����� �� 1 �� 2 �� ���� �� 5 ���. - 5/180 = 28 ��
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

// ���������� ����������
void TIM7_IRQHandler(void)
{
	TIM7->SR &= ~TIM_SR_UIF; // ����� �����
	//GPIOD->ODR ^= GPIO_ODR_ODR_12;
	if(st_LED_PD12.cnt > 0) st_LED_PD12.cnt--; // ���������� ��������
	else
	{
		st_LED_PD12.cnt = st_LED_PD12.time; // ������������ ��������
		st_LED_PD12.flag = 1;
	}

	if(st_LED_PD13.cnt > 0) st_LED_PD13.cnt--; // ���������� ��������
	else
	{
		st_LED_PD13.cnt = st_LED_PD13.time; // ������������ ��������
		st_LED_PD13.flag = 1;
	}

	if(st_LED_PD14.en)
	{
		if(st_LED_PD14.cnt > 0) st_LED_PD14.cnt--; // ���������� ��������
		else
		{
			st_LED_PD14.cnt = st_LED_PD14.time; // ������������ ��������
			st_LED_PD14.flag = 1;
		}
	}
	else
	{
		GPIOD->ODR &= ~GPIO_ODR_ODR_14;
	}

	if(st_LED_PD15.cnt > 0) st_LED_PD15.cnt--; // ���������� ��������
	else
	{
		st_LED_PD15.cnt = st_LED_PD15.time; // ������������ ��������
		st_LED_PD15.flag = 1;
	}
	
}

void timer_settings(void) {
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // �������� ������������
	//GPIOD->MODER |= GPIO_MODER_MODER12_0; // �� �����
	//GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0; // �� �����
	GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0; // �� �����
	// 0: Output push-pull (reset state)
	// 00: 2 MHz Low speed
	
	// 16000000/PSC/ARR = ������� ������������ �������
	// 1 ms - 1 kHz
	// 16000000/PSC/ARR = 1000
	// 16000000/16/1000 = 1000 < 65535
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // ���. ������������
	TIM7->PSC = 16 - 1;
	//TIM7->PSC = 16000 - 1;
	TIM7->ARR = 1000;
	TIM7->DIER |= TIM_DIER_UIE; // ���� ����.
	NVIC_EnableIRQ(TIM7_IRQn); // ����. ����. � NVIC
	// ���������� ���������� ���������� 
	TIM7->CR1 |= TIM_CR1_CEN; // �������� ������
	
	//���
	GPIOD->MODER |= GPIO_MODER_MODER15_1; // �������������� �-�
	GPIOD->AFR[1] |= 0x20000000; // AFR[0] = AFRL, AFR[1] = AFRH, TIM4_CH4 - AF2
	//
	// ������� ���  - 100 Hz
	// ARR - ������ - 256 ������� �������
	// 16000000/PSC/ARR = 100
	// 16000000/PSC/256 = 100
	// 16000000/625/256 = 1000 < 65535
	// ����� ������
	// ������� ���  - 50 Hz (������ 20 ��)
	// ������������ �������� ~1��(���� �������) - 1,5��(�������) - 2,0��(������ �������)
	// �� 1 �� �� 2 �� - 180 ����.
	// � ��������� 1 ���� - 20�� * 180 = 3600
	// 16000000/PSC/3600 = 50
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // ���. ������������
	/*
	TIM4->PSC = 625 - 1;
	TIM4->ARR = 256;
	*/
	TIM4->PSC = 89 - 1;
	TIM4->ARR = 3600;
	
	// CC4S: Capture/Compare 1 selection - 00: CC4 channel is configured as output.
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; // 110: PWM mode 1
	TIM4->CCER |= TIM_CCER_CC4E; // ���������� ������ � �����
	TIM4->CR1 |= TIM_CR1_CEN; // �������� ������
	//TIM4->CCR4 = 0;
	TIM4->CCR4 = 180;
	
}

void delay_ms(uint16_t ms)
{
	// 16000000/PSC/ARR = ������� ������������ ������� 
	// 1 ms - 1 kHz
	// 16000000/PSC = 1000
	// PSC = 16000000/1000 = 16000 < 65535
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // ���. ������������
	TIM6->CR1 |= TIM_CR1_OPM; // ���������. ������.
	TIM6->PSC = 16000 - 1;
	TIM6->ARR = ms;
	TIM6->EGR |= TIM_EGR_UG; // ����������������� �������
	TIM6->SR &= ~ TIM_SR_UIF; // ������� ����
	TIM6->CR1 |= TIM_CR1_CEN; // �������� ������
	while(!(TIM6->SR & TIM_SR_UIF)) {} // ����
	
}

