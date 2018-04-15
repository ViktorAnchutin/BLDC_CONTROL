#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "mylib.h"



void TIM5_ini(void) /// Time measuring timer
{
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		//base settings
	TIM_TimeBaseInitTypeDef TIM5_Base;
	TIM5_Base.TIM_Period = 0xFFFFFFFF;
  TIM5_Base.TIM_Prescaler = 84-1; // 1MHz
  TIM5_Base.TIM_ClockDivision = TIM_CKD_DIV1 ; //
  TIM5_Base.TIM_CounterMode = TIM_CounterMode_Up;
  TIM5_Base.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit(TIM5, &TIM5_Base);
	TIM_Cmd(TIM5, ENABLE);
}






void led15_ini(void)
{

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef LED_pin;
	GPIO_StructInit(&LED_pin);
	LED_pin.GPIO_Pin = GPIO_Pin_15;
	LED_pin.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOD, &LED_pin);
}













 //Delay


uint32_t time1;


void myDelay_microsec(uint32_t delay)
{
	time1 = TIM5->CNT;
	while( ((TIM5->CNT) - time1) < delay) {}
}


void myDelay_ms(uint32_t delay)
{
	time1 = TIM5->CNT;
	while( ((TIM5->CNT) - time1) < 1000*delay) {}
	
}
	






void USATRT2_SendStr(char* str_p)
{
	uint16_t i = 0;
	while(str_p[i]!=0)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET){}
		USART_SendData(USART2,str_p[i]);
		i++;
	}
}




	
	














	uint32_t SYSCLK_Frequency_; /*!<  SYSCLK clock frequency expressed in Hz */
  uint32_t HCLK_Frequency_;   /*!<  HCLK clock frequency expressed in Hz   */
  uint32_t PCLK1_Frequency_;  /*!<  PCLK1 clock frequency expressed in Hz  */
  uint32_t PCLK2_Frequency_; 
	
void get_clock(void)
{
		RCC_ClocksTypeDef Clocks;
	RCC_GetClocksFreq(&Clocks) ;
	
	SYSCLK_Frequency_ = Clocks.SYSCLK_Frequency;
	HCLK_Frequency_ = Clocks.HCLK_Frequency;
	PCLK1_Frequency_ = Clocks.PCLK1_Frequency;
	PCLK2_Frequency_ = Clocks.PCLK2_Frequency;
}





float moving_average(moving_average_type* filter_x, float input, float window_f)
{

	if(!filter_x->init)
	{
		if(window_f > FILTER_BUF - 1)
			{
				filter_x->error=1;
				while(1){}
			}
				
		filter_x->init=1;		
			
	}
	
		if(!filter_x->filled)
		{
			
			
			filter_x->buffer[filter_x->counter1] = input ;
			filter_x->sum = filter_x->sum + filter_x->buffer[filter_x->counter1];
				if(filter_x->counter1 < window_f - 1) 
				{
					filter_x->counter1++;
				}
				else
				{
					filter_x->output  = filter_x->sum / window_f ; // out of the filter
					filter_x->filled = 1;
					filter_x->counter2=0;
					
					
				}
			 
		}
		
		// 2) start filtering
		
		else
		{
			
			
		
			filter_x->sum = filter_x->sum - filter_x->buffer[filter_x->counter2] + input;
			filter_x->output  = filter_x->sum / window_f ; // out of the filter
			filter_x->buffer[filter_x->counter2] = input; // substitute thrown out value with new value for cycling
			filter_x->counter2++;
			if(filter_x->counter2 >= window_f) filter_x->counter2=0; // array loop
			
			
		}
	
	
	return filter_x->output;
	
}








