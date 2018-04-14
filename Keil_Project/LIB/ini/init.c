#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "init.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"




void TIM4_ini(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_TimeBaseInitTypeDef timer_init;
	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Period = PWM_period-1;// ---> 20kHz
	timer_init.TIM_Prescaler = 2-1; // ---> 42 MHz
	TIM_TimeBaseInit(TIM4, &timer_init);
	
	//channel 1;	
	TIM_OCInitTypeDef tim_oc_init1;
	TIM_OCStructInit(&tim_oc_init1);
	tim_oc_init1.TIM_Pulse = 0;  
	tim_oc_init1.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init1.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM4, &tim_oc_init1);// 
	
	
	//channel 2;
	TIM_OCInitTypeDef tim_oc_init2;
	TIM_OCStructInit(&tim_oc_init2);
	tim_oc_init2.TIM_Pulse = 0;  
	tim_oc_init2.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init2.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM4, &tim_oc_init2);// 
	
	//channel 3;
	TIM_OCInitTypeDef tim_oc_init3;
	TIM_OCStructInit(&tim_oc_init3);
	tim_oc_init3.TIM_Pulse = 0;  
	tim_oc_init3.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init3.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM4, &tim_oc_init3);// 
	
	
	TIM_Cmd(TIM4, ENABLE);
	
	//	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);// Interrupts for Update
	//NVIC_EnableIRQ(TIM4_IRQn);
	
}



// PWM out for 12th pin

void PWM_ENx_ini(void) // INx, IN - PWMpins
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	
	//EN1 --> PD12--------------------------
	GPIO_InitTypeDef ledinit_AF;//
	GPIO_StructInit(&ledinit_AF);// 
	ledinit_AF.GPIO_Mode = GPIO_Mode_AF;//
	ledinit_AF.GPIO_Pin = GPIO_Pin_12;// 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);// 
	GPIO_Init(GPIOD, &ledinit_AF);// 
	
	//EN2 --> PD13--------------------------
	GPIO_InitTypeDef ledinit_AF2;//
	GPIO_StructInit(&ledinit_AF2);// 
	ledinit_AF2.GPIO_Mode = GPIO_Mode_AF;//
	ledinit_AF2.GPIO_Pin = GPIO_Pin_13;// 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);// 
	GPIO_Init(GPIOD, &ledinit_AF2);//
	
	
	//EN3 --> PD14--------------------------
	GPIO_InitTypeDef ledinit_AF3;//
	GPIO_StructInit(&ledinit_AF2);// 
	ledinit_AF3.GPIO_Mode = GPIO_Mode_AF;//
	ledinit_AF3.GPIO_Pin = GPIO_Pin_14;// 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);// 
	GPIO_Init(GPIOD, &ledinit_AF3);//
		
}



void port_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_InitTypeDef ledinit;
	GPIO_StructInit(&ledinit);
	ledinit.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5  ;
	ledinit.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_Init(GPIOC,&ledinit);
	GPIO_Init(GPIOA,&ledinit);
	
	
	//Pin 0 - Button
GPIO_InitTypeDef GPIO_InitDef;
GPIO_InitDef.GPIO_Pin = GPIO_Pin_0;
//Mode output
GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
//Output type push-pull
GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
//With pull down resistor
GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_DOWN;
//50MHz pin speed
GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
 
//Initialize pin on GPIOA port
GPIO_Init(GPIOA, &GPIO_InitDef);
	
	
	
	/*
	GPIO_InitTypeDef ledinit2;
	GPIO_StructInit(&ledinit2);
	ledinit2.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	ledinit2.GPIO_Mode = GPIO_Mode_OUT;*/
	

	
//	GPIO_Init(GPIOD, &ledinit2); // TENPORARY FOR TEST!!
	
	
}


void ADC_initt(void) // PB1
{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	GPIO_InitTypeDef GPIO_AI;
	GPIO_StructInit(&GPIO_AI);
	GPIO_AI.GPIO_Pin = GPIO_Pin_1;
	GPIO_AI.GPIO_Mode = GPIO_Mode_AN;
	GPIO_AI.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_AI.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_AI);
	
	
	
	ADC_CommonInitTypeDef ADC_init;
	ADC_InitTypeDef ADC_InitStructure;

	ADC_StructInit(&ADC_InitStructure);
	ADC_CommonStructInit(&ADC_init);
	ADC_CommonInit (&ADC_init);
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_480Cycles);
	ADC_SoftwareStartConv(ADC1);
	
}


void PWM_INx_ini(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	
	//IN1 --> PD12--------------------------
	GPIO_InitTypeDef ledinit_AF;//
	GPIO_StructInit(&ledinit_AF);// 
	ledinit_AF.GPIO_Mode = GPIO_Mode_AF;//
	ledinit_AF.GPIO_Pin = GPIO_Pin_12;// 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);// 
	GPIO_Init(GPIOD, &ledinit_AF);// 
	
	//IN2 --> PD13--------------------------
	GPIO_InitTypeDef ledinit_AF2;//
	GPIO_StructInit(&ledinit_AF2);// 
	ledinit_AF2.GPIO_Mode = GPIO_Mode_AF;//
	ledinit_AF2.GPIO_Pin = GPIO_Pin_13;// 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);// 
	GPIO_Init(GPIOD, &ledinit_AF2);//
	
	
	//IN3 --> PD14--------------------------
	GPIO_InitTypeDef ledinit_AF3;//
	GPIO_StructInit(&ledinit_AF2);// 
	ledinit_AF3.GPIO_Mode = GPIO_Mode_AF;//
	ledinit_AF3.GPIO_Pin = GPIO_Pin_14;// 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);// 
	GPIO_Init(GPIOD, &ledinit_AF3);//
		
}


 void USART_2_init(void) //PD6,PD5
 {
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	 
	GPIO_InitTypeDef USART2_ini;//
	USART2_ini.GPIO_Mode = GPIO_Mode_AF;//
	USART2_ini.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;// 
	USART2_ini.GPIO_Speed = GPIO_Speed_2MHz;
  USART2_ini.GPIO_OType = GPIO_OType_PP;
  USART2_ini.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOD, &USART2_ini);// 
	 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);// 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);//
	 
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE) ;
	 
	USART_InitTypeDef USART2_user;
	USART2_user.USART_BaudRate= 115200;
	USART2_user.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 USART2_user.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	 USART2_user.USART_Parity = USART_Parity_No;
	 USART2_user.USART_StopBits = USART_StopBits_1;
	 USART2_user.USART_WordLength = USART_WordLength_8b;
	 
	 USART_Init(USART2, &USART2_user);
	 
	 NVIC_EnableIRQ(USART2_IRQn);
	 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	 
	 
	 
	 USART_Cmd(USART2, ENABLE);
 }

void user_button_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef User_But_ini;//
	User_But_ini.GPIO_Mode = GPIO_Mode_IN;//
	User_But_ini.GPIO_Pin = GPIO_Pin_0;// 
	User_But_ini.GPIO_Speed = GPIO_Speed_2MHz;
  User_But_ini.GPIO_OType = GPIO_OType_PP;
  User_But_ini.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA, &User_But_ini);//
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);
	
	
	
	
	NVIC_EnableIRQ(EXTI0_IRQn);
	
}