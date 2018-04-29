#ifndef INIT_H
#define INIT_H

#include "stm32f4xx_adc.h"

void TIM4_ini(void);
void PWM_INx_init(void);
void ENx_init(void);
void ADC_initt(void);

void TIM5_ini(void);
void myDelay_microsec(uint32_t delay);
void myDelay_ms(uint32_t delay);

 void USART_2_init(void);


void TIM2_ini(void);

void TIM1_ini(void);



#define PWM_period 2100//3200



#endif


