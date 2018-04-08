#ifndef INIT_H
#define INIT_H

#include "stm32f4xx_adc.h"

void TIM4_ini(void);
void PWM_ENx_ini(void);
void port_init(void);
void ADC_initt(void);
void PWM_INx_ini(void);
void TIM5_ini(void);
void myDelay_microsec(uint32_t delay);
void myDelay_ms(uint32_t delay);

 void USART_2_init(void);

#define PWM_period 2100//3200



#endif


