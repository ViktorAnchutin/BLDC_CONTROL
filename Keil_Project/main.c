#include "stm32f4xx.h" 
#include "as5048.h"

                 // Device header
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "mylib.h"
#include "init.h"
#include "motor.h"
#include <stdio.h>


	uint8_t mode;
float angle, angle_error, angle_test, angle1;
float des_val,des_val1;
uint32_t t1, t2, t3, t4, dt1, dt22, p, timex, t1_IMU, t2_IMU, dt_IMU;

float voltage, current;

float sin_x, cos_x, tv_g, t_g, t_d;

// moving average filter variables


uint8_t filled_ADC;
	uint16_t i_ADC; // counter for filling in
	float data_ADC;
	float arr_ADC[window_ADC],  angle_average, a_i_ADC,  des_val_raw; // variables for first order moving average
	uint16_t k_ADC; // counter
	uint32_t ADC_average;
	uint8_t flag_ADC;
  uint32_t sum_ADC;
	
	
	//moving average Roll
	
	uint8_t filled_Roll;
	uint32_t i_Roll; // counter for filling in
	float data_Roll;
	uint32_t arr_Roll[window_Roll],  Roll_average, a_i_Roll; // variables for first order moving average
	uint32_t k_Roll; // counter
	
	uint8_t flag_Roll;
  uint32_t sum_Roll;
	
	
	
	
	
	
	
	


uint16_t USART_count;
uint8_t IMU_data_ready;

uint16_t IMU_Recieve_Buf[11];
uint8_t IMU_count;
uint16_t USART_test_rec;
uint8_t IMU_data;

float Pitch, Roll, Yaw;
float Roll_raw, roll_sine, roll_cos, Roll_raw_test, roll_sine_test, roll_cos_test, Roll_test;

int main(void)
	
{
	
	
	
	//USART_2_init();
	SPI3_ini();
	user_button_init();
	
	
	TIM5_ini(); // Delay timer
	
	led15_ini();
	
	TIM4_ini(); // PWM timer
	
	
	PWM_ENx_ini();
	port_init();
	ADC_initt();
	Set_nRes_nSleep();
	GPIO_SetBits(GPIOA, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3); // EN1,2,3 to 1 enable all half-bridges
	FOC_InitPosition();
	
	
	

	
//		des_val = -36; ///////////////








	while(1)
	{
		
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
		{
			
		}			
		
			
	t1 = TIM5->CNT;
		
	angle = CQ_average_angle();//ThirdOrder_average();//average_angle();//	angle = get_angle();
		
  des_val = (float)ADC_average*360/4095;
  
	angle_error = des_val - angle;
	
if(mode==0)	FOC(angle, angle_error, 0.5,   0,  0.01,  dt1)	;
		
 
		
	if(mode==1) sinus_control_V2(angle_error);
		
	 if(mode==2)combined_control_V3(angle, angle_error, 0.2 , 0, 0, 0);
		
		t2 = TIM5->CNT;
	  dt1 = t2 - t1;
	

		
	
	}
}




void EXTI0_IRQHandler(void) {
	
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {		
			
			EXTI_ClearITPendingBit(EXTI_Line0);
		 NVIC_DisableIRQ(EXTI0_IRQn);
		
		mode++;
			if(mode>2) mode=0;
		
		NVIC_EnableIRQ(EXTI0_IRQn);
		
	}
	
}




void ADC_IRQHandler()
	{
		if(ADC_GetITStatus(ADC1,ADC_IT_EOC) !=RESET)
		{
			
			
			ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
			
			// averaging	
		if(!filled_ADC)
		{
			
			
				arr_ADC[i_ADC] = ADC_GetConversionValue(ADC1) ;
				sum_ADC = sum_ADC + arr_ADC[i_ADC];
				if(i_ADC < window_ADC - 1) 
				{
					i_ADC++;
				}
				else
				{
					ADC_average  = sum_ADC / (uint32_t)window_ADC ; // out of the filter
					filled_ADC = 1;
					k_ADC=0;
					
				}
			 
		}
		
		// 2) start filtering
		
		else
		{
			a_i_ADC = ADC_GetConversionValue(ADC1) ;
			des_val_raw = a_i_ADC*360/4095;
		//	flag_ADC = 1;
			sum_ADC = sum_ADC - arr_ADC[k_ADC] + a_i_ADC;
			ADC_average  = sum_ADC / (uint32_t)window_ADC ; // out of the filter
			arr_ADC[k_ADC] = a_i_ADC; // substitute thrown out value with new value for cycling
			k_ADC++;
			if(k_ADC >= window_ADC) k_ADC=0; // array loop
			
			
		}
		
		 
		
			//Speed = (float)(ADC_GetConversionValue(ADC1))*6/4095; // used for potentiometr speed control
		//	des_val_raw = ADC_GetConversionValue(ADC1);//*360/4095; // used for potentiometr angle tracking
			
			
			
			//alpha = (float)(ADC_GetConversionValue(ADC1))*360/4095;
			
			ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_480Cycles);
	    ADC_SoftwareStartConv(ADC1);
			//p++;
			
		}
		
	}  
	
	