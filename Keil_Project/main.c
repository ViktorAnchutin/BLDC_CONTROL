#include "stm32f4xx.h" 
#include "as5048.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "mylib.h"
#include "init.h"
#include "motor.h"
#include <stdio.h>


uint8_t mode;
float angle, angle_error, angle_test, angle1, angle_error_mem_in1, angle_error_mem_in2, angle_error_mem_in3, angle_error_mem_out1, angle_error_mem_out2, angle_error_mem_out3;
float des_val,des_val1;
uint32_t t1, t2, t3, t4, dt1, dt22, p, timex, t1_IMU, t2_IMU, dt_IMU;

float voltage, current;

float sin_x, cos_x, tv_g, t_g, t_d;

uint8_t ADC_started;

// moving average filter variables


uint8_t filled_ADC;
	uint16_t i_ADC; // counter for filling in
	float data_ADC;
	float arr_ADC[window_ADC],  angle_average, a_i_ADC,  des_val_raw; // variables for first order moving average
	uint16_t k_ADC; // counter
	float ADC_average;
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
float Roll_raw, roll_sine, roll_cos, Roll_raw_test, roll_sine_test, roll_cos_test, Roll_test, Roll_cor;

 moving_average_type ADC_filter;
 
 
 uint8_t first_ini;
 
 
 
 
 

int main(void)
	
{
	
	
	
	USART_2_init();
	SPI3_ini();
	user_button_init();
	
	
	TIM5_ini(); // Delay timer
		
	TIM4_ini(); // PWM timer	
	
	PWM_INx_init();
	ENx_init();
	ADC_initt();
	Set_nRes_nSleep();
	Set_ENx();
	FOC_InitPosition();
	
	des_val = 36; ///////////////


	while(1)
	{
		
		
		if(!ADC_filter.filled)
		{
			while(!ADC_filter.filled); // wait for adc filter ready
		} 
		
		
			
			t1 = TIM5->CNT;
		
		
	

	/*	
	if(IMU_data_ready)
		{
			
			Roll_raw = ((float)(IMU_Recieve_Buf[1]<<8|IMU_Recieve_Buf[0]))/32768*PI;	
			 roll_sine = arm_sin_f32(Roll_raw); 
				roll_cos = arm_cos_f32(Roll_raw);
			  Roll = atan2(roll_sine, roll_cos)*57.295779513082320876798154814105 - Roll_cor;
			
			
			
		}
			
		
		if(!first_ini)
		{
			Roll_cor = Roll;
			first_ini=1;
		}
		*/
		
		
		
	angle = CQ_average_angle();//ThirdOrder_average();//average_angle();//	angle = get_angle();
		
  des_val = ADC_average*360/4095;
  
	//angle_error = des_val - angle;
		
		angle_error = Roll;
		
	
	if(mode==0)	FOC(angle, angle_error, 1.1,   0,  0.01,  dt1)	;		
 
	if(mode==1) sinus_control_V2(angle_error, 6, 0.001, 0.05);
		
	if(mode==2)combined_control_V3(angle, angle_error, 6, 0.001, 0.05);
		
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
			
			ADC_average =  moving_average(&ADC_filter, (float)ADC_GetConversionValue(ADC1), 2000);
						
			ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_480Cycles);
	    
			ADC_SoftwareStartConv(ADC1);
									
		}
		
	}  
	
	
	
	
	void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_TXE)==SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
		
		
	}
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	//	USART_test_rec = (uint8_t)USART_ReceiveData(USART2);
		
		USART_test_rec = USART_ReceiveData(USART2);
		
		if(IMU_data==0)
		{
			if(USART_test_rec==0x55)
			{
				IMU_data=1;
			}
		}
		
		else if(IMU_data==1)
		{
			if(USART_test_rec==0x53)
			{
				IMU_data = 2;
			}
			else
			{
				IMU_data=0;
			}
		}
		
		else if (IMU_data==2)
		{
			if(IMU_count<6)
			{
			IMU_Recieve_Buf[IMU_count]=USART_test_rec;
			IMU_count++;
			}
			else
			{
				IMU_data_ready=1;
				IMU_count=0;
				IMU_data=0;
			}
		}
		
		
	}
	
	
}
	