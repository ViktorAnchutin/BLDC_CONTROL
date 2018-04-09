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

//#include "tm_stm32f4_delay.h"
	
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
	
	
	
	USART_2_init();
	SPI3_ini();
	// encoder_as5048_SPI3();
	//angle = get_angle();
	
	TIM5_ini(); // Delay timer
	
	led15_ini();
	
	TIM4_ini(); // PWM timer
	
	
	PWM_ENx_ini();
	port_init();
	ADC_initt();
	Set_nRes_nSleep();
	GPIO_SetBits(GPIOA, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3); // EN1,2,3 to 1 enable all half-bridges
	FOC_InitPosition();
	
	
	
	//angle_test = get_angle_once();
	
		des_val = -36; ///////////////



/*
  TIM4->CCR1 = 1000  ; 
  TIM4->CCR2 = 1500  ;
  TIM4->CCR3 = 2000  ; */


char str[30];
sprintf(str, "Hello world");


timex = TIM5->CNT; //init delay getting angle
t1_IMU = timex; //init measure time IMU

	while(1)
	{
		
				t1 = TIM5->CNT;
		
		
		if(IMU_data_ready)
		{
			t2_IMU = TIM5->CNT;
			dt_IMU = t2_IMU-t1_IMU;
			t1_IMU = t2_IMU;
			Roll_raw = ((float)((IMU_Recieve_Buf[1]<<8)|IMU_Recieve_Buf[0]))/32768*PI;
			//Roll_raw= ((float)((IMU_Recieve_Buf[1]<<8)))/32768*PI;
			//Pitch = ((float)((IMU_Recieve_Buf[3]<<8)|IMU_Recieve_Buf[2]))/32768*180;
			//Yaw = ((float)((IMU_Recieve_Buf[5]<<8)|IMU_Recieve_Buf[4]))/32768*180;
			
			
					
			
			
			  roll_sine = arm_sin_f32(Roll_raw); 
				roll_cos = arm_cos_f32(Roll_raw);
			  Roll = atan2(roll_sine, roll_cos)*57.295779513082320876798154814105 ;
			
			/*
			
			if(!filled_Roll)
		{
			
			
				arr_Roll[i_Roll] = Roll ;
				sum_Roll = sum_Roll + arr_Roll[i_Roll];
				if(i_Roll < window_Roll - 1) 
				{
					i_Roll++;
				}
				else
				{
					Roll_average  = sum_Roll / (uint32_t)window_Roll ; // out of the filter
					filled_Roll = 1;
					k_Roll=0;
					
				}
			 
		}
		
		// 2) start filtering
		
		else
		{
			a_i_Roll = Roll ;
			
		//	flag_ADC = 1;
			sum_Roll = sum_Roll - arr_Roll[k_Roll] + a_i_Roll;
			Roll_average  = sum_Roll / (uint32_t)window_Roll ; // out of the filter
			arr_Roll[k_Roll] = a_i_Roll; // substitute thrown out value with new value for cycling
			k_Roll++;
			if(k_Roll >= window_Roll) k_Roll=0; // array loop
			
			
		}
		*/
			
			IMU_data_ready=0;
		}
		
		
	//	myDelay_ms(500);
		//USATRT2_SendStr(str);
	//	USART_SendData(USART2,0xFF);
	//	USART_ReceiveData(USART2);
		
			
			/*
			if(IMU_data_ready)
		{
			t2_IMU = TIM5->CNT;
			dt_IMU = t2_IMU-t1_IMU;
			t1_IMU = t2_IMU;
			Roll_raw_test = ((float)((IMU_Recieve_Buf[1]<<8)|IMU_Recieve_Buf[0]))/32768*PI;
			//Pitch = ((float)((IMU_Recieve_Buf[3]<<8)|IMU_Recieve_Buf[2]))/32768*180;
			//Yaw = ((float)((IMU_Recieve_Buf[5]<<8)|IMU_Recieve_Buf[4]))/32768*180;
			
			  roll_sine_test = arm_sin_f32(Roll_raw_test); 
				roll_cos_test = arm_cos_f32(Roll_raw_test);
		
			  Roll_test = atan2(roll_sine, roll_cos)*57.295779513082320876798154814105 ;
			
			if(!filled_Roll)
		{
			
			
				arr_Roll[i_Roll] = IMU_Recieve_Buf[0] ;
				sum_Roll = sum_Roll + arr_Roll[i_Roll];
				if(i_Roll < window_Roll - 1) 
				{
					i_Roll++;
				}
				else
				{
					Roll_average  = (uint16_t)(sum_Roll / (uint32_t)window_Roll) ; // out of the filter
					filled_Roll = 1;
					k_Roll=0;
					
				}
			 
		}
		
		// 2) start filtering
		
		else
		{
			a_i_Roll = IMU_Recieve_Buf[0] ;
			
		//	flag_ADC = 1;
			sum_Roll = sum_Roll - arr_Roll[k_Roll] + a_i_Roll;
			Roll_average  = (uint16_t)(sum_Roll / (uint32_t)window_Roll) ; // out of the filter
			arr_Roll[k_Roll] = a_i_Roll; // substitute thrown out value with new value for cycling
			k_Roll++;
			if(k_Roll >= window_Roll) k_Roll=0; // array loop
			
			
			Roll_raw = ((float)((IMU_Recieve_Buf[1]<<8)|(uint16_t)Roll_average))/32768*PI;
			roll_sine = arm_sin_f32(Roll_raw); 
			roll_cos = arm_cos_f32(Roll_raw);
			Roll = atan2(roll_sine, roll_cos)*57.295779513082320876798154814105 ;
			
		}
		
		
		
			
			IMU_data_ready=0;
		}
		
		*/
		
		
	
	
	
	
	angle = CQ_average_angle();//ThirdOrder_average();//average_angle();//	angle = get_angle();
   


		if(((TIM5->CNT) - timex) > 10*1000) // get discrete angle from encoder
		{
			angle1 = get_angle();
			//des_val = (float)ADC_average*360/4095 + 100;
			timex = TIM5->CNT;
		}
		
		
				
	des_val = (float)ADC_average*360/4095+100-360; // des val with Roll
	//	des_val = (float)ADC_average*360/4095 + 100; // des val with Encoder
		
  //angle_error = des_val - angle;
	//angle_error = des_val + Roll;
	
	angle_error =  Roll;
	
	//---------------------------------------------------

	//FOC(angle, Roll, 0.1,   0,  0.1,  dt1)	;
	FOC(angle, angle_error, 0.2,   0,  0.2,  dt1)	;
	//=----------------------------------------------
	
	
	
	
	
	
//	sinus_control(des_val);
	//sinus_control_V2(angle_error);
//	 combined_control_V3(angle, angle_error, 0.2 , 0, 0, 0);
		t2 = TIM5->CNT;
	  dt1 = t2 - t1;
	

		
	
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
	
	
	
	
	
	



/*	
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
					ADC_average  = sum_ADC / window_ADC ; // out of the filter
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
			ADC_average  = sum_ADC / window_ADC ; // out of the filter
			arr_ADC[k_ADC] = a_i_ADC; // substitute thrown out value with new value for cycling
			k_ADC++;
			if(k_ADC >= window_ADC) k_ADC=0; // array loop
			
			
		}
		*/
