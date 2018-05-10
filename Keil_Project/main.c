#include "stm32f4xx.h" 
#include "as5048.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "mylib.h"
#include "init.h"
#include "motor.h"
#include <stdio.h>


#define IMU_I2C      1 
#define IMU_UART     0            // 0 - MPU I2C tim2, 1- MPU UART

#ifdef IMU_I2C
	
	#ifdef IMU_UART
	#define  IMU_UART  0
	#endif

#endif






uint8_t mode;
float angle, angle_error, angle_test, angle1, angle_error_mem_in1, angle_error_mem_in2, angle_error_mem_in3, angle_error_mem_out1, angle_error_mem_out2, angle_error_mem_out3;
float des_val,des_val1;
uint32_t t1, t2, t3, t4, dt1, dt22, p, timex, t1_IMU, t2_IMU, dt_IMU, t1_1 , dt_1, t1_2 , dt_2, t1_3 , dt_3, t1_4 , dt_4;

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
	
	uint8_t flag_Roll, started_MPU6050;
  uint32_t sum_Roll;
	
	float K_p, K_d;
	
	
	
	
	
	


uint16_t USART_count;
uint8_t IMU_data_ready;

uint16_t IMU_Recieve_Buf[11];
uint8_t IMU_count;
uint16_t USART_test_rec;
uint8_t IMU_data;

float Pitch, Roll, Yaw;
float Roll_raw, roll_sine, roll_cos, Roll_raw_test, roll_sine_test, roll_cos_test, Roll_test, Roll_cor;
uint8_t dif_ready;



 moving_average_type ADC_filter;
 
 
 uint8_t first_ini;
 
 unsigned char accel_data[6], gyro_data[6];
	
	int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, gyro_x1, gyro_y1, gyro_z1;
	int32_t	gyro_y_cor_sum, gyro_x_cor_sum, gyro_z_cor_sum;
	float gyro_x_cor, gyro_y_cor, gyro_z_cor;
	float acc_total_vector, angle_pitch_acc, angle_roll_acc, angle_pitch_gyro, angle_roll_gyro, acc_x1;
  float gyro_x_1;
	uint8_t ready;
 uint32_t cnt, time_1;
 
 
 
 
 
 
 
 
 
 
 
 
 
//main----------------------------------------------------------------------------------------------------------------------------------------------------- 
 

int main(void)
	
{
	
	#ifdef IMU_UART
	
	USART_2_init();
	
	#endif
	
	
	
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

	
	

#ifdef IMU_I2C

  I2cMaster_Init();
	mpu_ini();



//*PWM_Sound "Put into intial position"----------------------

TIM4->CCR1 = 0  ; 
  TIM4->CCR2 = 0 ;
  TIM4->CCR3 =0;


	
	TIM4->CCR1 = 500 ;
	
		TIM4->ARR = 1400000;		
		myDelay_ms(500);		
		TIM4->ARR = 0;
		myDelay_ms(500);
		TIM4->ARR = 1400000;		
		myDelay_ms(500);	
		TIM4->ARR = 0;
		myDelay_ms(500);
		TIM4->ARR = 1400000;		
		myDelay_ms(500);
		TIM4->ARR = 0;
		myDelay_ms(500);
		TIM4->ARR = 1400000;
		myDelay_ms(500);
		TIM4->ARR = 0;
		myDelay_ms(500);
		TIM4->ARR = 1400000;
		
    TIM4->CCR1 = 0 ;		
		myDelay_ms(1000);
		
//----------------------		
		
	
			
	// calculate gyro bias value -------------------------------------------------------------------------------------------------------------		
			
			
			for(uint16_t i=0; i<1000; i++)
			{
			
				
				
				t1 = TIM5->CNT;
			ST_Sensors_I2C_ReadRegister(MPU_6050_addr, 0x43, 2, gyro_data );
		
					
			gyro_x = (uint16_t)gyro_data[0]<<8 | (uint16_t)gyro_data[1];	
		//	gyro_y = (uint16_t)gyro_data[2]<<8 | (uint16_t)gyro_data[3];
		//	gyro_z = (uint16_t)gyro_data[4]<<8 | (uint16_t)gyro_data[5];
			
			gyro_x_cor_sum += gyro_x;                                              //Add the gyro x offset to the gyro_x_cal variable
			//gyro_y_cor_sum += gyro_y;                                              //Add the gyro y offset to the gyro_y_cal variable
		//	gyro_z_cor += gyro_z;  
				
				
				
			while(TIM5->CNT - t1 < 1000){}
				
			
			}
			
			
			gyro_x_cor = (float)gyro_x_cor_sum/1000;                                                 
		//	gyro_y_cor = (float)gyro_y_cor_sum/1000;                                                 
		//	gyro_z_cor = (float)gyro_z_cor_sum/1000;
			
			started_MPU6050 = 1; //
			
		//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------	
			
			
			
			
			
	// "ready sound"		--------------
		
TIM4->CCR1 = 500 ; // ready sound
TIM4->ARR = 1400000;			
myDelay_ms(1000);			
			
		//----------------------------


// make TIM4 ready for FOC PWM ----
			
TIM4->CCR1 = 0 ;	
TIM4->ARR = PWM_period-1;

//--------

TIM2_ini(); //IMU cycle timer / 1000 microsec


ready=1; // ready flag

#endif






TIM1_ini(); // FOC cycle timer / 100 microsec
	
















// while loop -----------------------------------------------------

	while(1)
	{
		
		
	}
}

//----------------------------------------------------------------




// FOC interrupt ---------------------------------------------------------------

void TIM1_UP_TIM10_IRQHandler(void)
{
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {		
			
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
			
			if(ready)
			{
			//t1_2 = TIM5->CNT;			
			angle = CQ_average_angle();
			//des_val = ADC_average*360/4095;
				K_d = ADC_average*200/4095;
			angle_error = des_val - angle;
			FOC(angle, -angle_pitch_gyro, 2.5,   K_d,  0,  dt_1)	;
		//	dt_2 = TIM5->CNT - t1_2;
			
			GPIO_ToggleBits(GPIOB, GPIO_Pin_2);
			
			dt_1 = TIM5->CNT - t1_1;
			t1_1 = TIM5->CNT;
			}
		}
		
	}

	//-------------------------------------------------------------------------------------
	

#ifdef IMU_I2C
	
	// I2C IMU interrupt --------------------------------------------------------------------------------------------------

 void TIM2_IRQHandler(void)
 {
	 if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {		
			
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
			
		 cnt++;
		 
			if(started_MPU6050)
			{
			t1_3 = TIM5->CNT;			
		
			ST_Sensors_I2C_ReadRegister(MPU_6050_addr, 0x43, 2, gyro_data );	
			gyro_x = (uint16_t)gyro_data[0]<<8 | (uint16_t)gyro_data[1];	
			gyro_x_1 = (float)gyro_x - gyro_x_cor;
			angle_pitch_gyro += ((gyro_x_1)/16.4)*0.001;	
				
				
			
			}
			
			dt_4 = TIM5->CNT - t1_4;
			t1_4 = TIM5->CNT;
			
		}
 }
// ------------------------------------------------------------------------------------------------

#endif
 
 
 
	

// User button interrupt ----------------------------------------------------
 
void EXTI0_IRQHandler(void) {
	
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {		
			
			EXTI_ClearITPendingBit(EXTI_Line0);
		 NVIC_DisableIRQ(EXTI0_IRQn);
		
		mode++;
			if(mode>2) mode=0;
		
		NVIC_EnableIRQ(EXTI0_IRQn);
		
	}
	
}

//------------------------------------------------------------





// ADC interrupt ---------------------------------------------------------

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
	//----------------------------------------------------------------------------------------------------------
	
	
	
	
	#ifdef IMU_UART
	
	// USART IMU(100 Hz) interrupt ---------------------------------------------------------------------
	
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

//-------------------------------------------------------------------------------------------------------------------
#endif










	