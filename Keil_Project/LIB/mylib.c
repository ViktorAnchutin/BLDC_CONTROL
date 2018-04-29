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
















void I2cMaster_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;

    // Enable I2Cx clock 
  RCC_APB1PeriphClockCmd(SENSORS_I2C_RCC_CLK, ENABLE);

  // Enable I2C GPIO clock 
  RCC_AHB1PeriphClockCmd(SENSORS_I2C_SCL_GPIO_CLK | SENSORS_I2C_SDA_GPIO_CLK, ENABLE);

  // Configure I2Cx pin: SCL ----------------------------------------
  GPIO_InitStructure.GPIO_Pin =  SENSORS_I2C_SCL_GPIO_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  // Connect pins to Periph 
  GPIO_PinAFConfig(SENSORS_I2C_SCL_GPIO_PORT, SENSORS_I2C_SCL_GPIO_PINSOURCE, SENSORS_I2C_AF);  
  GPIO_Init(SENSORS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  // Configure I2Cx pin: SDA ----------------------------------------
  GPIO_InitStructure.GPIO_Pin = SENSORS_I2C_SDA_GPIO_PIN; 

  // Connect pins to Periph 
  GPIO_PinAFConfig(SENSORS_I2C_SDA_GPIO_PORT, SENSORS_I2C_SDA_GPIO_PINSOURCE, SENSORS_I2C_AF);  
  GPIO_Init(SENSORS_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);  
  
  I2C_DeInit(SENSORS_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_OWN_ADDRESS;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
    
  // Enable the I2C peripheral 
  I2C_Cmd(SENSORS_I2C, ENABLE);  
    
  // Initialize the I2C peripheral 
  I2C_Init(SENSORS_I2C, &I2C_InitStructure);
  
  return;
}




//---------------------------------------------------------------------------------------MPU------------------------------------------------
//--------------------------------------
//--------------------------
//-------------------
//--------------
//------------
//-------
//---
//

void CLEAR_ADDR_BIT(void)
{
I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR1);
I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR2);
}


 //----------------------------------------------------------Wait for flag
void WAIT_FOR_FLAG(uint32_t flag, uint8_t value)
 {	 
      while(I2C_GetFlagStatus(SENSORS_I2C, flag) != value) {}
 }

 
 
 
 
 //----------------------------------------------------------Write register
 
 
 unsigned long ST_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
  uint32_t  result = 0;
  uint32_t  i = 0; // i = RegisterLen;
    
//  RegisterValue = RegisterValue + (RegisterLen - 1);

  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET);// chek whether there is communication ongoing on the bus or not
  
  /* Start the config sequence */
  I2C_GenerateSTART(SENSORS_I2C, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET);

  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET); //wait for flag ADDR in SR1
  
  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR1);
  I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR2);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET);

  /* Transmit the first address for write operation */
  I2C_SendData(SENSORS_I2C, RegisterAddr);

//  while (i--)
//  {
//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
//  
//    /* Prepare the register value to be sent */
//    I2C_SendData(SENSORS_I2C, *RegisterValue--);
//  }
  
  for(i=0; i<(RegisterLen); i++)
  {
    /* Wait for address bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET);
  
    /* Prepare the register value to be sent */
    I2C_SendData(SENSORS_I2C, RegisterValue[i]);
  }  
   
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_BTF, SET);
	
  
  /* End the configuration sequence */
  I2C_GenerateSTOP(SENSORS_I2C, ENABLE);  
  
  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return result;  
}






void mpu_ini(void)
{
	unsigned char data[6];

 /*   // Reset device. -------------------------------------------------------------------------
    data[0] = BIT_RESET;
    ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x6B, 1, data);
    Delay(100); */

    /* Wake up chip. -------------------------------------------------------------------------*/
    data[0] = 0x00;
    ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x6B, 1, data);
    
		//mpu_set_gyro_fsr(2000)) reg 1B ---> 0x18-------------------------------------------------------------------------
		data[0] = 0x18;
		ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x1B, 1, data);
    
    //mpu_set_accel_fsr(2) reg 0x1C ---> 0x00
		data[0] = 0x00;
		ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x1C, 1, data);
    
    
		
		//mpu_set_lpf-------------------------------------------------------------------------
		data[0] = 0x00;//0x03;
		ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x1A, 1, data); 
    
		
		
		
  /*  
    //mpu_set_sample_rate(50)-------------------------------------------------------------------------
		uint16_t rate = 50;
		data[0] = 1000 / rate - 1;
    ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x19, 1, data);
		/// Automatically set LPF to 1/2 sampling rate. ---> 20Hz --> 0x04 
		data[0] = 0x04; //20Hz
		ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x1A, 1, data);
   
    
		//mpu_configure_fifo(0)) Select which sensors are pushed to FIFO.
        
    // choose clock source
		 data[0] = 0x01; 				//PLL with X axis gyroscope reference
		 ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x6B, 1, data);
		 */
		 
	/*	 // FIFO enable
		data[0] = 0x40;  
		ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x6A, 1, data); */
		 
		 
		 
 /*		// Push both gyro and accel data into the FIFO. FIFO enable, reg 0x23 
		data[0] = 0x78;  // gyro+accel ---> fifo
		ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x6B, 1, data); */
		
		// Select clock sourse ---> GYRO_PLL
		//data[0] = 0x01; //GYRO_X_PLL
		//ST_Sensors_I2C_WriteRegister(MPU_6050_addr, 0x6B, 1, data);
		
		
	
		
			
		//dmp_state = 0;
}

unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
  uint32_t i=0, result = 0;
  
   
  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET);
  
  /* Start the config sequence */
  I2C_GenerateSTART(SENSORS_I2C, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET);
  
  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);

  /* Wait for the addr to be sent */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET);

  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT();
  
  /* Wait for ack */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET);
  
  /* Transmit the register address to be read */
  I2C_SendData(SENSORS_I2C, RegisterAddr);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET);  

  /*!< Send START condition a second time */  
  I2C_GenerateSTART(SENSORS_I2C, ENABLE);
  
  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET);
  
  /*!< Send address for read */
  I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Receiver);  
  
  /* Wait for the addr to be sent */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET);
  
  if (RegisterLen == 1) 
  {
    /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
    
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT();
    
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
    
    /* Wait for the RXNE bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET);
    
    RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C); //read data from buffer
  } 
  else if( RegisterLen == 2) 
  {
     /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
    
   /* Set POS bit */ 
   SENSORS_I2C->CR1 |= I2C_CR1_POS;//0x0800
   
   /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
   CLEAR_ADDR_BIT(); 
   
   /* Wait for the buffer full bit to be set */
   WAIT_FOR_FLAG (I2C_FLAG_BTF, SET);
   
   /*!< Send STOP Condition */
   I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

   /* Read 2 bytes */
   RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
   RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
  } 
  else if( RegisterLen == 3)
  {
    CLEAR_ADDR_BIT();
    
    /* Wait for the buffer full bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_BTF, SET);
    /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
    /* Read 1 bytes */
    RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(SENSORS_I2C, ENABLE);        
    /* Read 1 bytes */
    RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
    /* Wait for the buffer full bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET);
    /* Read 1 bytes */
    RegisterValue[2] = I2C_ReceiveData(SENSORS_I2C);  
  }  
  else /* more than 2 bytes */
  { 
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT();
    
    for(i=0; i<(RegisterLen); i++)
    {
      if(i==(RegisterLen-3))
      {
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_BTF, SET);
        
        /*!< Disable Acknowledgment */
        I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
        
        /*!< Send STOP Condition */
        I2C_GenerateSTOP(SENSORS_I2C, ENABLE);        
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
        
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET);
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);  
        goto endReadLoop;
      }
            
      /* Wait for the RXNE bit to be set */
      WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET);
      RegisterValue[i] = I2C_ReceiveData(SENSORS_I2C); 
    }   
  } 
  
endReadLoop:  
  /* Clear BTF flag */
  I2C_ClearFlag(SENSORS_I2C, I2C_FLAG_BTF);
  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET);  
  /*!< Re-Enable Acknowledgment to be ready for another reception */
  I2C_AcknowledgeConfig(SENSORS_I2C, ENABLE);
  //Disable POS -- TODO
  SENSORS_I2C->CR1 &= ~I2C_CR1_POS;  
     
  /* Return the byte read from sensor */
  return result;
}


