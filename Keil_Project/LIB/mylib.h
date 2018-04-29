#include "stm32f4xx_adc.h"



#define FILTER_BUF 2500


//-----------------------------------------------
//----------------------------------I2C defines from MPU library---------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------

#define MPU_6050_addr 0x68 
#define I2Cx_FLAG_TIMEOUT             ((uint32_t) 900) //0x1100
#define I2Cx_LONG_TIMEOUT             ((uint32_t) (300 * I2Cx_FLAG_TIMEOUT)) //was300
 

#define SENSORS_I2C_SCL_GPIO_PORT         GPIOB
#define SENSORS_I2C_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define SENSORS_I2C_SCL_GPIO_PIN          GPIO_Pin_10
#define SENSORS_I2C_SCL_GPIO_PINSOURCE    GPIO_PinSource10
 
#define SENSORS_I2C_SDA_GPIO_PORT         GPIOB
#define SENSORS_I2C_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define SENSORS_I2C_SDA_GPIO_PIN          GPIO_Pin_11
#define SENSORS_I2C_SDA_GPIO_PINSOURCE    GPIO_PinSource11

#define SENSORS_I2C_RCC_CLK               RCC_APB1Periph_I2C2
#define SENSORS_I2C_AF                    GPIO_AF_I2C2


#define SENSORS_I2C               I2C2

#define I2C_SPEED                 400000
#define I2C_OWN_ADDRESS           0x00
//-----------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------




typedef struct
{
	uint8_t filled; // filter filled flag
	
	uint8_t error; // error flag;
	
	uint8_t init; //
	
	float buffer[FILTER_BUF]; // buffer
	
	float sum; // sum of buf
	
	float output; // output of the filter
	
	uint16_t counter1; // counter for filling buffer in
	
	uint16_t counter2; // counter for filtration
	
	float input;
		  
} moving_average_type;





void TIM5_ini(void);
void led15_ini(void);
void myDelay_microsec(uint32_t delay);
void myDelay_ms(uint32_t delay);
float moving_average(moving_average_type* filter_x, float input, float window_f);

#define window_Roll 10000
