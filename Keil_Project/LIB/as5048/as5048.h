#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "math.h"
#include "arm_math.h"



#define window 100
#define window_ADC 2000


#define CS3_ON() GPIO_ResetBits(GPIOC, GPIO_Pin_9)
#define CS3_OFF() GPIO_SetBits(GPIOC, GPIO_Pin_9)






/*
uint8_t filled_ADC;
	uint16_t i_ADC; // counter for filling in
	float data_ADC;
	float arr_ADC[window],  angle_average, a_i_ADC,  des_val_raw; // variables for first order moving average
	uint16_t k_ADC; // counter
	uint32_t ADC_average;
	uint8_t flag_ADC;
  uint32_t sum_ADC;
*/



/*


typedef struct
{
  uint8_t filled;              

  uint16_t i_buf;     // Scounter for buffering
                                     

  uint16_t window_av   //Specifies the speed for the selected pins.
                                       

  uint16_t arr[window_av];   // Specifies the operating output type for the selected pins.
                                       

  GPIOPuPd_TypeDef GPIO_PuPd;     // Specifies the operating Pull-up/Pull down for the selected pins.
                                       
}MovingAveage_TypeDef;

*/
	







void encoder_as5048_SPI3(void);
void SPI3_ini(void);
float get_angle(void);
float average_angle(void) ;
float get_angle_once(void);
float  SecondOrder_average(void);
float ThirdOrder_average(void);
float CQ_average_angle(void);
