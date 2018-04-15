#include "stm32f4xx_adc.h"



#define FILTER_BUF 2500



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
