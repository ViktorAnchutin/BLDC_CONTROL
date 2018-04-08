#include "motor.h"
#include "as5048.h"
#include "math.h"
#include "init.h"

#include "stm32f4xx_spi.h"

int change;

/*




void dead_time(uint32_t time)
{
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	GPIO_ResetBits(GPIOA, GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_5);
	Delay(time);
}


void Use_HalfBridge(int num, int Transistor, int power) // num - 1,2,3;  Switch - High or Low; power - duty cycle from 0 to 142(or any other which was chosen)
{
	
	if(num ==1)
	{
		if(Transistor == High)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_5);		//IN1 = 1
			TIM4->CCR1 = power;			
		}
		if(Transistor == Low)
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);		//IN1 = 0
			TIM4->CCR1 = 1000;	
		}
	}
	
	else if(num ==2)
	{
		if(Transistor == High)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_1);		//IN2 = 1
			TIM4->CCR2 = power;			
		}
		if(Transistor == Low)
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);		//IN2 = 0
			TIM4->CCR2 = 1000;	
		}
	}
	
	else if(num ==3)
	{
		if(Transistor == High)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_2);		//IN3 = 1
			TIM4->CCR3 = power;			
		}
		if(Transistor == Low)
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_2);		//IN13= 0
			TIM4->CCR3 = 1000;	
		}		
	}
	
}





void Change_winding(void)
{
	dead_time(1000000);
	
	if (change == 0)
	{
	Use_HalfBridge(2, High, 71); // num - 1,2,3;  Transistor - High or Low , power from 0 to 142
	Use_HalfBridge(1, Low, 142); // num - 1,2,3;  Transistor - High or Low , power from 0 to 142
	change = 1;
	}
	else
	{
	Use_HalfBridge(1, High, 71); // num - 1,2,3;  Transistor - High or Low , power from 0 to 142
	Use_HalfBridge(3, Low, 142); // num - 1,2,3;  Transistor - High or Low , power from 0 to 142
	change = 0;
	}
	
}

void try_to_turn(void)
{
	
	
	Use_HalfBridge(1, High, 71);
	Use_HalfBridge(2, Low, 142);
	
}


*/






void Set_nRes_nSleep(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_4|GPIO_Pin_5);
}






#include "mylib.h"

float Vq;
float	Vd;
float Va, Vb, Vc, Va_1, Vb_1, Vc_1;
float theta, theta_elec_degrees;
float Vinv1,Vinv2, Vinv3;
float angle_init , error_angle_last;
 extern float angle ;
//extern float Speed;
//extern float alpha;

//CCR1 - A, CCR2 - B, CCR3 - C

void FOC_InitPosition(void) // establishing zero position, d-axis directed to A winding, theta = 90
{
	/*
	TIM4->CCR1 = (uint32_t)(12*PWM_period/Vdc) ; 
  TIM4->CCR2 = (uint32_t)(3*PWM_period/Vdc)  ;
  TIM4->CCR3 = (uint32_t)(3*PWM_period/Vdc)  ;
	
	myDelay_ms(1000);	
	*/
	// init angle was calculated once. Now it is used like starting point for electrical angles and engine does not need position initialization
	
	angle_init = 238.066;//average_angle() ;//get_angle();//average_angle() ;
	error_angle_last = 0;
	
}



float error_in_proc, er_mem, angle_mem,integral;

void FOC(float angle, float error_angle, float K_p, float K_d, float K_I, uint32_t dt)
{
	theta_elec_degrees = ((angle - angle_init)*Pole_Pairs + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90  
	theta = theta_elec_degrees*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians
	 
	
	Vd = 0; // !
	
	if(error_angle > 180)
	{
		error_angle = 360 - error_angle;
		error_angle = - error_angle;
	}
	if(error_angle < -180)
	{
		error_angle = 360 + error_angle;
		//error_angle = - error_angle;
	}
	
	
	error_in_proc = error_angle;
	if ((error_angle > 100)||(error_angle< -100)) 
	{
		er_mem = error_angle;
		angle_mem = angle;
	}
	integral = dt*0.000001*error_angle_last+integral;
	Vq = K_p*error_angle + ((error_angle - error_angle_last)/(dt*0.000001))*K_d + integral*K_I; //Speed; //
	error_angle_last = error_angle;
	
	
	
	
	if(Vq < -6) Vq = -6; // 6V = Vdc/2 , voltage limitation
	if(Vq > 6) Vq = 6;
	
	
	
	Va_1 = arm_cos_f32(theta);//cos(theta         );     
	Vb_1 = arm_cos_f32(theta - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	Vc_1 = arm_cos_f32(theta + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
	
	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
	
	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM4->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM4->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM4->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;
	
	
}





void sinus_control(float des_val_)
{
	theta_elec_degrees = ((des_val_)*11 + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90  
	theta = theta_elec_degrees*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians
	
	Vq=6;
	
	
	Va_1 = arm_cos_f32(theta);//cos(theta         );     
	Vb_1 = arm_cos_f32(theta - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	Vc_1 = arm_cos_f32(theta + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
	
	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
	
	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM4->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM4->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM4->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;
	
	
	
	
	
}




uint8_t started;
float step, step2;

void sinus_control_V2(float err)
{
	if(!started)
	{
		theta = 0;
		Va_1 = arm_cos_f32(theta);//cos(theta         );     
	Vb_1 = arm_cos_f32(theta - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	Vc_1 = arm_cos_f32(theta + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
	
	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
	
	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM4->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM4->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM4->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;
		started =1;
	}
	
	
	
	step = err*0.0002;
	step2 = step;
	//theta_elec_degrees = ((err)*11 + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90  
	if(step>0.006) { step=0.006;}
	if(step< -0.006) {step=-0.006; }
	theta = theta+step;
	
	
	Vq=6;
	
	
	Va_1 = arm_cos_f32(theta);//cos(theta         );     
	Vb_1 = arm_cos_f32(theta - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	Vc_1 = arm_cos_f32(theta + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
	
	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
	
	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM4->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM4->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM4->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;
	
	
	
	
	
}