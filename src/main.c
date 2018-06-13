/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "main.h"

uint16_t h1,h2,h3;
uint16_t dtc_u,dtc_v,dtc_w;
uint8_t en_gate;
int main(void)
{

	MAIN_INIT();

    h1=0;
    h2=0;
    h3=0;
    dtc_u = 50;
	dtc_v = 50;
	dtc_w = 50;
	en_gate = 0;
	for(;;)
	{
        HALL_GET_STATE(&h1,&h2,&h3);

	    TIM1->CCR1 = ((float)dtc_u/100.0F)*TIM1->ARR;
		TIM1->CCR2 = ((float)dtc_v/100.0F)*TIM1->ARR;
		TIM1->CCR3 = ((float)dtc_w/100.0F)*TIM1->ARR;
		HAL_GPIO_WritePin(GPIOB,PWM_EN_GATE,en_gate);

	    if(!FMSTR_DISABLE)
	    {
	      FMSTR_Poll();
	      FMSTR_Recorder();
	    }
	}
}
