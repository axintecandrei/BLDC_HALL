/*
 * mcu_init.c
 *
 *  Created on: 20 mar. 2018
 *      Author: uidp7521
 */
#include "main_init.h"


void MAIN_INIT(void)
{
	RCC_INIT();

	HALL_INIT();
	MIP_SPEED_EST_INIT();
	//MOC_SPEED_CTRL_INIT();

	USART2_UART_Init();
	FMSTR_Init();

	ADC_INIT();
	BLDC_PWM_INIT();
}



