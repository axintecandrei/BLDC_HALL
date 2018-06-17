/*
 * moc_speed_ctrl.c
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */
#include "moc_speed_ctrl.h"

void MOC_SPEED_CTRL_INIT()
{

}

void MOC_SPEED_CTRL_MAIN()
{
	Set_Bldc_Pwm_U(((float)50/100.0F)*4199);
	Set_Bldc_Pwm_V(((float)50/100.0F)*4199);
	Set_Bldc_Pwm_W(((float)50/100.0F)*4199);
	Set_Bldc_En_Gate(GATE_DISABLE);

}
