/*
 * moc_speed_ctrl.c
 *
 *  Created on: 16 iun. 2018
 *      Author: AxinteA
 */
#include "moc_speed_ctrl.h"

void MOC_SPEED_CTRL_INIT()
{
   MOC_SPEED_CTRL.OUTPUT_VOLT=0;
   MOC_SPEED_CTRL.OUTPUT_VOLT_K1=0;
   MOC_SPEED_CTRL.ERROR_SPEED=0;
   MOC_SPEED_CTRL.P_GAIN=0.0000002;/*gains for speed controller when used only*/
   MOC_SPEED_CTRL.I_GAIN=0;

   Set_Moc_Req_Speed(50);

}

void MOC_SPEED_CTRL_MAIN()
{
   uint16_t to_be_applied_DTC;
   MOC_SPEED_CTRL.ERROR_SPEED = (float)(Get_Moc_Req_Speed()) - (float)(Get_Mip_Est_Speed());
   MOC_SPEED_CTRL.P_PART = (MOC_SPEED_CTRL.P_GAIN * MOC_SPEED_CTRL.ERROR_SPEED);
   MOC_SPEED_CTRL.I_PART = MOC_SPEED_CTRL.OUTPUT_VOLT_K1 + (MOC_SPEED_CTRL.I_GAIN * MOC_SPEED_CTRL.ERROR_SPEED)/Ts;
   MOC_SPEED_CTRL.OUTPUT_VOLT = MOC_SPEED_CTRL.P_PART + MOC_SPEED_CTRL.I_PART;

   MOC_SPEED_CTRL.OUTPUT_VOLT = LIM(MOC_SPEED_CTRL.OUTPUT_VOLT,(-DC_LINK/2.0F), (DC_LINK/2.0F));

   MOC_SPEED_CTRL.OUTPUT_VOLT_K1 = MOC_SPEED_CTRL.OUTPUT_VOLT;

   to_be_applied_DTC = (((DC_LINK/2.0F) + MOC_SPEED_CTRL.OUTPUT_VOLT)/DC_LINK)*PWM_MAX_TICKS;

   Set_Bldc_Pwm_U(to_be_applied_DTC);
   Set_Bldc_Pwm_V(to_be_applied_DTC);
   Set_Bldc_Pwm_W(to_be_applied_DTC);

#if 1
   Set_Bldc_Pwm_U(((float)Get_Moc_Req_Speed()/100.0F)*4199);
   Set_Bldc_Pwm_V(((float)Get_Moc_Req_Speed()/100.0F)*4199);
   Set_Bldc_Pwm_W(((float)Get_Moc_Req_Speed()/100.0F)*4199);
#endif

}
